#define _CRT_SECURE_NO_DEPRECATE
#include "AdbController.h"

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>

// ---------------------------------------------------------------------------
// Path resolution
// ---------------------------------------------------------------------------
namespace {

// Returns the directory containing the running exe. The overlay is
// WKOpenVR.exe; adb lives at <exe-dir>/bin/adb/adb.exe.
std::string ResolveAdbPath()
{
    char buf[MAX_PATH] = {};
    const DWORD len = GetModuleFileNameA(nullptr, buf, MAX_PATH);
    if (len == 0 || len >= MAX_PATH) return {};

    std::filesystem::path exeDir = std::filesystem::path(buf).parent_path();
    std::filesystem::path candidate = exeDir / "bin" / "adb" / "adb.exe";
    return candidate.string();
}

// Convert a narrow UTF-8 string to a wide string for Win32 APIs.
std::wstring ToWide(const std::string& s)
{
    if (s.empty()) return {};
    int n = MultiByteToWideChar(CP_UTF8, 0, s.c_str(), -1, nullptr, 0);
    if (n <= 0) return {};
    std::wstring out(n, L'\0');
    MultiByteToWideChar(CP_UTF8, 0, s.c_str(), -1, out.data(), n);
    if (!out.empty() && out.back() == L'\0') out.pop_back();
    return out;
}

// Convert a wide string back to UTF-8 narrow.
std::string ToNarrow(const std::wstring& w)
{
    if (w.empty()) return {};
    int n = WideCharToMultiByte(CP_UTF8, 0, w.c_str(), -1, nullptr, 0, nullptr, nullptr);
    if (n <= 0) return {};
    std::string out(n, '\0');
    WideCharToMultiByte(CP_UTF8, 0, w.c_str(), -1, out.data(), n, nullptr, nullptr);
    if (!out.empty() && out.back() == '\0') out.pop_back();
    return out;
}

// Drain a read-end pipe handle into a std::string until EOF or the write end
// closes. Non-blocking: reads whatever is available, returns immediately when
// there is nothing more (handle should be drained after the process exits).
void DrainPipe(HANDLE hRead, std::string& out)
{
    char chunk[4096];
    DWORD avail = 0;
    while (PeekNamedPipe(hRead, nullptr, 0, nullptr, &avail, nullptr) && avail > 0) {
        DWORD got = 0;
        if (!ReadFile(hRead, chunk, static_cast<DWORD>(sizeof(chunk)), &got, nullptr)) break;
        out.append(chunk, got);
    }
}

// Blocking drain: read until ReadFile returns false (write end closed or error).
void DrainPipeFull(HANDLE hRead, std::string& out)
{
    char chunk[4096];
    DWORD got = 0;
    while (ReadFile(hRead, chunk, static_cast<DWORD>(sizeof(chunk)), &got, nullptr) && got > 0) {
        out.append(chunk, got);
    }
}

} // namespace

// ---------------------------------------------------------------------------
// AdbController
// ---------------------------------------------------------------------------

AdbController::AdbController()
    : m_adbPath(ResolveAdbPath())
{
    if (m_adbPath.empty()) {
        fprintf(stderr, "[adb] failed to resolve adb.exe path from module location\n");
    } else {
        fprintf(stderr, "[adb] resolved binary: %s\n", m_adbPath.c_str());
    }
}

AdbController::~AdbController() = default;

bool AdbController::BinaryAvailable() const
{
    if (m_adbPath.empty()) return false;
    std::error_code ec;
    return std::filesystem::exists(m_adbPath, ec);
}

std::string AdbController::ResolvedBinaryPath() const
{
    return m_adbPath;
}

// static
std::string AdbController::QuoteArg(const std::string& arg)
{
    // Quote if empty or if any space/tab/quote is present.
    bool needsQuote = arg.empty()
        || arg.find(' ')  != std::string::npos
        || arg.find('\t') != std::string::npos
        || arg.find('"')  != std::string::npos;

    if (!needsQuote) return arg;

    std::string out;
    out.reserve(arg.size() + 2);
    out.push_back('"');
    for (char c : arg) {
        if (c == '"') out.push_back('\\');
        out.push_back(c);
    }
    out.push_back('"');
    return out;
}

// static
std::string AdbController::TrimAscii(std::string s)
{
    // Trim trailing whitespace including \r\n.
    while (!s.empty() && static_cast<unsigned char>(s.back()) <= ' ') {
        s.pop_back();
    }
    // Trim leading whitespace.
    size_t start = 0;
    while (start < s.size() && static_cast<unsigned char>(s[start]) <= ' ') {
        ++start;
    }
    return s.substr(start);
}

AdbController::Result AdbController::Run(
    const std::vector<std::string>& args,
    std::chrono::milliseconds timeout)
{
    Result result;

    if (m_adbPath.empty()) {
        result.err = "adb path not resolved";
        return result;
    }

    // Build the command line: "<adb.exe>" [args...]
    std::string cmdNarrow = QuoteArg(m_adbPath);
    for (const auto& arg : args) {
        cmdNarrow += ' ';
        cmdNarrow += QuoteArg(arg);
    }
    std::wstring cmdLine = ToWide(cmdNarrow);
    std::wstring exePath = ToWide(m_adbPath);

    // Pipe handles: stdout and stderr are each an anonymous pipe.
    // The write ends are inherited by the child; the read ends stay in this process.
    HANDLE hStdoutRead  = nullptr, hStdoutWrite = nullptr;
    HANDLE hStderrRead  = nullptr, hStderrWrite = nullptr;

    SECURITY_ATTRIBUTES sa{};
    sa.nLength              = sizeof(sa);
    sa.bInheritHandle       = TRUE;
    sa.lpSecurityDescriptor = nullptr;

    if (!CreatePipe(&hStdoutRead, &hStdoutWrite, &sa, 0) ||
        !CreatePipe(&hStderrRead, &hStderrWrite, &sa, 0)) {
        fprintf(stderr, "[adb] CreatePipe failed err=%lu\n", GetLastError());
        if (hStdoutRead)  CloseHandle(hStdoutRead);
        if (hStdoutWrite) CloseHandle(hStdoutWrite);
        if (hStderrRead)  CloseHandle(hStderrRead);
        if (hStderrWrite) CloseHandle(hStderrWrite);
        return result;
    }

    // The parent's read ends must not be inherited.
    SetHandleInformation(hStdoutRead, HANDLE_FLAG_INHERIT, 0);
    SetHandleInformation(hStderrRead, HANDLE_FLAG_INHERIT, 0);

    STARTUPINFOW si{};
    si.cb          = sizeof(si);
    si.dwFlags     = STARTF_USESTDHANDLES;
    si.hStdInput   = GetStdHandle(STD_INPUT_HANDLE);
    si.hStdOutput  = hStdoutWrite;
    si.hStdError   = hStderrWrite;

    PROCESS_INFORMATION pi{};

    const BOOL ok = CreateProcessW(
        exePath.c_str(),
        cmdLine.data(),
        nullptr, nullptr,
        TRUE,                       // inherit handles (the pipe write ends)
        CREATE_NO_WINDOW,           // hide console
        nullptr, nullptr,
        &si, &pi);

    // Close the write ends in the parent immediately after the child inherits
    // them. If we hold them open, ReadFile on the read ends will never see EOF.
    CloseHandle(hStdoutWrite);
    CloseHandle(hStderrWrite);

    if (!ok) {
        DWORD err = GetLastError();
        fprintf(stderr, "[adb] CreateProcessW failed err=%lu cmd='%s'\n",
                err, cmdNarrow.c_str());
        CloseHandle(hStdoutRead);
        CloseHandle(hStderrRead);
        result.err = "CreateProcessW failed";
        return result;
    }
    CloseHandle(pi.hThread);

    fprintf(stderr, "[adb] spawned pid=%lu cmd='%s'\n",
            pi.dwProcessId, cmdNarrow.c_str());

    // Wait up to `timeout` for the process to finish, then terminate.
    const DWORD waitMs = static_cast<DWORD>(timeout.count() > 0 ? timeout.count() : 1);
    const DWORD waitResult = WaitForSingleObject(pi.hProcess, waitMs);

    if (waitResult == WAIT_TIMEOUT) {
        TerminateProcess(pi.hProcess, 1);
        WaitForSingleObject(pi.hProcess, 2000);
        result.timedOut = true;
        fprintf(stderr, "[adb] process timed out after %lldms; terminated\n",
                static_cast<long long>(timeout.count()));
    }

    // Drain pipes after the process has exited (or been terminated).
    DrainPipeFull(hStdoutRead, result.out);
    DrainPipeFull(hStderrRead, result.err);

    DWORD exitCode = static_cast<DWORD>(-1);
    GetExitCodeProcess(pi.hProcess, &exitCode);
    result.exitCode = static_cast<int>(exitCode);

    CloseHandle(pi.hProcess);
    CloseHandle(hStdoutRead);
    CloseHandle(hStderrRead);

    if (!result.timedOut) {
        fprintf(stderr, "[adb] pid exited code=%d\n", result.exitCode);
    }
    return result;
}

AdbController::Result AdbController::Shell(
    const std::string& cmd,
    std::chrono::milliseconds timeout)
{
    return Run({"shell", cmd}, timeout);
}

bool AdbController::Connect(const std::string& endpoint)
{
    Result r = Run({"connect", endpoint});
    if (r.timedOut) {
        fprintf(stderr, "[adb] connect timed out endpoint='%s'\n", endpoint.c_str());
        return false;
    }
    // adb connect prints "connected to <endpoint>" on success and on
    // already-connected, so a substring check is idempotent.
    const bool ok = (r.exitCode == 0)
        && (r.out.find("connected to") != std::string::npos);
    fprintf(stderr, "[adb] connect endpoint='%s' ok=%d exit=%d out='%s'\n",
            endpoint.c_str(), ok ? 1 : 0, r.exitCode, TrimAscii(r.out).c_str());
    return ok;
}

bool AdbController::Connected()
{
    Result r = Run({"get-state"});
    if (r.timedOut) return false;
    return (r.exitCode == 0) && (r.out.find("device") != std::string::npos);
}

bool AdbController::Disconnect(const std::string& endpoint)
{
    std::vector<std::string> args = {"disconnect"};
    if (!endpoint.empty()) {
        args.push_back(endpoint);
    }

    Result r = Run(args, std::chrono::seconds(5));
    if (r.timedOut) {
        fprintf(stderr, "[adb] disconnect timed out endpoint='%s'\n", endpoint.c_str());
        return false;
    }
    const bool ok = (r.exitCode == 0);
    fprintf(stderr, "[adb] disconnect endpoint='%s' ok=%d exit=%d out='%s'\n",
            endpoint.c_str(), ok ? 1 : 0, r.exitCode, TrimAscii(r.out).c_str());
    return ok;
}

bool AdbController::DisableWirelessAdb(const std::string& endpoint)
{
    std::vector<std::string> args;
    if (!endpoint.empty()) {
        args.push_back("-s");
        args.push_back(endpoint);
    }
    args.push_back("usb");

    Result r = Run(args, std::chrono::seconds(8));
    if (r.timedOut) {
        fprintf(stderr, "[adb] usb timed out endpoint='%s'\n", endpoint.c_str());
        return false;
    }
    const bool ok = (r.exitCode == 0);
    fprintf(stderr, "[adb] usb endpoint='%s' ok=%d exit=%d out='%s' err='%s'\n",
            endpoint.c_str(), ok ? 1 : 0, r.exitCode,
            TrimAscii(r.out).c_str(), TrimAscii(r.err).c_str());
    return ok;
}

bool AdbController::SetGuardianPaused(bool /*paused*/, int valueToWrite)
{
    Result r = Shell("setprop debug.oculus.guardian_pause " + std::to_string(valueToWrite));
    if (r.timedOut) {
        fprintf(stderr, "[adb] SetGuardianPaused timed out\n");
        return false;
    }
    return r.exitCode == 0;
}

int AdbController::GetGuardianPaused()
{
    Result r = Shell("getprop debug.oculus.guardian_pause");
    if (r.timedOut || r.exitCode != 0) return -1;

    std::string trimmed = TrimAscii(r.out);
    if (trimmed.empty()) return -1;

    try {
        return std::stoi(trimmed);
    } catch (...) {
        fprintf(stderr, "[adb] GetGuardianPaused: could not parse '%s'\n", trimmed.c_str());
        return -1;
    }
}
