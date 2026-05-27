#define _CRT_SECURE_NO_DEPRECATE
#include "AdbController.h"

#include "DiagnosticsLog.h"
#include "Win32Paths.h"

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <filesystem>
#include <regex>
#include <string>
#include <utility>
#include <vector>

// ---------------------------------------------------------------------------
// Path resolution
// ---------------------------------------------------------------------------
namespace {

// Quest App installs platform-tools into the user data tree instead of
// shipping adb in the main installer.
std::string ResolveAdbPath()
{
    const std::wstring toolsDir =
        openvr_pair::common::WkOpenVrSubdirectoryPath(L"questapp\\platform-tools", false);
    if (toolsDir.empty()) return {};
    std::filesystem::path candidate = std::filesystem::path(toolsDir) / "adb.exe";
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

std::string RedactAdbText(std::string text)
{
    if (text.empty()) return text;
    text = std::regex_replace(
        text,
        std::regex(R"(([^\s]+)\s+(device|unauthorized|offline)(\s|$))"),
        "<device> $2$3");
    text = std::regex_replace(
        text,
        std::regex(R"(\b\d{1,3}(?:\.\d{1,3}){3}(?::\d+)?\b)"),
        "<ip>");
    text = std::regex_replace(
        text,
        std::regex(R"(\b\d{6}\b)"),
        "<code>");
    return text;
}

std::string TrimAsciiForLog(std::string s)
{
    while (!s.empty() && static_cast<unsigned char>(s.back()) <= ' ') {
        s.pop_back();
    }
    size_t start = 0;
    while (start < s.size() && static_cast<unsigned char>(s[start]) <= ' ') {
        ++start;
    }
    return s.substr(start);
}

std::string TruncateForLog(std::string text, size_t limit = 420)
{
    text = TrimAsciiForLog(RedactAdbText(std::move(text)));
    if (text.size() <= limit) return text;
    text.resize(limit);
    text += "...";
    return text;
}

std::string ArgsForLog(const std::vector<std::string>& args)
{
    std::string out;
    for (size_t i = 0; i < args.size(); ++i) {
        if (!out.empty()) out += ' ';
        if (i == 2 && args.size() >= 3 && args[0] == "pair") {
            out += "<pairing-code>";
        } else if (i > 0 && args[i - 1] == "wkopenvr_pairing_key") {
            out += "<pairing-key>";
        } else {
            out += RedactAdbText(args[i]);
        }
    }
    return out;
}

std::string LowerAscii(std::string text)
{
    for (char& c : text) {
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }
    return text;
}

bool LooksLikeConnectionRefused(const std::string& out, const std::string& err)
{
    const std::string text = LowerAscii(out + "\n" + err);
    return text.find("refused") != std::string::npos
        || text.find("actively refused") != std::string::npos
        || text.find("cannot connect") != std::string::npos
        || text.find("failed to connect") != std::string::npos;
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
        openvr_pair::common::DiagnosticLog("adb", "resolve_binary failed");
    } else {
        fprintf(stderr, "[adb] resolved binary: %s\n", m_adbPath.c_str());
        openvr_pair::common::DiagnosticLog("adb", "resolve_binary ok");
    }
}

AdbController::AdbController(std::string adbPath)
    : m_adbPath(std::move(adbPath))
{
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

void AdbController::RefreshResolvedBinaryPath()
{
    m_adbPath = ResolveAdbPath();
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
        openvr_pair::common::DiagnosticLog("adb", "run skipped command='%s' reason=path_not_resolved",
                                           ArgsForLog(args).c_str());
        return result;
    }

    // Build the command line: "<adb.exe>" [args...]
    std::string cmdNarrow = QuoteArg(m_adbPath);
    for (const auto& arg : args) {
        cmdNarrow += ' ';
        cmdNarrow += QuoteArg(arg);
    }
    const std::string commandForLog = ArgsForLog(args);
    openvr_pair::common::DiagnosticLog("adb", "run_start command='%s' timeout_ms=%lld",
                                       commandForLog.c_str(),
                                       static_cast<long long>(timeout.count()));
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
        const DWORD pipeErr = GetLastError();
        fprintf(stderr, "[adb] CreatePipe failed err=%lu\n", pipeErr);
        openvr_pair::common::DiagnosticLog("adb", "run_pipe_failed command='%s' winerr=%lu",
                                           commandForLog.c_str(), pipeErr);
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
        openvr_pair::common::DiagnosticLog("adb", "run_spawn_failed command='%s' winerr=%lu",
                                           commandForLog.c_str(), err);
        CloseHandle(hStdoutRead);
        CloseHandle(hStderrRead);
        result.err = "CreateProcessW failed";
        return result;
    }
    CloseHandle(pi.hThread);

    fprintf(stderr, "[adb] spawned pid=%lu cmd='%s'\n",
            pi.dwProcessId, cmdNarrow.c_str());
    openvr_pair::common::DiagnosticLog("adb", "run_spawned command='%s' pid=%lu",
                                       commandForLog.c_str(), pi.dwProcessId);

    // Wait up to `timeout` for the process to finish, then terminate.
    const DWORD waitMs = static_cast<DWORD>(timeout.count() > 0 ? timeout.count() : 1);
    const DWORD waitResult = WaitForSingleObject(pi.hProcess, waitMs);

    if (waitResult == WAIT_TIMEOUT) {
        TerminateProcess(pi.hProcess, 1);
        WaitForSingleObject(pi.hProcess, 2000);
        result.timedOut = true;
        fprintf(stderr, "[adb] process timed out after %lldms; terminated\n",
                static_cast<long long>(timeout.count()));
        openvr_pair::common::DiagnosticLog("adb", "run_timeout command='%s' timeout_ms=%lld",
                                           commandForLog.c_str(),
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
    openvr_pair::common::DiagnosticLog("adb",
        "run_exit command='%s' exit=%d timed_out=%d stdout='%s' stderr='%s'",
        commandForLog.c_str(),
        result.exitCode,
        result.timedOut ? 1 : 0,
        TruncateForLog(result.out).c_str(),
        TruncateForLog(result.err).c_str());
    return result;
}

AdbController::Result AdbController::Shell(
    const std::string& cmd,
    std::chrono::milliseconds timeout)
{
    std::vector<std::string> args;
    if (!m_targetSerial.empty()) {
        args.push_back("-s");
        args.push_back(m_targetSerial);
    }
    args.push_back("shell");
    args.push_back(cmd);
    return Run(args, timeout);
}

bool AdbController::Connect(const std::string& endpoint)
{
    return Connect(endpoint, std::chrono::seconds(10));
}

bool AdbController::Connect(const std::string& endpoint, std::chrono::milliseconds timeout)
{
    m_lastConnectTimedOut = false;
    m_lastConnectWasRefused = false;

    Result r = Run({"connect", endpoint}, timeout);
    m_lastConnectTimedOut = r.timedOut;
    m_lastConnectWasRefused = LooksLikeConnectionRefused(r.out, r.err);
    if (r.timedOut) {
        fprintf(stderr, "[adb] connect timed out endpoint='%s'\n", endpoint.c_str());
        openvr_pair::common::DiagnosticLog("adb",
            "connect_result endpoint='%s' ok=0 timed_out=1 refused=%d exit=%d stdout='%s' stderr='%s'",
            RedactAdbText(endpoint).c_str(),
            m_lastConnectWasRefused ? 1 : 0,
            r.exitCode,
            TruncateForLog(r.out).c_str(),
            TruncateForLog(r.err).c_str());
        return false;
    }
    // adb connect prints "connected to <endpoint>" on success and on
    // already-connected, so a substring check is idempotent.
    const bool ok = (r.exitCode == 0)
        && (r.out.find("connected to") != std::string::npos);
    if (ok) {
        m_targetSerial = endpoint;
        openvr_pair::common::DiagnosticLog(
            "adb", "target_serial_set source=connect endpoint='%s'", endpoint.c_str());
    }
    openvr_pair::common::DiagnosticLog("adb",
        "connect_result endpoint='%s' ok=%d timed_out=0 refused=%d exit=%d stdout='%s' stderr='%s'",
        RedactAdbText(endpoint).c_str(),
        ok ? 1 : 0,
        m_lastConnectWasRefused ? 1 : 0,
        r.exitCode,
        TruncateForLog(r.out).c_str(),
        TruncateForLog(r.err).c_str());
    fprintf(stderr, "[adb] connect endpoint='%s' ok=%d exit=%d out='%s'\n",
            endpoint.c_str(), ok ? 1 : 0, r.exitCode, TrimAscii(r.out).c_str());
    return ok;
}

bool AdbController::LastConnectTimedOut() const
{
    return m_lastConnectTimedOut;
}

bool AdbController::LastConnectWasRefused() const
{
    return m_lastConnectWasRefused;
}

bool AdbController::Connected()
{
    std::vector<std::string> args;
    if (!m_targetSerial.empty()) {
        args.push_back("-s");
        args.push_back(m_targetSerial);
    }
    args.push_back("get-state");

    Result r = Run(args);
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
    if (ok && (endpoint.empty() || endpoint == m_targetSerial)) {
        m_targetSerial.clear();
        openvr_pair::common::DiagnosticLog(
            "adb", "target_serial_cleared source=disconnect endpoint='%s'", endpoint.c_str());
    }
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
    if (ok && !endpoint.empty() && endpoint == m_targetSerial) {
        m_targetSerial.clear();
        openvr_pair::common::DiagnosticLog(
            "adb", "target_serial_cleared source=usb endpoint='%s'", endpoint.c_str());
    }
    fprintf(stderr, "[adb] usb endpoint='%s' ok=%d exit=%d out='%s' err='%s'\n",
            endpoint.c_str(), ok ? 1 : 0, r.exitCode,
            TrimAscii(r.out).c_str(), TrimAscii(r.err).c_str());
    return ok;
}
