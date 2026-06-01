#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>

#include <gtest/gtest.h>

#include "DriverModule.h"
#include "FaceFrameReader.h"
#include "FaceOscPublisher.h"
#include "OscRouter.h"
#include "OscWire.h"
#include "Protocol.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace {

using namespace std::chrono_literals;

std::wstring QuoteArg(const std::wstring &arg)
{
    std::wstring out = L"\"";
    for (wchar_t ch : arg) {
        if (ch == L'"') out += L"\\\"";
        else out.push_back(ch);
    }
    out += L"\"";
    return out;
}

std::wstring Utf8ToWide(const std::string &value)
{
    if (value.empty()) return {};
    int needed = MultiByteToWideChar(CP_UTF8, 0, value.c_str(),
        static_cast<int>(value.size()), nullptr, 0);
    if (needed <= 0) return {};
    std::wstring out(static_cast<size_t>(needed), L'\0');
    MultiByteToWideChar(CP_UTF8, 0, value.c_str(),
        static_cast<int>(value.size()), out.data(), needed);
    return out;
}

std::filesystem::path CurrentExePath()
{
    std::wstring buf(32768, L'\0');
    DWORD n = GetModuleFileNameW(nullptr, buf.data(), static_cast<DWORD>(buf.size()));
    buf.resize(n);
    return std::filesystem::path(buf);
}

std::filesystem::path BuildRoot()
{
    std::filesystem::path dir = CurrentExePath().parent_path();
    // build/artifacts/Release/e2e_tests.exe -> build
    return dir.parent_path().parent_path();
}

std::filesystem::path FaceHostPath()
{
    return BuildRoot() / L"driver_wkopenvr" / L"resources" / L"facetracking" /
        L"host" / L"WKOpenVR.FaceModuleHost.exe";
}

std::filesystem::path FaceTestModuleArtifactDir()
{
    return BuildRoot() / L"artifacts" / L"FaceTestModule";
}

std::filesystem::path FaceTestModuleDllPath()
{
    return FaceTestModuleArtifactDir() / L"WKOpenVR.FaceTracking.TestModule.dll";
}

std::filesystem::path CaptionsHostPath()
{
    return BuildRoot() / L"driver_wkopenvr" / L"resources" / L"captions" /
        L"host" / L"WKOpenVR.CaptionsHost.exe";
}

std::filesystem::path MakeTempDir(const wchar_t *name)
{
    wchar_t tempBuf[MAX_PATH] = {};
    GetTempPathW(MAX_PATH, tempBuf);
    std::filesystem::path dir(tempBuf);
    dir /= std::wstring(L"WKOpenVR_E2E_") +
        std::to_wstring(GetCurrentProcessId()) + L"_" +
        std::to_wstring(GetTickCount64()) + L"_" + name;
    std::filesystem::create_directories(dir);
    return dir;
}

std::string ReadFileUtf8(const std::filesystem::path &path)
{
    std::ifstream f(path, std::ios::binary);
    return std::string(std::istreambuf_iterator<char>(f), {});
}

void WriteFileUtf8(const std::filesystem::path &path, const std::string &content)
{
    std::filesystem::create_directories(path.parent_path());
    std::ofstream f(path, std::ios::binary);
    f << content;
}

void CopyDirectoryContents(
    const std::filesystem::path &source,
    const std::filesystem::path &target)
{
    std::filesystem::create_directories(target);
    for (const auto &entry : std::filesystem::recursive_directory_iterator(source)) {
        const auto relative = std::filesystem::relative(entry.path(), source);
        const auto dst = target / relative;
        if (entry.is_directory()) {
            std::filesystem::create_directories(dst);
        } else if (entry.is_regular_file()) {
            std::filesystem::create_directories(dst.parent_path());
            std::filesystem::copy_file(
                entry.path(),
                dst,
                std::filesystem::copy_options::overwrite_existing);
        }
    }
}

struct ProcessResult
{
    DWORD exitCode = 0;
    bool timedOut = false;
};

class RunningProcess
{
public:
    ~RunningProcess()
    {
        if (process_ != nullptr) {
            TerminateProcess(process_, 0xE2E00002);
            WaitForSingleObject(process_, 5000);
            CloseHandle(process_);
        }
    }

    bool Start(const std::filesystem::path &exe,
        const std::vector<std::wstring> &args)
    {
        std::wstring commandLine = QuoteArg(exe.wstring());
        for (const auto &arg : args) {
            commandLine += L" ";
            commandLine += QuoteArg(arg);
        }

        STARTUPINFOW si{};
        si.cb = sizeof(si);
        PROCESS_INFORMATION pi{};

        BOOL ok = CreateProcessW(
            exe.wstring().c_str(),
            commandLine.data(),
            nullptr,
            nullptr,
            FALSE,
            CREATE_NO_WINDOW,
            nullptr,
            exe.parent_path().wstring().c_str(),
            &si,
            &pi);
        if (!ok) {
            exitCode_ = GetLastError();
            return false;
        }

        CloseHandle(pi.hThread);
        process_ = pi.hProcess;
        pid_ = pi.dwProcessId;
        return true;
    }

    bool Wait(DWORD timeoutMs)
    {
        if (!process_) return true;
        DWORD wait = WaitForSingleObject(process_, timeoutMs);
        if (wait == WAIT_TIMEOUT) return false;
        GetExitCodeProcess(process_, &exitCode_);
        CloseHandle(process_);
        process_ = nullptr;
        return true;
    }

    DWORD ExitCode() const { return exitCode_; }
    DWORD Pid() const { return pid_; }

private:
    HANDLE process_ = nullptr;
    DWORD pid_ = 0;
    DWORD exitCode_ = 0;
};

ProcessResult RunProcess(
    const std::filesystem::path &exe,
    const std::vector<std::wstring> &args,
    DWORD timeoutMs)
{
    std::wstring commandLine = QuoteArg(exe.wstring());
    for (const auto &arg : args) {
        commandLine += L" ";
        commandLine += QuoteArg(arg);
    }

    STARTUPINFOW si{};
    si.cb = sizeof(si);
    PROCESS_INFORMATION pi{};

    BOOL ok = CreateProcessW(
        exe.wstring().c_str(),
        commandLine.data(),
        nullptr,
        nullptr,
        FALSE,
        CREATE_NO_WINDOW,
        nullptr,
        exe.parent_path().wstring().c_str(),
        &si,
        &pi);
    if (!ok) {
        return ProcessResult{ GetLastError(), false };
    }

    CloseHandle(pi.hThread);
    DWORD wait = WaitForSingleObject(pi.hProcess, timeoutMs);
    ProcessResult result{};
    if (wait == WAIT_TIMEOUT) {
        result.timedOut = true;
        TerminateProcess(pi.hProcess, 0xE2E00001);
        WaitForSingleObject(pi.hProcess, 5000);
    }
    GetExitCodeProcess(pi.hProcess, &result.exitCode);
    CloseHandle(pi.hProcess);
    return result;
}

template<typename Predicate>
bool WaitUntil(Predicate pred, std::chrono::milliseconds timeout)
{
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    do {
        if (pred()) return true;
        std::this_thread::sleep_for(20ms);
    } while (std::chrono::steady_clock::now() < deadline);
    return pred();
}

struct DecodedOsc
{
    std::string address;
    std::string typetag;
    float floatValue = 0.0f;
    std::string stringValue;
    bool hasFloat = false;
    bool hasString = false;
};

class FakeVrchatReceiver
{
public:
    ~FakeVrchatReceiver() { Stop(); }

    bool Start()
    {
        WSADATA wd{};
        WSAStartup(MAKEWORD(2, 2), &wd);

        sock_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (sock_ == INVALID_SOCKET) return false;

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        addr.sin_port = 0;
        if (bind(sock_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == SOCKET_ERROR) {
            return false;
        }

        sockaddr_in bound{};
        int len = sizeof(bound);
        if (getsockname(sock_, reinterpret_cast<sockaddr*>(&bound), &len) == SOCKET_ERROR) {
            return false;
        }
        port_ = ntohs(bound.sin_port);
        running_ = true;
        thread_ = std::thread([this] { Run(); });
        return true;
    }

    void Stop()
    {
        running_ = false;
        if (thread_.joinable()) thread_.join();
        if (sock_ != INVALID_SOCKET) {
            closesocket(sock_);
            sock_ = INVALID_SOCKET;
        }
    }

    uint16_t Port() const { return port_; }

    bool WaitForFloat(const std::string &address, float &out,
        std::chrono::milliseconds timeout)
    {
        return WaitUntil([&] {
            for (const auto &msg : DecodeSnapshot()) {
                if (msg.address == address && msg.hasFloat) {
                    out = msg.floatValue;
                    return true;
                }
            }
            return false;
        }, timeout);
    }

    bool WaitForString(const std::string &address, std::string &out,
        std::string &typetag, std::chrono::milliseconds timeout)
    {
        return WaitUntil([&] {
            for (const auto &msg : DecodeSnapshot()) {
                if (msg.address == address && msg.hasString) {
                    out = msg.stringValue;
                    typetag = msg.typetag;
                    return true;
                }
            }
            return false;
        }, timeout);
    }

private:
    SOCKET sock_ = INVALID_SOCKET;
    uint16_t port_ = 0;
    std::atomic<bool> running_{ false };
    std::thread thread_;
    std::mutex mutex_;
    std::vector<std::vector<uint8_t>> packets_;

    void Run()
    {
        while (running_) {
            fd_set readSet;
            FD_ZERO(&readSet);
            FD_SET(sock_, &readSet);
            timeval tv{};
            tv.tv_sec = 0;
            tv.tv_usec = 50000;
            int ready = select(0, &readSet, nullptr, nullptr, &tv);
            if (ready <= 0) continue;

            uint8_t buf[1024];
            int got = recvfrom(sock_, reinterpret_cast<char*>(buf), sizeof(buf),
                0, nullptr, nullptr);
            if (got <= 0) continue;
            std::lock_guard<std::mutex> lk(mutex_);
            packets_.push_back(std::vector<uint8_t>(buf, buf + got));
        }
    }

    std::vector<DecodedOsc> DecodeSnapshot()
    {
        std::vector<std::vector<uint8_t>> packets;
        {
            std::lock_guard<std::mutex> lk(mutex_);
            packets = packets_;
        }

        std::vector<DecodedOsc> out;
        for (const auto &packet : packets) {
            oscrouter::OscDispatch(packet.data(), packet.size(),
                [&](const char *addr, const char *tag,
                    const uint8_t *args, size_t argSize) {
                    DecodedOsc msg;
                    msg.address = addr ? addr : "";
                    msg.typetag = tag ? tag : "";
                    oscrouter::OscReader reader(args, argSize);
                    if (msg.typetag == ",f") {
                        msg.floatValue = reader.ReadFloat();
                        msg.hasFloat = reader.IsValid();
                    } else if (msg.typetag.rfind(",s", 0) == 0) {
                        const char *s = reader.ReadStr();
                        if (reader.IsValid() && s) {
                            msg.stringValue = s;
                            msg.hasString = true;
                        }
                    }
                    out.push_back(std::move(msg));
                });
        }
        return out;
    }
};

class RouterHarness
{
public:
    bool Start()
    {
        if (!receiver.Start()) return false;
        router.SetSendEndpointForTesting("127.0.0.1", receiver.Port());
        DriverModuleContext ctx{};
        return router.Init(ctx);
    }

    void Stop()
    {
        router.Shutdown();
        receiver.Stop();
    }

    FakeVrchatReceiver receiver;
    oscrouter::OscRouter router;
};

void CborAppendText(std::string &out, const std::string &value)
{
    if (value.size() < 24) {
        out.push_back(static_cast<char>(0x60 | value.size()));
    } else {
        out.push_back(static_cast<char>(0x78));
        out.push_back(static_cast<char>(value.size() & 0xff));
    }
    out.append(value);
}

std::string EncodeFaceHostMessage(
    const std::string &type,
    const std::string &uuid = {})
{
    std::string body;
    body.push_back(static_cast<char>(type == "SelectModule" ? 0xA2 : 0xA1));
    CborAppendText(body, "type");
    CborAppendText(body, type);
    if (type == "SelectModule") {
        CborAppendText(body, "uuid");
        CborAppendText(body, uuid);
    }

    std::string wire;
    const uint32_t len = static_cast<uint32_t>(body.size());
    wire.push_back(static_cast<char>(len & 0xff));
    wire.push_back(static_cast<char>((len >> 8) & 0xff));
    wire.push_back(static_cast<char>((len >> 16) & 0xff));
    wire.push_back(static_cast<char>((len >> 24) & 0xff));
    wire.append(body);
    return wire;
}

bool SendFaceHostMessage(const std::wstring &pipeName, const std::string &wire)
{
    if (!WaitNamedPipeW(pipeName.c_str(), 5000)) return false;
    HANDLE h = CreateFileW(
        pipeName.c_str(),
        GENERIC_WRITE,
        0,
        nullptr,
        OPEN_EXISTING,
        FILE_ATTRIBUTE_NORMAL,
        nullptr);
    if (h == INVALID_HANDLE_VALUE) return false;

    DWORD written = 0;
    BOOL ok = WriteFile(h, wire.data(), static_cast<DWORD>(wire.size()),
        &written, nullptr);
    CloseHandle(h);
    return ok && written == wire.size();
}

void CreateFakeFaceModule(
    const std::filesystem::path &modulesDir,
    const std::string &uuid,
    const std::string &version)
{
    auto moduleDir = modulesDir / Utf8ToWide(uuid) / Utf8ToWide(version);
    WriteFileUtf8(moduleDir / L"manifest.json",
        "{\n"
        "  \"schema\": 1,\n"
        "  \"uuid\": \"" + uuid + "\",\n"
        "  \"name\": \"E2E Fake Face Module\",\n"
        "  \"vendor\": \"WKOpenVR Tests\",\n"
        "  \"version\": \"" + version + "\",\n"
        "  \"sdk_version\": \"1.0.0\",\n"
        "  \"min_host_version\": \"1.0.0\",\n"
        "  \"supported_hmds\": [\"any\"],\n"
        "  \"capabilities\": [\"eye\", \"expression\"],\n"
        "  \"platforms\": [\"win-x64\"],\n"
        "  \"entry_assembly\": \"FakeFaceModule.dll\",\n"
        "  \"entry_type\": \"Fake.Face.Module\",\n"
        "  \"dependencies\": [],\n"
        "  \"payload_sha256\": \"\",\n"
        "  \"payload_size\": 0\n"
        "}\n");
    WriteFileUtf8(moduleDir / L"assemblies" / L"FakeFaceModule.dll", "");
}

void CreateLegacyVrcftCompatFaceModule(
    const std::filesystem::path &modulesDir,
    const std::string &uuid,
    const std::string &version)
{
    auto moduleDir = modulesDir / Utf8ToWide(uuid) / Utf8ToWide(version);
    WriteFileUtf8(moduleDir / L"manifest.json",
        "{\n"
        "  \"schema\": 1,\n"
        "  \"uuid\": \"" + uuid + "\",\n"
        "  \"name\": \"E2E Legacy VRCFT Compat Module\",\n"
        "  \"vendor\": \"WKOpenVR Tests\",\n"
        "  \"version\": \"" + version + "\",\n"
        "  \"sdk_version\": \"1.0.0\",\n"
        "  \"min_host_version\": \"1.0.0\",\n"
        "  \"supported_hmds\": [\"any\"],\n"
        "  \"capabilities\": [\"eye\", \"expression\"],\n"
        "  \"platforms\": [\"win-x64\"],\n"
        "  \"entry_assembly\": \"OpenVRPair.FaceTracking.VrcftCompat.dll\",\n"
        "  \"entry_type\": \"OpenVRPair.FaceTracking.VrcftCompat.ReflectingExtTrackingModuleAdapter\",\n"
        "  \"dependencies\": [],\n"
        "  \"payload_sha256\": \"\",\n"
        "  \"payload_size\": 0\n"
        "}\n");
    WriteFileUtf8(moduleDir / L"assemblies" / L"bridge.json",
        "{ \"upstream_assembly\": \"VirtualDesktop.FaceTracking.dll\" }\n");
    WriteFileUtf8(moduleDir / L"assemblies" / L"VirtualDesktop.FaceTracking.dll", "");
}

void CreateLoadableTestFaceModule(
    const std::filesystem::path &modulesDir,
    const std::string &uuid,
    const std::string &version)
{
    auto moduleDir = modulesDir / Utf8ToWide(uuid) / Utf8ToWide(version);
    WriteFileUtf8(moduleDir / L"manifest.json",
        "{\n"
        "  \"schema\": 1,\n"
        "  \"uuid\": \"" + uuid + "\",\n"
        "  \"name\": \"E2E Loadable Face Module\",\n"
        "  \"vendor\": \"WKOpenVR Tests\",\n"
        "  \"version\": \"" + version + "\",\n"
        "  \"sdk_version\": \"1.0.0\",\n"
        "  \"min_host_version\": \"1.0.0\",\n"
        "  \"supported_hmds\": [\"any\"],\n"
        "  \"capabilities\": [\"eye\", \"expression\"],\n"
        "  \"platforms\": [\"win-x64\"],\n"
        "  \"entry_assembly\": \"WKOpenVR.FaceTracking.TestModule.dll\",\n"
        "  \"entry_type\": \"WKOpenVR.FaceTracking.TestModule.DeterministicFaceModule\",\n"
        "  \"dependencies\": [],\n"
        "  \"payload_sha256\": \"\",\n"
        "  \"payload_size\": 0\n"
        "}\n");
    CopyDirectoryContents(FaceTestModuleArtifactDir(), moduleDir / L"assemblies");
}

} // namespace

TEST(E2E, FaceHostFakeFramesReachFakeVrchat)
{
    ASSERT_TRUE(std::filesystem::exists(FaceHostPath()))
        << "Face host missing at " << FaceHostPath().string();

    RouterHarness harness;
    ASSERT_TRUE(harness.Start());

    auto temp = MakeTempDir(L"face");
    const std::string shmemName = "WKOpenVR_E2E_Face_" +
        std::to_string(GetCurrentProcessId()) + "_" +
        std::to_string(GetTickCount64());

    protocol::FaceTrackingFrameShmem shmem;
    ASSERT_TRUE(shmem.Create(shmemName.c_str()));

    auto statusPath = temp / L"face_status.json";
    auto logPath = temp / L"face_host.log";
    // 60s budget covers .NET cold start + Defender real-time scanning of a
    // freshly-extracted module on GitHub-hosted Windows runners, which has
    // been observed taking 7-10s for the singleton+pipe phase alone. Healthy
    // local runs finish in <5s; the headroom catches real hangs without
    // false-failing on runner load.
    ProcessResult result = RunProcess(FaceHostPath(), {
        L"--e2e-fake-face-output",
        L"--shmem-name", Utf8ToWide(shmemName),
        L"--e2e-fake-frame-count", L"4",
        L"--e2e-fake-frame-interval-ms", L"1",
        L"--status-file", statusPath.wstring(),
        L"--log-file", logPath.wstring(),
        L"--debug-logging", L"1",
    }, 60000);

    ASSERT_FALSE(result.timedOut) << "face host log: " << ReadFileUtf8(logPath);
    ASSERT_EQ(result.exitCode, 0u) << "face host log: " << ReadFileUtf8(logPath);

    std::string status = ReadFileUtf8(statusPath);
    EXPECT_NE(status.find("\"phase\": \"e2e-fake-complete\""), std::string::npos);
    EXPECT_NE(status.find("\"frames_written\": 4"), std::string::npos);

    facetracking::FaceFrameReader reader;
    reader.Open(shmemName.c_str());
    protocol::FaceTrackingFrameBody frame{};
    ASSERT_TRUE(reader.TryRead(frame));
    EXPECT_NE(frame.flags & 0x1u, 0u);
    EXPECT_NE(frame.flags & 0x2u, 0u);
    EXPECT_NEAR(frame.eye_openness_l, 0.62f, 0.001f);
    EXPECT_NEAR(frame.expressions[26], 0.75f, 0.001f);
    // Upstream MouthCornerPullLeft (slot 57 in v5) bridges via the semantic
    // alias to ours.MouthSmileLeft (slot 45). End-to-end verification that
    // the host->wire->driver->remap chain delivers the value to the right
    // ours-slot.
    EXPECT_NEAR(frame.expressions[45], 0.25f, 0.001f);

    facetracking::FaceOscPublishCounts counts =
        facetracking::PublishFaceFrameOsc(frame);
    EXPECT_GT(counts.sent, 0u);
    EXPECT_EQ(counts.dropped, 0u);

    float jawLegacy = 0.0f;
    float jawV2 = 0.0f;
    float lidLegacy = 0.0f;
    float lidV2 = 0.0f;
    ASSERT_TRUE(harness.receiver.WaitForFloat(
        "/avatar/parameters/JawOpen", jawLegacy, 5000ms));
    ASSERT_TRUE(harness.receiver.WaitForFloat(
        "/avatar/parameters/v2/JawOpen", jawV2, 5000ms));
    ASSERT_TRUE(harness.receiver.WaitForFloat(
        "/avatar/parameters/LeftEyeLid", lidLegacy, 5000ms));
    ASSERT_TRUE(harness.receiver.WaitForFloat(
        "/avatar/parameters/v2/EyeOpenLeft", lidV2, 5000ms));

    EXPECT_NEAR(jawLegacy, 0.75f, 0.001f);
    EXPECT_NEAR(jawV2, 0.75f, 0.001f);
    EXPECT_NEAR(lidLegacy, 0.62f, 0.001f);
    EXPECT_NEAR(lidV2, 0.62f, 0.001f);

    // Dual-emit smoke check: an avatar built against legacy VRCFT binds to
    // MouthSmileLeft, an avatar built against modern v5 VRCFT binds to
    // MouthCornerPullLeft. Both names must carry the value the upstream
    // module wrote so neither avatar style gets stuck at zero.
    float smileLegacy = 0.0f, smileV2 = 0.0f;
    float cornerLegacy = 0.0f, cornerV2 = 0.0f;
    ASSERT_TRUE(harness.receiver.WaitForFloat(
        "/avatar/parameters/MouthSmileLeft", smileLegacy, 5000ms));
    ASSERT_TRUE(harness.receiver.WaitForFloat(
        "/avatar/parameters/v2/MouthSmileLeft", smileV2, 5000ms));
    ASSERT_TRUE(harness.receiver.WaitForFloat(
        "/avatar/parameters/MouthCornerPullLeft", cornerLegacy, 5000ms));
    ASSERT_TRUE(harness.receiver.WaitForFloat(
        "/avatar/parameters/v2/MouthCornerPullLeft", cornerV2, 5000ms));
    EXPECT_NEAR(smileLegacy,  0.25f, 0.001f);
    EXPECT_NEAR(smileV2,      0.25f, 0.001f);
    EXPECT_NEAR(cornerLegacy, 0.25f, 0.001f);
    EXPECT_NEAR(cornerV2,     0.25f, 0.001f);

    harness.Stop();
}

TEST(E2E, FaceHostLoadsTestModuleAndPublishesFrames)
{
    ASSERT_TRUE(std::filesystem::exists(FaceHostPath()))
        << "Face host missing at " << FaceHostPath().string();
    ASSERT_TRUE(std::filesystem::exists(FaceTestModuleDllPath()))
        << "Face test module missing at " << FaceTestModuleDllPath().string();

    RouterHarness harness;
    ASSERT_TRUE(harness.Start());

    auto temp = MakeTempDir(L"face_loadable_module");
    auto modulesDir = temp / L"modules";
    auto statusPath = temp / L"face_status.json";
    auto logPath = temp / L"face_host.log";
    std::filesystem::create_directories(modulesDir);

    const std::string uuid = "33333333-4444-5555-6666-777777777777";
    CreateLoadableTestFaceModule(modulesDir, uuid, "1.0.0");

    const std::string shmemName = "WKOpenVR_E2E_FaceLoadable_" +
        std::to_string(GetCurrentProcessId()) + "_" +
        std::to_string(GetTickCount64());
    protocol::FaceTrackingFrameShmem shmem;
    ASSERT_TRUE(shmem.Create(shmemName.c_str()));

    const std::wstring pipeName = L"\\\\.\\pipe\\WKOpenVR-E2E-FaceHost-" +
        std::to_wstring(GetCurrentProcessId()) + L"-" +
        std::to_wstring(GetTickCount64());

    RunningProcess host;
    ASSERT_TRUE(host.Start(FaceHostPath(), {
        L"--driver-handshake-pipe", pipeName,
        L"--shmem-name", Utf8ToWide(shmemName),
        L"--modules-dir", modulesDir.wstring(),
        L"--status-file", statusPath.wstring(),
        L"--log-file", logPath.wstring(),
        L"--debug-logging", L"1",
    })) << "CreateProcess failed: " << host.ExitCode();

    ASSERT_TRUE(WaitUntil([&] {
        std::string status = ReadFileUtf8(statusPath);
        return status.find("\"module_count\": 1") != std::string::npos;
    }, 30000ms)) << "host did not discover loadable test module. status: "
                 << ReadFileUtf8(statusPath) << " log: " << ReadFileUtf8(logPath);

    ASSERT_TRUE(SendFaceHostMessage(
        pipeName,
        EncodeFaceHostMessage("SelectModule", uuid)));

    facetracking::FaceFrameReader reader;
    reader.Open(shmemName.c_str());
    protocol::FaceTrackingFrameBody frame{};

    ASSERT_TRUE(WaitUntil([&] {
        if (!reader.TryRead(frame)) return false;
        return (frame.flags & 0x1u) != 0u &&
               (frame.flags & 0x2u) != 0u &&
               frame.eye_openness_l > 0.6f &&
               frame.expressions[26] > 0.7f &&
               frame.expressions[45] > 0.2f;
    }, 30000ms)) << "loadable test module did not publish expected frame. status: "
                 << ReadFileUtf8(statusPath) << " log: " << ReadFileUtf8(logPath);

    EXPECT_NEAR(frame.eye_openness_l, 0.62f, 0.001f);
    EXPECT_NEAR(frame.eye_openness_r, 0.58f, 0.001f);
    EXPECT_NEAR(frame.expressions[26], 0.75f, 0.001f);
    EXPECT_NEAR(frame.expressions[45], 0.25f, 0.001f);

    const std::string log = ReadFileUtf8(logPath);
    EXPECT_NE(log.find("RECV ReplyInit"), std::string::npos);
    EXPECT_NE(log.find("WKOpenVR E2E Face Module"), std::string::npos);
    EXPECT_NE(log.find("first non-zero shapes"), std::string::npos);

    facetracking::FaceOscPublishCounts counts =
        facetracking::PublishFaceFrameOsc(frame);
    EXPECT_GT(counts.sent, 0u);
    EXPECT_EQ(counts.dropped, 0u);

    float jawV2 = 0.0f;
    float lidV2 = 0.0f;
    float smileV2 = 0.0f;
    ASSERT_TRUE(harness.receiver.WaitForFloat(
        "/avatar/parameters/v2/JawOpen", jawV2, 5000ms));
    ASSERT_TRUE(harness.receiver.WaitForFloat(
        "/avatar/parameters/v2/EyeOpenLeft", lidV2, 5000ms));
    ASSERT_TRUE(harness.receiver.WaitForFloat(
        "/avatar/parameters/v2/MouthSmileLeft", smileV2, 5000ms));
    EXPECT_NEAR(jawV2, 0.75f, 0.001f);
    EXPECT_NEAR(lidV2, 0.62f, 0.001f);
    EXPECT_NEAR(smileV2, 0.25f, 0.001f);

    ASSERT_TRUE(SendFaceHostMessage(
        pipeName,
        EncodeFaceHostMessage("Shutdown")));
    ASSERT_TRUE(host.Wait(10000)) << "face host did not shut down. log: "
                                  << ReadFileUtf8(logPath);
    EXPECT_EQ(host.ExitCode(), 0u);

    harness.Stop();
}

TEST(E2E, FaceHostReloadsInstalledModulesAndDisablesSelection)
{
    ASSERT_TRUE(std::filesystem::exists(FaceHostPath()))
        << "Face host missing at " << FaceHostPath().string();

    auto temp = MakeTempDir(L"face_lifecycle");
    auto modulesDir = temp / L"modules";
    auto statusPath = temp / L"face_status.json";
    auto logPath = temp / L"face_host.log";
    std::filesystem::create_directories(modulesDir);

    const std::string shmemName = "WKOpenVR_E2E_FaceLifecycle_" +
        std::to_string(GetCurrentProcessId()) + "_" +
        std::to_string(GetTickCount64());
    protocol::FaceTrackingFrameShmem shmem;
    ASSERT_TRUE(shmem.Create(shmemName.c_str()));

    const std::wstring pipeName = L"\\\\.\\pipe\\WKOpenVR-E2E-FaceHost-" +
        std::to_wstring(GetCurrentProcessId()) + L"-" +
        std::to_wstring(GetTickCount64());

    RunningProcess host;
    ASSERT_TRUE(host.Start(FaceHostPath(), {
        L"--driver-handshake-pipe", pipeName,
        L"--shmem-name", Utf8ToWide(shmemName),
        L"--modules-dir", modulesDir.wstring(),
        L"--status-file", statusPath.wstring(),
        L"--log-file", logPath.wstring(),
        L"--debug-logging", L"1",
    })) << "CreateProcess failed: " << host.ExitCode();

    // 30s covers the same cold-startup tail that FaceHostFakeFramesReachFakeVrchat
    // bumped above; this WaitUntil only fires the first status write, which
    // happens after module discovery, so it's the tightest window on the
    // host's cold-start path.
    ASSERT_TRUE(WaitUntil([&] {
        return std::filesystem::exists(statusPath);
    }, 30000ms)) << "face host did not write status. log: "
                 << ReadFileUtf8(logPath);

    const std::string uuid = "11111111-2222-3333-4444-555555555555";
    CreateFakeFaceModule(modulesDir, uuid, "1.0.0");

    ASSERT_TRUE(SendFaceHostMessage(
        pipeName,
        EncodeFaceHostMessage("SelectModule", uuid)));

    ASSERT_TRUE(WaitUntil([&] {
        std::string status = ReadFileUtf8(statusPath);
        return status.find("\"module_count\": 1") != std::string::npos &&
               status.find("\"active_module_uuid\": \"" + uuid + "\"") != std::string::npos;
    }, 8000ms)) << "host did not reload/select new module. status: "
                << ReadFileUtf8(statusPath) << " log: " << ReadFileUtf8(logPath);

    ASSERT_TRUE(SendFaceHostMessage(
        pipeName,
        EncodeFaceHostMessage("SelectModule", "")));

    ASSERT_TRUE(WaitUntil([&] {
        std::string status = ReadFileUtf8(statusPath);
        return status.find("\"active_module_uuid\": \"\"") != std::string::npos;
    }, 8000ms)) << "host did not disable active module. status: "
                << ReadFileUtf8(statusPath) << " log: " << ReadFileUtf8(logPath);

    ASSERT_TRUE(SendFaceHostMessage(
        pipeName,
        EncodeFaceHostMessage("Shutdown")));
    ASSERT_TRUE(host.Wait(10000)) << "face host did not shut down. log: "
                                  << ReadFileUtf8(logPath);
    EXPECT_EQ(host.ExitCode(), 0u);
}

TEST(E2E, FaceHostResolvesLegacyBridgeManifestWithoutAdapterAssembly)
{
    ASSERT_TRUE(std::filesystem::exists(FaceHostPath()))
        << "Face host missing at " << FaceHostPath().string();

    auto temp = MakeTempDir(L"face_legacy_vrcft");
    auto modulesDir = temp / L"modules";
    auto statusPath = temp / L"face_status.json";
    auto logPath = temp / L"face_host.log";
    std::filesystem::create_directories(modulesDir);

    const std::string uuid = "22222222-3333-4444-5555-666666666666";
    CreateLegacyVrcftCompatFaceModule(modulesDir, uuid, "1.0.0");
    EXPECT_FALSE(std::filesystem::exists(
        modulesDir / Utf8ToWide(uuid) / L"1.0.0" / L"assemblies" /
        L"OpenVRPair.FaceTracking.VrcftCompat.dll"));

    const std::string shmemName = "WKOpenVR_E2E_FaceLegacy_" +
        std::to_string(GetCurrentProcessId()) + "_" +
        std::to_string(GetTickCount64());
    protocol::FaceTrackingFrameShmem shmem;
    ASSERT_TRUE(shmem.Create(shmemName.c_str()));

    const std::wstring pipeName = L"\\\\.\\pipe\\WKOpenVR-E2E-FaceHost-" +
        std::to_wstring(GetCurrentProcessId()) + L"-" +
        std::to_wstring(GetTickCount64());

    RunningProcess host;
    ASSERT_TRUE(host.Start(FaceHostPath(), {
        L"--driver-handshake-pipe", pipeName,
        L"--shmem-name", Utf8ToWide(shmemName),
        L"--modules-dir", modulesDir.wstring(),
        L"--status-file", statusPath.wstring(),
        L"--log-file", logPath.wstring(),
        L"--debug-logging", L"1",
    })) << "CreateProcess failed: " << host.ExitCode();

    ASSERT_TRUE(WaitUntil([&] {
        std::string status = ReadFileUtf8(statusPath);
        std::string log = ReadFileUtf8(logPath);
        return status.find("\"module_count\": 1") != std::string::npos &&
               log.find("bridge.json -> upstream_assembly=VirtualDesktop.FaceTracking.dll")
                   != std::string::npos;
    }, 30000ms)) << "host did not resolve legacy bridge manifest. status: "
                 << ReadFileUtf8(statusPath) << " log: " << ReadFileUtf8(logPath);

    ASSERT_TRUE(SendFaceHostMessage(
        pipeName,
        EncodeFaceHostMessage("Shutdown")));
    ASSERT_TRUE(host.Wait(10000)) << "face host did not shut down. log: "
                                  << ReadFileUtf8(logPath);
    EXPECT_EQ(host.ExitCode(), 0u);
}

TEST(E2E, CaptionsFakeOutputReachesFakeVrchat)
{
    ASSERT_TRUE(std::filesystem::exists(CaptionsHostPath()))
        << "Captions host missing at " << CaptionsHostPath().string();

    RouterHarness harness;
    ASSERT_TRUE(harness.Start());

    auto temp = MakeTempDir(L"captions");
    auto statusPath = temp / L"captions_status.json";
    const std::string expected = "WKOpenVR E2E translated line";

    ProcessResult result = RunProcess(CaptionsHostPath(), {
        L"--e2e-fake-chatbox", Utf8ToWide(expected),
        L"--status-file", statusPath.wstring(),
    }, 15000);

    ASSERT_FALSE(result.timedOut);
    ASSERT_EQ(result.exitCode, 0u);

    std::string status = ReadFileUtf8(statusPath);
    EXPECT_NE(status.find("\"phase\": \"e2e-fake-complete\""), std::string::npos);
    EXPECT_NE(status.find("\"packets_sent\": 1"), std::string::npos);

    std::string text;
    std::string typetag;
    ASSERT_TRUE(harness.receiver.WaitForString(
        "/chatbox/input", text, typetag, 5000ms));
    EXPECT_EQ(text, expected);
    EXPECT_EQ(typetag, ",sTF");

    harness.Stop();
}
