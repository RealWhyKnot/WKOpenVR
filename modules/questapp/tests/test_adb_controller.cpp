// Tests for AdbController.
//
// Real adb.exe is not present in CI. Subprocess tests use cmd.exe as a
// stand-in binary. Parsing tests use a stub subclass that overrides Run to
// inject canned output without spawning a process.

#include "AdbController.h"

#include <gtest/gtest.h>

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

#include <algorithm>
#include <cstring>
#include <filesystem>
#include <string>
#include <vector>

namespace {

// Returns the full path to cmd.exe for use as a stand-in binary.
std::string SystemCmdExe()
{
    char sysDir[MAX_PATH] = {};
    GetSystemDirectoryA(sysDir, MAX_PATH);
    return std::string(sysDir) + "\\cmd.exe";
}

// Test subclass that replaces the resolved adb binary path at construction.
// Used for subprocess tests where we want to launch cmd.exe instead of adb.
class OverriddenPathController : public AdbController {
public:
    explicit OverriddenPathController(std::string path)
        : AdbController(std::move(path))
    {
    }
};

// Stub subclass that overrides Run to inject canned results without spawning
// any subprocess. Used to test the parsing logic in GetGuardianPaused, Connect,
// Connected, etc. without requiring a real binary or OS call.
class StubAdbController : public AdbController {
public:
    std::string stubOut;
    std::string stubErr;
    int         stubExit   = 0;
    bool        stubTimedOut = false;
    std::vector<std::vector<std::string>> calls;

    AdbController::Result Run(const std::vector<std::string>& args,
                              std::chrono::milliseconds /*timeout*/) override
    {
        calls.push_back(args);
        AdbController::Result r;
        r.out      = stubOut;
        r.err      = stubErr;
        r.exitCode = stubExit;
        r.timedOut = stubTimedOut;
        return r;
    }
};

} // namespace

// ---------------------------------------------------------------------------
// BinaryPath_resolves_to_profile_platform_tools
//
// The test executable runs from the build artifacts directory. GetModuleFileNameA
// Quest App downloads platform-tools into LocalLow. We verify only the
// structural shape -- BinaryAvailable() may return false in CI, which is
// expected.
// ---------------------------------------------------------------------------
TEST(AdbControllerTest, BinaryPath_resolves_to_profile_platform_tools)
{
    AdbController ctrl;
    const std::string path = ctrl.ResolvedBinaryPath();

    EXPECT_FALSE(path.empty())
        << "ResolvedBinaryPath returned empty";

    if (!path.empty()) {
        std::string normalised = path;
        std::replace(normalised.begin(), normalised.end(), '/', '\\');

        const std::string suffix = "questapp\\platform-tools\\adb.exe";
        const bool endsWith = normalised.size() >= suffix.size() &&
            normalised.compare(normalised.size() - suffix.size(),
                               suffix.size(), suffix) == 0;
        EXPECT_TRUE(endsWith)
            << "Expected path ending with 'questapp\\platform-tools\\adb.exe', got: " << path;
    }
}

TEST(AdbControllerTest, Disconnect_targets_saved_tcp_endpoint)
{
    StubAdbController ctrl;
    ctrl.stubOut = "disconnected 192.168.1.10:5555\n";
    ctrl.stubExit = 0;

    EXPECT_TRUE(ctrl.Disconnect("192.168.1.10:5555"));
    ASSERT_EQ(ctrl.calls.size(), 1u);
    EXPECT_EQ(ctrl.calls[0],
              (std::vector<std::string>{"disconnect", "192.168.1.10:5555"}));
}

TEST(AdbControllerTest, DisableWirelessAdb_targets_saved_tcp_endpoint)
{
    StubAdbController ctrl;
    ctrl.stubOut = "restarting in USB mode\n";
    ctrl.stubExit = 0;

    EXPECT_TRUE(ctrl.DisableWirelessAdb("192.168.1.10:5555"));
    ASSERT_EQ(ctrl.calls.size(), 1u);
    EXPECT_EQ(ctrl.calls[0],
              (std::vector<std::string>{"-s", "192.168.1.10:5555", "usb"}));
}

TEST(AdbControllerTest, EnableWirelessAdb_uses_tcpip_port)
{
    StubAdbController ctrl;
    ctrl.stubOut = "restarting in TCP mode port: 5555\n";
    ctrl.stubExit = 0;

    EXPECT_TRUE(ctrl.EnableWirelessAdb(5555));
    ASSERT_EQ(ctrl.calls.size(), 1u);
    EXPECT_EQ(ctrl.calls[0],
              (std::vector<std::string>{"tcpip", "5555"}));
}

TEST(AdbControllerTest, EnableWirelessAdb_targets_saved_endpoint_when_present)
{
    StubAdbController ctrl;
    ctrl.stubOut = "connected to 192.168.1.10:5555\n";
    ctrl.stubExit = 0;

    EXPECT_TRUE(ctrl.Connect("192.168.1.10:5555"));

    ctrl.stubOut = "restarting in TCP mode port: 5555\n";
    EXPECT_TRUE(ctrl.EnableWirelessAdb(5555));

    ASSERT_EQ(ctrl.calls.size(), 2u);
    EXPECT_EQ(ctrl.calls[1],
              (std::vector<std::string>{
                  "-s", "192.168.1.10:5555", "tcpip", "5555"}));
}

TEST(AdbControllerTest, Shell_targets_last_connected_endpoint)
{
    StubAdbController ctrl;
    ctrl.stubOut = "connected to 192.168.1.10:5555\n";
    ctrl.stubExit = 0;

    EXPECT_TRUE(ctrl.Connect("192.168.1.10:5555"));

    ctrl.stubOut = "Quest 3\n";
    const auto result = ctrl.Shell("getprop ro.product.model");

    EXPECT_EQ(result.out, "Quest 3\n");
    ASSERT_EQ(ctrl.calls.size(), 2u);
    EXPECT_EQ(ctrl.calls[1],
              (std::vector<std::string>{
                  "-s", "192.168.1.10:5555", "shell", "getprop ro.product.model"}));
}

TEST(AdbControllerTest, Connected_targets_last_connected_endpoint)
{
    StubAdbController ctrl;
    ctrl.stubOut = "already connected to 192.168.1.10:5555\n";
    ctrl.stubExit = 0;

    EXPECT_TRUE(ctrl.Connect("192.168.1.10:5555"));

    ctrl.stubOut = "device\n";
    EXPECT_TRUE(ctrl.Connected());

    ASSERT_EQ(ctrl.calls.size(), 2u);
    EXPECT_EQ(ctrl.calls[1],
              (std::vector<std::string>{
                  "-s", "192.168.1.10:5555", "get-state"}));
}

// ---------------------------------------------------------------------------
// Run_with_fake_binary_times_out
//
// Points the controller at cmd.exe and runs a long-running command with a
// 150ms timeout. Verifies that timedOut==true is reported.
// ---------------------------------------------------------------------------
TEST(AdbControllerTest, Run_with_fake_binary_times_out)
{
    OverriddenPathController ctrl(SystemCmdExe());

    // ping -n 10 sends 10 ICMP packets with ~1s between each: blocks ~9s.
    const auto result = ctrl.Run({"/c", "ping -n 10 127.0.0.1"},
                                 std::chrono::milliseconds(150));
    EXPECT_TRUE(result.timedOut);
}

// ---------------------------------------------------------------------------
// Shell_parses_stdout
//
// Points the controller at cmd.exe and runs `echo hello`. Verifies that
// result.out contains "hello" and exit code is 0.
// ---------------------------------------------------------------------------
TEST(AdbControllerTest, Shell_parses_stdout)
{
    OverriddenPathController ctrl(SystemCmdExe());

    // Call Run directly: cmd.exe /c echo hello -> stdout "hello\r\n"
    const auto result = ctrl.Run({"/c", "echo hello"});
    EXPECT_EQ(result.exitCode, 0);
    EXPECT_FALSE(result.timedOut);
    EXPECT_NE(result.out.find("hello"), std::string::npos)
        << "stdout: '" << result.out << "'";
}
