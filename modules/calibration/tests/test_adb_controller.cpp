// Tests for AdbController.
//
// Real adb.exe is not present in CI. Subprocess tests (timeout, stdout capture)
// use cmd.exe as a stand-in for the binary by pointing m_adbPath at it via a
// thin test subclass. Parsing tests (GetGuardianPaused) use a stub subclass that
// overrides Run to inject canned output. Path-resolution tests call the real
// AdbController and inspect only the shape of the resolved path.

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
        : AdbController()
    {
        m_adbPath = std::move(path);
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
// BinaryPath_resolves_to_install_relative
//
// The test executable runs from the build artifacts directory. GetModuleFileNameA
// returns the test runner path, so the resolved adb path will be relative to
// that directory rather than the real install root. We verify only the structural
// shape (ends with bin\adb\adb.exe) -- BinaryAvailable() will return false in CI
// (the real adb binary is not there), which is expected.
// ---------------------------------------------------------------------------
TEST(AdbControllerTest, BinaryPath_resolves_to_install_relative)
{
    AdbController ctrl;
    const std::string path = ctrl.ResolvedBinaryPath();

    EXPECT_FALSE(path.empty())
        << "ResolvedBinaryPath returned empty";

    if (!path.empty()) {
        std::string normalised = path;
        std::replace(normalised.begin(), normalised.end(), '/', '\\');

        const std::string suffix = "bin\\adb\\adb.exe";
        const bool endsWith = normalised.size() >= suffix.size() &&
            normalised.compare(normalised.size() - suffix.size(),
                               suffix.size(), suffix) == 0;
        EXPECT_TRUE(endsWith)
            << "Expected path ending with 'bin\\adb\\adb.exe', got: " << path;
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

// ---------------------------------------------------------------------------
// GetGuardianPaused_parses_integer
//
// Uses a stub subclass to inject canned shell output and verifies that
// GetGuardianPaused returns the parsed integer value, or -1 on failure.
// ---------------------------------------------------------------------------
TEST(AdbControllerTest, GetGuardianPaused_parses_integer)
{
    // Valid integer output.
    {
        StubAdbController ctrl;
        ctrl.stubOut  = "1\n";
        ctrl.stubExit = 0;
        EXPECT_EQ(ctrl.GetGuardianPaused(), 1);
    }

    // Output that cannot be parsed as integer.
    {
        StubAdbController ctrl;
        ctrl.stubOut  = "garbage";
        ctrl.stubExit = 0;
        EXPECT_EQ(ctrl.GetGuardianPaused(), -1);
    }

    // Shell command fails (non-zero exit).
    {
        StubAdbController ctrl;
        ctrl.stubOut  = "";
        ctrl.stubExit = 1;
        EXPECT_EQ(ctrl.GetGuardianPaused(), -1);
    }

    // Whitespace around the value should be stripped before parsing.
    {
        StubAdbController ctrl;
        ctrl.stubOut  = "  42  \r\n";
        ctrl.stubExit = 0;
        EXPECT_EQ(ctrl.GetGuardianPaused(), 42);
    }

    // Timed out: should return -1.
    {
        StubAdbController ctrl;
        ctrl.stubOut     = "1";
        ctrl.stubExit    = 0;
        ctrl.stubTimedOut = true;
        EXPECT_EQ(ctrl.GetGuardianPaused(), -1);
    }
}
