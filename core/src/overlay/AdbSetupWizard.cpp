#include "AdbSetupWizard.h"

#include <algorithm>
#include <cassert>
#include <cstdio>
#include <sstream>
#include <string>

namespace wkopenvr::adb {

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
namespace {

bool Contains(const std::string& haystack, const std::string& needle)
{
    return haystack.find(needle) != std::string::npos;
}

// True if the devices output has at least one non-header, non-empty line.
// Any status (unauthorized, offline, device) counts as "something present."
bool DevicesHasAnyEntry(const std::string& output)
{
    std::istringstream ss(output);
    std::string line;
    bool pastHeader = false;
    while (std::getline(ss, line)) {
        // Strip \r
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (line == "List of devices attached") {
            pastHeader = true;
            continue;
        }
        if (!pastHeader) continue;
        if (line.empty()) continue;
        return true;
    }
    return false;
}

// True if the devices output contains an authorized device line (status == "device").
bool DevicesHasAuthorized(const std::string& output)
{
    std::istringstream ss(output);
    std::string line;
    bool pastHeader = false;
    while (std::getline(ss, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (line == "List of devices attached") {
            pastHeader = true;
            continue;
        }
        if (!pastHeader) continue;
        if (line.empty()) continue;
        // A device line looks like: "<serial>\tdevice" or "<ip>:<port>\tdevice"
        if (Contains(line, "\tdevice") || line.find("device") != std::string::npos) {
            // Exclude lines that say "unauthorized" or "offline"
            if (!Contains(line, "unauthorized") && !Contains(line, "offline")) {
                return true;
            }
        }
    }
    return false;
}

} // anonymous namespace

// ---------------------------------------------------------------------------
// SetupWizard
// ---------------------------------------------------------------------------

SetupWizard::SetupWizard(AdbController& adb)
    : m_adb(adb)
    , m_step(WizardStep::Start)
{
    m_results.fill(StepResult{StepStatus::NotStarted, {}});
}

WizardStep SetupWizard::currentStep() const
{
    return m_step;
}

StepResult SetupWizard::stepResult(WizardStep step) const
{
    const int idx = static_cast<int>(step);
    if (idx < 0 || idx >= kStepCount) return {};
    return m_results[idx];
}

std::string SetupWizard::DiscoveredEndpoint() const
{
    return m_discoveredEndpoint;
}

bool SetupWizard::IsDone() const
{
    return m_step == WizardStep::Done;
}

void SetupWizard::Reset()
{
    m_step = WizardStep::Start;
    m_results.fill(StepResult{StepStatus::NotStarted, {}});
    m_discoveredEndpoint.clear();
    fprintf(stderr, "[adb-wizard] reset to Start\n");
}

StepResult SetupWizard::Commit(WizardStep step, StepResult result)
{
    const int idx = static_cast<int>(step);
    assert(idx >= 0 && idx < kStepCount);
    m_results[idx] = result;

    if (result.status == StepStatus::Passed) {
        // Advance to the next step.
        m_step = static_cast<WizardStep>(static_cast<int>(step) + 1);
        fprintf(stderr, "[adb-wizard] step %d passed -> now at step %d\n",
                idx, static_cast<int>(m_step));
    } else {
        // Stay on the current step so the UI can retry.
        fprintf(stderr, "[adb-wizard] step %d failed: %s\n",
                idx, result.detail.c_str());
    }
    return result;
}

// ---------------------------------------------------------------------------
// CheckBinary
// ---------------------------------------------------------------------------
StepResult SetupWizard::RunCheckBinary()
{
    StepResult r;
    r.status = StepStatus::InProgress;

    if (!m_adb.BinaryAvailable()) {
        r.status = StepStatus::Failed;
        r.detail = "adb.exe not found -- reinstall WKOpenVR.";
        return Commit(WizardStep::CheckBinary, r);
    }

    const auto result = m_adb.Run({"version"}, std::chrono::seconds(5));
    if (result.timedOut) {
        r.status = StepStatus::Failed;
        r.detail = "adb version timed out.";
        return Commit(WizardStep::CheckBinary, r);
    }
    if (result.exitCode != 0 || !Contains(result.out, "Android Debug Bridge")) {
        r.status = StepStatus::Failed;
        r.detail = "adb.exe did not return expected version output.";
        return Commit(WizardStep::CheckBinary, r);
    }

    r.status = StepStatus::Passed;
    r.detail = "adb binary OK.";
    return Commit(WizardStep::CheckBinary, r);
}

// ---------------------------------------------------------------------------
// CheckDevAccount
// ---------------------------------------------------------------------------
StepResult SetupWizard::RunCheckDevAccount()
{
    StepResult r;
    r.status = StepStatus::InProgress;

    const auto result = m_adb.Run({"devices"}, std::chrono::seconds(5));
    if (result.timedOut) {
        r.status = StepStatus::Failed;
        r.detail = "adb devices timed out.";
        return Commit(WizardStep::CheckDevAccount, r);
    }
    if (result.exitCode != 0) {
        r.status = StepStatus::Failed;
        r.detail = "adb devices failed (exit " + std::to_string(result.exitCode) + ").";
        return Commit(WizardStep::CheckDevAccount, r);
    }

    if (!DevicesHasAnyEntry(result.out)) {
        r.status = StepStatus::Failed;
        r.detail = "No Quest detected -- turn Developer Mode on in the Meta Horizon app, then reconnect USB.";
        return Commit(WizardStep::CheckDevAccount, r);
    }

    r.status = StepStatus::Passed;
    r.detail = "Device visible to adb (account approved).";
    return Commit(WizardStep::CheckDevAccount, r);
}

// ---------------------------------------------------------------------------
// CheckDevMode
// ---------------------------------------------------------------------------
StepResult SetupWizard::RunCheckDevMode()
{
    StepResult r;
    r.status = StepStatus::InProgress;

    const auto result = m_adb.Run({"devices"}, std::chrono::seconds(5));
    if (result.timedOut) {
        r.status = StepStatus::Failed;
        r.detail = "adb devices timed out.";
        return Commit(WizardStep::CheckDevMode, r);
    }
    if (result.exitCode != 0) {
        r.status = StepStatus::Failed;
        r.detail = "adb devices failed (exit " + std::to_string(result.exitCode) + ").";
        return Commit(WizardStep::CheckDevMode, r);
    }

    if (Contains(result.out, "unauthorized")) {
        r.status = StepStatus::Failed;
        r.detail = "Accept the USB debugging prompt on the headset. If it is not visible, open Settings > Developer, turn on MTP Notification, then reconnect USB.";
        return Commit(WizardStep::CheckDevMode, r);
    }

    if (!DevicesHasAuthorized(result.out)) {
        r.status = StepStatus::Failed;
        r.detail = "No authorized device found -- turn Developer Mode on in the Meta Horizon app and MTP Notification on in headset Developer settings.";
        return Commit(WizardStep::CheckDevMode, r);
    }

    r.status = StepStatus::Passed;
    r.detail = "Developer mode confirmed.";
    return Commit(WizardStep::CheckDevMode, r);
}

// ---------------------------------------------------------------------------
// UsbPair
// ---------------------------------------------------------------------------
StepResult SetupWizard::RunUsbPair()
{
    StepResult r;
    r.status = StepStatus::InProgress;

    const auto result = m_adb.Run({"devices"}, std::chrono::seconds(5));
    if (result.timedOut) {
        r.status = StepStatus::Failed;
        r.detail = "adb devices timed out.";
        return Commit(WizardStep::UsbPair, r);
    }

    if (!DevicesHasAuthorized(result.out)) {
        r.status = StepStatus::Failed;
        r.detail = "Quest not paired over USB -- accept the 'Allow USB debugging?' prompt in-headset, or toggle MTP Notification and reconnect USB.";
        return Commit(WizardStep::UsbPair, r);
    }

    r.status = StepStatus::Passed;
    r.detail = "USB pairing confirmed.";
    return Commit(WizardStep::UsbPair, r);
}

// ---------------------------------------------------------------------------
// WifiTcpip
// ---------------------------------------------------------------------------
StepResult SetupWizard::RunWifiTcpip()
{
    StepResult r;
    r.status = StepStatus::InProgress;

    const auto result = m_adb.Run({"tcpip", "5555"}, std::chrono::seconds(5));
    if (result.timedOut) {
        r.status = StepStatus::Failed;
        r.detail = "adb tcpip timed out.";
        return Commit(WizardStep::WifiTcpip, r);
    }
    if (result.exitCode != 0) {
        r.status = StepStatus::Failed;
        r.detail = "adb tcpip 5555 failed (exit " + std::to_string(result.exitCode) + ").";
        return Commit(WizardStep::WifiTcpip, r);
    }

    r.status = StepStatus::Passed;
    r.detail = "Wi-Fi ADB port set to 5555.";
    return Commit(WizardStep::WifiTcpip, r);
}

// ---------------------------------------------------------------------------
// WifiDiscover
// ---------------------------------------------------------------------------

// static
std::string SetupWizard::ParseIpFromIpRoute(const std::string& output)
{
    // Parse `ip route` output.
    // Look for a line containing "wlan" (preferred) or any non-loopback route.
    // The gateway/source IP we want is on the "src" field, or we can extract
    // the device's IP from the "dev wlan0 src <ip>" pattern.
    // Fallback: look for "default via <gw> dev wlan0 src <ip>" or
    //           "src <ip>" anywhere on a wlan line.
    std::istringstream ss(output);
    std::string line;
    while (std::getline(ss, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (!Contains(line, "wlan")) continue;

        // Try to find "src <ip>" on the line.
        const std::string srcToken = " src ";
        const size_t srcPos = line.find(srcToken);
        if (srcPos != std::string::npos) {
            std::string rest = line.substr(srcPos + srcToken.size());
            // The IP is the first whitespace-delimited token.
            std::istringstream tok(rest);
            std::string ip;
            tok >> ip;
            if (!ip.empty() && ip.find('.') != std::string::npos) {
                return ip;
            }
        }

        // Fallback: the route itself may be "192.168.x.0/24 dev wlan0 ..."
        // Extract the network address and strip the mask.
        std::istringstream tok(line);
        std::string first;
        tok >> first;
        const size_t slash = first.find('/');
        if (slash != std::string::npos) {
            first = first.substr(0, slash);
        }
        if (!first.empty() && first.find('.') != std::string::npos
            && first != "0.0.0.0") {
            // This is a network prefix, not the device IP -- skip.
            // We'll only fall back to this if there's nothing better.
        }
    }
    return {};
}

// static
std::string SetupWizard::ParseIpFromIfconfig(const std::string& output)
{
    // Parse `ifconfig wlan0` output.
    // Look for "inet addr:<ip>" (older busybox) or "inet <ip>" (newer iproute2-style).
    std::istringstream ss(output);
    std::string line;
    while (std::getline(ss, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();

        // "inet addr:192.168.x.y" (busybox style)
        {
            const std::string token = "inet addr:";
            const size_t pos = line.find(token);
            if (pos != std::string::npos) {
                std::string rest = line.substr(pos + token.size());
                std::istringstream tok(rest);
                std::string ip;
                tok >> ip;
                if (!ip.empty() && ip.find('.') != std::string::npos
                    && ip != "127.0.0.1") {
                    return ip;
                }
            }
        }

        // "inet 192.168.x.y" (newer style, also appears in some iproute output)
        {
            const std::string token = "inet ";
            const size_t pos = line.find(token);
            if (pos != std::string::npos) {
                std::string rest = line.substr(pos + token.size());
                std::istringstream tok(rest);
                std::string ip;
                tok >> ip;
                // Strip mask suffix if present ("192.168.x.y/24")
                const size_t slash = ip.find('/');
                if (slash != std::string::npos) ip = ip.substr(0, slash);
                if (!ip.empty() && ip.find('.') != std::string::npos
                    && ip != "127.0.0.1") {
                    return ip;
                }
            }
        }
    }
    return {};
}

StepResult SetupWizard::RunWifiDiscover()
{
    StepResult r;
    r.status = StepStatus::InProgress;
    m_discoveredEndpoint.clear();

    // Try `ip route` first.
    auto result = m_adb.Shell("ip route", std::chrono::seconds(5));
    std::string ip;

    if (!result.timedOut && result.exitCode == 0) {
        ip = ParseIpFromIpRoute(result.out);
    }

    // Fall back to `ifconfig wlan0`.
    if (ip.empty()) {
        result = m_adb.Shell("ifconfig wlan0", std::chrono::seconds(5));
        if (!result.timedOut && result.exitCode == 0) {
            ip = ParseIpFromIfconfig(result.out);
        }
    }

    if (ip.empty()) {
        r.status = StepStatus::Failed;
        r.detail = "Could not determine Quest Wi-Fi IP. Make sure the Quest is on Wi-Fi.";
        return Commit(WizardStep::WifiDiscover, r);
    }

    m_discoveredEndpoint = ip + ":5555";
    r.status = StepStatus::Passed;
    r.detail = "Quest IP: " + ip + " (endpoint: " + m_discoveredEndpoint + ").";
    return Commit(WizardStep::WifiDiscover, r);
}

// ---------------------------------------------------------------------------
// WifiPair
// ---------------------------------------------------------------------------
StepResult SetupWizard::RunWifiPair(const std::string& pairingHostPort,
                                    const std::string& pairingCode)
{
    StepResult r;
    r.status = StepStatus::InProgress;

    if (pairingHostPort.empty() || pairingCode.empty()) {
        r.status = StepStatus::Failed;
        r.detail = "Pairing host:port and 6-digit code are required.";
        return Commit(WizardStep::WifiPair, r);
    }

    // `adb pair <host:pairingPort>` reads the code from stdin.
    // We pass the code via a shell pipe because there is no stdin in our subprocess model:
    // `echo <code> | adb pair <host:port>` -- but adb pair reads from stdin.
    // The cleanest approach with our CreateProcessW model is to pass pairingCode via
    // stdin of the child. AdbController does not currently wire stdin; adb pair on
    // newer versions also accepts the code as a second argument on some builds.
    // Use: `adb pair <host:port> <code>` (supported in platform-tools >= 31).
    const auto result = m_adb.Run({"pair", pairingHostPort, pairingCode},
                                  std::chrono::seconds(10));

    if (result.timedOut) {
        r.status = StepStatus::Failed;
        r.detail = "adb pair timed out. Confirm the pairing code is still shown on the Quest.";
        return Commit(WizardStep::WifiPair, r);
    }
    if (result.exitCode != 0 || Contains(result.out, "Failed")) {
        r.status = StepStatus::Failed;
        r.detail = "Pairing failed. Verify the code and that the Quest pairing screen is open. "
                   "adb output: " + result.out;
        return Commit(WizardStep::WifiPair, r);
    }
    if (!Contains(result.out, "Successfully paired") &&
        !Contains(result.out, "paired") &&
        !Contains(result.out, "success")) {
        r.status = StepStatus::Failed;
        r.detail = "Unexpected adb pair output: " + result.out;
        return Commit(WizardStep::WifiPair, r);
    }

    r.status = StepStatus::Passed;
    r.detail = "Wi-Fi paired with " + pairingHostPort + ".";
    return Commit(WizardStep::WifiPair, r);
}

// ---------------------------------------------------------------------------
// WifiVerify
// ---------------------------------------------------------------------------
StepResult SetupWizard::RunWifiVerify()
{
    StepResult r;
    r.status = StepStatus::InProgress;

    const std::string endpoint = m_discoveredEndpoint;
    if (endpoint.empty()) {
        r.status = StepStatus::Failed;
        r.detail = "No endpoint discovered -- re-run the Discover step.";
        return Commit(WizardStep::WifiVerify, r);
    }

    if (!m_adb.Connect(endpoint)) {
        r.status = StepStatus::Failed;
        r.detail = "Failed to connect to " + endpoint +
                   ". Unplug the USB cable and ensure Quest is on Wi-Fi.";
        return Commit(WizardStep::WifiVerify, r);
    }

    const auto prop = m_adb.Shell("getprop ro.product.model", std::chrono::seconds(5));
    if (prop.timedOut || prop.exitCode != 0) {
        r.status = StepStatus::Failed;
        r.detail = "Connected but getprop timed out or failed.";
        return Commit(WizardStep::WifiVerify, r);
    }

    const std::string model = prop.out;
    if (!Contains(model, "Quest")) {
        r.status = StepStatus::Failed;
        r.detail = "Connected but device does not look like a Quest (model: " + model + ").";
        return Commit(WizardStep::WifiVerify, r);
    }

    r.status = StepStatus::Passed;
    r.detail = "Wi-Fi connection verified -- Quest model: " + [&]{
        std::string m = model;
        while (!m.empty() && (m.back() == '\r' || m.back() == '\n' || m.back() == ' '))
            m.pop_back();
        return m;
    }() + ".";
    return Commit(WizardStep::WifiVerify, r);
}

// ---------------------------------------------------------------------------
// ProbeGuardianPolarity
// ---------------------------------------------------------------------------
PolarityResult ProbeGuardianPolarity(AdbController& adb)
{
    PolarityResult out{};
    out.writtenValue    = 1;
    out.readBackValue   = -1;
    out.readMatchesWrite = false;

    // Write 1; read back.
    if (!adb.SetGuardianPaused(true, out.writtenValue)) {
        fprintf(stderr, "[adb-wizard] ProbeGuardianPolarity: SetGuardianPaused failed\n");
        return out;
    }

    out.readBackValue    = adb.GetGuardianPaused();
    out.readMatchesWrite = (out.readBackValue == out.writtenValue);

    fprintf(stderr, "[adb-wizard] ProbeGuardianPolarity: wrote=%d read=%d match=%d\n",
            out.writtenValue, out.readBackValue, out.readMatchesWrite ? 1 : 0);
    return out;
}

} // namespace wkopenvr::adb
