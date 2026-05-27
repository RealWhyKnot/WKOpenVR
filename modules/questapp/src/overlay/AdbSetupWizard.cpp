#include "AdbSetupWizard.h"

#include "DiagnosticsLog.h"
#include "QuestAppConfig.h"

#include <algorithm>
#include <cctype>
#include <string>

namespace wkopenvr::questapp {
namespace {

std::string LowerAscii(std::string text)
{
    for (char& c : text) {
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }
    return text;
}

bool ContainsNoCase(const std::string& text, const char* needle)
{
    if (!needle || !needle[0]) return true;
    return LowerAscii(text).find(LowerAscii(needle)) != std::string::npos;
}

} // namespace

bool DevicesOutputHasAuthorizedQuest(const std::string& output)
{
    const std::string lower = LowerAscii(output);
    if (lower.find("\tdevice") == std::string::npos) return false;
    return lower.find("quest") != std::string::npos
        || lower.find("seacliff") != std::string::npos
        || lower.find("hollywood") != std::string::npos
        || lower.find("eureka") != std::string::npos
        || lower.find("panther") != std::string::npos
        || lower.find("product:") != std::string::npos;
}

AdbSetupWizard::AdbSetupWizard(AdbController& adb)
    : adb_(adb)
{
    Reset();
}

SetupStep AdbSetupWizard::currentStep() const
{
    return step_;
}

StepResult AdbSetupWizard::stepResult(SetupStep step) const
{
    const int idx = static_cast<int>(step);
    if (idx < 0 || idx >= kStepCount) return {};
    return results_[idx];
}

void AdbSetupWizard::Reset()
{
    step_ = SetupStep::Start;
    for (auto& result : results_) result = {};
}

bool AdbSetupWizard::IsDone() const
{
    return step_ == SetupStep::Done;
}

StepResult AdbSetupWizard::Commit(SetupStep step, StepResult result)
{
    const int idx = static_cast<int>(step);
    if (idx >= 0 && idx < kStepCount) {
        results_[idx] = result;
    }
    if (result.status == StepStatus::Passed) {
        const int next = idx + 1;
        step_ = next >= kStepCount ? SetupStep::Done : static_cast<SetupStep>(next);
    } else {
        step_ = step;
    }
    openvr_pair::common::DiagnosticLog(
        "questapp",
        "setup_step step=%d status=%d detail='%s'",
        idx,
        static_cast<int>(result.status),
        result.detail.c_str());
    return result;
}

StepResult AdbSetupWizard::CheckPlatformTools()
{
    adb_.RefreshResolvedBinaryPath();
    StepResult result;
    if (!adb_.BinaryAvailable()) {
        result.status = StepStatus::Failed;
        result.detail = "ADB is not installed yet.";
        return Commit(SetupStep::PlatformTools, result);
    }

    const auto version = adb_.Run({"version"});
    if (version.timedOut || version.exitCode != 0 || !ContainsNoCase(version.out, "Android Debug Bridge")) {
        result.status = StepStatus::Failed;
        result.detail = "adb.exe is present but did not return a valid version.";
        return Commit(SetupStep::PlatformTools, result);
    }

    result.status = StepStatus::Passed;
    result.detail = "ADB is installed.";
    return Commit(SetupStep::PlatformTools, result);
}

StepResult AdbSetupWizard::MarkDeveloperModeReady()
{
    StepResult result;
    result.status = StepStatus::Passed;
    result.detail = "Developer Mode guide acknowledged.";
    return Commit(SetupStep::DeveloperMode, result);
}

StepResult AdbSetupWizard::CheckUsbAuthorization()
{
    StepResult result;
    const auto devices = adb_.Run({"devices", "-l"});
    if (devices.timedOut || devices.exitCode != 0) {
        result.status = StepStatus::Failed;
        result.detail = "ADB could not list devices. Reconnect USB and try again.";
        return Commit(SetupStep::UsbAuthorization, result);
    }

    const std::string lower = LowerAscii(devices.out);
    if (lower.find("unauthorized") != std::string::npos) {
        result.status = StepStatus::Failed;
        result.detail = "Quest is visible but unauthorized. Put on the headset and allow USB debugging.";
        return Commit(SetupStep::UsbAuthorization, result);
    }
    if (lower.find("offline") != std::string::npos) {
        result.status = StepStatus::Failed;
        result.detail = "Quest is visible but offline. Replug USB, unlock the headset, and try again.";
        return Commit(SetupStep::UsbAuthorization, result);
    }
    if (!DevicesOutputHasAuthorizedQuest(devices.out)) {
        result.status = StepStatus::Failed;
        result.detail = "No authorized Quest was found over USB.";
        return Commit(SetupStep::UsbAuthorization, result);
    }

    result.status = StepStatus::Passed;
    result.detail = "USB debugging is authorized.";
    return Commit(SetupStep::UsbAuthorization, result);
}

StepResult AdbSetupWizard::InstallCompanionApk(
    const std::string& apkPath,
    const std::string& pairingKey)
{
    StepResult result;
    if (apkPath.empty()) {
        result.status = StepStatus::Failed;
        result.detail = "Companion APK is not bundled.";
        return Commit(SetupStep::CompanionInstall, result);
    }
    if (!IsValidPairingKey(pairingKey)) {
        result.status = StepStatus::Failed;
        result.detail = "Install key is missing. Reinstall to create a new key.";
        return Commit(SetupStep::CompanionInstall, result);
    }

    const auto install = adb_.Run({"install", "-r", apkPath});
    if (install.timedOut || install.exitCode != 0 ||
        (!ContainsNoCase(install.out, "success") && !ContainsNoCase(install.err, "success"))) {
        result.status = StepStatus::Failed;
        result.detail = "Companion install failed.";
        return Commit(SetupStep::CompanionInstall, result);
    }

    const auto configure = adb_.Run({
        "shell", "am", "start-foreground-service",
        "-n", "org.wkopenvr.quest/.QuestCompanionService",
        "--es", "wkopenvr_pairing_key", pairingKey});
    if (configure.timedOut || configure.exitCode != 0) {
        result.status = StepStatus::Failed;
        result.detail = "Companion installed, but initial key setup did not confirm.";
        return Commit(SetupStep::CompanionInstall, result);
    }

    result.status = StepStatus::Passed;
    result.detail = "Companion app installed and paired.";
    return Commit(SetupStep::CompanionInstall, result);
}

} // namespace wkopenvr::questapp
