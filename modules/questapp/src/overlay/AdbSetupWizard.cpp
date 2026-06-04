#include "AdbSetupWizard.h"

#include "DiagnosticsLog.h"
#include "QuestAppConfig.h"

#include <algorithm>
#include <cctype>
#include <sstream>
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

std::string TrimAscii(std::string s)
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

bool ParseAdbDeviceLine(const std::string& line, std::string& serial, std::string& state)
{
	std::istringstream fields(line);
	if (!(fields >> serial >> state)) return false;
	state = LowerAscii(state);
	return state == "device" || state == "unauthorized" || state == "offline";
}

bool IsUsbSerial(const std::string& serial)
{
	return !serial.empty() && serial.find(':') == std::string::npos;
}

bool LineIdentifiesQuest(const std::string& lowerLine)
{
	return lowerLine.find("quest") != std::string::npos || lowerLine.find("seacliff") != std::string::npos ||
	       lowerLine.find("hollywood") != std::string::npos || lowerLine.find("eureka") != std::string::npos ||
	       lowerLine.find("panther") != std::string::npos || lowerLine.find("product:") != std::string::npos;
}

std::string FindAuthorizedQuestSerial(const std::string& output, bool requireUsb)
{
	std::istringstream lines(output);
	std::string line;
	while (std::getline(lines, line)) {
		line = TrimAscii(line);
		if (line.empty()) continue;

		std::string serial;
		std::string state;
		if (!ParseAdbDeviceLine(line, serial, state)) continue;
		if (IsUsbSerial(serial) != requireUsb) continue;

		const std::string lowerLine = LowerAscii(line);
		if (state == "device" && LineIdentifiesQuest(lowerLine)) {
			return serial;
		}
	}
	return {};
}

enum class UsbDeviceState
{
	None,
	Authorized,
	Unauthorized,
	Offline,
};

UsbDeviceState FindUsbQuestState(const std::string& output)
{
	bool sawUnauthorized = false;
	bool sawOffline = false;

	std::istringstream lines(output);
	std::string line;
	while (std::getline(lines, line)) {
		line = TrimAscii(line);
		if (line.empty()) continue;

		std::string serial;
		std::string state;
		if (!ParseAdbDeviceLine(line, serial, state)) continue;
		if (!IsUsbSerial(serial)) continue;

		const std::string lowerLine = LowerAscii(line);
		if (state == "device" && LineIdentifiesQuest(lowerLine)) {
			return UsbDeviceState::Authorized;
		}
		if (state == "unauthorized") {
			sawUnauthorized = true;
		}
		else if (state == "offline") {
			sawOffline = true;
		}
	}

	if (sawUnauthorized) return UsbDeviceState::Unauthorized;
	if (sawOffline) return UsbDeviceState::Offline;
	return UsbDeviceState::None;
}

} // namespace

bool DevicesOutputHasAuthorizedQuest(const std::string& output)
{
	return FindUsbQuestState(output) == UsbDeviceState::Authorized;
}

std::string FindAuthorizedUsbQuestSerial(const std::string& output)
{
	return FindAuthorizedQuestSerial(output, true);
}

std::string FindAuthorizedWifiQuestSerial(const std::string& output)
{
	return FindAuthorizedQuestSerial(output, false);
}

AdbSetupWizard::AdbSetupWizard(AdbController& adb) : adb_(adb)
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
	for (auto& result : results_)
		result = {};
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
	}
	else {
		step_ = step;
	}
	openvr_pair::common::DiagnosticLog("questapp", "setup_step step=%d status=%d detail='%s'", idx,
	                                   static_cast<int>(result.status), result.detail.c_str());
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

	const UsbDeviceState state = FindUsbQuestState(devices.out);
	if (state == UsbDeviceState::Unauthorized) {
		result.status = StepStatus::Failed;
		result.detail = "Quest is visible but unauthorized. Put on the headset and allow USB debugging.";
		return Commit(SetupStep::UsbAuthorization, result);
	}
	if (state == UsbDeviceState::Offline) {
		result.status = StepStatus::Failed;
		result.detail = "Quest is visible but offline. Replug USB, unlock the headset, and try again.";
		return Commit(SetupStep::UsbAuthorization, result);
	}
	if (state != UsbDeviceState::Authorized) {
		result.status = StepStatus::Failed;
		result.detail = "No authorized Quest was found over USB.";
		return Commit(SetupStep::UsbAuthorization, result);
	}

	result.status = StepStatus::Passed;
	result.detail = "USB debugging is authorized.";
	return Commit(SetupStep::UsbAuthorization, result);
}

StepResult AdbSetupWizard::InstallCompanionApk(const std::string& apkPath, const std::string& pairingKey)
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

	const auto configure =
	    adb_.Run({"shell", "am", "start-foreground-service", "-n", "org.wkopenvr.quest/.QuestCompanionService", "--es",
	              "wkopenvr_pairing_key", pairingKey});
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
