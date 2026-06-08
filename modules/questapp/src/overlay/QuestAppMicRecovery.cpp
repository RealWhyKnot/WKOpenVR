#include "QuestAppMicRecovery.h"

#include "AdbSetupWizard.h"

#include <chrono>
#include <string>
#include <vector>

namespace wkopenvr::questapp {
namespace {

constexpr const char* kRecordAudioPermission = "android.permission.RECORD_AUDIO";

std::vector<std::string> TargetAdbArgs(std::string serial, std::vector<std::string> args)
{
	if (!serial.empty()) {
		args.insert(args.begin(), serial);
		args.insert(args.begin(), "-s");
	}
	return args;
}

std::string FindPreferredQuestSerial(AdbController& adb)
{
	auto devices = adb.Run({"devices", "-l"}, std::chrono::seconds(8));
	if (devices.timedOut || devices.exitCode != 0) return {};

	std::string serial = FindAuthorizedUsbQuestSerial(devices.out);
	if (!serial.empty()) return serial;
	return FindAuthorizedWifiQuestSerial(devices.out);
}

bool CommandSucceeded(const AdbController::Result& result)
{
	return !result.timedOut && result.exitCode == 0;
}

} // namespace

OperationResult ResetSelectedAppMicrophonePermission(AdbController& adb, const QuestCompanionSettings& settings)
{
	OperationResult out;
	if (settings.selectedPackage.empty()) {
		out.message = "Select an app before resetting its microphone permission.";
		return out;
	}

	adb.RefreshResolvedBinaryPath();
	const std::string serial = FindPreferredQuestSerial(adb);
	const auto revoke =
	    adb.Run(TargetAdbArgs(serial, {"shell", "pm", "revoke", settings.selectedPackage, kRecordAudioPermission}),
	            std::chrono::seconds(10));
	if (!CommandSucceeded(revoke)) {
		out.message = "Could not deny microphone permission for the selected app over ADB.";
		return out;
	}

	const auto grant =
	    adb.Run(TargetAdbArgs(serial, {"shell", "pm", "grant", settings.selectedPackage, kRecordAudioPermission}),
	            std::chrono::seconds(10));
	if (!CommandSucceeded(grant)) {
		out.message = "Microphone permission was denied but could not be re-enabled. Re-enable it in headset settings.";
		return out;
	}

	out.ok = true;
	out.message = "Microphone permission reset for " + settings.selectedPackage + ".";
	return out;
}

} // namespace wkopenvr::questapp
