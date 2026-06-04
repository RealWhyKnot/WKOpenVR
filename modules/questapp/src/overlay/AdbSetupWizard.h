#pragma once

#include "AdbController.h"

#include <array>
#include <string>

namespace wkopenvr::questapp {

enum class SetupStep
{
	Start = 0,
	PlatformTools,
	DeveloperMode,
	UsbAuthorization,
	CompanionInstall,
	Done,
};

enum class StepStatus
{
	NotStarted,
	Failed,
	Passed,
};

struct StepResult
{
	StepStatus status = StepStatus::NotStarted;
	std::string detail;
};

class AdbSetupWizard
{
public:
	explicit AdbSetupWizard(AdbController& adb);

	SetupStep currentStep() const;
	StepResult stepResult(SetupStep step) const;
	void Reset();
	bool IsDone() const;

	StepResult CheckPlatformTools();
	StepResult MarkDeveloperModeReady();
	StepResult CheckUsbAuthorization();
	StepResult InstallCompanionApk(const std::string& apkPath, const std::string& pairingKey);

private:
	AdbController& adb_;
	SetupStep step_ = SetupStep::Start;
	static constexpr int kStepCount = 6;
	std::array<StepResult, kStepCount> results_{};

	StepResult Commit(SetupStep step, StepResult result);
};

bool DevicesOutputHasAuthorizedQuest(const std::string& output);
std::string FindAuthorizedUsbQuestSerial(const std::string& output);
std::string FindAuthorizedWifiQuestSerial(const std::string& output);

} // namespace wkopenvr::questapp
