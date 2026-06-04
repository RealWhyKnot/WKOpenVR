#include "AdbSetupWizard.h"

#include <gtest/gtest.h>

#include <string>
#include <utility>
#include <vector>

namespace {

class StubAdb : public AdbController
{
public:
	bool binaryExists = true;
	std::vector<AdbController::Result> results;
	mutable std::vector<std::vector<std::string>> calls;

	explicit StubAdb() : AdbController("C:\\fake\\adb.exe") {}

	bool BinaryAvailable() const override { return binaryExists; }

	AdbController::Result Run(const std::vector<std::string>& args, std::chrono::milliseconds) override
	{
		calls.push_back(args);
		if (calls.size() <= results.size()) return results[calls.size() - 1];
		AdbController::Result r;
		r.exitCode = 0;
		return r;
	}
};

AdbController::Result AdbResult(std::string out, int exitCode = 0, bool timedOut = false)
{
	AdbController::Result r;
	r.out = std::move(out);
	r.exitCode = exitCode;
	r.timedOut = timedOut;
	return r;
}

} // namespace

TEST(QuestAdbSetupWizard, CheckPlatformToolsRequiresAdb)
{
	StubAdb adb;
	adb.binaryExists = false;

	wkopenvr::questapp::AdbSetupWizard wiz(adb);
	const auto result = wiz.CheckPlatformTools();

	EXPECT_EQ(result.status, wkopenvr::questapp::StepStatus::Failed);
	EXPECT_EQ(wiz.currentStep(), wkopenvr::questapp::SetupStep::PlatformTools);
}

TEST(QuestAdbSetupWizard, CheckPlatformToolsAcceptsAdbVersionOutput)
{
	StubAdb adb;
	adb.results = {AdbResult("Android Debug Bridge version 1.0.41\n")};

	wkopenvr::questapp::AdbSetupWizard wiz(adb);
	const auto result = wiz.CheckPlatformTools();

	EXPECT_EQ(result.status, wkopenvr::questapp::StepStatus::Passed);
	EXPECT_EQ(wiz.currentStep(), wkopenvr::questapp::SetupStep::DeveloperMode);
	ASSERT_EQ(adb.calls.size(), 1u);
	EXPECT_EQ(adb.calls[0], (std::vector<std::string>{"version"}));
}

TEST(QuestAdbSetupWizard, UsbAuthorizationDetectsAuthorizedQuest)
{
	StubAdb adb;
	adb.results = {AdbResult(
	    "List of devices attached\n230YC01DBK003Q\tdevice product:seacliff model:Quest_Pro device:seacliff\n")};

	wkopenvr::questapp::AdbSetupWizard wiz(adb);
	const auto result = wiz.CheckUsbAuthorization();

	EXPECT_EQ(result.status, wkopenvr::questapp::StepStatus::Passed);
	EXPECT_EQ(wiz.currentStep(), wkopenvr::questapp::SetupStep::CompanionInstall);
}

TEST(QuestAdbSetupWizard, UsbAuthorizationAcceptsSpacePaddedAdbOutput)
{
	StubAdb adb;
	adb.results = {AdbResult("List of devices attached\n230YC01DBK003Q         device product:seacliff model:Quest_Pro "
	                         "device:seacliff transport_id:6\n")};

	wkopenvr::questapp::AdbSetupWizard wiz(adb);
	const auto result = wiz.CheckUsbAuthorization();

	EXPECT_EQ(result.status, wkopenvr::questapp::StepStatus::Passed);
}

TEST(QuestAdbSetupWizard, UsbAuthorizationReportsUnauthorized)
{
	StubAdb adb;
	adb.results = {AdbResult("List of devices attached\n230YC01DBK003Q\tunauthorized\n")};

	wkopenvr::questapp::AdbSetupWizard wiz(adb);
	const auto result = wiz.CheckUsbAuthorization();

	EXPECT_EQ(result.status, wkopenvr::questapp::StepStatus::Failed);
	EXPECT_NE(result.detail.find("unauthorized"), std::string::npos);
}

TEST(QuestAdbSetupWizard, UsbAuthorizationIgnoresWifiOnlyQuest)
{
	StubAdb adb;
	adb.results = {AdbResult("List of devices attached\n192.168.50.93:5555     device product:seacliff model:Quest_Pro "
	                         "device:seacliff transport_id:3\n")};

	wkopenvr::questapp::AdbSetupWizard wiz(adb);
	const auto result = wiz.CheckUsbAuthorization();

	EXPECT_EQ(result.status, wkopenvr::questapp::StepStatus::Failed);
	EXPECT_NE(result.detail.find("USB"), std::string::npos);
}

TEST(QuestAdbSetupWizard, UsbAuthorizationPassesWithUsbAndWifiQuest)
{
	StubAdb adb;
	adb.results = {
	    AdbResult("List of devices attached\n"
	              "230YC01DBK003Q         device product:seacliff model:Quest_Pro device:seacliff transport_id:6\n"
	              "192.168.50.93:5555     device product:seacliff model:Quest_Pro device:seacliff transport_id:3\n")};

	wkopenvr::questapp::AdbSetupWizard wiz(adb);
	const auto result = wiz.CheckUsbAuthorization();

	EXPECT_EQ(result.status, wkopenvr::questapp::StepStatus::Passed);
}

TEST(QuestAdbSetupWizard, DeviceParserReturnsUsbAndWifiSerials)
{
	const std::string output =
	    "List of devices attached\n"
	    "230YC01DBK003Q         device product:seacliff model:Quest_Pro device:seacliff transport_id:6\n"
	    "192.168.50.93:5555     device product:seacliff model:Quest_Pro device:seacliff transport_id:3\n";

	EXPECT_EQ(wkopenvr::questapp::FindAuthorizedUsbQuestSerial(output), "230YC01DBK003Q");
	EXPECT_EQ(wkopenvr::questapp::FindAuthorizedWifiQuestSerial(output), "192.168.50.93:5555");
}

TEST(QuestAdbSetupWizard, UsbAuthorizationDoesNotFailOnUnauthorizedWifiWhenUsbIsAuthorized)
{
	StubAdb adb;
	adb.results = {
	    AdbResult("List of devices attached\n"
	              "230YC01DBK003Q         device product:seacliff model:Quest_Pro device:seacliff transport_id:6\n"
	              "192.168.50.93:5555     unauthorized\n")};

	wkopenvr::questapp::AdbSetupWizard wiz(adb);
	const auto result = wiz.CheckUsbAuthorization();

	EXPECT_EQ(result.status, wkopenvr::questapp::StepStatus::Passed);
}

TEST(QuestAdbSetupWizard, InstallCompanionRequiresPairingKey)
{
	StubAdb adb;

	wkopenvr::questapp::AdbSetupWizard wiz(adb);
	const auto result = wiz.InstallCompanionApk("C:\\tmp\\app.apk", "");

	EXPECT_EQ(result.status, wkopenvr::questapp::StepStatus::Failed);
	EXPECT_TRUE(adb.calls.empty());
}

TEST(QuestAdbSetupWizard, InstallCompanionRejectsMalformedPairingKey)
{
	StubAdb adb;

	wkopenvr::questapp::AdbSetupWizard wiz(adb);
	const auto result = wiz.InstallCompanionApk("C:\\tmp\\app.apk", "abc123");

	EXPECT_EQ(result.status, wkopenvr::questapp::StepStatus::Failed);
	EXPECT_TRUE(adb.calls.empty());
}

TEST(QuestAdbSetupWizard, InstallCompanionInstallsAndStartsServiceWithKey)
{
	StubAdb adb;
	adb.results = {AdbResult("Success\n"), AdbResult("Starting service: Intent {}\n")};

	wkopenvr::questapp::AdbSetupWizard wiz(adb);
	const auto result =
	    wiz.InstallCompanionApk("C:\\tmp\\app.apk", "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef");

	EXPECT_EQ(result.status, wkopenvr::questapp::StepStatus::Passed);
	EXPECT_EQ(wiz.currentStep(), wkopenvr::questapp::SetupStep::Done);
	ASSERT_EQ(adb.calls.size(), 2u);
	EXPECT_EQ(adb.calls[0], (std::vector<std::string>{"install", "-r", "C:\\tmp\\app.apk"}));
	EXPECT_EQ(adb.calls[1][0], "shell");
	EXPECT_EQ(adb.calls[1][2], "start-foreground-service");
	EXPECT_EQ(adb.calls[1][4], "org.wkopenvr.quest/.QuestCompanionService");
	EXPECT_EQ(adb.calls[1][5], "--es");
	EXPECT_EQ(adb.calls[1][6], "wkopenvr_pairing_key");
}

TEST(QuestAdbSetupWizard, DeviceParserRejectsOfflineOnlyOutput)
{
	EXPECT_FALSE(
	    wkopenvr::questapp::DevicesOutputHasAuthorizedQuest("List of devices attached\n230YC01DBK003Q\toffline\n"));
	EXPECT_TRUE(wkopenvr::questapp::DevicesOutputHasAuthorizedQuest(
	    "List of devices attached\n230YC01DBK003Q\tdevice product:seacliff model:Quest_Pro\n"));
}
