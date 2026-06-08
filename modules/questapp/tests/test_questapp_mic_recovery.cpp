#include "QuestAppMicRecovery.h"

#include <gtest/gtest.h>

#include <chrono>
#include <string>
#include <utility>
#include <vector>

namespace {

class ScriptedAdbController : public AdbController
{
public:
	std::vector<AdbController::Result> results;
	std::vector<std::vector<std::string>> calls;

	ScriptedAdbController() : AdbController("adb.exe") {}

	AdbController::Result Run(const std::vector<std::string>& args, std::chrono::milliseconds /*timeout*/) override
	{
		calls.push_back(args);
		if (results.empty()) return {};
		AdbController::Result result = results.front();
		results.erase(results.begin());
		return result;
	}
};

AdbController::Result Success(std::string out = {})
{
	AdbController::Result result;
	result.exitCode = 0;
	result.out = std::move(out);
	return result;
}

AdbController::Result Failure(std::string err = {})
{
	AdbController::Result result;
	result.exitCode = 1;
	result.err = std::move(err);
	return result;
}

} // namespace

TEST(QuestAppMicRecovery, RequiresSelectedPackage)
{
	ScriptedAdbController adb;
	wkopenvr::questapp::QuestCompanionSettings settings;

	const auto result = wkopenvr::questapp::ResetSelectedAppMicrophonePermission(adb, settings);

	EXPECT_FALSE(result.ok);
	EXPECT_TRUE(adb.calls.empty());
}

TEST(QuestAppMicRecovery, RevokesAndGrantsRecordAudioForSelectedPackage)
{
	ScriptedAdbController adb;
	adb.results.push_back(Success("List of devices attached\r\n230YC01DBK003Q device product:quest model:Quest_3\r\n"));
	adb.results.push_back(Success());
	adb.results.push_back(Success());

	wkopenvr::questapp::QuestCompanionSettings settings;
	settings.selectedPackage = "VirtualDesktop.Android";

	const auto result = wkopenvr::questapp::ResetSelectedAppMicrophonePermission(adb, settings);

	EXPECT_TRUE(result.ok);
	ASSERT_EQ(adb.calls.size(), 3u);
	EXPECT_EQ(adb.calls[0], (std::vector<std::string>{"devices", "-l"}));
	EXPECT_EQ(adb.calls[1], (std::vector<std::string>{"-s", "230YC01DBK003Q", "shell", "pm", "revoke",
	                                                  "VirtualDesktop.Android", "android.permission.RECORD_AUDIO"}));
	EXPECT_EQ(adb.calls[2], (std::vector<std::string>{"-s", "230YC01DBK003Q", "shell", "pm", "grant",
	                                                  "VirtualDesktop.Android", "android.permission.RECORD_AUDIO"}));
}

TEST(QuestAppMicRecovery, ReportsGrantFailureAfterSuccessfulRevoke)
{
	ScriptedAdbController adb;
	adb.results.push_back(Success("List of devices attached\r\n230YC01DBK003Q device product:quest model:Quest_3\r\n"));
	adb.results.push_back(Success());
	adb.results.push_back(Failure("grant failed"));

	wkopenvr::questapp::QuestCompanionSettings settings;
	settings.selectedPackage = "com.valvesoftware.steamlinkvr";

	const auto result = wkopenvr::questapp::ResetSelectedAppMicrophonePermission(adb, settings);

	EXPECT_FALSE(result.ok);
	ASSERT_EQ(adb.calls.size(), 3u);
	EXPECT_EQ(adb.calls[2],
	          (std::vector<std::string>{"-s", "230YC01DBK003Q", "shell", "pm", "grant", "com.valvesoftware.steamlinkvr",
	                                    "android.permission.RECORD_AUDIO"}));
}
