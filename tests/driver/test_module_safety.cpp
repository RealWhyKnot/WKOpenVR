#include "ModuleSafety.h"

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

#include <gtest/gtest.h>

#include <filesystem>
#include <string>
#include <system_error>

namespace module_safety = openvr_pair::common::module_safety;

namespace {

std::wstring ReadEnvironmentVariable(const wchar_t* name, bool& present)
{
	present = false;
	DWORD len = GetEnvironmentVariableW(name, nullptr, 0);
	if (len == 0) return {};
	std::wstring value(len, L'\0');
	DWORD written = GetEnvironmentVariableW(name, value.data(), len);
	if (written == 0 || written >= len) return {};
	value.resize(written);
	present = true;
	return value;
}

std::wstring MakeTempDirectory()
{
	wchar_t tempRoot[MAX_PATH + 1] = {};
	EXPECT_NE(GetTempPathW(MAX_PATH, tempRoot), 0u);

	wchar_t tempName[MAX_PATH + 1] = {};
	EXPECT_NE(GetTempFileNameW(tempRoot, L"wks", 0, tempName), 0u);
	DeleteFileW(tempName);
	EXPECT_NE(CreateDirectoryW(tempName, nullptr), FALSE);
	return tempName;
}

class ModuleSafetyTest : public ::testing::Test
{
protected:
	void SetUp() override
	{
		previousRoot = ReadEnvironmentVariable(kRootEnv, hadPreviousRoot);
		root = MakeTempDirectory();
		ASSERT_NE(SetEnvironmentVariableW(kRootEnv, root.c_str()), FALSE);
	}

	void TearDown() override
	{
		SetEnvironmentVariableW(kRootEnv, hadPreviousRoot ? previousRoot.c_str() : nullptr);
		if (!root.empty()) {
			std::error_code ec;
			std::filesystem::remove_all(root, ec);
		}
	}

	static constexpr const wchar_t* kRootEnv = L"WKOPENVR_MODULE_SAFETY_ROOT";
	std::wstring root;
	std::wstring previousRoot;
	bool hadPreviousRoot = false;
};

} // namespace

TEST_F(ModuleSafetyTest, StaleActiveMarkerRecordsConcernWithoutImmediateDisable)
{
	const module_safety::ModuleSpec* spec = module_safety::FindByFlagFileName("enable_facetracking.flag");
	ASSERT_NE(spec, nullptr);

	EXPECT_FALSE(module_safety::HasActiveMarker(*spec));
	EXPECT_FALSE(module_safety::HasAutoDisabledMarker(*spec));

	EXPECT_TRUE(module_safety::MarkActive(*spec));
	EXPECT_TRUE(module_safety::HasActiveMarker(*spec));
	EXPECT_FALSE(module_safety::HasAutoDisabledMarker(*spec));

	const module_safety::LaunchAssessment assessment = module_safety::AssessLaunch(*spec);
	EXPECT_TRUE(assessment.had_stale_active);
	EXPECT_FALSE(assessment.had_stale_suspect);
	EXPECT_FALSE(assessment.auto_disabled);
	EXPECT_EQ(assessment.active_unclean_count, 1u);
	EXPECT_FALSE(module_safety::HasActiveMarker(*spec));
	EXPECT_FALSE(module_safety::HasAutoDisabledMarker(*spec));
}

TEST_F(ModuleSafetyTest, StaleSuspectMarkerAutoDisablesModule)
{
	const module_safety::ModuleSpec* spec = module_safety::FindByFlagFileName("enable_phantom.flag");
	ASSERT_NE(spec, nullptr);

	EXPECT_TRUE(module_safety::MarkActive(*spec));
	EXPECT_TRUE(module_safety::MarkSuspect(*spec, "pose_pipeline"));

	const module_safety::LaunchAssessment assessment = module_safety::AssessLaunch(*spec);
	EXPECT_TRUE(assessment.had_stale_active);
	EXPECT_TRUE(assessment.had_stale_suspect);
	EXPECT_TRUE(assessment.auto_disabled);
	EXPECT_EQ(assessment.suspect_unclean_count, 1u);
	EXPECT_EQ(assessment.reason, "unclean_exit_during_module_operation");
	EXPECT_FALSE(module_safety::HasActiveMarker(*spec));
	EXPECT_FALSE(module_safety::HasSuspectMarker(*spec));
	EXPECT_TRUE(module_safety::HasAutoDisabledMarker(*spec));
	EXPECT_EQ(module_safety::AutoDisabledReason(*spec), "unclean_exit_during_module_operation");
}

TEST_F(ModuleSafetyTest, RepeatedStaleActiveMarkersAutoDisableAfterBackoff)
{
	const module_safety::ModuleSpec* spec = module_safety::FindByFlagFileName("enable_inputhealth.flag");
	ASSERT_NE(spec, nullptr);

	for (unsigned i = 1; i <= 2; ++i) {
		EXPECT_TRUE(module_safety::MarkActive(*spec));
		const module_safety::LaunchAssessment assessment = module_safety::AssessLaunch(*spec);
		EXPECT_TRUE(assessment.had_stale_active);
		EXPECT_FALSE(assessment.auto_disabled);
		EXPECT_EQ(assessment.active_unclean_count, i);
	}

	EXPECT_TRUE(module_safety::MarkActive(*spec));
	const module_safety::LaunchAssessment assessment = module_safety::AssessLaunch(*spec, true);
	EXPECT_TRUE(assessment.had_stale_active);
	EXPECT_TRUE(assessment.auto_disabled);
	EXPECT_EQ(assessment.active_unclean_count, 3u);
	EXPECT_EQ(assessment.reason, "repeated_unclean_driver_exit");
	EXPECT_TRUE(module_safety::HasAutoDisabledMarker(*spec));
	EXPECT_EQ(module_safety::AutoDisabledReason(*spec), "repeated_unclean_driver_exit");
}

TEST_F(ModuleSafetyTest, CleanShutdownClearsRuntimeMarkersAndCounters)
{
	const module_safety::ModuleSpec* spec = module_safety::FindByFlagFileName("enable_smoothing.flag");
	ASSERT_NE(spec, nullptr);

	EXPECT_TRUE(module_safety::MarkActive(*spec));
	module_safety::AssessLaunch(*spec);
	EXPECT_TRUE(module_safety::MarkActive(*spec));
	EXPECT_TRUE(module_safety::MarkSuspect(*spec, "request"));
	EXPECT_TRUE(module_safety::MarkClean(*spec));

	EXPECT_FALSE(module_safety::HasActiveMarker(*spec));
	EXPECT_FALSE(module_safety::HasSuspectMarker(*spec));
	EXPECT_TRUE(module_safety::HasCleanMarker(*spec));
	EXPECT_FALSE(module_safety::HasAutoDisabledMarker(*spec));

	EXPECT_TRUE(module_safety::MarkActive(*spec));
	const module_safety::LaunchAssessment assessment = module_safety::AssessLaunch(*spec);
	EXPECT_EQ(assessment.active_unclean_count, 1u);
}

TEST_F(ModuleSafetyTest, ClearAutoDisabledForFlagReenablesModule)
{
	const module_safety::ModuleSpec* spec = module_safety::FindByFlagFileName("enable_phantom.flag");
	ASSERT_NE(spec, nullptr);

	EXPECT_TRUE(module_safety::MarkActive(*spec));
	EXPECT_TRUE(module_safety::MarkSuspect(*spec, "request"));
	EXPECT_TRUE(module_safety::MarkFault(*spec, "test_fault"));
	EXPECT_FALSE(module_safety::HasActiveMarker(*spec));
	EXPECT_FALSE(module_safety::HasSuspectMarker(*spec));
	EXPECT_TRUE(module_safety::HasAutoDisabledMarker(*spec));

	EXPECT_TRUE(module_safety::ClearAutoDisabledForFlag("enable_phantom.flag"));
	EXPECT_FALSE(module_safety::HasActiveMarker(*spec));
	EXPECT_FALSE(module_safety::HasSuspectMarker(*spec));
	EXPECT_FALSE(module_safety::HasAutoDisabledMarker(*spec));
}

TEST_F(ModuleSafetyTest, UnknownFlagDoesNotClearKnownMarkers)
{
	const module_safety::ModuleSpec* spec = module_safety::FindByFlagFileName("enable_smoothing.flag");
	ASSERT_NE(spec, nullptr);

	EXPECT_TRUE(module_safety::MarkFault(*spec, "test_fault"));
	EXPECT_FALSE(module_safety::ClearAutoDisabledForFlag("enable_missing.flag"));
	EXPECT_TRUE(module_safety::HasAutoDisabledMarker(*spec));
}
