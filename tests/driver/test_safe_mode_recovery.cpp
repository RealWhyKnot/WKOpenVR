#include "JsonUtil.h"
#include "ModuleSafety.h"
#include "SteamVrControl.h"

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>
#include <system_error>
#include <vector>

namespace svc = openvr_pair::common::steamvr_control;
namespace module_safety = openvr_pair::common::module_safety;
namespace json = openvr_pair::common::json;

// =============================================================================
// Pure log parsing.
// =============================================================================

TEST(SafeModeParse, ExtractsBlockedDriversFromRealLog)
{
	// Verbatim lines from the user's vrserver.txt at the safe-mode lockout.
	const std::string log =
	    "Sun Jun 07 2026 20:43:28.075 [Info] - Found saved crash timestamp time 1780883006 v1780421623 uptime 300 -- "
	    "Using safe mode\n"
	    "Sun Jun 07 2026 20:43:28.700 [Warning] - Not loading driver VirtualDesktop because it was blocked by a "
	    "previous safe mode event\n"
	    "Sun Jun 07 2026 20:43:28.707 [Warning] - Not loading driver 01wkopenvr because it was blocked by a previous "
	    "safe mode event\n"
	    "Sun Jun 07 2026 20:43:28.725 [Warning] - Not loading driver SmoothTracking because it was blocked by a "
	    "previous safe mode event\n";

	std::vector<std::string> blocked;
	EXPECT_TRUE(svc::ParseVrServerSafeModeBlock(log, blocked));
	ASSERT_EQ(blocked.size(), 3u);
	EXPECT_EQ(blocked[0], "VirtualDesktop");
	EXPECT_EQ(blocked[1], "01wkopenvr");
	EXPECT_EQ(blocked[2], "SmoothTracking");
}

TEST(SafeModeParse, DeduplicatesRepeatedBlockLines)
{
	const std::string log = "Not loading driver 01wkopenvr because it was blocked by a previous safe mode event\n"
	                        "Not loading driver 01wkopenvr because it was blocked by a previous safe mode event\n";
	std::vector<std::string> blocked;
	EXPECT_TRUE(svc::ParseVrServerSafeModeBlock(log, blocked));
	ASSERT_EQ(blocked.size(), 1u);
	EXPECT_EQ(blocked[0], "01wkopenvr");
}

TEST(SafeModeParse, CleanLogReportsNoBlock)
{
	const std::string log = "Sun Jun 07 2026 21:00:00.000 [Info] - Driver 01wkopenvr loaded\n"
	                        "Sun Jun 07 2026 21:00:01.000 [Info] - Scene application VRChat connected\n";
	std::vector<std::string> blocked;
	EXPECT_FALSE(svc::ParseVrServerSafeModeBlock(log, blocked));
	EXPECT_TRUE(blocked.empty());
}

// =============================================================================
// Pure vrsettings transform.
// =============================================================================

TEST(SafeModeClearJson, ClearsFlagsAndPreservesOtherKeys)
{
	const std::string input = "{ \"steamvr\" : { \"enableSafeMode\" : true, \"installID\" : \"123\" },"
	                          "  \"driver_01wkopenvr\" : { \"blocked_by_safe_mode\" : true },"
	                          "  \"GpuSpeed\" : { \"gpuSpeed0\" : 7 } }";

	std::string out;
	ASSERT_TRUE(svc::ClearSafeModeInVrSettingsJson(input, {"01wkopenvr", "VirtualDesktop"}, out));

	picojson::value root;
	ASSERT_TRUE(json::Parse(root, out));

	const picojson::value* steamvr = json::ValueAt(root, "steamvr");
	ASSERT_NE(steamvr, nullptr);
	EXPECT_FALSE(json::BoolAt(*steamvr, "enableSafeMode", true));
	EXPECT_EQ(json::StringAt(*steamvr, "installID"), "123"); // untouched

	const picojson::value* ours = json::ValueAt(root, "driver_01wkopenvr");
	ASSERT_NE(ours, nullptr);
	EXPECT_FALSE(json::BoolAt(*ours, "blocked_by_safe_mode", true));

	// Driver block that did not exist is created and cleared.
	const picojson::value* vd = json::ValueAt(root, "driver_VirtualDesktop");
	ASSERT_NE(vd, nullptr);
	EXPECT_FALSE(json::BoolAt(*vd, "blocked_by_safe_mode", true));

	// Unrelated key preserved.
	const picojson::value* gpu = json::ValueAt(root, "GpuSpeed");
	ASSERT_NE(gpu, nullptr);
	EXPECT_EQ(json::IntAt(*gpu, "gpuSpeed0"), 7);
}

TEST(SafeModeClearJson, CreatesSteamvrSectionWhenAbsent)
{
	const std::string input = "{ \"driver_01wkopenvr\" : { \"blocked_by_safe_mode\" : true } }";
	std::string out;
	ASSERT_TRUE(svc::ClearSafeModeInVrSettingsJson(input, {"01wkopenvr"}, out));
	picojson::value root;
	ASSERT_TRUE(json::Parse(root, out));
	const picojson::value* steamvr = json::ValueAt(root, "steamvr");
	ASSERT_NE(steamvr, nullptr);
	EXPECT_FALSE(json::BoolAt(*steamvr, "enableSafeMode", true));
}

TEST(SafeModeClearJson, RejectsNonObjectInput)
{
	std::string out;
	EXPECT_FALSE(svc::ClearSafeModeInVrSettingsJson("[1,2,3]", {"01wkopenvr"}, out));
	EXPECT_FALSE(svc::ClearSafeModeInVrSettingsJson("not json", {"01wkopenvr"}, out));
}

// =============================================================================
// Pure loop guard.
// =============================================================================

TEST(SafeModeLoopGuard, FirstAttemptStartsWindow)
{
	auto d = svc::EvaluateLoopGuard({/*count*/ 0, /*windowStart*/ 0}, /*now*/ 1000, /*max*/ 2, /*window*/ 600);
	EXPECT_TRUE(d.allowed);
	EXPECT_EQ(d.next.count, 1u);
	EXPECT_EQ(d.next.windowStartEpoch, 1000);
}

TEST(SafeModeLoopGuard, SecondAttemptWithinWindowAllowed)
{
	auto d = svc::EvaluateLoopGuard({1, 1000}, 1100, 2, 600);
	EXPECT_TRUE(d.allowed);
	EXPECT_EQ(d.next.count, 2u);
	EXPECT_EQ(d.next.windowStartEpoch, 1000); // window preserved
}

TEST(SafeModeLoopGuard, ExceedingCapWithinWindowDenied)
{
	auto d = svc::EvaluateLoopGuard({2, 1000}, 1200, 2, 600);
	EXPECT_FALSE(d.allowed);
	EXPECT_EQ(d.next.count, 2u); // unchanged
	EXPECT_EQ(d.next.windowStartEpoch, 1000);
}

TEST(SafeModeLoopGuard, WindowRolloverResetsCount)
{
	auto d = svc::EvaluateLoopGuard({2, 1000}, 1000 + 600, 2, 600);
	EXPECT_TRUE(d.allowed);
	EXPECT_EQ(d.next.count, 1u);
	EXPECT_EQ(d.next.windowStartEpoch, 1600);
}

TEST(SafeModeLoopGuard, ClockMovedBackwardResetsWindow)
{
	auto d = svc::EvaluateLoopGuard({2, 5000}, 1000, 2, 600);
	EXPECT_TRUE(d.allowed);
	EXPECT_EQ(d.next.count, 1u);
	EXPECT_EQ(d.next.windowStartEpoch, 1000);
}

// =============================================================================
// Attribution + file-backed state (need the module_safety temp root override).
// =============================================================================

namespace {

std::wstring MakeTempDirectory()
{
	wchar_t tempRoot[MAX_PATH + 1] = {};
	EXPECT_NE(GetTempPathW(MAX_PATH, tempRoot), 0u);
	wchar_t tempName[MAX_PATH + 1] = {};
	EXPECT_NE(GetTempFileNameW(tempRoot, L"wkr", 0, tempName), 0u);
	DeleteFileW(tempName);
	EXPECT_NE(CreateDirectoryW(tempName, nullptr), FALSE);
	return tempName;
}

class SafeModeStateTest : public ::testing::Test
{
protected:
	void SetUp() override
	{
		DWORD len = GetEnvironmentVariableW(kRootEnv, nullptr, 0);
		if (len > 0) {
			previousRoot.resize(len);
			DWORD w = GetEnvironmentVariableW(kRootEnv, previousRoot.data(), len);
			previousRoot.resize(w);
			hadPreviousRoot = true;
		}
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

TEST_F(SafeModeStateTest, NoMarkersMeansNoCulprit)
{
	EXPECT_TRUE(svc::FindUncontainedCrashCulprits().empty());
}

TEST_F(SafeModeStateTest, StaleSuspectMarkerIsCulprit)
{
	const module_safety::ModuleSpec* spec = module_safety::FindByFlagFileName("enable_smoothing.flag");
	ASSERT_NE(spec, nullptr);
	ASSERT_TRUE(module_safety::MarkActive(*spec));
	ASSERT_TRUE(module_safety::MarkSuspect(*spec, "pose_pipeline"));

	auto culprits = svc::FindUncontainedCrashCulprits();
	ASSERT_EQ(culprits.size(), 1u);
	EXPECT_EQ(culprits[0], spec);
}

TEST_F(SafeModeStateTest, ActiveOnlyMarkerIsNotCulprit)
{
	const module_safety::ModuleSpec* spec = module_safety::FindByFlagFileName("enable_calibration.flag");
	ASSERT_NE(spec, nullptr);
	ASSERT_TRUE(module_safety::MarkActive(*spec)); // active, but never marked suspect

	EXPECT_TRUE(svc::FindUncontainedCrashCulprits().empty());
}

TEST_F(SafeModeStateTest, AutoDisabledOnlyMarkerIsNotCulprit)
{
	// MarkFault writes the auto-disabled marker and clears active/suspect, which
	// represents a *contained* fault -- not an uncontained crash, so it must not
	// drive the self-heal trigger on its own.
	const module_safety::ModuleSpec* spec = module_safety::FindByFlagFileName("enable_phantom.flag");
	ASSERT_NE(spec, nullptr);
	ASSERT_TRUE(module_safety::MarkFault(*spec, "pose_exception"));
	ASSERT_TRUE(module_safety::HasAutoDisabledMarker(*spec));

	EXPECT_TRUE(svc::FindUncontainedCrashCulprits().empty());
}

TEST_F(SafeModeStateTest, LoopGuardStateRoundTrips)
{
	svc::LoopGuardState absent;
	EXPECT_FALSE(svc::ReadLoopGuardState(absent)); // nothing written yet

	svc::LoopGuardState write{2, 1234567};
	ASSERT_TRUE(svc::WriteLoopGuardState(write));

	svc::LoopGuardState read;
	ASSERT_TRUE(svc::ReadLoopGuardState(read));
	EXPECT_EQ(read.count, 2u);
	EXPECT_EQ(read.windowStartEpoch, 1234567);
}

// =============================================================================
// File-level clear (backup + rewrite of a real-on-disk vrsettings).
// =============================================================================

TEST(SafeModeClearFile, BacksUpAndRewritesSettings)
{
	const std::wstring dir = MakeTempDirectory();
	const std::wstring settings = dir + L"\\steamvr.vrsettings";
	{
		std::ofstream f(settings, std::ios::binary);
		f << "{ \"steamvr\" : { \"enableSafeMode\" : true },"
		     "  \"driver_01wkopenvr\" : { \"blocked_by_safe_mode\" : true } }";
	}

	svc::SteamPaths paths;
	paths.ok = true;
	paths.vrSettingsPath = settings;

	ASSERT_TRUE(svc::ClearSafeMode(paths, {"01wkopenvr", "VirtualDesktop", "SmoothTracking"}));

	// A timestamped backup was created alongside the original.
	bool foundBackup = false;
	for (const auto& entry : std::filesystem::directory_iterator(dir)) {
		const std::string name = entry.path().filename().string();
		if (name.rfind("steamvr.vrsettings.wkopenvr-safe-mode-", 0) == 0) foundBackup = true;
	}
	EXPECT_TRUE(foundBackup);

	// The live file now has safe mode cleared.
	std::ifstream in(settings, std::ios::binary);
	std::string body((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
	picojson::value root;
	ASSERT_TRUE(json::Parse(root, body));
	const picojson::value* steamvr = json::ValueAt(root, "steamvr");
	ASSERT_NE(steamvr, nullptr);
	EXPECT_FALSE(json::BoolAt(*steamvr, "enableSafeMode", true));
	const picojson::value* sm = json::ValueAt(root, "driver_SmoothTracking");
	ASSERT_NE(sm, nullptr);
	EXPECT_FALSE(json::BoolAt(*sm, "blocked_by_safe_mode", true));

	std::error_code ec;
	std::filesystem::remove_all(dir, ec);
}
