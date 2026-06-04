// Tests that Profiles.cpp's load path prunes records whose paths fall into
// the Unsupported or DiagnosticsOnly classifier categories, while preserving
// valid controller records.
//
// The test writes a synthetic JSON profile to the WKOpenVR profiles directory,
// calls ProfileStore::LoadAll(), and then verifies the in-memory state.

#include <gtest/gtest.h>

#include "Profiles.h"
#include "Win32Paths.h"
#include "Win32Text.h"

#include <cstdio>
#include <fstream>
#include <string>

namespace {

// Write a minimal profile JSON to disk and return the file path.
// The profile has one valid trigger record, one valid stick record, and one
// eye/openness record that should be pruned on load.
std::wstring WriteTestProfile(uint64_t serial_hash)
{
	const std::wstring dir = openvr_pair::common::WkOpenVrSubdirectoryPath(L"profiles", true);
	if (dir.empty()) return {};

	char hashHex[32];
	snprintf(hashHex, sizeof(hashHex), "%016llx", (unsigned long long)serial_hash);
	std::wstring path = dir + L"\\" + openvr_pair::common::Utf8ToWide(std::string(hashHex) + ".json");

	// Build JSON by hand to avoid pulling in picojson in the test.
	const std::string body = std::string(R"({
  "serial": "TEST_SERIAL",
  "serial_hash_hex": ")") + hashHex +
	                         R"(",
  "display_name": "Test Controller",
  "corrections_enabled": true,
  "learned_paths": [
    {
      "path": "/input/trigger/value",
      "kind": "scalar_single",
      "sample_count": 1024,
      "ready": true,
      "learned_rest_offset": 0.02,
      "learned_stddev": 0.003,
      "learned_trigger_min": 0.02,
      "learned_trigger_max": 0.98,
      "learned_deadzone_radius": 0.0,
      "learned_debounce_us": 0,
      "last_updated_unix": 1700000000,
      "drift_shift_resets": 0
    },
    {
      "path": "/input/joystick/x",
      "kind": "stick_x",
      "sample_count": 1024,
      "ready": true,
      "learned_rest_offset": 0.01,
      "learned_stddev": 0.002,
      "learned_trigger_min": 0.0,
      "learned_trigger_max": 0.0,
      "learned_deadzone_radius": 0.03,
      "learned_debounce_us": 0,
      "last_updated_unix": 1700000000,
      "drift_shift_resets": 0
    },
    {
      "path": "/input/eye/left/openness",
      "kind": "scalar_single",
      "sample_count": 512,
      "ready": false,
      "learned_rest_offset": 0.0,
      "learned_stddev": 0.0,
      "learned_trigger_min": 0.0,
      "learned_trigger_max": 0.0,
      "learned_deadzone_radius": 0.0,
      "learned_debounce_us": 0,
      "last_updated_unix": 1700000000,
      "drift_shift_resets": 0
    },
    {
      "path": "/input/pupil/left/dilation",
      "kind": "scalar_single",
      "sample_count": 128,
      "ready": false,
      "learned_rest_offset": 0.0,
      "learned_stddev": 0.0,
      "learned_trigger_min": 0.0,
      "learned_trigger_max": 0.0,
      "learned_deadzone_radius": 0.0,
      "learned_debounce_us": 0,
      "last_updated_unix": 1700000000,
      "drift_shift_resets": 0
    },
    {
      "path": "/input/finger/index",
      "kind": "scalar_single",
      "sample_count": 512,
      "ready": true,
      "learned_rest_offset": 0.25,
      "learned_stddev": 0.01,
      "learned_trigger_min": 0.0,
      "learned_trigger_max": 0.0,
      "learned_deadzone_radius": 0.0,
      "learned_debounce_us": 0,
      "last_updated_unix": 1700000000,
      "drift_shift_resets": 0
    },
    {
      "path": "/input/trackpad/x",
      "kind": "stick_x",
      "sample_count": 512,
      "ready": true,
      "learned_rest_offset": 0.20,
      "learned_stddev": 0.01,
      "learned_trigger_min": 0.0,
      "learned_trigger_max": 0.0,
      "learned_deadzone_radius": 0.05,
      "learned_debounce_us": 0,
      "last_updated_unix": 1700000000,
      "drift_shift_resets": 0
    }
  ]
})";

	std::ofstream f(path);
	if (!f.is_open()) return {};
	f << body;
	return path;
}

void DeleteFile(const std::wstring& path)
{
	_wremove(path.c_str());
}

} // namespace

// ---------------------------------------------------------------------------
// DeviceProfile default values.
// ---------------------------------------------------------------------------

TEST(DeviceProfileDefaults, RestRecenterIsOnByDefault)
{
	const DeviceProfile p;
	EXPECT_TRUE(p.enable_rest_recenter) << "New profiles must default rest-recenter to on";
}

TEST(ProfileSaveScheduler, SampleCountOnlyChurnIsNotMaterial)
{
	LearnedPathRecord a;
	a.path = "/input/trigger/value";
	a.kind = "scalar_single";
	a.sample_count = 1024;
	a.ready = true;
	a.learned_rest_offset = 0.02;
	a.learned_stddev = 0.002;
	a.learned_trigger_min = 0.02;
	a.learned_trigger_max = 0.98;
	a.last_updated_unix = 1700000000;

	LearnedPathRecord b = a;
	b.sample_count = 4096;

	EXPECT_TRUE(LearnedPathMaterialEqual(a, b));

	b.learned_trigger_min = 0.03;
	EXPECT_FALSE(LearnedPathMaterialEqual(a, b));
}

// ---------------------------------------------------------------------------
// After LoadAll, a profile with eye/pupil entries has those pruned.
// Valid trigger and stick entries are preserved.
// ---------------------------------------------------------------------------

TEST(ProfilePrune, LegacyEyeAndPupilRecordsPruned)
{
	const uint64_t testHash = 0xDEADBEEFCAFEBABEULL;
	const std::wstring testFile = WriteTestProfile(testHash);
	if (testFile.empty()) {
		GTEST_SKIP() << "Cannot write to profiles directory (CI path issue)";
	}

	ProfileStore store;
	store.LoadAll();

	const auto& all = store.All();
	const auto it = all.find(testHash);
	ASSERT_NE(it, all.end()) << "Profile not loaded at all";

	const DeviceProfile& p = it->second;

	// The two valid records (trigger and stick) must be present.
	bool hasTrigger = false;
	bool hasStick = false;
	bool hasEye = false;
	bool hasPupil = false;
	bool hasFinger = false;
	bool hasTrackpad = false;

	for (const auto& r : p.learned_paths) {
		if (r.path == "/input/trigger/value") hasTrigger = true;
		if (r.path == "/input/joystick/x") hasStick = true;
		if (r.path == "/input/eye/left/openness") hasEye = true;
		if (r.path == "/input/pupil/left/dilation") hasPupil = true;
		if (r.path == "/input/finger/index") hasFinger = true;
		if (r.path == "/input/trackpad/x") hasTrackpad = true;
	}

	EXPECT_TRUE(hasTrigger) << "/input/trigger/value must survive prune";
	EXPECT_TRUE(hasStick) << "/input/joystick/x must survive prune";
	EXPECT_FALSE(hasEye) << "/input/eye/left/openness must be pruned (DiagnosticsOnly)";
	EXPECT_FALSE(hasPupil) << "/input/pupil/left/dilation must be pruned (Unsupported)";
	EXPECT_FALSE(hasFinger) << "/input/finger/index must be pruned (FingerCapsense)";
	EXPECT_FALSE(hasTrackpad) << "/input/trackpad/x must be pruned (TrackpadAxis)";

	// Total: only 2 records should remain.
	EXPECT_EQ(p.learned_paths.size(), 2u);

	DeleteFile(testFile);
}

// ---------------------------------------------------------------------------
// Force/grip records survive, but legacy trigger-range fields are removed.
// ---------------------------------------------------------------------------

TEST(ProfilePrune, ForceAndGripRecordsAreFloorOnlyOnLoad)
{
	const uint64_t testHash = 0xCAFEBABEDEAD0001ULL;

	const std::wstring dir = openvr_pair::common::WkOpenVrSubdirectoryPath(L"profiles", true);
	if (dir.empty()) {
		GTEST_SKIP() << "Cannot access profiles directory";
	}

	char hashHex[32];
	snprintf(hashHex, sizeof(hashHex), "%016llx", (unsigned long long)testHash);
	const std::wstring testFile = dir + L"\\" + openvr_pair::common::Utf8ToWide(std::string(hashHex) + ".json");

	const std::string body = std::string(R"({
  "serial": "TEST2",
  "serial_hash_hex": ")") + hashHex +
	                         R"(",
  "display_name": "Test 2",
  "corrections_enabled": true,
  "learned_paths": [
    {
      "path": "/input/grip/value",
      "kind": "scalar_single",
      "sample_count": 1024,
      "ready": true,
      "learned_rest_offset": 0.10,
      "learned_stddev": 0.002,
      "learned_trigger_min": 0.10,
      "learned_trigger_max": 0.80,
      "learned_deadzone_radius": 0.05,
      "learned_debounce_us": 1000,
      "last_updated_unix": 1700000001,
      "drift_shift_resets": 0
    }
  ]
})";

	{
		std::ofstream f(testFile);
		if (!f.is_open()) {
			GTEST_SKIP() << "Cannot write test profile";
		}
		f << body;
	}

	ProfileStore store;
	store.LoadAll();

	const auto& all = store.All();
	const auto it = all.find(testHash);
	ASSERT_NE(it, all.end());
	EXPECT_EQ(it->second.learned_paths.size(), 1u);
	EXPECT_EQ(it->second.learned_paths[0].path, "/input/grip/value");
	EXPECT_EQ(it->second.learned_paths[0].kind, "scalar_single");
	EXPECT_DOUBLE_EQ(it->second.learned_paths[0].learned_rest_offset, 0.05);
	EXPECT_DOUBLE_EQ(it->second.learned_paths[0].learned_trigger_min, 0.0);
	EXPECT_DOUBLE_EQ(it->second.learned_paths[0].learned_trigger_max, 0.0);
	EXPECT_DOUBLE_EQ(it->second.learned_paths[0].learned_deadzone_radius, 0.0);
	EXPECT_EQ(it->second.learned_paths[0].learned_debounce_us, 0u);

	DeleteFile(testFile);
}
