#include <gtest/gtest.h>

#include "AvatarStatePoller.h"
#include "JsonUtil.h"
#include "Profiles.h"
#include "Win32Text.h"

#include <windows.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

namespace {

std::filesystem::path MakeProfileTempDir()
{
	wchar_t tempBuf[MAX_PATH] = {};
	GetTempPathW(MAX_PATH, tempBuf);
	auto stamp = std::to_wstring(GetCurrentProcessId()) + L"_" +
	             std::to_wstring(std::chrono::steady_clock::now().time_since_epoch().count());
	std::filesystem::path dir = std::filesystem::path(tempBuf) / (L"WKOpenVR_ProfileTests_" + stamp);
	std::filesystem::create_directories(dir);
	return dir;
}

class ScopedEnvVar
{
public:
	ScopedEnvVar(const wchar_t* name, const std::wstring& value) : name_(name)
	{
		DWORD needed = GetEnvironmentVariableW(name_.c_str(), nullptr, 0);
		if (needed > 0) {
			previous_.resize(needed, L'\0');
			DWORD written = GetEnvironmentVariableW(name_.c_str(), previous_.data(), needed);
			if (written > 0 && written < needed) {
				previous_.resize(written);
				hadPrevious_ = true;
			}
			else {
				previous_.clear();
			}
		}
		SetEnvironmentVariableW(name_.c_str(), value.c_str());
	}

	~ScopedEnvVar() { SetEnvironmentVariableW(name_.c_str(), hadPrevious_ ? previous_.c_str() : nullptr); }

private:
	std::wstring name_;
	std::wstring previous_;
	bool hadPrevious_ = false;
};

std::filesystem::path ProfilePathUnder(const std::filesystem::path& localLow)
{
	return localLow / L"WKOpenVR" / L"profiles" / L"facetracking.json";
}

std::filesystem::path AvatarStatePathUnder(const std::filesystem::path& localLow)
{
	return localLow / L"WKOpenVR" / L"facetracking" / L"avatar_parameter_cache" / L"state.json";
}

void WriteText(const std::filesystem::path& path, const std::string& body)
{
	std::filesystem::create_directories(path.parent_path());
	std::ofstream out(path, std::ios::binary | std::ios::trunc);
	ASSERT_TRUE(out.is_open());
	out << body;
}

picojson::value ReadProfileJson(const std::filesystem::path& path)
{
	std::ifstream in(path, std::ios::binary);
	EXPECT_TRUE(in.is_open());
	std::stringstream ss;
	ss << in.rdbuf();
	picojson::value root;
	std::string err;
	EXPECT_TRUE(openvr_pair::common::json::ParseObject(root, ss.str(), &err)) << err;
	return root;
}

} // namespace

TEST(FacetrackingProfiles, DefaultContinuousCalibrationModeIsOff)
{
	FacetrackingProfile p;
	EXPECT_EQ(p.continuous_calib_mode, 0);
}

TEST(FacetrackingProfiles, EyeCloseAssistDefaultsOffAndRoundTrips)
{
	FacetrackingProfile defaults;
	EXPECT_FALSE(defaults.eye_close_assist_enabled);

	auto temp = MakeProfileTempDir();
	ScopedEnvVar overrideLocalLow(L"WKOPENVR_LOCALAPPDATA_OVERRIDE", temp.wstring());
	const auto path = ProfilePathUnder(temp);

	FacetrackingProfileStore store;
	store.current.eye_close_assist_enabled = true;
	store.current.eye_close_assist_strength = 85;
	ASSERT_TRUE(store.Save());

	FacetrackingProfileStore loaded;
	ASSERT_TRUE(loaded.Load());
	EXPECT_TRUE(loaded.current.eye_close_assist_enabled);
	EXPECT_EQ(loaded.current.eye_close_assist_strength, 85);

	std::error_code ec;
	std::filesystem::remove_all(temp, ec);
}

TEST(FacetrackingProfiles, LegacyContinuousCalibrationModeLoadsOffAndIsPersisted)
{
	auto temp = MakeProfileTempDir();
	ScopedEnvVar overrideLocalLow(L"WKOPENVR_LOCALAPPDATA_OVERRIDE", temp.wstring());
	const auto path = ProfilePathUnder(temp);
	WriteText(path, "{\n  \"continuous_calib_mode\": 2,\n  \"output_osc_enabled\": true\n}\n");

	FacetrackingProfileStore store;
	ASSERT_TRUE(store.Load());
	EXPECT_EQ(store.current.continuous_calib_mode, 0);

	picojson::value saved = ReadProfileJson(path);
	EXPECT_EQ(openvr_pair::common::json::IntAt(saved, "continuous_calib_mode", -1), 0);

	std::error_code ec;
	std::filesystem::remove_all(temp, ec);
}

TEST(FacetrackingProfiles, SaveAlwaysEmitsContinuousCalibrationOff)
{
	auto temp = MakeProfileTempDir();
	ScopedEnvVar overrideLocalLow(L"WKOPENVR_LOCALAPPDATA_OVERRIDE", temp.wstring());
	const auto path = ProfilePathUnder(temp);

	FacetrackingProfileStore store;
	store.current.continuous_calib_mode = 2;
	ASSERT_TRUE(store.Save());

	picojson::value saved = ReadProfileJson(path);
	EXPECT_EQ(openvr_pair::common::json::IntAt(saved, "continuous_calib_mode", -1), 0);

	std::error_code ec;
	std::filesystem::remove_all(temp, ec);
}

TEST(FacetrackingProfiles, AvatarShapeTuningRoundTripsSparseValues)
{
	auto temp = MakeProfileTempDir();
	ScopedEnvVar overrideLocalLow(L"WKOPENVR_LOCALAPPDATA_OVERRIDE", temp.wstring());
	const auto path = ProfilePathUnder(temp);

	FacetrackingProfileStore store;
	FaceShapeScaleArray values = DefaultFaceShapeScales();
	values[26].max_percent = 80;  // JawOpen
	values[45].max_percent = 60;  // MouthSmileLeft
	values[46].max_percent = 150; // MouthSmileRight
	store.current.avatar_shape_tuning["avtr_test"] = values;
	ASSERT_TRUE(store.Save());

	picojson::value saved = ReadProfileJson(path);
	const picojson::value* tuning = openvr_pair::common::json::ValueAt(saved, "avatar_shape_tuning");
	ASSERT_NE(tuning, nullptr);
	ASSERT_TRUE(tuning->is<picojson::object>());
	const auto& tuningObj = tuning->get<picojson::object>();
	auto avatarIt = tuningObj.find("avtr_test");
	ASSERT_NE(avatarIt, tuningObj.end());
	ASSERT_TRUE(avatarIt->second.is<picojson::object>());
	const auto& shapeObj = avatarIt->second.get<picojson::object>();
	// min at default 0 -> shorthand encodes max as a bare number.
	EXPECT_EQ(shapeObj.at("JawOpen").get<double>(), 80.0);
	EXPECT_EQ(shapeObj.at("MouthSmileLeft").get<double>(), 60.0);
	EXPECT_EQ(shapeObj.at("MouthSmileRight").get<double>(), 150.0);
	EXPECT_EQ(shapeObj.find("MouthSadLeft"), shapeObj.end());

	FacetrackingProfileStore loaded;
	ASSERT_TRUE(loaded.Load());
	const FaceShapeScaleArray* loadedValues = FindShapeTuningForAvatar(loaded.current, "avtr_test");
	ASSERT_NE(loadedValues, nullptr);
	EXPECT_EQ((*loadedValues)[26].max_percent, 80);
	EXPECT_EQ((*loadedValues)[45].max_percent, 60);
	EXPECT_EQ((*loadedValues)[46].max_percent, 150);
	EXPECT_TRUE(IsDefaultFaceShapeTuningValue((*loadedValues)[47]));

	std::error_code ec;
	std::filesystem::remove_all(temp, ec);
}

TEST(FacetrackingProfiles, AvatarShapeTuningRoundTripsMinAndMax)
{
	auto temp = MakeProfileTempDir();
	ScopedEnvVar overrideLocalLow(L"WKOPENVR_LOCALAPPDATA_OVERRIDE", temp.wstring());
	const auto path = ProfilePathUnder(temp);

	FacetrackingProfileStore store;
	FaceShapeScaleArray values = DefaultFaceShapeScales();
	values[8].min_percent = 10; // EyeWideLeft: non-default min -> {min,max} object
	values[8].max_percent = 70;
	store.current.avatar_shape_tuning["avtr_test"] = values;
	ASSERT_TRUE(store.Save());

	picojson::value saved = ReadProfileJson(path);
	const picojson::value* tuning = openvr_pair::common::json::ValueAt(saved, "avatar_shape_tuning");
	ASSERT_NE(tuning, nullptr);
	ASSERT_TRUE(tuning->is<picojson::object>());
	const auto& tuningObj = tuning->get<picojson::object>();
	const auto& shapeObj = tuningObj.at("avtr_test").get<picojson::object>();
	ASSERT_TRUE(shapeObj.at("EyeWideLeft").is<picojson::object>());
	const auto& valueObj = shapeObj.at("EyeWideLeft").get<picojson::object>();
	EXPECT_EQ(valueObj.find("scale"), valueObj.end());
	EXPECT_EQ(valueObj.at("min").get<double>(), 10.0);
	EXPECT_EQ(valueObj.at("max").get<double>(), 70.0);

	FacetrackingProfileStore loaded;
	ASSERT_TRUE(loaded.Load());
	const FaceShapeScaleArray* loadedValues = FindShapeTuningForAvatar(loaded.current, "avtr_test");
	ASSERT_NE(loadedValues, nullptr);
	EXPECT_EQ((*loadedValues)[8].min_percent, 10);
	EXPECT_EQ((*loadedValues)[8].max_percent, 70);

	std::error_code ec;
	std::filesystem::remove_all(temp, ec);
}

TEST(FacetrackingProfiles, GlobalShapeTuningRoundTripsSparseValues)
{
	auto temp = MakeProfileTempDir();
	ScopedEnvVar overrideLocalLow(L"WKOPENVR_LOCALAPPDATA_OVERRIDE", temp.wstring());
	const auto path = ProfilePathUnder(temp);

	FacetrackingProfileStore store;
	store.current.global_shape_tuning[26].max_percent = 80;  // JawOpen
	store.current.global_shape_tuning[45].max_percent = 60;  // MouthSmileLeft
	store.current.global_shape_tuning[46].max_percent = 150; // MouthSmileRight
	ASSERT_TRUE(store.Save());

	picojson::value saved = ReadProfileJson(path);
	const picojson::value* tuning = openvr_pair::common::json::ValueAt(saved, "global_shape_tuning");
	ASSERT_NE(tuning, nullptr);
	ASSERT_TRUE(tuning->is<picojson::object>());
	const auto& shapeObj = tuning->get<picojson::object>();
	EXPECT_EQ(shapeObj.at("JawOpen").get<double>(), 80.0);
	EXPECT_EQ(shapeObj.at("MouthSmileLeft").get<double>(), 60.0);
	EXPECT_EQ(shapeObj.at("MouthSmileRight").get<double>(), 150.0);
	EXPECT_EQ(shapeObj.find("MouthSadLeft"), shapeObj.end());

	FacetrackingProfileStore loaded;
	ASSERT_TRUE(loaded.Load());
	EXPECT_EQ(loaded.current.global_shape_tuning[26].max_percent, 80);
	EXPECT_EQ(loaded.current.global_shape_tuning[45].max_percent, 60);
	EXPECT_EQ(loaded.current.global_shape_tuning[46].max_percent, 150);
	EXPECT_TRUE(IsDefaultFaceShapeTuningValue(loaded.current.global_shape_tuning[47]));

	std::error_code ec;
	std::filesystem::remove_all(temp, ec);
}

TEST(FacetrackingProfiles, GlobalShapeTuningRoundTripsMinAndMax)
{
	auto temp = MakeProfileTempDir();
	ScopedEnvVar overrideLocalLow(L"WKOPENVR_LOCALAPPDATA_OVERRIDE", temp.wstring());
	const auto path = ProfilePathUnder(temp);

	FacetrackingProfileStore store;
	store.current.global_shape_tuning[8].min_percent = 10; // EyeWideLeft
	store.current.global_shape_tuning[8].max_percent = 70;
	ASSERT_TRUE(store.Save());

	picojson::value saved = ReadProfileJson(path);
	const picojson::value* tuning = openvr_pair::common::json::ValueAt(saved, "global_shape_tuning");
	ASSERT_NE(tuning, nullptr);
	ASSERT_TRUE(tuning->is<picojson::object>());
	const auto& shapeObj = tuning->get<picojson::object>();
	ASSERT_TRUE(shapeObj.at("EyeWideLeft").is<picojson::object>());
	const auto& valueObj = shapeObj.at("EyeWideLeft").get<picojson::object>();
	EXPECT_EQ(valueObj.find("scale"), valueObj.end());
	EXPECT_EQ(valueObj.at("min").get<double>(), 10.0);
	EXPECT_EQ(valueObj.at("max").get<double>(), 70.0);

	FacetrackingProfileStore loaded;
	ASSERT_TRUE(loaded.Load());
	EXPECT_EQ(loaded.current.global_shape_tuning[8].min_percent, 10);
	EXPECT_EQ(loaded.current.global_shape_tuning[8].max_percent, 70);

	std::error_code ec;
	std::filesystem::remove_all(temp, ec);
}

TEST(FacetrackingProfiles, LegacyScaleTuningMigratesToMinMax)
{
	auto temp = MakeProfileTempDir();
	ScopedEnvVar overrideLocalLow(L"WKOPENVR_LOCALAPPDATA_OVERRIDE", temp.wstring());
	const auto path = ProfilePathUnder(temp);
	// Old-format profile: bare number was scale-only; object carried scale/min/max.
	WriteText(path, "{\n"
	                "  \"global_shape_tuning\": {\n"
	                "    \"JawOpen\": 150,\n"
	                "    \"MouthSmileLeft\": { \"scale\": 150, \"min\": 0, \"max\": 200 },\n"
	                "    \"EyeWideLeft\": { \"scale\": 175, \"min\": 10, \"max\": 70 }\n"
	                "  }\n"
	                "}\n");

	FacetrackingProfileStore store;
	ASSERT_TRUE(store.Load());
	// Bare number (old scale) -> new max, min stays 0. Exact behavior preservation.
	EXPECT_EQ(store.current.global_shape_tuning[26].min_percent, 0);
	EXPECT_EQ(store.current.global_shape_tuning[26].max_percent, 150);
	// Object with default ceiling (max==200): old scale becomes the new max.
	EXPECT_EQ(store.current.global_shape_tuning[45].min_percent, 0);
	EXPECT_EQ(store.current.global_shape_tuning[45].max_percent, 150);
	// Object with explicit floor+ceiling: floor -> at-rest min, ceiling -> max.
	EXPECT_EQ(store.current.global_shape_tuning[8].min_percent, 10);
	EXPECT_EQ(store.current.global_shape_tuning[8].max_percent, 70);

	std::error_code ec;
	std::filesystem::remove_all(temp, ec);
}

TEST(FacetrackingProfiles, GlobalShapeTuningOmitsAllDefaultEntries)
{
	auto temp = MakeProfileTempDir();
	ScopedEnvVar overrideLocalLow(L"WKOPENVR_LOCALAPPDATA_OVERRIDE", temp.wstring());
	const auto path = ProfilePathUnder(temp);

	FacetrackingProfileStore store;
	store.current.global_shape_tuning = DefaultFaceShapeScales();
	ASSERT_TRUE(store.Save());

	picojson::value saved = ReadProfileJson(path);
	EXPECT_EQ(openvr_pair::common::json::ValueAt(saved, "global_shape_tuning"), nullptr);

	std::error_code ec;
	std::filesystem::remove_all(temp, ec);
}

TEST(FacetrackingProfiles, MissingGlobalShapeTuningLoadsDefault)
{
	auto temp = MakeProfileTempDir();
	ScopedEnvVar overrideLocalLow(L"WKOPENVR_LOCALAPPDATA_OVERRIDE", temp.wstring());
	const auto path = ProfilePathUnder(temp);
	WriteText(path, "{\n  \"output_osc_enabled\": true\n}\n");

	FacetrackingProfileStore store;
	ASSERT_TRUE(store.Load());
	EXPECT_TRUE(IsDefaultFaceShapeScales(store.current.global_shape_tuning));

	std::error_code ec;
	std::filesystem::remove_all(temp, ec);
}

TEST(FacetrackingProfiles, GlobalAndAvatarShapeTuningCoexist)
{
	auto temp = MakeProfileTempDir();
	ScopedEnvVar overrideLocalLow(L"WKOPENVR_LOCALAPPDATA_OVERRIDE", temp.wstring());
	const auto path = ProfilePathUnder(temp);

	FacetrackingProfileStore store;
	store.current.global_shape_tuning[26].max_percent = 70;
	FaceShapeScaleArray avatar = DefaultFaceShapeScales();
	avatar[26].max_percent = 140;
	store.current.avatar_shape_tuning["avtr_test"] = avatar;
	ASSERT_TRUE(store.Save());

	picojson::value saved = ReadProfileJson(path);
	EXPECT_NE(openvr_pair::common::json::ValueAt(saved, "global_shape_tuning"), nullptr);
	EXPECT_NE(openvr_pair::common::json::ValueAt(saved, "avatar_shape_tuning"), nullptr);

	FacetrackingProfileStore loaded;
	ASSERT_TRUE(loaded.Load());
	EXPECT_EQ(loaded.current.global_shape_tuning[26].max_percent, 70);
	const FaceShapeScaleArray* loadedAvatar = FindShapeTuningForAvatar(loaded.current, "avtr_test");
	ASSERT_NE(loadedAvatar, nullptr);
	EXPECT_EQ((*loadedAvatar)[26].max_percent, 140);

	std::error_code ec;
	std::filesystem::remove_all(temp, ec);
}

TEST(FacetrackingProfiles, CombineShapeTuningUsesAvatarOverridesOverGlobal)
{
	FaceShapeScaleArray global = DefaultFaceShapeScales();
	FaceShapeScaleArray avatar = DefaultFaceShapeScales();

	global[26].max_percent = 70;
	global[45].min_percent = 10;
	global[45].max_percent = 90;
	avatar[45].max_percent = 150;

	const FaceShapeScaleArray combined = CombineShapeTuning(global, avatar);
	EXPECT_EQ(combined[26].max_percent, 70);
	// Avatar override wins wholesale over the global entry for slot 45.
	EXPECT_EQ(combined[45].max_percent, 150);
	EXPECT_EQ(combined[45].min_percent, protocol::FACETRACKING_SHAPE_TUNING_DEFAULT_MIN_PERCENT);
	EXPECT_TRUE(IsDefaultFaceShapeTuningValue(combined[46]));
}

TEST(FacetrackingProfiles, AvatarShapeMetadataRoundTrips)
{
	auto temp = MakeProfileTempDir();
	ScopedEnvVar overrideLocalLow(L"WKOPENVR_LOCALAPPDATA_OVERRIDE", temp.wstring());
	const auto path = ProfilePathUnder(temp);

	FacetrackingProfileStore store;
	store.current.avatar_shape_metadata["avtr_test"] =
	    AvatarShapeTuningMetadata{"My Alias", "OSC Avatar", "2026-06-09T02:14:58.4249754Z", "C:\\avatars\\a.json"};
	ASSERT_TRUE(store.Save());

	picojson::value saved = ReadProfileJson(path);
	const picojson::value* metadata = openvr_pair::common::json::ValueAt(saved, "avatar_shape_metadata");
	ASSERT_NE(metadata, nullptr);
	ASSERT_TRUE(metadata->is<picojson::object>());
	const auto& metadataObj = metadata->get<picojson::object>();
	auto avatarIt = metadataObj.find("avtr_test");
	ASSERT_NE(avatarIt, metadataObj.end());
	EXPECT_EQ(openvr_pair::common::json::StringAt(avatarIt->second, "custom_name"), "My Alias");
	EXPECT_EQ(openvr_pair::common::json::StringAt(avatarIt->second, "auto_name"), "OSC Avatar");
	EXPECT_EQ(openvr_pair::common::json::StringAt(avatarIt->second, "last_used_utc"), "2026-06-09T02:14:58.4249754Z");
	EXPECT_EQ(openvr_pair::common::json::StringAt(avatarIt->second, "config_path"), "C:\\avatars\\a.json");

	FacetrackingProfileStore loaded;
	ASSERT_TRUE(loaded.Load());
	const AvatarShapeTuningMetadata* loadedMetadata = FindMetadataForAvatar(loaded.current, "avtr_test");
	ASSERT_NE(loadedMetadata, nullptr);
	EXPECT_EQ(loadedMetadata->custom_name, "My Alias");
	EXPECT_EQ(loadedMetadata->auto_name, "OSC Avatar");
	EXPECT_EQ(loadedMetadata->last_used_utc, "2026-06-09T02:14:58.4249754Z");
	EXPECT_EQ(loadedMetadata->config_path, "C:\\avatars\\a.json");

	std::error_code ec;
	std::filesystem::remove_all(temp, ec);
}

TEST(FacetrackingProfiles, AvatarShapeTuningOmitsAllDefaultEntries)
{
	auto temp = MakeProfileTempDir();
	ScopedEnvVar overrideLocalLow(L"WKOPENVR_LOCALAPPDATA_OVERRIDE", temp.wstring());
	const auto path = ProfilePathUnder(temp);

	FacetrackingProfileStore store;
	store.current.avatar_shape_tuning["avtr_default"] = DefaultFaceShapeScales();
	ASSERT_TRUE(store.Save());

	picojson::value saved = ReadProfileJson(path);
	EXPECT_EQ(openvr_pair::common::json::ValueAt(saved, "avatar_shape_tuning"), nullptr);

	std::error_code ec;
	std::filesystem::remove_all(temp, ec);
}

TEST(FacetrackingProfiles, NormalizeAvatarShapeTuningKeyFallsBackToDefault)
{
	EXPECT_EQ(NormalizeAvatarShapeTuningKey(" \t\r\n"), kDefaultAvatarShapeTuningKey);
	EXPECT_EQ(NormalizeAvatarShapeTuningKey(" avtr_123 "), "avtr_123");
}

TEST(FacetrackingProfiles, AvatarDisplayNamePrefersAliasThenOscNameThenCompactId)
{
	AvatarShapeTuningMetadata metadata;
	metadata.auto_name = "OSC Avatar";
	EXPECT_EQ(AvatarDisplayName("avtr_67bb9fae-11fe-441a-afa7-3d3086a99cd7", &metadata), "OSC Avatar");
	EXPECT_EQ(AvatarDisplaySourceLabel("avtr_67bb9fae-11fe-441a-afa7-3d3086a99cd7", &metadata), "OSC");

	metadata.custom_name = "  Favorite Smile Test  ";
	EXPECT_EQ(AvatarDisplayName("avtr_67bb9fae-11fe-441a-afa7-3d3086a99cd7", &metadata), "Favorite Smile Test");
	EXPECT_EQ(AvatarDisplaySourceLabel("avtr_67bb9fae-11fe-441a-afa7-3d3086a99cd7", &metadata), "Alias");

	EXPECT_EQ(AvatarDisplayName("avtr_67bb9fae-11fe-441a-afa7-3d3086a99cd7", nullptr), "avtr_67bb9fae");
	EXPECT_EQ(AvatarDisplaySourceLabel("avtr_67bb9fae-11fe-441a-afa7-3d3086a99cd7", nullptr), "ID");
	EXPECT_EQ(AvatarDisplayName(kDefaultAvatarShapeTuningKey, nullptr), "Default profile");
	EXPECT_EQ(AvatarDisplaySourceLabel(kDefaultAvatarShapeTuningKey, nullptr), "Default");
}

TEST(FacetrackingProfiles, FormatAvatarLastUsedAgeUsesHumanUnits)
{
	const int64_t now = AvatarLastUsedUnixSeconds("2026-06-09T02:15:58Z");
	ASSERT_GT(now, 0);

	EXPECT_EQ(FormatAvatarLastUsedAge("2026-06-09T02:15:56Z", now), "used just now");
	EXPECT_EQ(FormatAvatarLastUsedAge("2026-06-09T02:15:11Z", now), "used 47 seconds ago");
	EXPECT_EQ(FormatAvatarLastUsedAge("2026-06-09T02:14:58.4249754Z", now), "used 1 minute ago");
	EXPECT_EQ(FormatAvatarLastUsedAge("2026-06-09T00:15:58Z", now), "used 2 hours ago");
	EXPECT_EQ(FormatAvatarLastUsedAge("2026-06-06T02:15:58Z", now), "used 3 days ago");
	EXPECT_EQ(FormatAvatarLastUsedAge("not-a-date", now), "last used unknown");
}

TEST(FacetrackingProfiles, AvatarStatePollerReadsOscConfigName)
{
	auto temp = MakeProfileTempDir();
	ScopedEnvVar overrideLocalLow(L"WKOPENVR_LOCALAPPDATA_OVERRIDE", temp.wstring());
	const auto statePath = AvatarStatePathUnder(temp);
	const auto configPath = temp / L"VRChat" / L"OSC" / L"usr_test" / L"Avatars" / L"avtr_test.json";
	const std::string configPathUtf8 = openvr_pair::common::WideToUtf8(configPath.wstring());

	WriteText(configPath, "{\n"
	                      "  \"id\": \"avtr_test\",\n"
	                      "  \"name\": \"Readable Avatar\",\n"
	                      "  \"parameters\": []\n"
	                      "}\n");

	picojson::object state;
	state["AvatarId"] = picojson::value(std::string("avtr_test"));
	state["ConfigPath"] = picojson::value(configPathUtf8);
	state["UpdatedAtUtc"] = picojson::value(std::string("2026-06-09T02:14:58.4249754Z"));
	WriteText(statePath, picojson::value(state).serialize(true));

	facetracking::AvatarStatePoller poller;
	poller.Tick();

	const facetracking::AvatarStateSnapshot& snapshot = poller.Snapshot();
	ASSERT_TRUE(snapshot.valid);
	EXPECT_EQ(snapshot.avatar_id, "avtr_test");
	EXPECT_EQ(snapshot.avatar_name, "Readable Avatar");
	EXPECT_EQ(snapshot.config_path, configPathUtf8);
	EXPECT_EQ(snapshot.updated_at_utc, "2026-06-09T02:14:58.4249754Z");

	std::error_code ec;
	std::filesystem::remove_all(temp, ec);
}
