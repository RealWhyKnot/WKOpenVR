#include <gtest/gtest.h>

#include "JsonUtil.h"
#include "Profiles.h"

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
	values[26] = 80;  // JawOpen
	values[45] = 60;  // MouthSmileLeft
	values[46] = 150; // MouthSmileRight
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
	EXPECT_EQ(shapeObj.at("JawOpen").get<double>(), 80.0);
	EXPECT_EQ(shapeObj.at("MouthSmileLeft").get<double>(), 60.0);
	EXPECT_EQ(shapeObj.at("MouthSmileRight").get<double>(), 150.0);
	EXPECT_EQ(shapeObj.find("MouthSadLeft"), shapeObj.end());

	FacetrackingProfileStore loaded;
	ASSERT_TRUE(loaded.Load());
	const FaceShapeScaleArray* loadedValues = FindShapeTuningForAvatar(loaded.current, "avtr_test");
	ASSERT_NE(loadedValues, nullptr);
	EXPECT_EQ((*loadedValues)[26], 80);
	EXPECT_EQ((*loadedValues)[45], 60);
	EXPECT_EQ((*loadedValues)[46], 150);
	EXPECT_EQ((*loadedValues)[47], protocol::FACETRACKING_SHAPE_TUNING_DEFAULT_PERCENT);

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
