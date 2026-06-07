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
