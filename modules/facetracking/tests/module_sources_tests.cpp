#include <gtest/gtest.h>

#include "ModuleSources.h"

#include <windows.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>

namespace {

std::filesystem::path MakeSourceTempDir()
{
	wchar_t tempBuf[MAX_PATH] = {};
	GetTempPathW(MAX_PATH, tempBuf);
	auto stamp = std::to_wstring(GetCurrentProcessId()) + L"_" +
	             std::to_wstring(std::chrono::steady_clock::now().time_since_epoch().count());
	std::filesystem::path dir = std::filesystem::path(tempBuf) / (L"WKOpenVR_SourceTests_" + stamp);
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

const facetracking::ModuleSource* FindSource(const facetracking::SourcesCatalogue& cat, const std::string& id)
{
	for (const auto& source : cat.sources) {
		if (source.id == id) return &source;
	}
	return nullptr;
}

} // namespace

TEST(ModuleSources, SeedsNativeAndLegacyRegistrySources)
{
	auto temp = MakeSourceTempDir();
	ScopedEnvVar overrideLocalLow(L"WKOPENVR_LOCALAPPDATA_OVERRIDE", temp.wstring());

	facetracking::SourcesCatalogue cat = facetracking::EnsureSourcesCatalogue();

	ASSERT_EQ(cat.sources.size(), 2u);
	const auto* native = FindSource(cat, "00000000000000000000000000000002");
	const auto* legacy = FindSource(cat, "00000000000000000000000000000001");
	ASSERT_NE(native, nullptr);
	ASSERT_NE(legacy, nullptr);
	EXPECT_EQ(native->label, "WKOpenVR native registry");
	EXPECT_EQ(native->url, "https://wkopenvr-module-registry.whyknot.dev");
	EXPECT_FALSE(native->include_prerelease);
	EXPECT_EQ(legacy->label, "VRCFT legacy registry");
	EXPECT_EQ(legacy->url, "https://registry.vrcft.io");
	EXPECT_FALSE(legacy->include_prerelease);
	EXPECT_EQ(cat.sources.front().id, "00000000000000000000000000000002");

	facetracking::SourcesCatalogue again = facetracking::EnsureSourcesCatalogue();
	EXPECT_EQ(again.sources.size(), 2u);

	std::error_code ec;
	std::filesystem::remove_all(temp, ec);
}

TEST(ModuleSources, PersistsPrereleaseSourceOptIn)
{
	auto temp = MakeSourceTempDir();
	ScopedEnvVar overrideLocalLow(L"WKOPENVR_LOCALAPPDATA_OVERRIDE", temp.wstring());

	facetracking::SourcesCatalogue cat = facetracking::EnsureSourcesCatalogue();
	ASSERT_FALSE(cat.sources.empty());
	cat.sources.front().include_prerelease = true;
	ASSERT_TRUE(facetracking::SaveSourcesCatalogue(cat));

	facetracking::SourcesCatalogue loaded = facetracking::LoadSourcesCatalogue();
	const auto* native = FindSource(loaded, "00000000000000000000000000000002");
	ASSERT_NE(native, nullptr);
	EXPECT_TRUE(native->include_prerelease);

	std::error_code ec;
	std::filesystem::remove_all(temp, ec);
}

TEST(ModuleSources, FiltersAvailablePrereleaseModulesBySourceOptIn)
{
	facetracking::SourcesCatalogue cat;
	facetracking::ModuleSource source;
	source.id = "source-a";
	cat.sources.push_back(source);

	facetracking::AvailableModule stable;
	stable.uuid = "stable-module";
	stable.source_id = "source-a";
	EXPECT_TRUE(facetracking::ShouldShowAvailableModule(cat, stable));

	facetracking::AvailableModule prerelease;
	prerelease.uuid = "beta-module";
	prerelease.source_id = "source-a";
	prerelease.prerelease = true;
	EXPECT_FALSE(facetracking::ShouldShowAvailableModule(cat, prerelease));

	prerelease.source_id = "missing-source";
	EXPECT_FALSE(facetracking::ShouldShowAvailableModule(cat, prerelease));

	prerelease.source_id = "source-a";
	cat.sources.front().include_prerelease = true;
	EXPECT_TRUE(facetracking::ShouldShowAvailableModule(cat, prerelease));
}

TEST(ModuleSources, LoadsAvailablePrereleaseMetadata)
{
	auto temp = MakeSourceTempDir();
	ScopedEnvVar overrideLocalLow(L"WKOPENVR_LOCALAPPDATA_OVERRIDE", temp.wstring());

	const std::filesystem::path availableDir = temp / L"WKOpenVR" / L"facetracking" / L"available";
	std::filesystem::create_directories(availableDir);
	const std::filesystem::path cachePath = availableDir / L"source-a.json";
	std::ofstream out(cachePath);
	out << R"json({
  "schema_version": 1,
  "source_id": "source-a",
  "source_label": "Test registry",
  "registry_url": "https://example.invalid",
  "modules": [
    {
      "uuid": "module-a",
      "version": "2026.6.13.0-beta",
      "name": "Module A",
      "vendor": "WhyKnot",
      "prerelease": true,
      "release_channel": "beta"
    }
  ]
})json";
	out.close();

	const std::vector<facetracking::AvailableModule> modules = facetracking::LoadAvailableModules();
	ASSERT_EQ(modules.size(), 1u);
	EXPECT_EQ(modules.front().uuid, "module-a");
	EXPECT_TRUE(modules.front().prerelease);
	EXPECT_EQ(modules.front().release_channel, "beta");

	std::error_code ec;
	std::filesystem::remove_all(temp, ec);
}
