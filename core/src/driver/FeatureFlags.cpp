#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

#include "FeatureFlags.h"
#include "Logging.h"

#include <string>

namespace pairdriver {

namespace {

// Returns the absolute path of <root>\resources, where <root> is the driver
// folder SteamVR loaded the DLL from. We resolve our own DLL path with
// GetModuleHandleEx(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS) using the address
// of this very function, which works regardless of what the DLL was renamed
// to and regardless of how SteamVR resolved it. Walk up from
//   <root>\bin\win64\driver_wkopenvr.dll
// to <root> (three pop-segments) then append "\resources".
std::wstring GetResourcesDir()
{
	HMODULE hMod = nullptr;
	if (!GetModuleHandleExW(
			GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
			reinterpret_cast<LPCWSTR>(&GetResourcesDir),
			&hMod)) {
		return {};
	}

	wchar_t buf[MAX_PATH];
	DWORD len = GetModuleFileNameW(hMod, buf, MAX_PATH);
	if (len == 0 || len >= MAX_PATH) return {};

	std::wstring path(buf, len);
	for (int i = 0; i < 3; ++i) {
		size_t slash = path.find_last_of(L"\\/");
		if (slash == std::wstring::npos) return {};
		path.resize(slash);
	}
	path += L"\\resources";
	return path;
}

bool FlagFileExists(const std::wstring &resourcesDir, const wchar_t *flagName)
{
	std::wstring path = resourcesDir + L"\\" + flagName;
	DWORD attr = GetFileAttributesW(path.c_str());
	return attr != INVALID_FILE_ATTRIBUTES && !(attr & FILE_ATTRIBUTE_DIRECTORY);
}

} // namespace

uint32_t DetectFeatureFlags()
{
	std::wstring dir = GetResourcesDir();
	if (dir.empty()) {
		LOG("DetectFeatureFlags: unable to resolve driver resources directory; treating all features as disabled");
		return 0;
	}

	const bool calOn = FlagFileExists(dir, L"enable_calibration.flag");
	const bool smoOn = FlagFileExists(dir, L"enable_smoothing.flag");
	const bool ihOn  = FlagFileExists(dir, L"enable_inputhealth.flag");
	const bool ftOn  = FlagFileExists(dir, L"enable_facetracking.flag");
	const bool orOn  = FlagFileExists(dir, L"enable_oscrouter.flag");
	// Legacy alias: pre-rename installs dropped enable_translator.flag. Treat
	// either name as the same signal so an upgrade-in-place keeps the feature
	// enabled without forcing the user to re-toggle. Future release can drop
	// the legacy name.
	const bool capOn = FlagFileExists(dir, L"enable_captions.flag")
		|| FlagFileExists(dir, L"enable_translator.flag");
	const bool phOn  = FlagFileExists(dir, L"enable_phantom.flag");
	uint32_t flags = ComposeFeatureFlags(calOn, smoOn, ihOn, ftOn, orOn, capOn, phOn);
	const bool orEffective = (flags & kFeatureOscRouter) != 0;
	if (orEffective && !orOn) {
		LOG("DetectFeatureFlags: enabling oscrouter because a module requires centralized OSC routing");
	}

	// %ls expects wide string on MSVC's CRT. Cap the printed length so a
	// pathological install path doesn't blow the log line.
	LOG("DetectFeatureFlags: resources=%.260ls calibration=%d smoothing=%d inputhealth=%d facetracking=%d oscrouter_flag=%d oscrouter_effective=%d captions=%d phantom=%d (mask=0x%x)",
		dir.c_str(), (int)calOn, (int)smoOn, (int)ihOn, (int)ftOn, (int)orOn, (int)orEffective, (int)capOn, (int)phOn, (unsigned)flags);
	return flags;
}

} // namespace pairdriver
