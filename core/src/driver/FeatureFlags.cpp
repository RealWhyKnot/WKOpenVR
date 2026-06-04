#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

#include "FeatureFlags.h"
#include "Logging.h"
#include "ModuleSafety.h"

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

constexpr unsigned kActiveOnlyAutoDisableThreshold = 3;

struct SafetyGateResult
{
	bool *enabled = nullptr;
	const openvr_pair::common::module_safety::ModuleSpec *spec = nullptr;
	openvr_pair::common::module_safety::LaunchAssessment assessment;
};

SafetyGateResult ApplySafetyGate(bool &enabled, const char *flagFileName)
{
	SafetyGateResult result{&enabled, nullptr, {}};
	if (!enabled) return result;
	const auto *spec = openvr_pair::common::module_safety::FindByFlagFileName(flagFileName);
	result.spec = spec;
	if (!spec) return result;

	const auto assessment = openvr_pair::common::module_safety::AssessLaunch(*spec);
	result.assessment = assessment;
	if (assessment.had_stale_suspect) {
		LOG("Module safety: stale guarded-operation marker for '%s' suspect_count=%u auto_disabled=%d",
			spec->slug,
			assessment.suspect_unclean_count,
			assessment.auto_disabled ? 1 : 0);
	} else if (assessment.had_stale_active) {
		LOG("Module safety: stale active marker for '%s' active_count=%u auto_disabled=%d",
			spec->slug,
			assessment.active_unclean_count,
			assessment.auto_disabled ? 1 : 0);
	}
	if (openvr_pair::common::module_safety::HasAutoDisabledMarker(*spec)) {
		enabled = false;
		LOG("Module safety: '%s' masked off by auto-disable marker", spec->slug);
	}
	return result;
}

void ApplyRepeatedActiveOnlyBackoff(SafetyGateResult *gates, size_t gateCount)
{
	SafetyGateResult *candidate = nullptr;
	size_t candidates = 0;
	for (size_t i = 0; i < gateCount; ++i) {
		SafetyGateResult &gate = gates[i];
		if (!gate.enabled || !*gate.enabled || !gate.spec) continue;
		const auto &assessment = gate.assessment;
		if (!assessment.had_stale_active || assessment.had_stale_suspect ||
			assessment.auto_disabled ||
			assessment.active_unclean_count < kActiveOnlyAutoDisableThreshold) {
			continue;
		}
		candidate = &gate;
		++candidates;
	}

	if (candidates == 1 && candidate && candidate->enabled && candidate->spec) {
		openvr_pair::common::module_safety::MarkFault(
			*candidate->spec,
			"repeated_unclean_driver_exit");
		*candidate->enabled = false;
		LOG("Module safety: '%s' auto-disabled after repeated isolated unclean exits",
			candidate->spec->slug);
	} else if (candidates > 1) {
		LOG("Module safety: repeated active-only unclean exits touched %zu modules; leaving modules enabled until a suspect marker isolates the fault",
			candidates);
	}
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

	bool calSafe = calOn;
	bool smoSafe = smoOn;
	bool ihSafe = ihOn;
	bool ftSafe = ftOn;
	bool capSafe = capOn;
	bool phSafe = phOn;
	SafetyGateResult calGate = ApplySafetyGate(calSafe, "enable_calibration.flag");
	SafetyGateResult smoGate = ApplySafetyGate(smoSafe, "enable_smoothing.flag");
	SafetyGateResult ihGate = ApplySafetyGate(ihSafe, "enable_inputhealth.flag");
	SafetyGateResult ftGate = ApplySafetyGate(ftSafe, "enable_facetracking.flag");
	SafetyGateResult capGate = ApplySafetyGate(capSafe, "enable_captions.flag");
	SafetyGateResult phGate = ApplySafetyGate(phSafe, "enable_phantom.flag");

	bool orSafe = orOn || ftSafe || capSafe;
	SafetyGateResult orGate = ApplySafetyGate(orSafe, "enable_oscrouter.flag");
	SafetyGateResult gates[] = {
		calGate,
		smoGate,
		ihGate,
		ftGate,
		capGate,
		phGate,
		orGate,
	};
	ApplyRepeatedActiveOnlyBackoff(gates, sizeof(gates) / sizeof(gates[0]));
	if (!orSafe && (ftSafe || capSafe)) {
		LOG("Module safety: disabling OSC-dependent modules because oscrouter is auto-disabled");
		ftSafe = false;
		capSafe = false;
	}

	uint32_t flags = ComposeFeatureFlags(calSafe, smoSafe, ihSafe, ftSafe, orSafe, capSafe, phSafe);
	const bool orEffective = (flags & kFeatureOscRouter) != 0;
	if (orEffective && !orOn) {
		LOG("DetectFeatureFlags: enabling oscrouter because a module requires centralized OSC routing");
	}

	// %ls expects wide string on MSVC's CRT. Cap the printed length so a
	// pathological install path doesn't blow the log line.
	LOG("DetectFeatureFlags: resources=%.260ls calibration=%d/%d smoothing=%d/%d inputhealth=%d/%d facetracking=%d/%d oscrouter_flag=%d/%d oscrouter_effective=%d captions=%d/%d phantom=%d/%d (mask=0x%x)",
		dir.c_str(),
		(int)calOn, (int)calSafe,
		(int)smoOn, (int)smoSafe,
		(int)ihOn, (int)ihSafe,
		(int)ftOn, (int)ftSafe,
		(int)orOn, (int)orSafe,
		(int)orEffective,
		(int)capOn, (int)capSafe,
		(int)phOn, (int)phSafe,
		(unsigned)flags);
	return flags;
}

} // namespace pairdriver
