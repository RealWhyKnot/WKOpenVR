#include "ModuleRegistry.h"

#include "ProtocolNames.h"

#include <algorithm>

namespace openvr_pair::common::modules {
namespace {

constexpr ModuleInfo kModules[] = {
	{
		ModuleId::Calibration,
		"calibration",
		"enable_calibration.flag",
		L"enable_calibration.flag",
		nullptr,
		nullptr,
		"Space Calibrator",
		L"Space Calibrator",
		L"--launch=calibration",
		OPENVR_PAIRDRIVER_CALIBRATION_PIPE_NAME,
		false,
		true,
	},
	{
		ModuleId::Smoothing,
		"smoothing",
		"enable_smoothing.flag",
		L"enable_smoothing.flag",
		nullptr,
		nullptr,
		"Smoothing",
		L"Smoothing",
		L"--launch=smoothing",
		OPENVR_PAIRDRIVER_SMOOTHING_PIPE_NAME,
		false,
		true,
	},
	{
		ModuleId::InputHealth,
		"inputhealth",
		"enable_inputhealth.flag",
		L"enable_inputhealth.flag",
		nullptr,
		nullptr,
		"Input Health",
		L"Input Health",
		L"--launch=inputhealth",
		OPENVR_PAIRDRIVER_INPUTHEALTH_PIPE_NAME,
		false,
		true,
	},
	{
		ModuleId::FaceTracking,
		"facetracking",
		"enable_facetracking.flag",
		L"enable_facetracking.flag",
		nullptr,
		nullptr,
		"Face Tracking",
		L"Face Tracking",
		L"--launch=facetracking",
		OPENVR_PAIRDRIVER_FACETRACKING_PIPE_NAME,
		true,
		true,
	},
	{
		ModuleId::OscRouter,
		"oscrouter",
		"enable_oscrouter.flag",
		L"enable_oscrouter.flag",
		nullptr,
		nullptr,
		"OSC Router",
		L"OSC Router",
		L"--launch=oscrouter",
		OPENVR_PAIRDRIVER_OSCROUTER_PIPE_NAME,
		false,
		true,
	},
	{
		ModuleId::Captions,
		"captions",
		"enable_captions.flag",
		L"enable_captions.flag",
		"enable_translator.flag",
		L"enable_translator.flag",
		"Captions",
		L"Captions",
		L"--launch=captions",
		OPENVR_PAIRDRIVER_CAPTIONS_PIPE_NAME,
		true,
		true,
	},
	{
		ModuleId::Phantom,
		"phantom",
		"enable_phantom.flag",
		L"enable_phantom.flag",
		nullptr,
		nullptr,
		"Phantom Trackers",
		L"Phantom Trackers",
		L"--launch=phantom",
		OPENVR_PAIRDRIVER_PHANTOM_PIPE_NAME,
		false,
		true,
	},
	{
		ModuleId::QuestApp,
		"questapp",
		"enable_questapp.flag",
		L"enable_questapp.flag",
		nullptr,
		nullptr,
		"Quest App",
		L"Quest App",
		L"--launch=questapp",
		"",
		false,
		false,
	},
};
constexpr size_t kDriverSafetyModuleCount = 7;
static_assert(kModules[0].participates_in_driver_safety);
static_assert(kModules[1].participates_in_driver_safety);
static_assert(kModules[2].participates_in_driver_safety);
static_assert(kModules[3].participates_in_driver_safety);
static_assert(kModules[4].participates_in_driver_safety);
static_assert(kModules[5].participates_in_driver_safety);
static_assert(kModules[6].participates_in_driver_safety);
static_assert(!kModules[7].participates_in_driver_safety);

bool EqualAscii(std::string_view a, std::string_view b)
{
	return a.size() == b.size()
		&& std::equal(a.begin(), a.end(), b.begin(), [](char left, char right) {
			return left == right;
		});
}

const ModuleInfo &FallbackModule()
{
	return kModules[0];
}

} // namespace

const ModuleInfo *All(size_t *count)
{
	if (count) *count = sizeof(kModules) / sizeof(kModules[0]);
	return kModules;
}

const ModuleInfo *DriverSafetyModules(size_t *count)
{
	if (count) *count = kDriverSafetyModuleCount;
	return kModules;
}

const ModuleInfo &Get(ModuleId id)
{
	const ModuleInfo *module = FindById(id);
	return module ? *module : FallbackModule();
}

const ModuleInfo *FindById(ModuleId id)
{
	for (const auto &module : kModules) {
		if (module.id == id) return &module;
	}
	return nullptr;
}

const ModuleInfo *FindBySlug(std::string_view slug)
{
	for (const auto &module : kModules) {
		if (EqualAscii(slug, module.slug)) return &module;
	}
	return nullptr;
}

const ModuleInfo *FindByFlagFileName(std::string_view flagFileName)
{
	for (const auto &module : kModules) {
		if (EqualAscii(flagFileName, module.flag_file)) return &module;
	}
	return nullptr;
}

const ModuleInfo *FindByAnyFlagFileName(std::string_view flagFileName)
{
	for (const auto &module : kModules) {
		if (EqualAscii(flagFileName, module.flag_file)) return &module;
		if (module.legacy_flag_file && EqualAscii(flagFileName, module.legacy_flag_file)) return &module;
	}
	return nullptr;
}

const char *Slug(ModuleId id)
{
	return Get(id).slug;
}

const char *FlagFileName(ModuleId id)
{
	return Get(id).flag_file;
}

const wchar_t *FlagFileNameWide(ModuleId id)
{
	return Get(id).flag_file_wide;
}

const char *DisplayName(ModuleId id)
{
	return Get(id).display_name;
}

const wchar_t *ShortcutLabel(ModuleId id)
{
	return Get(id).shortcut_label;
}

const wchar_t *ShortcutArgument(ModuleId id)
{
	return Get(id).shortcut_argument;
}

const char *PipeName(ModuleId id)
{
	return Get(id).pipe_name;
}

bool RequiresOscRouter(ModuleId id)
{
	return Get(id).requires_osc_router;
}

bool ParticipatesInDriverSafety(ModuleId id)
{
	return Get(id).participates_in_driver_safety;
}

} // namespace openvr_pair::common::modules
