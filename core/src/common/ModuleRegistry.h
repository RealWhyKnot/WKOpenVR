#pragma once

#include <cstddef>
#include <string_view>

namespace openvr_pair::common::modules {

enum class ModuleId
{
	Calibration,
	Smoothing,
	InputHealth,
	FaceTracking,
	OscRouter,
	Captions,
	Phantom,
	QuestApp,
	DynamicResolution,
};

struct ModuleInfo
{
	ModuleId id;
	const char* slug;
	const char* flag_file;
	const wchar_t* flag_file_wide;
	const char* legacy_flag_file;
	const wchar_t* legacy_flag_file_wide;
	const char* display_name;
	const wchar_t* shortcut_label;
	const wchar_t* shortcut_argument;
	const char* pipe_name;
	bool requires_osc_router;
	bool participates_in_driver_safety;
};

const ModuleInfo* All(size_t* count = nullptr);
const ModuleInfo* DriverSafetyModules(size_t* count = nullptr);
const ModuleInfo& Get(ModuleId id);
const ModuleInfo* FindById(ModuleId id);
const ModuleInfo* FindBySlug(std::string_view slug);
const ModuleInfo* FindByFlagFileName(std::string_view flagFileName);
const ModuleInfo* FindByAnyFlagFileName(std::string_view flagFileName);

const char* Slug(ModuleId id);
const char* FlagFileName(ModuleId id);
const wchar_t* FlagFileNameWide(ModuleId id);
const char* DisplayName(ModuleId id);
const wchar_t* ShortcutLabel(ModuleId id);
const wchar_t* ShortcutArgument(ModuleId id);
const char* PipeName(ModuleId id);
bool RequiresOscRouter(ModuleId id);
bool ParticipatesInDriverSafety(ModuleId id);

} // namespace openvr_pair::common::modules
