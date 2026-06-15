#pragma once

#include "ModuleRegistry.h"

#include <cstddef>
#include <string>
#include <string_view>

namespace openvr_pair::common::module_safety {

using ModuleSpec = openvr_pair::common::modules::ModuleInfo;

struct LaunchAssessment
{
	bool had_stale_active = false;
	bool had_stale_suspect = false;
	bool auto_disabled = false;
	unsigned active_unclean_count = 0;
	unsigned suspect_unclean_count = 0;
	std::string suspect_reason;
	std::string reason;
};

const ModuleSpec* Specs(size_t* count = nullptr);
const ModuleSpec* FindById(openvr_pair::common::modules::ModuleId id);
const ModuleSpec* FindBySlug(std::string_view slug);
const ModuleSpec* FindByFlagFileName(std::string_view flagFileName);

std::wstring RootPath(bool create = true);
std::wstring ActiveMarkerPath(const ModuleSpec& spec, bool create = true);
std::wstring SuspectMarkerPath(const ModuleSpec& spec, bool create = true);
std::wstring CleanMarkerPath(const ModuleSpec& spec, bool create = true);
std::wstring CounterPath(const ModuleSpec& spec, bool create = true);
std::wstring AutoDisabledMarkerPath(const ModuleSpec& spec, bool create = true);

bool HasActiveMarker(const ModuleSpec& spec);
bool HasSuspectMarker(const ModuleSpec& spec);
bool HasCleanMarker(const ModuleSpec& spec);
bool HasAutoDisabledMarker(const ModuleSpec& spec);
std::string AutoDisabledReason(const ModuleSpec& spec);
bool MarkActive(const ModuleSpec& spec);
bool MarkSuspect(const ModuleSpec& spec, std::string_view reason);
bool ClearSuspect(const ModuleSpec& spec);
bool MarkClean(const ModuleSpec& spec);
bool MarkFault(const ModuleSpec& spec, std::string_view reason);
LaunchAssessment AssessLaunch(const ModuleSpec& spec, bool allowActiveOnlyAutoDisable = false);
bool ConvertStaleActiveToAutoDisabled(const ModuleSpec& spec);
bool ClearAutoDisabled(const ModuleSpec& spec);
bool ClearAutoDisabledForFlag(std::string_view flagFileName);

} // namespace openvr_pair::common::module_safety
