#pragma once

#include "DynamicResolutionLogic.h"

#include <iosfwd>
#include <string>

namespace wkopenvr::dynamicres {

struct DynamicResolutionRestoreState
{
	bool restorePending = false;
	double baselineScale = 1.0;
	bool baselineManualOverride = false;
	uint32_t sceneProcessId = 0;
	double lastWrittenScale = 0.0;
};

struct DynamicResolutionProfile
{
	DynamicResolutionSettings settings;
	DynamicResolutionRestoreState restore;
};

DynamicResolutionProfile LoadDynamicResolutionProfile();
void SaveDynamicResolutionProfile(const DynamicResolutionProfile& profile);
DynamicResolutionProfile ParseDynamicResolutionProfile(std::istream& in);
void WriteDynamicResolutionProfile(const DynamicResolutionProfile& profile, std::ostream& out);
std::wstring DynamicResolutionProfilePath();

} // namespace wkopenvr::dynamicres
