#pragma once

#include "FeaturePlugin.h"
#include "ShellContext.h"

#include <vector>

namespace openvr_pair::overlay {

struct ModuleToggleTableOptions
{
	bool markDevelopmentModules = false;
	const char* developmentMarker = "(dev only)";
	const char* developmentTooltip = "This module is compiled into dev builds and omitted from release builds.";
	bool allowTabReorder = false;
};

void DrawModuleToggleTable(ShellContext& context, const std::vector<FeaturePlugin*>& plugins, const char* tableId,
                           const char* emptyMessage, ModuleToggleTableOptions options = {});

} // namespace openvr_pair::overlay
