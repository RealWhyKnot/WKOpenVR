#pragma once

#include "FeaturePlugin.h"
#include "ShellContext.h"

#include <vector>

namespace openvr_pair::overlay {

void DrawModuleToggleTable(
	ShellContext &context,
	const std::vector<FeaturePlugin *> &plugins,
	const char *tableId,
	const char *emptyMessage);

} // namespace openvr_pair::overlay
