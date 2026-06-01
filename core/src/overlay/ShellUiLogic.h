#pragma once

#include <string>
#include <string_view>

namespace openvr_pair::overlay {

constexpr std::string_view kDefaultLogsPanelPluginFlag = "enable_calibration.flag";

inline bool ShouldSelectDesktopDefaultTab(
	bool vrConnected,
	const char *pluginFlagFileName,
	const std::string &desktopDefaultFlagFileName,
	const std::string &appliedFlagFileName)
{
	if (vrConnected || !pluginFlagFileName || desktopDefaultFlagFileName.empty()) return false;
	return appliedFlagFileName != desktopDefaultFlagFileName
		&& desktopDefaultFlagFileName == pluginFlagFileName;
}

inline bool IsDefaultLogsPanelPlugin(const char *pluginFlagFileName)
{
	return pluginFlagFileName && std::string_view(pluginFlagFileName) == kDefaultLogsPanelPluginFlag;
}

} // namespace openvr_pair::overlay
