#pragma once

#include <string>
#include <string_view>

namespace openvr_pair::overlay {

constexpr std::string_view kDefaultLogsPanelPluginFlag = "enable_calibration.flag";
constexpr double kShellStatusDefaultSeconds = 8.0;

inline bool ShouldSelectDesktopDefaultTab(bool vrConnected, const char* pluginFlagFileName,
                                          const std::string& desktopDefaultFlagFileName,
                                          const std::string& appliedFlagFileName)
{
	if (vrConnected || !pluginFlagFileName || desktopDefaultFlagFileName.empty()) return false;
	return appliedFlagFileName != desktopDefaultFlagFileName && desktopDefaultFlagFileName == pluginFlagFileName;
}

inline bool IsDefaultLogsPanelPlugin(const char* pluginFlagFileName)
{
	return pluginFlagFileName && std::string_view(pluginFlagFileName) == kDefaultLogsPanelPluginFlag;
}

inline bool ShouldClearTransientStatus(double nowSeconds, double clearAtSeconds)
{
	return clearAtSeconds > 0.0 && nowSeconds >= clearAtSeconds;
}

} // namespace openvr_pair::overlay
