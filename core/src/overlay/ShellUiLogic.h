#pragma once

#include <string>
#include <string_view>
#include <vector>

namespace openvr_pair::overlay {

constexpr std::string_view kDefaultLogsPanelPluginFlag = "enable_calibration.flag";
constexpr double kShellStatusDefaultSeconds = 8.0;

struct FeaturePickerSelection
{
	std::string_view flag;
	bool applyDesktopDefault = false;
};

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

inline bool ContainsFeatureFlag(const std::vector<std::string_view>& installedFlags, std::string_view flag)
{
	for (std::string_view installed : installedFlags) {
		if (installed == flag) return true;
	}
	return false;
}

inline FeaturePickerSelection ResolveFeaturePickerSelection(bool vrConnected, std::string_view currentFlag,
                                                            std::string_view desktopDefaultFlag,
                                                            std::string_view appliedDesktopDefaultFlag,
                                                            const std::vector<std::string_view>& installedFlags)
{
	if (installedFlags.empty()) return {};

	if (!vrConnected && !desktopDefaultFlag.empty() && appliedDesktopDefaultFlag != desktopDefaultFlag &&
	    ContainsFeatureFlag(installedFlags, desktopDefaultFlag)) {
		return {desktopDefaultFlag, true};
	}

	if (!currentFlag.empty() && ContainsFeatureFlag(installedFlags, currentFlag)) {
		return {currentFlag, false};
	}

	return {installedFlags.front(), false};
}

inline bool ShouldClearTransientStatus(double nowSeconds, double clearAtSeconds)
{
	return clearAtSeconds > 0.0 && nowSeconds >= clearAtSeconds;
}

} // namespace openvr_pair::overlay
