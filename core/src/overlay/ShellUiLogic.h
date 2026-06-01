#pragma once

#include <string>

namespace openvr_pair::overlay {

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

} // namespace openvr_pair::overlay
