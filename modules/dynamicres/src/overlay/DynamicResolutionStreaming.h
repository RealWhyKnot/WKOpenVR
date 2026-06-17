#pragma once

#include <cstdint>
#include <string>

namespace wkopenvr::dynamicres {

struct VirtualDesktopStreamerSettings
{
	bool settingsFilePresent = false;
	bool parsed = false;
	int preferredCodec = -1;
	std::string codecName;
	std::string deviceName;
	std::string error;
	std::wstring path;
	int64_t lastWriteTime = 0;
};

bool ParseVirtualDesktopStreamerSettings(const std::string& body, VirtualDesktopStreamerSettings& out,
                                         std::string* error = nullptr);
std::wstring DefaultVirtualDesktopStreamerSettingsPath();
VirtualDesktopStreamerSettings LoadVirtualDesktopStreamerSettings();
bool VirtualDesktopCodecAllowsDefaultAction(const VirtualDesktopStreamerSettings& settings);
std::string VirtualDesktopCodecLabel(const VirtualDesktopStreamerSettings& settings);

} // namespace wkopenvr::dynamicres
