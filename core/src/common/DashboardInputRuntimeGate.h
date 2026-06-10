#pragma once

namespace openvr_pair::common::dashboardinput {

inline constexpr const char* kRuntimeOptInFlagFileName = "enable_dashboardinput_runtime.flag";
inline constexpr const wchar_t* kRuntimeOptInFlagFileNameWide = L"enable_dashboardinput_runtime.flag";

constexpr bool RuntimeEnabled(bool moduleFlagPresent, bool runtimeOptInFlagPresent)
{
	return moduleFlagPresent && runtimeOptInFlagPresent;
}

} // namespace openvr_pair::common::dashboardinput
