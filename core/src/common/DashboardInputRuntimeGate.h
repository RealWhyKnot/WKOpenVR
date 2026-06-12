#pragma once

namespace openvr_pair::common::dashboardinput {

inline constexpr const char* kRuntimeOptInFlagFileName = "enable_dashboardinput_runtime.flag";
inline constexpr const wchar_t* kRuntimeOptInFlagFileNameWide = L"enable_dashboardinput_runtime.flag";

enum class RuntimeGateState
{
	ModuleOff,
	MissingRuntimeOptIn,
	Enabled,
};

constexpr bool RuntimeEnabled(bool moduleFlagPresent, bool runtimeOptInFlagPresent)
{
	return moduleFlagPresent && runtimeOptInFlagPresent;
}

constexpr RuntimeGateState RuntimeState(bool moduleFlagPresent, bool runtimeOptInFlagPresent)
{
	if (!moduleFlagPresent) return RuntimeGateState::ModuleOff;
	return runtimeOptInFlagPresent ? RuntimeGateState::Enabled : RuntimeGateState::MissingRuntimeOptIn;
}

inline const char* RuntimeStateLabel(RuntimeGateState state)
{
	switch (state) {
		case RuntimeGateState::ModuleOff:
			return "Module off";
		case RuntimeGateState::MissingRuntimeOptIn:
			return "Waiting for opt-in";
		case RuntimeGateState::Enabled:
			return "Opted in";
	}
	return "Disabled";
}

} // namespace openvr_pair::common::dashboardinput
