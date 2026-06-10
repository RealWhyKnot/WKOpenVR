#pragma once

namespace openvr_pair::overlay {

// Pure visibility policy for the dashboardinput safe overlay. The overlay
// is a user-summoned panel; while the real SteamVR dashboard is open the
// main dashboard overlay already feeds the shared ImGui IO, so the safe
// overlay yields to keep a single laser-event stream.
inline bool DashboardInputSafeOverlayShouldBeVisible(bool featureEnabled, bool userRequestedVisible,
                                                     bool dashboardVisible)
{
	return featureEnabled && userRequestedVisible && !dashboardVisible;
}

} // namespace openvr_pair::overlay
