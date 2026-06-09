#pragma once

#include <openvr.h>

#include <cstdint>

namespace openvr_pair::overlay {

inline int32_t DashboardInputSafeOverlayPriority()
{
	return vr::k_nActionSetOverlayGlobalPriorityMin;
}

inline bool DashboardInputSafeOverlayToggleActive(bool featureEnabled, bool inputReady)
{
	return featureEnabled && inputReady;
}

inline bool DashboardInputSafeOverlayPointerActive(bool featureEnabled, bool inputReady, bool overlayVisible)
{
	return featureEnabled && inputReady && overlayVisible;
}

inline uint32_t DashboardInputSafeOverlayActionSetCount(bool featureEnabled, bool inputReady, bool overlayVisible)
{
	if (!DashboardInputSafeOverlayToggleActive(featureEnabled, inputReady)) return 0;
	return DashboardInputSafeOverlayPointerActive(featureEnabled, inputReady, overlayVisible) ? 4u : 2u;
}

} // namespace openvr_pair::overlay
