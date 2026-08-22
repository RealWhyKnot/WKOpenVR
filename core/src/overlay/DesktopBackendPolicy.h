#pragma once

#include <cstdint>

namespace openvr_pair::overlay {

// Retry bookkeeping for the in-app desktop backend. The host itself is started and stopped by
// main's poll loop; keeping the decisions here makes them testable without a live pipe server.
struct DesktopBackendState
{
	// The mask the running host was built for, not the mask that actually came up.
	uint32_t startedForMask = 0;
	int startAttempts = 0;
	double retryAfterSeconds = 0.0;
};

// The Modules tab changed what should be running (or SteamVR came up and nothing should). The host
// builds its modules in dependency order, so it is torn down and rebuilt rather than amended.
inline bool DesktopBackendShouldStop(const DesktopBackendState& state, uint32_t wantedMask)
{
	return wantedMask != state.startedForMask;
}

// Something is wanted, the host is not already serving exactly that, and the retry budget is not
// spent. Bounded because a module that faults every time would otherwise tear down its working
// neighbours forever.
inline bool DesktopBackendShouldStart(const DesktopBackendState& state, uint32_t wantedMask, uint32_t activeMask,
                                      double nowSeconds, int maxAttempts = 3)
{
	return wantedMask != 0 && activeMask != wantedMask && state.startAttempts < maxAttempts &&
	       nowSeconds >= state.retryAfterSeconds;
}

inline void DesktopBackendRecordStart(DesktopBackendState& state, uint32_t wantedMask, uint32_t startedMask,
                                      double nowSeconds, double retryDelaySeconds = 15.0)
{
	state.startedForMask = wantedMask;
	++state.startAttempts;
	if (startedMask != wantedMask) state.retryAfterSeconds = nowSeconds + retryDelaySeconds;
}

} // namespace openvr_pair::overlay
