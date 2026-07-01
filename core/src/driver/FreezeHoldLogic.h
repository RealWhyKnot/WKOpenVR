#pragma once

#include <openvr_driver.h>

// "Freeze all tracking" hold logic -- the pure decision + pose-rewrite used by
// the driver's pose hook when the user activates the time-freeze toggle. Kept
// header-only and free of any driver state so it is testable in isolation
// (tests/driver/test_freeze_hold_logic.cpp), mirroring TrackerLiveness.h.
//
// The driver caches each device's last forwarded (fully-processed) pose. While
// frozen, it replays that cached pose instead of the live one so nothing moves.
// Replaying the OUTPUT pose (not the raw input) means the device does not jump
// to its un-calibrated position at the instant of freeze.

namespace wkopenvr::freeze {

// Whether this device's pose should be held frozen this tick.
//   frozen      -- freeze is active (already heartbeat-checked by the caller)
//   includeHmd  -- also freeze device 0 (the HMD view)
//   openVRID    -- the device index (0 == HMD)
//   haveCached  -- a last-forwarded pose has been captured for this device
inline bool ShouldHold(bool frozen, bool includeHmd, uint32_t openVRID, bool haveCached)
{
	if (!frozen || !haveCached) return false;
	// The HMD is held only when the user opts in; otherwise it stays live so the
	// user can still look around a frozen scene.
	if (openVRID == 0 && !includeHmd) return false;
	return true;
}

// Rewrite `pose` to a perfectly still hold of `cached`. Zeroing the velocity /
// angular-velocity vectors stops the SteamVR compositor from extrapolating the
// device (which would drift a "frozen" pose); forcing valid/OK/connected keeps
// the device present rather than dropping to a grey/tracking-lost state.
inline void MakeHeldPose(vr::DriverPose_t& pose, const vr::DriverPose_t& cached)
{
	pose = cached;
	for (int i = 0; i < 3; ++i) {
		pose.vecVelocity[i] = 0.0;
		pose.vecAngularVelocity[i] = 0.0;
	}
	pose.poseIsValid = true;
	pose.result = vr::TrackingResult_Running_OK;
	pose.deviceIsConnected = true;
}

} // namespace wkopenvr::freeze
