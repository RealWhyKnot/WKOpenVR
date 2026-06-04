#pragma once

#include <cstdint>

// Tracker liveness detector -- pinpoints the case where SteamVR keeps a
// disconnected device's `eTrackingResult == Running_OK` and `poseIsValid ==
// true` while its pose is silently frozen at the last known position. The
// existing `!poseIsValid || result != Running_OK` gate in CollectSample
// cannot see this; samples flow into the buffer, the solver fits against
// the no-longer-real geometry, and the saved profile drifts. See
// `spacecal_log.2026-05-13T00-48-51.txt` for the empirical case (the
// head-mounted reference Vive tracker went silent from t=18,789 through
// session end and posOffset drifted ~7 cm before the user manually power-
// cycled the device).
//
// The detector is intended for the two non-HMD devices in the calibration
// pair: `referenceID` and `targetID`. The HMD has its own dedicated stall
// + relocalization handling upstream of this code path. State is caller-
// owned so the same logic can run independently for reference and target
// without one struct serving two concerns.
//
// Design pinned by tests in `tests/test_tracker_liveness.cpp`. The
// thresholds below are constexpr and guarded by static_assert so anyone
// loosening them must update the regression suite first.

namespace spacecal::liveness {

// Pose hash must be unchanged for this long before considering the device
// frozen. Eight seconds = 160 ticks at the 50 ms cadence; well past any
// legitimate sub-second freeze (e.g. brief radio glitches on Vive trackers
// that recover within a couple of ticks).
constexpr double kFrozenPoseThresholdSec = 8.0;

// The HMD must have crossed this speed at some point in the freeze window
// before we declare the device offline. Distinguishes "tracker silently
// dropped" from "user is sitting still with the tracker on a table." A
// stationary tracker is legitimate; a stationary tracker while everything
// else is moving is the bug. 0.02 m/s is low enough to catch leaning, head
// turns, and walking in place; high enough to ignore Quest IMU thermal
// drift noise on a perfectly stationary head.
constexpr double kHmdMovedThresholdMps = 0.02;

// Latency-EMA-update gap (s) is a *corroborating* signal only. The
// cross-correlator requires both devices to be feeding live poses; a gap
// here strongly suggests one side has stopped, but on its own it can also
// mean a quiet session with low motion. Never fire on this alone --
// require frozen-hash + HMD-moved as the primary trigger and treat the
// EMA gap as supporting evidence for the log annotation.
constexpr double kEmaGapThresholdSec = 15.0;

// After being declared offline, the pose must change for at least this
// long before we declare the device online again. Prevents flapping if
// SteamVR briefly re-issues the same frozen pose during a partial-recover
// glitch.
constexpr double kReconnectDebounceSec = 2.0;

static_assert(kFrozenPoseThresholdSec > kReconnectDebounceSec,
              "Frozen-pose detection window must exceed reconnect debounce or the "
              "detector will oscillate near the threshold");
static_assert(kHmdMovedThresholdMps > 0.0, "HMD-moved threshold must be positive; zero would let stationary "
                                           "sessions trigger the offline path");
static_assert(kFrozenPoseThresholdSec > 0.0 && kReconnectDebounceSec > 0.0, "Windows must be positive durations");

// Caller-owned state. One instance per tracked device; sentinel field
// `offlineSinceSec < 0` encodes "online", >= 0 encodes "offline since
// glfwGetTime() value." Initialise via brace-init or `Reset` -- never
// memset (the `< 0` sentinels need explicit values).
struct TrackerLivenessState
{
	double poseHashSinceSec = -1.0; // when the current hash was first observed
	uint64_t lastPoseHash = 0;
	double offlineSinceSec = -1.0;   // <0 = online
	double reconnectSinceSec = -1.0; // <0 = not in reconnect debounce
	bool hmdMovedDuringFreeze = false;
};

// Inputs packed into a struct so adding signals later does not churn the
// caller's argument list. Built once per tick from the live OpenVR pose
// arrays.
struct TrackerLivenessInputs
{
	uint64_t posHash;        // bitcast of vecPosition[0..2] into a uint64_t
	bool deviceIsConnected;  // per-pose deviceIsConnected flag
	double hmdSpeedMps;      // ||HMD vecVelocity|| this tick
	double lastEmaUpdateSec; // ctx.timeLastLatencyEstimate
	double now;              // glfwGetTime()
};

inline bool IsOffline(const TrackerLivenessState& s)
{
	return s.offlineSinceSec >= 0.0;
}

inline void Reset(TrackerLivenessState& s)
{
	s = TrackerLivenessState{};
}

// Tick the detector. Returns true *exactly once* on the edge that
// transitions online -> offline, so the caller can fire a one-off
// recovery (annotate the log, kick off StartContinuousCalibration on the
// reverse edge). All other ticks return false. `IsOffline` continues to
// reflect the steady-state classification after the edge.
//
// Edge semantics:
//   - online -> offline: this function returns true and `offlineSinceSec`
//     is set to `in.now`.
//   - offline (steady): returns false; `offlineSinceSec` keeps its value.
//   - offline -> online: returns false; `offlineSinceSec` flips back to
//     <0 once the pose has been changing for `kReconnectDebounceSec`.
//     The caller observes the transition via `IsOffline(s)` going from
//     true to false across ticks.
inline bool TickTrackerLiveness(TrackerLivenessState& s, const TrackerLivenessInputs& in)
{
	// Fast path: explicit disconnect from the OpenVR side wins, and must
	// fire even if the pose hash happens to be different this tick (e.g.
	// SteamVR pushed one last partial update on the way out). SteamVR is
	// often unreliable about flipping this flag, but when it does fire
	// there is no ambiguity -- trust it immediately and fire the offline
	// edge without waiting for a freeze window.
	if (!in.deviceIsConnected) {
		if (!IsOffline(s)) {
			s.offlineSinceSec = in.now;
			s.reconnectSinceSec = -1.0;
			return true;
		}
		// Already offline; remain offline. Don't re-fire the edge.
		return false;
	}

	// Track pose-hash continuity. Any change resets the freeze timer and
	// (if we were in reconnect debounce) advances the recovery state.
	if (in.posHash != s.lastPoseHash) {
		s.lastPoseHash = in.posHash;
		s.poseHashSinceSec = in.now;
		s.hmdMovedDuringFreeze = false;

		if (IsOffline(s)) {
			// First pose change while offline: start reconnect debounce.
			// Don't declare online yet -- a single coincidental hash
			// change (or a brief partial-pose-update from SteamVR mid-
			// reconnect) shouldn't snap us back to online prematurely.
			if (s.reconnectSinceSec < 0.0) {
				s.reconnectSinceSec = in.now;
			}
			else if ((in.now - s.reconnectSinceSec) >= kReconnectDebounceSec) {
				// Stable pose changes for long enough: back online.
				Reset(s);
				return false;
			}
		}
		return false;
	}

	// Hash unchanged this tick. If we're currently in a reconnect debounce
	// but the pose stopped moving again, abandon the debounce (the device
	// is not really live yet).
	if (s.reconnectSinceSec >= 0.0 && in.posHash == s.lastPoseHash) {
		s.reconnectSinceSec = -1.0;
	}

	// Record whether the HMD has moved during this freeze window. A
	// stationary HMD + stationary tracker is the "user is sitting still
	// and the tracker is on a table" case -- legitimate and must not
	// trigger.
	if (in.hmdSpeedMps >= kHmdMovedThresholdMps) {
		s.hmdMovedDuringFreeze = true;
	}

	if (IsOffline(s)) {
		// Already offline. Stay offline; nothing edge-y to report.
		return false;
	}

	if (s.poseHashSinceSec < 0.0) {
		// Very first tick we saw this hash. Stamp and wait.
		s.poseHashSinceSec = in.now;
		return false;
	}

	const double frozenFor = in.now - s.poseHashSinceSec;
	if (frozenFor >= kFrozenPoseThresholdSec && s.hmdMovedDuringFreeze) {
		s.offlineSinceSec = in.now;
		s.reconnectSinceSec = -1.0;
		return true;
	}

	return false;
}

} // namespace spacecal::liveness
