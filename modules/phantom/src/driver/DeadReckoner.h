#pragma once

#include "PoseHistory.h"

#include <openvr_driver.h>

#include <cstdint>

namespace phantom {

// Constant-acceleration dead-reckoning extrapolator. Reads the last few
// samples from PoseHistory and produces a forward-extrapolated DriverPose_t
// for a target observation time.
//
// Math: integrate position with v*dt + 0.5*a*dt^2 (clamped to the last
// observed velocity / acceleration); rotation extrapolates by axis-angle
// of the last angular velocity scaled by dt. Past ~100 ms of synthesis the
// extrapolation factor damps toward zero velocity so we do not walk the
// avatar through walls; the higher-level state machine should hand off
// to IK / ML before damping becomes visible.
class DeadReckoner
{
public:
	DeadReckoner() = default;

	// Project forward from the most recent real pose in `history` to
	// observation time `target_qpc_ns`. Returns false if there is no real
	// sample to project from. On success, `out_pose` is filled and may be
	// emitted to SteamVR; its velocity / acceleration fields are damped to
	// suppress SteamVR's own extrapolation (we already projected forward).
	bool Project(const PoseHistory& history, int64_t qpc_freq, int64_t target_qpc_ns, vr::DriverPose_t& out_pose) const;

	// Damping window. Past this many milliseconds since the last real pose,
	// the projected velocity / acceleration are scaled down to zero so the
	// pose comes to rest rather than continuing to drift.
	static constexpr int64_t kFullDampMs = 250;
};

} // namespace phantom
