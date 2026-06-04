#pragma once

#include <openvr_driver.h>

#include <cstdint>

namespace phantom {

// One entry in the per-device pose-history ring buffer. The phantom hook
// records every real pose the driver observes (including transformations
// already applied by smoothing / calibration) plus the QPC timestamp the
// hook saw it at. The synthesis path reads the most recent N samples to
// extrapolate forward and to detect impossible jumps.
//
// `was_real` distinguishes pure-pass-through poses from synthesised ones
// the phantom module itself emitted; in REAL state every entry is true,
// during synthesis the history retains the last real pose for hand-back
// matching but does not pollute extrapolation inputs.
struct PoseSample
{
	int64_t qpc_ns = 0;
	vr::DriverPose_t pose{};
	bool was_real = false;
};

} // namespace phantom
