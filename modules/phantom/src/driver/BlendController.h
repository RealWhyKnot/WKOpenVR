#pragma once

#include <openvr_driver.h>

namespace phantom {

// Match-and-fade blender between two DriverPose_t streams. Handles both
// directions:
//
//   real -> synth (BLEND_OUT): start from the latest real pose, ramp to the
//   dead-reckoned / IK / ML output over kBlendOutMs. Avoids a perceptible
//   step at the moment the real signal disappears.
//
//   synth -> real (BLEND_IN):  start from the synthesised pose currently
//   being published, ramp to the recovered real pose over kBlendInMs.
//   "Match-and-fade" means we record the delta between synth and real at
//   t=0 of the blend, then attenuate that delta to zero over the window,
//   which prevents a visible pop when the synthesised pose has drifted.
class BlendController
{
public:
	BlendController() = default;

	// Lerp between two poses; alpha=0 returns `a`, alpha=1 returns `b`.
	// Positions linearly, rotations slerp on the shortest arc.
	static void Lerp(const vr::DriverPose_t& a, const vr::DriverPose_t& b, double alpha, vr::DriverPose_t& out);

	// Public helper for tests: dot-product of two quaternions.
	static double QuatDot(const vr::HmdQuaternion_t& a, const vr::HmdQuaternion_t& b);
};

} // namespace phantom
