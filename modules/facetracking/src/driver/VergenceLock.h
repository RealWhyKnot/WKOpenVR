#pragma once

#include "Protocol.h"

#include <cstdint>

namespace facetracking {

// Vergence-lock: compute a binocular focus point via skew-line midpoint and
// lerp each eye's gaze direction toward it by `strength / 100`.
//
// The math:
//   Given two rays (o_L + t_L * d_L) and (o_R + t_R * d_R), the closest
//   approach midpoint is:
//       t_L = (b*e - c*d) / denom
//       t_R = (a*e - b*d) / denom
//       denom = a*c - b*b   where a=dot(d_L,d_L), b=dot(d_L,d_R), c=dot(d_R,d_R)
//                                 d=dot(d_L,r),   e=dot(d_R,r),   r=o_L-o_R
//       focus = 0.5 * ((o_L + t_L*d_L) + (o_R + t_R*d_R))
//
// Hot path: no allocations, no logging.
class VergenceLock
{
public:
	// Apply vergence lock in-place.  Strength 0 = no-op, 100 = full lock.
	// Leaves the frame untouched if the eye fields are invalid (flags bit 0
	// not set) or if parallel-gaze is detected.
	void Apply(protocol::FaceTrackingFrameBody& frame, uint8_t strength_0_to_100);

	// Distance (metres) to the last successfully computed focus point.
	// Returns 0 if vergence has never fired or the last frame was discarded
	// (parallel gaze, dropout, etc.).
	float LastFocusDistanceM() const noexcept { return last_focus_m_; }

	// IPD (metres) estimated from the last frame where both eye origins were
	// valid.  Returns 0 until at least one frame has been processed.
	float LastIpdM() const noexcept { return last_ipd_m_; }

private:
	float last_focus_m_ = 0.f;
	float last_ipd_m_ = 0.f;
};

} // namespace facetracking
