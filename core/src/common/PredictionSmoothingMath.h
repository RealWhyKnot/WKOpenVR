#pragma once

// Pure-function helpers for the driver-side prediction-smoothing path.
//
// Two concerns live here:
//
// 1. Factor curve.  The raw slider value (0..100 integer) maps to a
//    suppression factor via SmoothnessToFactor().  A super-linear (squared)
//    curve is used so the upper end of the slider produces noticeably stronger
//    suppression than the middle:
//      s=0:   factor=1.00 (no suppression)
//      s=50:  factor=0.25 (75% suppression)
//      s=80:  factor=0.04 (96% suppression)
//      s=100: factor=0.00 (fully zeroed)
//    The old linear map put s=80 at factor=0.20 -- barely perceptible compared
//    to s=100; the squared curve widens that gap.
//
// 2. Position EWM.  Velocity/acceleration suppression only kills SteamVR's
//    extrapolation lookahead; it does not reduce positional jitter from
//    physical tracker noise.  PositionEwmAlpha() converts the same smoothness
//    value into an EWM alpha: alpha=1 is pass-through; alpha near 0 is heavy
//    smoothing.  The smart-motion blend in the driver drives alpha toward 1
//    when the device is moving, so the filter releases during intentional
//    motion and latches back when the device goes still again.
//      s=0:   alpha=1.00 (pass-through)
//      s=50:  alpha=0.29 (light smoothing)
//      s=80:  alpha=0.08 (heavy smoothing)
//      s=100: alpha=0.00 (freeze -- only reached when device is truly still)
//
// All functions are pure; pinned by test_prediction_smoothing.cpp.

#include <cmath>
#include <cstdint>

namespace prediction {

// Velocity/acceleration suppression factor for a smoothness percent 0..100.
// Uses a squared curve: factor = (1 - s/100)^2.
constexpr double SmoothnessToFactor(uint8_t smoothness)
{
	double s = static_cast<double>(smoothness);
	if (s > 100.0) s = 100.0;
	double t = 1.0 - s / 100.0;
	return t * t;
}

// EWM alpha for position low-pass filtering.  alpha is the weight on the
// incoming sample; (1-alpha) is the weight on the running average.
// Uses t^1.8 so the position curve is slightly softer than the velocity curve
// at mid-settings (less lag at 50%), but both converge to 0 at s=100.
inline double PositionEwmAlpha(uint8_t smoothness)
{
	double s = static_cast<double>(smoothness);
	if (s > 100.0) s = 100.0;
	double t = 1.0 - s / 100.0;
	if (t <= 0.0) return 0.0;
	return std::pow(t, 1.8);
}

} // namespace prediction
