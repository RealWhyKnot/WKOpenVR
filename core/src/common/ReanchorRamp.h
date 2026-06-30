#pragma once

// Pure constant-velocity re-anchor ramp math, shared by the driver transform
// applier and its unit test. Kept free of OpenVR / Eigen types so it can be
// exercised without a runtime.

namespace spacecal::reanchor {

// Fixed constant-velocity caps for the re-anchor ramp. At 90 Hz these are
// ~0.5 mm/frame and ~0.05 deg/frame, which settles a 0.8 m universe flip in
// ~18 s while staying below the per-frame motion a user can perceive. A small
// 5 mm correction reaches its target in ~0.1 s. Fixed (not motion-masked) so
// the behaviour is deterministic and testable.
constexpr double kReanchorSlewTransMps = 0.045;  // 45 mm/s
constexpr double kReanchorSlewRotRadps = 0.0785; // ~4.5 deg/s

// Fraction in [0,1] to interpolate the device transform toward its target this
// frame, capping the per-frame translation/rotation step at maxTransStep
// (metres) and maxRotStep (radians). transFull / rotFull are the full remaining
// world-space translation and rotation at fraction 1. Returns 1 when the
// remaining motion fits within the caps (target reached this frame), so the
// caller can clear the ramp latch. The binding cap (whichever is proportionally
// larger) limits the step, keeping both translation and rotation imperceptible.
constexpr double ComputeReanchorFraction(double transFull, double rotFull, double maxTransStep, double maxRotStep)
{
	double frac = 1.0;
	if (transFull > 1e-9 && maxTransStep < transFull) {
		const double f = maxTransStep / transFull;
		if (f < frac) frac = f;
	}
	if (rotFull > 1e-9 && maxRotStep < rotFull) {
		const double f = maxRotStep / rotFull;
		if (f < frac) frac = f;
	}
	if (frac < 0.0) frac = 0.0;
	return frac;
}

} // namespace spacecal::reanchor
