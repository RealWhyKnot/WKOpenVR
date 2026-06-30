#pragma once

// Pure math for the continuous sub-30 cm witness correction (Stage 1, item 4).
//
// A Lighthouse witness puck gives the true head pose independent of SLAM. When
// the SLAM frame drifts by a small amount (below the 30 cm relocalization-
// recovery gate) the calibration rides uncorrected until the next paired-motion
// solve. This computes a slow, dead-banded corrective step that closes that gap
// without chasing tracking noise. Kept pure (no CalCtx / Eigen) so the policy is
// unit-testable; the caller supplies the measured drift and applies the step.

namespace spacecal::cont_correction {

// Slew + band limits. Within the 2-5 mm/s the spec calls for, on the slow end so
// the correction is imperceptible and never overshoots a stationary target.
constexpr double kCorrectionSlewMps = 0.003; // 3 mm/s
// Above this the drift is a real relocalization -> the recovery path (snap
// corroboration / re-anchor) owns it, not this slow loop.
constexpr double kMaxCorrectionM = 0.30; // 30 cm
// Never correct inside this floor even if mad_floor is smaller -- below it the
// "drift" is indistinguishable from tracker jitter.
constexpr double kDeadbandFloorM = 0.003; // 3 mm

// Corrective translation step (metres) to apply this tick toward closing a
// witness-vs-calibration drift of errorM, given the rolling mad_floor (metres)
// and frame dt (seconds). Dead-banded at max(madFloorM, kDeadbandFloorM):
// returns 0 within the band and 0 for errorM > kMaxCorrectionM. Otherwise the
// step is the smaller of the remaining over-band error and the per-tick slew
// budget, so the correction converges to the band edge at constant velocity.
constexpr double CorrectionStepM(double errorM, double madFloorM, double dt)
{
	const double deadband = madFloorM > kDeadbandFloorM ? madFloorM : kDeadbandFloorM;
	if (errorM <= deadband) return 0.0;
	if (errorM > kMaxCorrectionM) return 0.0;
	const double budget = kCorrectionSlewMps * (dt > 0.0 ? dt : 0.0);
	const double remaining = errorM - deadband;
	return remaining < budget ? remaining : budget;
}

} // namespace spacecal::cont_correction
