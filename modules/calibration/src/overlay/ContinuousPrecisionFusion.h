#pragma once

// Confidence-weighted fusion of the continuous calibration over time.
//
// The absolute calibration C (target driver space -> reference world) is
// physically CONSTANT -- the origin-to-origin offset does not move. But each
// re-solve's translation error scales with the lever arm (distance of the
// HMD/target from their tracking origins) times the tracker's angular jitter,
// so a far-from-origin re-solve is wildly uncertain. Overwriting C with every
// re-solve therefore lets far-from-origin noise drag a good calibration around
// -- the "flies off far from origin" report.
//
// Instead we treat each accepted candidate as a MEASUREMENT of the constant C
// with a precision set by its geometry, and fuse it into a running estimate
// (a scalar Kalman update): the running estimate carries accumulated confidence,
// and each new candidate moves it by gain = its precision / (accumulated + its
// precision). A near-origin candidate (high precision) moves C a lot; a
// far-from-origin candidate (low precision) barely moves it. Once a calibration
// is well established, far readings can't budge it. Nothing to tune per rig and
// no hard threshold -- the trust each reading gets is computed from the physics.
//
// Header-only pure helpers, testable in isolation. Same pattern as
// AutoLockHysteresis.h / MotionGate.h.

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>

namespace spacecal::precision {

// Positional/angular noise ratio (m^2). Regularizes the precision so a reading
// at the origin doesn't get infinite weight; small, so precision is dominated by
// the lever arm. ~ (lighthouse positional noise / angular noise)^2.
constexpr double kLeverRegM2 = 0.04;

// Cap on accumulated confidence, i.e. a floor on the fusion gain, so a genuinely
// changed rig can still be tracked (slowly) instead of freezing forever. Roughly
// "how much good evidence before the calibration is treated as locked".
constexpr double kMaxConfidence = 2000.0;

// Confidence assigned to a persisted calibration when continuous mode starts, so
// a calibration banked with good geometry in a previous session isn't wiped by
// the first far-from-origin reading of a new (far-the-whole-time) session. About
// the precision of ~40 near-origin (0.3 m) readings of trust; a bad seed is
// still corrected within ~a second of near-origin data, while far readings can't
// override it. Used only when a valid profile seeds the continuous solve.
constexpr double kSeedPriorPrecision = 300.0;

// Stale-seed breaker. The seed prior defends a banked calibration against
// far-from-origin noise, but a cross-session profile can be metres wrong when
// the target universe re-anchors between sessions (field logs: stored profile
// 2.5-4.7 m from the session's true calibration). With the prior in place the
// fusion then crawls toward the truth for the whole session instead of
// escaping. Far-from-origin solve noise is tens of cm and universe changes are
// metres, so a full metre of disagreement on several consecutive accepted
// candidates can only mean the current estimate (seed or a genuinely moved
// rig) is wrong -- adopt the candidate outright and rebuild confidence from
// its own precision.
constexpr double kStaleSeedDisagreeM = 1.0;
constexpr int kStaleSeedTripCount = 3;

// Tracks consecutive candidate-vs-estimate disagreements >= kStaleSeedDisagreeM.
// Returns true when the streak reaches kStaleSeedTripCount; the caller adopts
// the candidate wholesale and the streak resets.
inline bool NoteSeedDisagreement(int& streak, double distM)
{
	if (!(distM >= kStaleSeedDisagreeM)) {
		streak = 0;
		return false;
	}
	if (++streak < kStaleSeedTripCount) {
		return false;
	}
	streak = 0;
	return true;
}

// Precision (inverse variance, relative units) of a calibration measurement
// taken with the given mean-squared lever arm (m^2). Lever arm =
// |ref.trans|^2 + |target.trans|^2 averaged over the solve window.
inline double MeasurementPrecision(double meanSqLeverArmM2)
{
	const double lever = meanSqLeverArmM2 > 0.0 ? meanSqLeverArmM2 : 0.0;
	return 1.0 / (kLeverRegM2 + lever);
}

// Kalman-style fusion gain: fraction of the way to move the running estimate
// toward a new measurement, given the confidence accumulated so far and the new
// measurement's precision. accum=0 -> gain 1 (first measurement adopted whole).
inline double FusionGain(double accumPrecision, double measPrecision)
{
	const double denom = accumPrecision + measPrecision;
	return denom > 0.0 ? measPrecision / denom : 1.0;
}

// Fuse a candidate transform into the current running estimate by `gain`:
// translation lerp, rotation slerp. gain is clamped to [0,1].
inline Eigen::AffineCompact3d Fuse(const Eigen::AffineCompact3d& current, const Eigen::AffineCompact3d& candidate,
                                   double gain)
{
	const double g = gain < 0.0 ? 0.0 : (gain > 1.0 ? 1.0 : gain);
	const Eigen::Quaterniond cq(current.rotation());
	const Eigen::Quaterniond kq(candidate.rotation());
	Eigen::AffineCompact3d out(cq.slerp(g, kq));
	out.translation() = current.translation() + g * (candidate.translation() - current.translation());
	return out;
}

} // namespace spacecal::precision
