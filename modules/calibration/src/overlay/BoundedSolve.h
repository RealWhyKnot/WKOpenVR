#pragma once

// Pure helpers for the robust / bounded continuous-solve toggle.
//
// The continuous calibration solver fits a fresh ref<->target transform from a
// sliding sample buffer every tick. With clean motion that is exactly what you
// want, but when a burst of biased samples arrives (post-relocalization poses in
// the shifted Quest world frame, or a whole-universe origin shift) the
// unconstrained least-squares fit will happily walk the calibration hundreds of
// mm to satisfy them. These three independent guards bound how far a single tick
// can move the solution and reject the most pathological inputs outright:
//
//   (a) ApplyPrior   -- pull the solved relative pose toward the last-settled
//                       one (Tikhonov / damped update). A small lambda barely
//                       affects clean convergence but resists a transient yank.
//   (b) ClampStep    -- hard slew limit on the per-tick change of the SOLVE
//                       output. This is distinct from the driver-side
//                       BlendTransform mm/sec cap (which smooths what the runtime
//                       renders); this bounds what the solver itself commits.
//   (c) IsCommonModeJump -- reject a sample/update when the reference and target
//                       both translate together coherently this tick. Real
//                       relative motion moves them differently; a coherent joint
//                       translation is a frame shift (SLAM relocalization /
//                       origin reset), which carries no relative-pose
//                       information and only biases the fit.
//
// (c) is the single-pair, sample-level analog of spacecal::coherence (which
// works across multiple calibration pairs' residual streams); the continuous
// case has only the one ref+target pair, so coherence is read from their
// per-tick translation deltas instead.
//
// All functions are pure Eigen math: no CalCtx access, no vr::* calls.

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>

namespace spacecal::bounded_solve {

// (a) Blend the freshly-solved relative pose toward a prior (the last-settled
// relative pose) by lambda in [0, 1]. lambda = 0 returns the pure solve;
// lambda = 1 returns the pure prior; in between, translation is linearly
// interpolated and rotation is slerped. A modest lambda (~0.2) damps transient
// excursions while still tracking genuine slow drift over many ticks.
inline Eigen::AffineCompact3d ApplyPrior(const Eigen::AffineCompact3d& solved, const Eigen::AffineCompact3d& prior,
                                         double lambda)
{
	if (lambda <= 0.0) return solved;
	if (lambda >= 1.0) return prior;

	Eigen::AffineCompact3d out;
	out.translation() = (1.0 - lambda) * solved.translation() + lambda * prior.translation();

	const Eigen::Quaterniond qs(solved.rotation());
	const Eigen::Quaterniond qp(prior.rotation());
	out.linear() = qs.slerp(lambda, qp).toRotationMatrix();
	return out;
}

// (b) Limit how far `proposed` may move from `prev` in a single tick. Caps the
// translation step at maxTransM metres (along the proposed direction) and the
// rotation step at maxRotRad radians (slerp fraction toward proposed). A
// non-positive limit disables that axis's clamp. Returns the clamped pose.
inline Eigen::AffineCompact3d ClampStep(const Eigen::AffineCompact3d& prev, const Eigen::AffineCompact3d& proposed,
                                        double maxTransM, double maxRotRad)
{
	Eigen::AffineCompact3d out = proposed;

	if (maxTransM > 0.0) {
		const Eigen::Vector3d delta = proposed.translation() - prev.translation();
		const double dist = delta.norm();
		if (dist > maxTransM) {
			out.translation() = prev.translation() + delta * (maxTransM / dist);
		}
	}

	if (maxRotRad > 0.0) {
		const Eigen::Quaterniond qp(prev.rotation());
		const Eigen::Quaterniond qn(proposed.rotation());
		const double ang = qp.angularDistance(qn);
		if (ang > maxRotRad) {
			out.linear() = qp.slerp(maxRotRad / ang, qn).toRotationMatrix();
		}
	}

	return out;
}

// (c) Common-mode jump detection thresholds.
//   kCommonModeMinJumpM  -- both devices must translate at least this far this
//                           tick for the jump to count (filters normal jitter).
//   kCommonModeCoherence -- magnitude agreement (min/max norm) AND direction
//                           agreement (cos of the angle between the deltas) must
//                           both meet this bar for the joint motion to be
//                           classified as a single shared frame shift.
constexpr double kCommonModeMinJumpM = 0.05;  // 5 cm, aligned with the reloc detector
constexpr double kCommonModeCoherence = 0.85; // direction + magnitude agreement

// True when the ref and target translated together coherently this tick -- a
// whole-universe frame shift the solver should reject rather than fit. Returns
// false when either device barely moved (no shared jump), when the magnitudes
// disagree, or when the directions diverge (genuine relative motion).
inline bool IsCommonModeJump(const Eigen::Vector3d& refDelta, const Eigen::Vector3d& tgtDelta)
{
	const double rn = refDelta.norm();
	const double tn = tgtDelta.norm();
	if (rn < kCommonModeMinJumpM || tn < kCommonModeMinJumpM) return false;

	const double magRatio = std::min(rn, tn) / std::max(rn, tn);
	const double dirCos = refDelta.dot(tgtDelta) / (rn * tn);
	return magRatio >= kCommonModeCoherence && dirCos >= kCommonModeCoherence;
}

} // namespace spacecal::bounded_solve
