#pragma once

// Gravity-aligned calibration rotation handling. Both tracking universes are
// nominally gravity-aligned (SteamVR standing universe is +Y-up for the
// lighthouse driver and for the streamed headset driver alike), so the
// calibration rotation between them is dominated by yaw about +Y.
//
// The roll/pitch (tilt) component is NOT pure noise, though: corpus replay
// showed a small constant floor-alignment tilt is real on some rigs, and
// projecting it away worsens fit (hard projection is a replay-only A/B knob
// for that reason). What IS noise is the per-solve tilt random walk: tilt is
// weakly observable from a head-mounted co-moving pair, so unconstrained
// accepts wander it by degrees within a session. The damping helpers below
// treat tilt as a slowly-varying quantity -- yaw and translation follow the
// solver at full speed while the swing component is low-passed and capped.
//
// Header-only pure helpers, same pattern as ContinuousPrecisionFusion.h.

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace spacecal::gravity {

// Swing-twist decomposition: the yaw-about-+Y component of q is the
// normalized projection (w, 0, y, 0). Exact closest yaw rotation in the
// geodesic sense. Degenerate case (pure 180-degree roll/pitch, w==y==0)
// falls back to identity.
inline Eigen::Quaterniond YawTwist(const Eigen::Quaterniond& q)
{
	const double norm = std::sqrt(q.w() * q.w() + q.y() * q.y());
	if (norm < 1e-9) return Eigen::Quaterniond::Identity();
	return Eigen::Quaterniond(q.w() / norm, 0.0, q.y() / norm, 0.0);
}

// Replace the rotation of C with its yaw-about-gravity component; the
// translation is kept as solved. Replay-only A/B knob (worsens fit on rigs
// with a real floor tilt); live paths use DampTilt/ClampTilt instead.
inline Eigen::AffineCompact3d ProjectRotationToYaw(const Eigen::AffineCompact3d& c)
{
	Eigen::AffineCompact3d out(YawTwist(Eigen::Quaterniond(c.rotation())));
	out.translation() = c.translation();
	return out;
}

// Tilt low-pass time constant. At the continuous accept cadence (a few Hz)
// the swing component converges toward the solver's answer over minutes,
// absorbing the per-solve random walk while still tracking a real remount.
constexpr double kTiltTimeConstantSec = 300.0;

// Absolute swing-angle bound. Real floor misalignment between two leveled
// universes stays well under this; anything larger is banked solver error.
constexpr double kMaxTiltRad = 4.0 * EIGEN_PI / 180.0;

// Swing component of q: q = swing * twist with twist the yaw about +Y, so
// the swing axis is horizontal and its angle is the tilt.
inline Eigen::Quaterniond SwingOf(const Eigen::Quaterniond& q)
{
	return q * YawTwist(q).conjugate();
}

// Tilt angle of q in radians: geodesic distance from q to its closest pure
// yaw. Single definition used by the filter and by every tilt diagnostic.
inline double TiltAngleRad(const Eigen::Quaterniond& q)
{
	return q.angularDistance(YawTwist(q));
}

inline double TiltAngleDeg(const Eigen::Quaterniond& q)
{
	return TiltAngleRad(q) * 180.0 / EIGEN_PI;
}

// Cap the swing angle of C's rotation at maxTiltRad (slerp the swing toward
// identity); yaw and translation pass through unchanged.
inline Eigen::AffineCompact3d ClampTilt(const Eigen::AffineCompact3d& c, double maxTiltRad)
{
	const Eigen::Quaterniond q(c.rotation());
	const double tilt = TiltAngleRad(q);
	if (tilt <= maxTiltRad) return c;
	const Eigen::Quaterniond swing = Eigen::Quaterniond::Identity().slerp(maxTiltRad / tilt, SwingOf(q));
	Eigen::AffineCompact3d out(swing * YawTwist(q));
	out.translation() = c.translation();
	return out;
}

// Two-time-constant accept: take yaw + translation from `candidate` as-is,
// move the swing from `prior`'s toward `candidate`'s by `alpha`, then cap.
inline Eigen::AffineCompact3d DampTilt(const Eigen::AffineCompact3d& prior, const Eigen::AffineCompact3d& candidate,
                                       double alpha, double maxTiltRad)
{
	const Eigen::Quaterniond priorQ(prior.rotation());
	const Eigen::Quaterniond candQ(candidate.rotation());
	const Eigen::Quaterniond swing = SwingOf(priorQ).slerp(alpha, SwingOf(candQ));
	Eigen::AffineCompact3d out(swing * YawTwist(candQ));
	out.translation() = candidate.translation();
	return ClampTilt(out, maxTiltRad);
}

} // namespace spacecal::gravity
