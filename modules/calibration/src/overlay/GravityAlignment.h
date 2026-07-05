#pragma once

// Gravity-constrained calibration rotation. Both tracking universes are
// gravity-aligned (SteamVR standing universe is +Y-up for the lighthouse
// driver and for the streamed headset driver alike), so the true calibration
// rotation between them can only be a yaw about +Y. Any solved roll/pitch is
// lever-arm noise; projecting it out removes two drift-prone degrees of
// freedom at no observability cost.
//
// Header-only pure helper, same pattern as ContinuousPrecisionFusion.h.

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
// translation is kept as solved.
inline Eigen::AffineCompact3d ProjectRotationToYaw(const Eigen::AffineCompact3d& c)
{
	Eigen::AffineCompact3d out(YawTwist(Eigen::Quaterniond(c.rotation())));
	out.translation() = c.translation();
	return out;
}

} // namespace spacecal::gravity
