#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>

// Rotation vector axis*angle. The old skew-symmetric form (2*sin(angle)*axis)
// collapses toward zero approaching 180 degrees and flips sign past it; the
// exact form keeps the magnitude monotonic in the rotation angle.
inline Eigen::Vector3d AxisFromRotationMatrix3(const Eigen::Matrix3d& rot)
{
	const Eigen::AngleAxisd aa(rot);
	return aa.axis() * aa.angle();
}

inline double AngleFromRotationMatrix3(const Eigen::Matrix3d& rot)
{
	// Trace of a true rotation matrix satisfies trace in [-1, 3], so the
	// argument here lies in [-1, 1]. Composites of slightly non-orthogonal
	// matrices can produce trace values a few ULPs outside that interval;
	// without the clamp acos returns NaN, which silently corrupts every
	// downstream solver consumer.
	const double c = (rot(0, 0) + rot(1, 1) + rot(2, 2) - 1.0) * 0.5;
	return std::acos(std::clamp(c, -1.0, 1.0));
}
