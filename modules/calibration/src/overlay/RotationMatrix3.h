#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>

inline Eigen::Vector3d AxisFromRotationMatrix3(const Eigen::Matrix3d& rot)
{
	return Eigen::Vector3d(rot(2, 1) - rot(1, 2), rot(0, 2) - rot(2, 0), rot(1, 0) - rot(0, 1));
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
