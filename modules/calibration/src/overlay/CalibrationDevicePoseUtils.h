#pragma once

#include <cstdint>

#include <Eigen/Geometry>
#include <openvr.h>

bool IsHmdDevice(int32_t id);
uint64_t HashPositionLow64(const double v[3]);

// OpenVR 3x4 row-major pose matrix to an Eigen affine transform.
inline Eigen::Affine3d HmdMatrix34ToAffine(const vr::HmdMatrix34_t& m)
{
	Eigen::Affine3d affine = Eigen::Affine3d::Identity();
	affine.linear() << m.m[0][0], m.m[0][1], m.m[0][2], m.m[1][0], m.m[1][1], m.m[1][2], m.m[2][0], m.m[2][1],
	    m.m[2][2];
	affine.translation() = Eigen::Vector3d(m.m[0][3], m.m[1][3], m.m[2][3]);
	return affine;
}
