#pragma once

#include <openvr_driver.h>

#include <cmath>

namespace phantom {
namespace pose_space {

// Quaternion product: r = a * b (Hamilton).
inline vr::HmdQuaternion_t QMul(const vr::HmdQuaternion_t& a, const vr::HmdQuaternion_t& b)
{
	vr::HmdQuaternion_t r;
	r.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
	r.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
	r.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
	r.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
	return r;
}

inline void QNormalize(vr::HmdQuaternion_t& q)
{
	const double n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
	if (n > 1e-12) {
		q.w /= n;
		q.x /= n;
		q.y /= n;
		q.z /= n;
	}
	else {
		q.w = 1.0;
		q.x = q.y = q.z = 0.0;
	}
}

// Rotate vector v by unit quaternion q, in place. v' = q * v * q^-1.
inline void QRotateInPlace(const vr::HmdQuaternion_t& q, double v[3])
{
	const double ux = q.x, uy = q.y, uz = q.z, s = q.w;
	const double tx = 2.0 * (uy * v[2] - uz * v[1]);
	const double ty = 2.0 * (uz * v[0] - ux * v[2]);
	const double tz = 2.0 * (ux * v[1] - uy * v[0]);
	const double rx = v[0] + s * tx + (uy * tz - uz * ty);
	const double ry = v[1] + s * ty + (uz * tx - ux * tz);
	const double rz = v[2] + s * tz + (ux * ty - uy * tx);
	v[0] = rx;
	v[1] = ry;
	v[2] = rz;
}

} // namespace pose_space

// Fold a pose's world-from-driver transform into its driver-local fields so
// position/rotation/velocities read directly in SteamVR world space, and the
// remaining world-from-driver is identity. The runtime composes
// world = worldFromDriver * driverPose, so the folded pose is equivalent --
// but every consumer that compares positions across devices from different
// drivers (role inference, snap, the body solver) needs this fold first:
// the space calibration lives entirely in worldFromDriver, and raw
// vecPosition values from two drivers share no common frame.
//
// Velocities and accelerations are driver-space per openvr_driver.h, so they
// rotate by the world-from-driver rotation. Driver-from-head fields are a
// rigid per-device property and pass through untouched.
//
// The math here is deliberately plain doubles: this header is shared with the
// unit-test target, which builds driver sources without the core driver's
// linear-algebra dependencies.
inline vr::DriverPose_t ToWorldSpacePose(const vr::DriverPose_t& in)
{
	const vr::HmdQuaternion_t& q = in.qWorldFromDriverRotation;
	const double* t = in.vecWorldFromDriverTranslation;
	const bool identity_rot =
	    std::fabs(q.w) >= 1.0 - 1e-9 && std::fabs(q.x) <= 1e-9 && std::fabs(q.y) <= 1e-9 && std::fabs(q.z) <= 1e-9;
	const bool identity_trans = t[0] == 0.0 && t[1] == 0.0 && t[2] == 0.0;
	if (identity_rot && identity_trans) {
		return in;
	}

	vr::DriverPose_t out = in;
	pose_space::QRotateInPlace(q, out.vecPosition);
	out.vecPosition[0] += t[0];
	out.vecPosition[1] += t[1];
	out.vecPosition[2] += t[2];
	out.qRotation = pose_space::QMul(q, in.qRotation);
	pose_space::QNormalize(out.qRotation);
	pose_space::QRotateInPlace(q, out.vecVelocity);
	pose_space::QRotateInPlace(q, out.vecAcceleration);
	pose_space::QRotateInPlace(q, out.vecAngularVelocity);
	pose_space::QRotateInPlace(q, out.vecAngularAcceleration);
	out.qWorldFromDriverRotation = {1.0, 0.0, 0.0, 0.0};
	out.vecWorldFromDriverTranslation[0] = 0.0;
	out.vecWorldFromDriverTranslation[1] = 0.0;
	out.vecWorldFromDriverTranslation[2] = 0.0;
	return out;
}

} // namespace phantom
