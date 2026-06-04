#include "IkFallback.h"

#include <cmath>
#include <cstddef>
#include <cstring>

namespace phantom {

namespace {

// Quaternion product: r = a * b (Hamilton).
vr::HmdQuaternion_t QMul(const vr::HmdQuaternion_t& a, const vr::HmdQuaternion_t& b)
{
	vr::HmdQuaternion_t r;
	r.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
	r.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
	r.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
	r.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
	return r;
}

void QNormalize(vr::HmdQuaternion_t& q)
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

// Rotate vector v by quaternion q (q must be unit). Standard formula
// v' = q * v * q^-1; expressed without building intermediate quaternions.
void QRotate(const vr::HmdQuaternion_t& q, const double v[3], double out[3])
{
	const double ux = q.x, uy = q.y, uz = q.z, s = q.w;
	// t = 2 * cross(u, v)
	const double tx = 2.0 * (uy * v[2] - uz * v[1]);
	const double ty = 2.0 * (uz * v[0] - ux * v[2]);
	const double tz = 2.0 * (ux * v[1] - uy * v[0]);
	// out = v + s * t + cross(u, t)
	out[0] = v[0] + s * tx + (uy * tz - uz * ty);
	out[1] = v[1] + s * ty + (uz * tx - ux * tz);
	out[2] = v[2] + s * tz + (ux * ty - uy * tx);
}

} // namespace

void IkFallback::SetOffset(BodyRole role, const double rel_pos[3], const vr::HmdQuaternion_t& rel_rot)
{
	const auto idx = static_cast<size_t>(role);
	if (idx >= offsets_.size()) return;
	auto& slot = offsets_[idx];
	slot.role = role;
	slot.rel_position[0] = rel_pos[0];
	slot.rel_position[1] = rel_pos[1];
	slot.rel_position[2] = rel_pos[2];
	slot.rel_rotation = rel_rot;
	QNormalize(slot.rel_rotation);
	slot.calibrated = true;
}

void IkFallback::ClearOffset(BodyRole role)
{
	const auto idx = static_cast<size_t>(role);
	if (idx >= offsets_.size()) return;
	offsets_[idx] = TrackerOffset{};
}

void IkFallback::ClearAll()
{
	for (auto& o : offsets_)
		o = TrackerOffset{};
}

bool IkFallback::HasOffset(BodyRole role) const
{
	const auto idx = static_cast<size_t>(role);
	if (idx >= offsets_.size()) return false;
	return offsets_[idx].calibrated;
}

bool IkFallback::AnyCalibrated() const
{
	for (const auto& o : offsets_) {
		if (o.calibrated) return true;
	}
	return false;
}

bool IkFallback::Solve(BodyRole role, const vr::DriverPose_t& hmd_pose, vr::DriverPose_t& out_pose) const
{
	const auto idx = static_cast<size_t>(role);
	if (idx >= offsets_.size()) return false;
	const auto& o = offsets_[idx];
	if (!o.calibrated) return false;

	// Apply offset: world_tracker_pos = world_hmd_pos + R_hmd * rel_pos.
	double rotated[3];
	QRotate(hmd_pose.qRotation, o.rel_position, rotated);

	out_pose = hmd_pose;
	for (int i = 0; i < 3; ++i) {
		out_pose.vecPosition[i] = hmd_pose.vecPosition[i] + rotated[i];
		// Carry velocity from the HMD so the synth tracker moves with the
		// user's body translation; angular velocity is the HMD's angular
		// velocity because the rigid attachment rotates with the head.
		out_pose.vecVelocity[i] = hmd_pose.vecVelocity[i];
		out_pose.vecAcceleration[i] = hmd_pose.vecAcceleration[i];
		out_pose.vecAngularVelocity[i] = hmd_pose.vecAngularVelocity[i];
		out_pose.vecAngularAcceleration[i] = hmd_pose.vecAngularAcceleration[i];
	}
	out_pose.qRotation = QMul(hmd_pose.qRotation, o.rel_rotation);
	QNormalize(out_pose.qRotation);
	out_pose.poseTimeOffset = 0.0;
	out_pose.poseIsValid = true;
	out_pose.deviceIsConnected = true;
	out_pose.result = vr::TrackingResult_Running_OK;
	return true;
}

} // namespace phantom
