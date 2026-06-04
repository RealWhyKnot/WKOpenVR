#include "DeadReckoner.h"

#include <algorithm>
#include <cmath>

namespace phantom {

namespace {

// Quaternion product q = a * b (Hamilton convention used by openvr).
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

// Convert an axis-angle (axis * angle in radians, stored as a 3-vector) to a
// unit quaternion. The openvr DriverPose_t convention for vecAngularVelocity
// is axis-angle in radians/s, so integrating over dt gives radians-direct.
vr::HmdQuaternion_t QFromAxisAngle(double ax, double ay, double az)
{
	const double angle = std::sqrt(ax * ax + ay * ay + az * az);
	if (angle < 1e-9) {
		vr::HmdQuaternion_t q{1.0, 0.0, 0.0, 0.0};
		return q;
	}
	const double half = angle * 0.5;
	const double s = std::sin(half) / angle;
	vr::HmdQuaternion_t q;
	q.w = std::cos(half);
	q.x = ax * s;
	q.y = ay * s;
	q.z = az * s;
	return q;
}

bool IsFinite(const vr::HmdQuaternion_t& q)
{
	return std::isfinite(q.w) && std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z);
}

bool IsFinite3(const double v[3])
{
	return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

} // namespace

bool DeadReckoner::Project(const PoseHistory& history, int64_t qpc_freq, int64_t target_qpc_ns,
                           vr::DriverPose_t& out_pose) const
{
	const PoseSample* newest = history.GetNewest(0);
	if (!newest || !newest->was_real || qpc_freq <= 0) {
		return false;
	}

	// Bail on non-finite source data; SteamVR sometimes emits NaN/Inf on
	// tracking-recovery edges and we must not propagate that into the
	// synthesised pose stream.
	if (!IsFinite3(newest->pose.vecPosition) || !IsFinite3(newest->pose.vecVelocity) ||
	    !IsFinite3(newest->pose.vecAcceleration) || !IsFinite3(newest->pose.vecAngularVelocity) ||
	    !IsFinite(newest->pose.qRotation)) {
		return false;
	}

	// Convert QPC delta to seconds.
	const int64_t dt_ns = target_qpc_ns - newest->qpc_ns;
	const double dt_s = (dt_ns > 0) ? static_cast<double>(dt_ns) / static_cast<double>(qpc_freq) * 1.0 : 0.0;

	// Damp ratio: 1.0 at t=0, linearly decreasing to 0.0 at t = kFullDampMs.
	// Past kFullDampMs we still emit a pose, but velocity / acceleration are
	// zero so it comes to rest rather than drifting forever.
	const int64_t age_ms = (qpc_freq > 0) ? (dt_ns * 1000) / qpc_freq : 0;
	double damp = 1.0 - static_cast<double>(std::max<int64_t>(0, age_ms)) / static_cast<double>(kFullDampMs);
	damp = std::clamp(damp, 0.0, 1.0);

	// Start from the newest real pose and project forward.
	out_pose = newest->pose;

	for (int i = 0; i < 3; ++i) {
		const double v = newest->pose.vecVelocity[i] * damp;
		const double a = newest->pose.vecAcceleration[i] * damp;
		out_pose.vecPosition[i] = newest->pose.vecPosition[i] + v * dt_s + 0.5 * a * dt_s * dt_s;
		// Suppress SteamVR's own extrapolation on top of the projection by
		// scaling velocity/acceleration toward zero.
		out_pose.vecVelocity[i] = v * damp;
		out_pose.vecAcceleration[i] = a * damp;
	}

	// Rotation extrapolation: integrate angular velocity over dt, then
	// pre-multiply onto the source quaternion (world-frame rotation).
	vr::HmdQuaternion_t dq = QFromAxisAngle(newest->pose.vecAngularVelocity[0] * dt_s * damp,
	                                        newest->pose.vecAngularVelocity[1] * dt_s * damp,
	                                        newest->pose.vecAngularVelocity[2] * dt_s * damp);
	out_pose.qRotation = QMul(dq, newest->pose.qRotation);
	QNormalize(out_pose.qRotation);

	for (int i = 0; i < 3; ++i) {
		out_pose.vecAngularVelocity[i] *= damp * damp;
		out_pose.vecAngularAcceleration[i] *= damp * damp;
	}
	out_pose.poseTimeOffset = 0.0;

	out_pose.poseIsValid = true;
	out_pose.deviceIsConnected = true;
	out_pose.result = vr::TrackingResult_Running_OK;
	return true;
}

} // namespace phantom
