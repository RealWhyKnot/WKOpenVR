#include "BodyCompletionSolver.h"

#include <algorithm>
#include <cstddef>
#include <cmath>

namespace phantom {
namespace {

constexpr double kPi = 3.14159265358979323846;

double Clamp(double v, double lo, double hi)
{
	return std::max(lo, std::min(hi, v));
}

double Len3(const double v[3])
{
	return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

double PlanarLen3(const double v[3])
{
	return std::sqrt(v[0] * v[0] + v[2] * v[2]);
}

void Sub3(const double a[3], const double b[3], double out[3])
{
	out[0] = a[0] - b[0];
	out[1] = a[1] - b[1];
	out[2] = a[2] - b[2];
}

void Lerp3(const double a[3], const double b[3], double alpha, double out[3])
{
	out[0] = a[0] + (b[0] - a[0]) * alpha;
	out[1] = a[1] + (b[1] - a[1]) * alpha;
	out[2] = a[2] + (b[2] - a[2]) * alpha;
}

void NormalizeQuat(double q[4])
{
	const double n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	if (n > 1e-12) {
		q[0] /= n;
		q[1] /= n;
		q[2] /= n;
		q[3] /= n;
	}
	else {
		q[0] = 1.0;
		q[1] = q[2] = q[3] = 0.0;
	}
}

void CopyQuat(const double in[4], double out[4])
{
	out[0] = in[0];
	out[1] = in[1];
	out[2] = in[2];
	out[3] = in[3];
	NormalizeQuat(out);
}

double YawRadiansFromQuat(const double q[4])
{
	// Yaw around Y for a w,x,y,z quaternion.
	const double siny = 2.0 * (q[0] * q[2] + q[1] * q[3]);
	const double cosy = 1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]);
	return std::atan2(siny, cosy);
}

double NormalizeAngle(double a)
{
	while (a > kPi)
		a -= 2.0 * kPi;
	while (a < -kPi)
		a += 2.0 * kPi;
	return a;
}

double LerpAngle(double a, double b, double alpha)
{
	return NormalizeAngle(a + NormalizeAngle(b - a) * Clamp(alpha, 0.0, 1.0));
}

void YawQuat(double yaw, double out[4])
{
	out[0] = std::cos(yaw * 0.5);
	out[1] = 0.0;
	out[2] = std::sin(yaw * 0.5);
	out[3] = 0.0;
	NormalizeQuat(out);
}

void RotateByQuat(const double q[4], const double v[3], double out[3])
{
	const double ux = q[1], uy = q[2], uz = q[3], s = q[0];
	const double tx = 2.0 * (uy * v[2] - uz * v[1]);
	const double ty = 2.0 * (uz * v[0] - ux * v[2]);
	const double tz = 2.0 * (ux * v[1] - uy * v[0]);
	out[0] = v[0] + s * tx + (uy * tz - uz * ty);
	out[1] = v[1] + s * ty + (uz * tx - ux * tz);
	out[2] = v[2] + s * tz + (ux * ty - uy * tx);
}

void SlerpQuat(const double a[4], const double b[4], double alpha, double out[4])
{
	alpha = Clamp(alpha, 0.0, 1.0);
	double dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
	double bb[4] = {b[0], b[1], b[2], b[3]};
	if (dot < 0.0) {
		dot = -dot;
		bb[0] = -bb[0];
		bb[1] = -bb[1];
		bb[2] = -bb[2];
		bb[3] = -bb[3];
	}
	dot = Clamp(dot, -1.0, 1.0);
	if (dot > 0.9995) {
		out[0] = a[0] + (bb[0] - a[0]) * alpha;
		out[1] = a[1] + (bb[1] - a[1]) * alpha;
		out[2] = a[2] + (bb[2] - a[2]) * alpha;
		out[3] = a[3] + (bb[3] - a[3]) * alpha;
		NormalizeQuat(out);
		return;
	}
	const double theta = std::acos(dot);
	const double sinTheta = std::sin(theta);
	const double wa = std::sin((1.0 - alpha) * theta) / sinTheta;
	const double wb = std::sin(alpha * theta) / sinTheta;
	out[0] = wa * a[0] + wb * bb[0];
	out[1] = wa * a[1] + wb * bb[1];
	out[2] = wa * a[2] + wb * bb[2];
	out[3] = wa * a[3] + wb * bb[3];
	NormalizeQuat(out);
}

BodyCompletionPriors ClampPriors(BodyCompletionPriors c)
{
	c.floor_y_m = Clamp(c.floor_y_m, -2.0, 2.0);
	c.height_m = Clamp(c.height_m, 1.0, 2.4);
	c.forward_yaw_rad = NormalizeAngle(c.forward_yaw_rad);
	c.stance_width_m = Clamp(c.stance_width_m, 0.10, 0.70);
	c.shoulder_width_m = Clamp(c.shoulder_width_m, 0.20, 0.70);
	c.pelvis_width_m = Clamp(c.pelvis_width_m, 0.15, 0.60);
	c.upper_arm_m = Clamp(c.upper_arm_m, 0.15, 0.55);
	c.lower_arm_m = Clamp(c.lower_arm_m, 0.15, 0.55);
	c.upper_leg_m = Clamp(c.upper_leg_m, 0.20, 0.70);
	c.lower_leg_m = Clamp(c.lower_leg_m, 0.20, 0.70);
	c.virtual_min_confidence = Clamp(c.virtual_min_confidence, 0.0, 1.0);
	return c;
}

void SetRole(BodyCompletionResult& result, BodyRole role, const BodyCompletionPose& pose, float confidence,
             uint16_t source, BodyCompletionMode mode, uint32_t age_ms = 0)
{
	const auto idx = static_cast<std::size_t>(role);
	if (idx >= result.roles.size()) return;
	auto& out = result.roles[idx];
	out.pose = pose;
	NormalizeQuat(out.pose.rotation);
	out.confidence = static_cast<float>(Clamp(confidence, 0.0, 1.0));
	out.source_mask = source;
	out.mode = mode;
	out.age_ms = age_ms;
	out.valid = true;
}

BodyCompletionPose OffsetPose(const BodyCompletionPose& base, const double right[3], const double up[3],
                              const double forward[3], double r, double u, double f, const double rot[4])
{
	BodyCompletionPose pose = base;
	pose.position[0] += right[0] * r + up[0] * u + forward[0] * f;
	pose.position[1] += right[1] * r + up[1] * u + forward[1] * f;
	pose.position[2] += right[2] * r + up[2] * u + forward[2] * f;
	CopyQuat(rot, pose.rotation);
	return pose;
}

BodyCompletionPose MidPose(const BodyCompletionPose& a, const BodyCompletionPose& b, const double forward[3],
                           double forward_bias, const double rot[4])
{
	BodyCompletionPose pose;
	pose.position[0] = (a.position[0] + b.position[0]) * 0.5 + forward[0] * forward_bias;
	pose.position[1] = (a.position[1] + b.position[1]) * 0.5 + forward[1] * forward_bias;
	pose.position[2] = (a.position[2] + b.position[2]) * 0.5 + forward[2] * forward_bias;
	CopyQuat(rot, pose.rotation);
	return pose;
}

float DegradeByReach(const BodyCompletionPose& a, const BodyCompletionPose& b, double max_reach, float base_confidence)
{
	double d[3];
	Sub3(a.position, b.position, d);
	const double overshoot = std::max(0.0, Len3(d) - max_reach);
	return static_cast<float>(std::max(0.05, base_confidence - overshoot * 1.5));
}

bool ShouldPlantFoot(const BodyCompletionSensorPose& hmd, double floor_y)
{
	if (!hmd.valid) return true;
	return PlanarLen3(hmd.pose.velocity) < 0.20 && std::abs(hmd.pose.velocity[1]) < 0.15 &&
	       hmd.pose.position[1] > floor_y + 0.60;
}

BodyCompletionPose ApplyFootLock(BodyCompletionFootLockState& lock, const BodyCompletionPose& target, bool can_plant,
                                 bool& held)
{
	held = false;
	if (!can_plant) {
		lock.locked = false;
		return target;
	}

	if (!lock.locked) {
		lock.position[0] = target.position[0];
		lock.position[1] = target.position[1];
		lock.position[2] = target.position[2];
		lock.locked = true;
	}

	double delta[3];
	Sub3(target.position, lock.position, delta);
	if (PlanarLen3(delta) > 0.22 || std::abs(delta[1]) > 0.12) {
		lock.locked = false;
		return target;
	}

	BodyCompletionPose held_pose = target;
	held_pose.position[0] = lock.position[0];
	held_pose.position[1] = lock.position[1];
	held_pose.position[2] = lock.position[2];
	held = true;
	return held_pose;
}

double SmoothAlpha(BodyCompletionMode mode)
{
	switch (mode) {
		case BodyCompletionMode::Measured:
			return 1.0;
		case BodyCompletionMode::ControllerIk:
			return 0.55;
		case BodyCompletionMode::HmdRoot:
			return 0.45;
		case BodyCompletionMode::FloorContact:
			return 0.35;
		case BodyCompletionMode::HeldContact:
			return 0.25;
		case BodyCompletionMode::LowConfidence:
			return 0.25;
		case BodyCompletionMode::None:
			return 1.0;
	}
	return 1.0;
}

void SmoothOutput(BodyCompletionRoleOutput& out, BodyCompletionSolverRoleState& prev)
{
	if (!out.valid) return;
	if (!prev.valid || out.mode == BodyCompletionMode::Measured) {
		prev.pose = out.pose;
		prev.confidence = out.confidence;
		prev.valid = true;
		return;
	}

	const double alpha = SmoothAlpha(out.mode);
	BodyCompletionPose smoothed = out.pose;
	Lerp3(prev.pose.position, out.pose.position, alpha, smoothed.position);
	Lerp3(prev.pose.velocity, out.pose.velocity, alpha, smoothed.velocity);
	SlerpQuat(prev.pose.rotation, out.pose.rotation, alpha, smoothed.rotation);
	out.pose = smoothed;
	out.confidence = static_cast<float>(prev.confidence + (out.confidence - prev.confidence) * alpha);
	prev.pose = out.pose;
	prev.confidence = out.confidence;
	prev.valid = true;
}

} // namespace

const char* BodyCompletionModeLabel(BodyCompletionMode mode)
{
	switch (mode) {
		case BodyCompletionMode::None:
			return "none";
		case BodyCompletionMode::Measured:
			return "measured";
		case BodyCompletionMode::HmdRoot:
			return "hmd_root";
		case BodyCompletionMode::ControllerIk:
			return "controller_ik";
		case BodyCompletionMode::FloorContact:
			return "floor_contact";
		case BodyCompletionMode::HeldContact:
			return "held_contact";
		case BodyCompletionMode::LowConfidence:
			return "low_confidence";
	}
	return "unknown";
}

void BodyCompletionSolver::Reset()
{
	previous_ = {};
	left_foot_lock_ = {};
	right_foot_lock_ = {};
}

BodyCompletionResult BodyCompletionSolver::Solve(const BodyCompletionInput& input)
{
	BodyCompletionResult result;
	const auto cal = ClampPriors(input.priors);
	if (!input.hmd.valid) return result;

	const double hmd_yaw = YawRadiansFromQuat(input.hmd.pose.rotation);
	double body_yaw = hmd_yaw;
	if (cal.forward_estimated && ShouldPlantFoot(input.hmd, cal.floor_y_m)) {
		const double speed = PlanarLen3(input.hmd.pose.velocity);
		body_yaw = LerpAngle(cal.forward_yaw_rad, hmd_yaw, Clamp(speed / 0.35, 0.0, 1.0));
	}
	double body_rot[4];
	YawQuat(body_yaw, body_rot);
	double right[3];
	double up[3];
	double forward[3];
	const double local_right[3] = {1.0, 0.0, 0.0};
	const double local_up[3] = {0.0, 1.0, 0.0};
	const double local_forward[3] = {0.0, 0.0, 1.0};
	RotateByQuat(body_rot, local_right, right);
	RotateByQuat(body_rot, local_up, up);
	RotateByQuat(body_rot, local_forward, forward);

	const double head_height = Clamp(input.hmd.pose.position[1] - cal.floor_y_m, 0.55, cal.height_m + 0.25);
	const double crouch_scale = Clamp(head_height / cal.height_m, 0.45, 1.15);

	BodyCompletionPose hmd_pose = input.hmd.pose;
	SetRole(result, BodyRole::Hmd, hmd_pose, 1.0f, kBodySourceMeasured | kBodySourceHmd, BodyCompletionMode::Measured,
	        input.hmd.age_ms);

	BodyCompletionPose waist = hmd_pose;
	waist.position[0] = input.hmd.pose.position[0] - forward[0] * 0.05;
	waist.position[1] = cal.floor_y_m + cal.height_m * 0.54 * crouch_scale;
	waist.position[2] = input.hmd.pose.position[2] - forward[2] * 0.05;
	CopyQuat(body_rot, waist.rotation);

	BodyCompletionPose chest = hmd_pose;
	chest.position[0] = input.hmd.pose.position[0] - forward[0] * 0.03;
	chest.position[1] = cal.floor_y_m + cal.height_m * 0.78 * crouch_scale;
	chest.position[2] = input.hmd.pose.position[2] - forward[2] * 0.03;
	CopyQuat(body_rot, chest.rotation);

	SetRole(result, BodyRole::Waist, waist, 0.48f, kBodySourceHmd | kBodySourcePredicted, BodyCompletionMode::HmdRoot);
	SetRole(result, BodyRole::Chest, chest, 0.58f, kBodySourceHmd | kBodySourcePredicted, BodyCompletionMode::HmdRoot);

	BodyCompletionPose left_shoulder =
	    OffsetPose(chest, right, up, forward, -cal.shoulder_width_m * 0.5, 0.08, 0.0, body_rot);
	BodyCompletionPose right_shoulder =
	    OffsetPose(chest, right, up, forward, cal.shoulder_width_m * 0.5, 0.08, 0.0, body_rot);
	SetRole(result, BodyRole::LeftShoulder, left_shoulder, 0.42f, kBodySourceHmd | kBodySourcePredicted,
	        BodyCompletionMode::HmdRoot);
	SetRole(result, BodyRole::RightShoulder, right_shoulder, 0.42f, kBodySourceHmd | kBodySourcePredicted,
	        BodyCompletionMode::HmdRoot);

	auto solveArm = [&](BodyRole elbowRole, const BodyCompletionPose& shoulder,
	                    const BodyCompletionSensorPose& controller, double side) {
		if (controller.valid) {
			BodyCompletionPose hand = controller.pose;
			BodyCompletionPose elbow = MidPose(shoulder, hand, forward, -0.08, body_rot);
			elbow.position[0] += right[0] * side * 0.04;
			elbow.position[1] -= 0.04;
			elbow.position[2] += right[2] * side * 0.04;
			const float confidence = DegradeByReach(shoulder, hand, cal.upper_arm_m + cal.lower_arm_m + 0.05, 0.70f);
			SetRole(result, elbowRole, elbow, confidence, kBodySourceHmd | kBodySourceController | kBodySourcePredicted,
			        confidence >= 0.25f ? BodyCompletionMode::ControllerIk : BodyCompletionMode::LowConfidence,
			        controller.age_ms);
			return;
		}
		BodyCompletionPose elbow = OffsetPose(shoulder, right, up, forward, side * cal.shoulder_width_m * 0.10,
		                                      -cal.upper_arm_m * 0.65, 0.03, body_rot);
		SetRole(result, elbowRole, elbow, 0.18f, kBodySourceHmd | kBodySourcePredicted,
		        BodyCompletionMode::LowConfidence);
	};

	solveArm(BodyRole::LeftElbow, left_shoulder, input.left_controller, -1.0);
	solveArm(BodyRole::RightElbow, right_shoulder, input.right_controller, 1.0);

	const BodyCompletionPose left_hip =
	    OffsetPose(waist, right, up, forward, -cal.pelvis_width_m * 0.5, -0.02, 0.0, body_rot);
	const BodyCompletionPose right_hip =
	    OffsetPose(waist, right, up, forward, cal.pelvis_width_m * 0.5, -0.02, 0.0, body_rot);

	BodyCompletionPose left_foot = OffsetPose(waist, right, up, forward, -cal.stance_width_m * 0.5,
	                                          cal.floor_y_m - waist.position[1] + 0.04, 0.02, body_rot);
	BodyCompletionPose right_foot = OffsetPose(waist, right, up, forward, cal.stance_width_m * 0.5,
	                                           cal.floor_y_m - waist.position[1] + 0.04, 0.02, body_rot);

	bool leftHeld = false;
	bool rightHeld = false;
	const bool canPlant = ShouldPlantFoot(input.hmd, cal.floor_y_m);
	left_foot = ApplyFootLock(left_foot_lock_, left_foot, canPlant, leftHeld);
	right_foot = ApplyFootLock(right_foot_lock_, right_foot, canPlant, rightHeld);

	SetRole(result, BodyRole::LeftFoot, left_foot, leftHeld ? 0.42f : 0.26f,
	        kBodySourceHmd | kBodySourceFloor | kBodySourcePredicted |
	            (leftHeld ? kBodySourceContact | kBodySourceHeld : kBodySourceNone),
	        leftHeld ? BodyCompletionMode::HeldContact : BodyCompletionMode::FloorContact);
	SetRole(result, BodyRole::RightFoot, right_foot, rightHeld ? 0.42f : 0.26f,
	        kBodySourceHmd | kBodySourceFloor | kBodySourcePredicted |
	            (rightHeld ? kBodySourceContact | kBodySourceHeld : kBodySourceNone),
	        rightHeld ? BodyCompletionMode::HeldContact : BodyCompletionMode::FloorContact);

	BodyCompletionPose left_knee = MidPose(left_hip, left_foot, forward, 0.08, body_rot);
	BodyCompletionPose right_knee = MidPose(right_hip, right_foot, forward, 0.08, body_rot);
	left_knee.position[1] = std::max(cal.floor_y_m + cal.lower_leg_m * 0.55, left_knee.position[1]);
	right_knee.position[1] = std::max(cal.floor_y_m + cal.lower_leg_m * 0.55, right_knee.position[1]);
	SetRole(result, BodyRole::LeftKnee, left_knee, leftHeld ? 0.34f : 0.22f,
	        kBodySourceHmd | kBodySourceFloor | kBodySourcePredicted, BodyCompletionMode::FloorContact);
	SetRole(result, BodyRole::RightKnee, right_knee, rightHeld ? 0.34f : 0.22f,
	        kBodySourceHmd | kBodySourceFloor | kBodySourcePredicted, BodyCompletionMode::FloorContact);

	for (uint8_t i = 0; i < kBodyRoleCount; ++i) {
		const auto role = static_cast<BodyRole>(i);
		const auto& measured = input.real_roles[i];
		if (!measured.valid || role == BodyRole::None) continue;
		SetRole(result, role, measured.pose, 0.98f, kBodySourceMeasured, BodyCompletionMode::Measured, measured.age_ms);
	}

	double confidence_sum = 0.0;
	uint32_t confidence_count = 0;
	for (uint8_t i = 0; i < kBodyRoleCount; ++i) {
		auto& out = result.roles[i];
		if (!out.valid) continue;
		if (!input.enabled_roles[i] && i != static_cast<uint8_t>(BodyRole::Hmd) &&
		    (out.source_mask & kBodySourceMeasured) == 0) {
			continue;
		}
		SmoothOutput(out, previous_[i]);
		confidence_sum += out.confidence;
		++confidence_count;
	}
	result.global_confidence = confidence_count ? static_cast<float>(confidence_sum / confidence_count) : 0.0f;
	return result;
}

} // namespace phantom
