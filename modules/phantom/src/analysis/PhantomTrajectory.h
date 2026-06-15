#pragma once

#include "BodyCompletionSolver.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace phantom::analysis {

enum class PhantomMotion : uint8_t
{
	IdleStand,
	WalkInPlace,
	ForwardWalk,
	Crouch,
	SitStand,
	BendLean,
	FastTurn,
	ControllerReach,
};

struct PhantomTrajectoryOptions
{
	PhantomMotion motion = PhantomMotion::IdleStand;
	uint32_t frame_count = 180;
	double frame_rate_hz = 90.0;
	double floor_y_m = 0.0;
	double height_m = 1.70;
	double stance_width_m = 0.30;
	double shoulder_width_m = 0.40;
	double pelvis_width_m = 0.30;
	double walk_speed_mps = 0.80;
};

struct PhantomTrajectoryFrame
{
	double time_ms = 0.0;
	double dt_seconds = 1.0 / 90.0;
	BodyCompletionPose hmd;
	BodyCompletionPose left_controller;
	BodyCompletionPose right_controller;
	std::array<BodyCompletionPose, kBodyRoleCount> roles{};
	std::array<bool, kBodyRoleCount> role_valid{};
	std::array<bool, kBodyRoleCount> planted{};
};

inline double Clamp01(double value)
{
	return std::max(0.0, std::min(1.0, value));
}

inline double SmoothStep(double value)
{
	value = Clamp01(value);
	return value * value * (3.0 - 2.0 * value);
}

inline void SetYawQuat(double yaw_rad, double out[4])
{
	out[0] = std::cos(yaw_rad * 0.5);
	out[1] = 0.0;
	out[2] = std::sin(yaw_rad * 0.5);
	out[3] = 0.0;
}

inline BodyCompletionPose Pose(double x, double y, double z, double yaw_rad = 0.0)
{
	BodyCompletionPose pose;
	pose.position[0] = x;
	pose.position[1] = y;
	pose.position[2] = z;
	SetYawQuat(yaw_rad, pose.rotation);
	return pose;
}

inline BodyCompletionPose WithVelocity(BodyCompletionPose pose, const BodyCompletionPose& prev, double dt_seconds)
{
	if (dt_seconds <= 0.0) return pose;
	for (int i = 0; i < 3; ++i) {
		pose.velocity[i] = (pose.position[i] - prev.position[i]) / dt_seconds;
	}
	return pose;
}

inline BodyCompletionCalibration DefaultTrajectoryCalibration(const PhantomTrajectoryOptions& options)
{
	BodyCompletionCalibration cal;
	cal.floor_y_m = options.floor_y_m;
	cal.height_m = options.height_m;
	cal.stance_width_m = options.stance_width_m;
	cal.shoulder_width_m = options.shoulder_width_m;
	cal.pelvis_width_m = options.pelvis_width_m;
	cal.forward_calibrated = true;
	return cal;
}

inline void SetRole(PhantomTrajectoryFrame& frame, BodyRole role, const BodyCompletionPose& pose, bool planted = false)
{
	const auto idx = static_cast<size_t>(role);
	if (idx >= frame.roles.size()) return;
	frame.roles[idx] = pose;
	frame.role_valid[idx] = true;
	frame.planted[idx] = planted;
}

inline std::vector<PhantomTrajectoryFrame> GenerateTrajectory(const PhantomTrajectoryOptions& options)
{
	const double rate = std::max(1.0, options.frame_rate_hz);
	const double dt = 1.0 / rate;
	const double duration = std::max(dt, static_cast<double>(std::max<uint32_t>(1, options.frame_count - 1)) * dt);
	constexpr double kPi = 3.14159265358979323846;
	std::vector<PhantomTrajectoryFrame> frames;
	frames.reserve(options.frame_count);

	PhantomTrajectoryFrame prev;
	bool have_prev = false;

	for (uint32_t i = 0; i < options.frame_count; ++i) {
		const double t = static_cast<double>(i) * dt;
		const double u = duration > 0.0 ? t / duration : 0.0;
		const double gait = std::sin(2.0 * kPi * 1.5 * t);
		const double gait_other = std::sin(2.0 * kPi * 1.5 * t + kPi);

		double yaw = 0.0;
		double root_x = 0.0;
		double root_z = 0.0;
		double head_y = options.floor_y_m + options.height_m;
		double chest_forward = -0.03;
		double hand_reach = 0.0;

		switch (options.motion) {
			case PhantomMotion::IdleStand:
				break;
			case PhantomMotion::WalkInPlace:
				head_y += 0.025 * std::abs(gait);
				break;
			case PhantomMotion::ForwardWalk:
				root_z = options.walk_speed_mps * t;
				head_y += 0.02 * std::abs(gait);
				break;
			case PhantomMotion::Crouch:
				head_y -= 0.55 * SmoothStep(std::sin(kPi * u));
				break;
			case PhantomMotion::SitStand:
				head_y -= 0.62 * SmoothStep(std::sin(kPi * u));
				root_z -= 0.10 * SmoothStep(std::sin(kPi * u));
				break;
			case PhantomMotion::BendLean:
				head_y -= 0.28 * SmoothStep(std::sin(kPi * u));
				chest_forward = -0.28 * SmoothStep(std::sin(kPi * u));
				break;
			case PhantomMotion::FastTurn:
				yaw = kPi * SmoothStep(u);
				break;
			case PhantomMotion::ControllerReach:
				hand_reach = 0.45 * SmoothStep(std::sin(kPi * u));
				break;
		}

		PhantomTrajectoryFrame frame;
		frame.time_ms = t * 1000.0;
		frame.dt_seconds = dt;
		frame.hmd = Pose(root_x, head_y, root_z, yaw);

		const double crouch_scale =
		    std::clamp((head_y - options.floor_y_m) / std::max(1e-6, options.height_m), 0.45, 1.15);
		const double waist_y = options.floor_y_m + options.height_m * 0.54 * crouch_scale;
		const double chest_y = options.floor_y_m + options.height_m * 0.78 * crouch_scale;
		const double left_x = root_x - options.stance_width_m * 0.5;
		const double right_x = root_x + options.stance_width_m * 0.5;
		const double foot_lift =
		    options.motion == PhantomMotion::WalkInPlace || options.motion == PhantomMotion::ForwardWalk ? 0.08 : 0.0;
		const double step_z =
		    options.motion == PhantomMotion::WalkInPlace || options.motion == PhantomMotion::ForwardWalk ? 0.18 : 0.0;
		const double left_step = step_z * gait;
		const double right_step = step_z * gait_other;
		const double left_foot_y = options.floor_y_m + 0.04 + foot_lift * std::max(0.0, gait);
		const double right_foot_y = options.floor_y_m + 0.04 + foot_lift * std::max(0.0, gait_other);
		const bool left_planted = left_foot_y <= options.floor_y_m + 0.045;
		const bool right_planted = right_foot_y <= options.floor_y_m + 0.045;

		SetRole(frame, BodyRole::Hmd, frame.hmd);
		SetRole(frame, BodyRole::Waist, Pose(root_x, waist_y, root_z - 0.05, yaw));
		SetRole(frame, BodyRole::Chest, Pose(root_x, chest_y, root_z + chest_forward, yaw));
		SetRole(frame, BodyRole::LeftShoulder,
		        Pose(root_x - options.shoulder_width_m * 0.5, chest_y + 0.08, root_z + chest_forward, yaw));
		SetRole(frame, BodyRole::RightShoulder,
		        Pose(root_x + options.shoulder_width_m * 0.5, chest_y + 0.08, root_z + chest_forward, yaw));
		SetRole(frame, BodyRole::LeftFoot, Pose(left_x, left_foot_y, root_z + left_step, yaw), left_planted);
		SetRole(frame, BodyRole::RightFoot, Pose(right_x, right_foot_y, root_z + right_step, yaw), right_planted);
		SetRole(frame, BodyRole::LeftKnee,
		        Pose(left_x * 0.85, options.floor_y_m + 0.48, root_z + left_step * 0.45, yaw));
		SetRole(frame, BodyRole::RightKnee,
		        Pose(right_x * 0.85, options.floor_y_m + 0.48, root_z + right_step * 0.45, yaw));

		frame.left_controller = Pose(root_x - 0.36, chest_y - 0.30, root_z + 0.16 + hand_reach, yaw);
		frame.right_controller = Pose(root_x + 0.36, chest_y - 0.30, root_z + 0.16 + hand_reach, yaw);
		SetRole(frame, BodyRole::LeftHand, frame.left_controller);
		SetRole(frame, BodyRole::RightHand, frame.right_controller);
		SetRole(frame, BodyRole::LeftElbow, Pose(root_x - 0.34, chest_y - 0.12, root_z + 0.05 + hand_reach * 0.5, yaw));
		SetRole(frame, BodyRole::RightElbow,
		        Pose(root_x + 0.34, chest_y - 0.12, root_z + 0.05 + hand_reach * 0.5, yaw));

		if (have_prev) {
			frame.hmd = WithVelocity(frame.hmd, prev.hmd, dt);
			frame.left_controller = WithVelocity(frame.left_controller, prev.left_controller, dt);
			frame.right_controller = WithVelocity(frame.right_controller, prev.right_controller, dt);
			for (size_t role = 0; role < frame.roles.size(); ++role) {
				if (frame.role_valid[role] && prev.role_valid[role]) {
					frame.roles[role] = WithVelocity(frame.roles[role], prev.roles[role], dt);
				}
			}
		}
		prev = frame;
		have_prev = true;
		frames.push_back(frame);
	}
	return frames;
}

inline BodyCompletionSensorPose SensorFromPose(const BodyCompletionPose& pose, bool valid = true, uint32_t age_ms = 0)
{
	BodyCompletionSensorPose sensor;
	sensor.pose = pose;
	sensor.valid = valid;
	sensor.age_ms = age_ms;
	return sensor;
}

inline BodyCompletionInput MakeBodyCompletionInput(const PhantomTrajectoryFrame& frame,
                                                   const BodyCompletionCalibration& calibration,
                                                   const std::array<bool, kBodyRoleCount>& enabled_roles,
                                                   const std::array<bool, kBodyRoleCount>& measured_roles,
                                                   bool include_left_controller = true,
                                                   bool include_right_controller = true)
{
	BodyCompletionInput input;
	input.dt_seconds = frame.dt_seconds;
	input.calibration = calibration;
	input.hmd = SensorFromPose(frame.hmd);
	if (include_left_controller) input.left_controller = SensorFromPose(frame.left_controller);
	if (include_right_controller) input.right_controller = SensorFromPose(frame.right_controller);
	input.enabled_roles = enabled_roles;
	for (uint8_t i = 0; i < kBodyRoleCount; ++i) {
		if (!measured_roles[i] || !frame.role_valid[i]) continue;
		input.real_roles[i] = SensorFromPose(frame.roles[i]);
	}
	return input;
}

inline std::array<bool, kBodyRoleCount> RolesEnabled(std::initializer_list<BodyRole> roles)
{
	std::array<bool, kBodyRoleCount> enabled{};
	for (BodyRole role : roles) {
		const auto idx = static_cast<size_t>(role);
		if (idx < enabled.size()) enabled[idx] = true;
	}
	return enabled;
}

inline std::array<bool, kBodyRoleCount> AllVirtualTrackerRolesEnabled()
{
	return RolesEnabled({BodyRole::Waist, BodyRole::Chest, BodyRole::LeftFoot, BodyRole::RightFoot, BodyRole::LeftKnee,
	                     BodyRole::RightKnee, BodyRole::LeftElbow, BodyRole::RightElbow});
}

} // namespace phantom::analysis
