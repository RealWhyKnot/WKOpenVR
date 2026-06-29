#pragma once

#include "BodyCompletionSolver.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace phantom {

struct BodyPriorSample
{
	BodyCompletionSensorPose hmd;
	std::array<BodyCompletionSensorPose, kBodyRoleCount> measured_roles{};
};

struct BodyPriorEstimate
{
	BodyCompletionPriors priors;
	double settled = 0.0;
	uint32_t stable_sample_count = 0;
	bool floor_from_tracker = false;
};

inline double ClampBodyPriorValue(double v, double lo, double hi)
{
	return std::max(lo, std::min(hi, v));
}

inline void ApplyAnthropometricPriors(BodyCompletionPriors& priors)
{
	const double h = ClampBodyPriorValue(priors.height_m, 1.0, 2.4);
	priors.height_m = h;
	priors.stance_width_m = ClampBodyPriorValue(h * 0.165, 0.10, 0.70);
	priors.shoulder_width_m = ClampBodyPriorValue(h * 0.225, 0.20, 0.70);
	priors.pelvis_width_m = ClampBodyPriorValue(h * 0.165, 0.15, 0.60);
	priors.upper_arm_m = ClampBodyPriorValue(h * 0.176, 0.15, 0.55);
	priors.lower_arm_m = ClampBodyPriorValue(h * 0.159, 0.15, 0.55);
	priors.upper_leg_m = ClampBodyPriorValue(h * 0.265, 0.20, 0.70);
	priors.lower_leg_m = ClampBodyPriorValue(h * 0.265, 0.20, 0.70);
}

class BodyPriorEstimator
{
public:
	void Reset()
	{
		live_priors_ = BodyCompletionPriors{};
		ApplyAnthropometricPriors(live_priors_);
		floor_y_m_ = live_priors_.floor_y_m;
		height_m_ = live_priors_.height_m;
		forward_yaw_rad_ = live_priors_.forward_yaw_rad;
		stable_sample_count_ = 0;
		yaw_sample_count_ = 0;
		floor_initialized_ = false;
		height_initialized_ = false;
		yaw_initialized_ = false;
		floor_from_tracker_ = false;
	}

	void AddSample(const BodyPriorSample& sample)
	{
		if (!sample.hmd.valid) return;

		const auto& hmd = sample.hmd.pose;
		const bool stable = IsStableHmd(hmd);
		bool hasFootFloor = false;
		double footFloorY = 0.0;
		FindFootFloor(sample, hasFootFloor, footFloorY);
		if (hasFootFloor) {
			UpdateFloor(footFloorY, 0.08);
			floor_from_tracker_ = true;
		}
		else if (stable && !floor_initialized_ && IsPlausibleHmdY(hmd.position[1])) {
			UpdateFloor(hmd.position[1] - BodyCompletionPriors{}.height_m, 0.03);
		}

		if (!stable) return;

		const double floor = floor_initialized_ ? floor_y_m_ : BodyCompletionPriors{}.floor_y_m;
		const double height = hmd.position[1] - floor;
		if (height >= 1.0 && height <= 2.4) {
			if (!height_initialized_) {
				height_m_ = height;
				height_initialized_ = true;
			}
			else {
				height_m_ = height_m_ + (height - height_m_) * 0.05;
			}
			height_m_ = ClampBodyPriorValue(height_m_, 1.0, 2.4);
			++stable_sample_count_;
		}

		const double yaw = YawRadiansFromQuat(hmd.rotation);
		if (!yaw_initialized_) {
			forward_yaw_rad_ = yaw;
			yaw_initialized_ = true;
		}
		else {
			forward_yaw_rad_ = LerpAngle(forward_yaw_rad_, yaw, 0.04);
		}
		++yaw_sample_count_;
	}

	// Seed the learned height + floor from a one-shot snap so estimated trackers
	// start near-correct immediately instead of waiting out stand-still sampling.
	// Marks enough stable samples that Estimate() reports the seeded priors right
	// away; yaw is left to settle on its own from live motion (not faked).
	void Seed(double height_m, double floor_y_m)
	{
		floor_y_m_ = ClampBodyPriorValue(floor_y_m, -2.0, 2.0);
		floor_initialized_ = true;
		floor_from_tracker_ = false;
		height_m_ = ClampBodyPriorValue(height_m, 1.0, 2.4);
		height_initialized_ = true;
		if (stable_sample_count_ < 30) {
			stable_sample_count_ = 30;
		}
	}

	BodyPriorEstimate Estimate(double virtual_min_confidence) const
	{
		BodyPriorEstimate estimate;
		estimate.stable_sample_count = stable_sample_count_;
		estimate.floor_from_tracker = floor_from_tracker_;
		estimate.settled = ClampBodyPriorValue(static_cast<double>(stable_sample_count_) / 60.0, 0.0, 1.0);

		BodyCompletionPriors out;
		if (stable_sample_count_ >= 30 && height_initialized_) {
			out = live_priors_;
			out.floor_y_m = floor_initialized_ ? floor_y_m_ : BodyCompletionPriors{}.floor_y_m;
			out.height_m = height_m_;
			out.forward_yaw_rad = yaw_initialized_ ? NormalizeAngle(forward_yaw_rad_) : 0.0;
			out.forward_estimated = yaw_sample_count_ >= 60 && yaw_initialized_;
			ApplyAnthropometricPriors(out);
		}
		out.virtual_min_confidence = ClampBodyPriorValue(virtual_min_confidence, 0.0, 1.0);
		estimate.priors = out;
		return estimate;
	}

private:
	static bool IsPlausibleHmdY(double y) { return y >= -1.0 && y <= 4.0; }

	static bool IsStableHmd(const BodyCompletionPose& pose)
	{
		const double planar = std::sqrt(pose.velocity[0] * pose.velocity[0] + pose.velocity[2] * pose.velocity[2]);
		return planar < 0.25 && std::abs(pose.velocity[1]) < 0.20 && IsPlausibleHmdY(pose.position[1]);
	}

	static void FindFootFloor(const BodyPriorSample& sample, bool& found, double& floor_y)
	{
		found = false;
		floor_y = 0.0;
		const BodyRole footRoles[] = {BodyRole::LeftFoot, BodyRole::RightFoot};
		for (const auto role : footRoles) {
			const auto idx = static_cast<std::size_t>(role);
			if (idx >= sample.measured_roles.size()) continue;
			const auto& sensor = sample.measured_roles[idx];
			if (!sensor.valid || sensor.age_ms > 300u) continue;
			const double candidate = sensor.pose.position[1] - 0.04;
			if (!found || candidate < floor_y) {
				floor_y = candidate;
				found = true;
			}
		}
	}

	void UpdateFloor(double candidate, double alpha)
	{
		candidate = ClampBodyPriorValue(candidate, -2.0, 2.0);
		if (!floor_initialized_) {
			floor_y_m_ = candidate;
			floor_initialized_ = true;
			return;
		}
		floor_y_m_ = floor_y_m_ + (candidate - floor_y_m_) * ClampBodyPriorValue(alpha, 0.0, 1.0);
		floor_y_m_ = ClampBodyPriorValue(floor_y_m_, -2.0, 2.0);
	}

	static double NormalizeAngle(double a)
	{
		constexpr double kPi = 3.14159265358979323846;
		while (a > kPi)
			a -= 2.0 * kPi;
		while (a < -kPi)
			a += 2.0 * kPi;
		return a;
	}

	static double LerpAngle(double a, double b, double alpha)
	{
		return NormalizeAngle(a + NormalizeAngle(b - a) * ClampBodyPriorValue(alpha, 0.0, 1.0));
	}

	static double YawRadiansFromQuat(const double q[4])
	{
		const double siny = 2.0 * (q[0] * q[2] + q[1] * q[3]);
		const double cosy = 1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]);
		return std::atan2(siny, cosy);
	}

	BodyCompletionPriors live_priors_{};
	double floor_y_m_ = 0.0;
	double height_m_ = 1.70;
	double forward_yaw_rad_ = 0.0;
	uint32_t stable_sample_count_ = 0;
	uint32_t yaw_sample_count_ = 0;
	bool floor_initialized_ = false;
	bool height_initialized_ = false;
	bool yaw_initialized_ = false;
	bool floor_from_tracker_ = false;
};

} // namespace phantom
