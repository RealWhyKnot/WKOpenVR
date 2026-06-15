#include <gtest/gtest.h>

#include "BodyPriorEstimator.h"
#include "PhantomTrajectory.h"

#include <cstddef>
#include <cmath>

using namespace phantom;
using namespace phantom::analysis;

namespace {

BodyPriorSample SampleFromFrame(const PhantomTrajectoryFrame& frame, bool include_feet = true)
{
	BodyPriorSample sample;
	sample.hmd = SensorFromPose(frame.hmd);
	if (include_feet) {
		sample.measured_roles[static_cast<size_t>(BodyRole::LeftFoot)] =
		    SensorFromPose(frame.roles[static_cast<size_t>(BodyRole::LeftFoot)]);
		sample.measured_roles[static_cast<size_t>(BodyRole::RightFoot)] =
		    SensorFromPose(frame.roles[static_cast<size_t>(BodyRole::RightFoot)]);
	}
	return sample;
}

BodyPriorSample StillSample(double hmd_y, double floor_y, double yaw)
{
	BodyPriorSample sample;
	sample.hmd = SensorFromPose(Pose(0.0, hmd_y, 0.0, yaw));
	sample.measured_roles[static_cast<size_t>(BodyRole::LeftFoot)] = SensorFromPose(Pose(-0.15, floor_y + 0.04, 0.0));
	sample.measured_roles[static_cast<size_t>(BodyRole::RightFoot)] = SensorFromPose(Pose(0.15, floor_y + 0.04, 0.0));
	return sample;
}

} // namespace

TEST(BodyPriorEstimator, DefaultsHoldBeforeWarmup)
{
	BodyPriorEstimator estimator;
	const auto initial = estimator.Estimate(0.35);

	EXPECT_DOUBLE_EQ(initial.priors.height_m, 1.70);
	EXPECT_DOUBLE_EQ(initial.priors.floor_y_m, 0.0);
	EXPECT_FALSE(initial.priors.forward_estimated);
	EXPECT_DOUBLE_EQ(initial.priors.virtual_min_confidence, 0.35);

	estimator.AddSample(StillSample(1.86, -0.12, 0.0));
	const auto after_one = estimator.Estimate(0.35);
	EXPECT_DOUBLE_EQ(after_one.priors.height_m, 1.70);
	EXPECT_DOUBLE_EQ(after_one.priors.floor_y_m, 0.0);
	EXPECT_FALSE(after_one.priors.forward_estimated);
	EXPECT_LT(after_one.settled, 1.0);
}

TEST(BodyPriorEstimator, ConvergesHeightFloorAndRatiosFromFeet)
{
	PhantomTrajectoryOptions options;
	options.motion = PhantomMotion::IdleStand;
	options.frame_count = 180;
	options.floor_y_m = -0.12;
	options.height_m = 1.86;
	const auto frames = GenerateTrajectory(options);

	BodyPriorEstimator estimator;
	for (const auto& frame : frames) {
		estimator.AddSample(SampleFromFrame(frame));
	}

	const auto estimate = estimator.Estimate(0.20);
	EXPECT_NEAR(estimate.priors.floor_y_m, options.floor_y_m, 0.02);
	EXPECT_NEAR(estimate.priors.height_m, options.height_m, 0.04);
	EXPECT_NEAR(estimate.priors.stance_width_m, estimate.priors.height_m * 0.165, 0.005);
	EXPECT_NEAR(estimate.priors.shoulder_width_m, estimate.priors.height_m * 0.225, 0.005);
	EXPECT_NEAR(estimate.priors.pelvis_width_m, estimate.priors.height_m * 0.165, 0.005);
	EXPECT_NEAR(estimate.priors.upper_arm_m, estimate.priors.height_m * 0.176, 0.005);
	EXPECT_NEAR(estimate.priors.lower_arm_m, estimate.priors.height_m * 0.159, 0.005);
	EXPECT_NEAR(estimate.priors.upper_leg_m, estimate.priors.height_m * 0.265, 0.005);
	EXPECT_NEAR(estimate.priors.lower_leg_m, estimate.priors.height_m * 0.265, 0.005);
	EXPECT_TRUE(estimate.floor_from_tracker);
	EXPECT_DOUBLE_EQ(estimate.settled, 1.0);
}

TEST(BodyPriorEstimator, ForwardYawNeedsStableWarmup)
{
	BodyPriorEstimator estimator;
	constexpr double kYaw = 0.70;
	for (int i = 0; i < 90; ++i) {
		estimator.AddSample(StillSample(1.78, 0.0, kYaw));
	}

	const auto estimate = estimator.Estimate(0.20);
	EXPECT_TRUE(estimate.priors.forward_estimated);
	EXPECT_NEAR(estimate.priors.forward_yaw_rad, kYaw, 0.02);
}

TEST(BodyPriorEstimator, MovingHmdDoesNotPublishEarlyPriors)
{
	PhantomTrajectoryOptions options;
	options.motion = PhantomMotion::ForwardWalk;
	options.frame_count = 60;
	options.height_m = 1.92;
	const auto frames = GenerateTrajectory(options);

	BodyPriorEstimator estimator;
	for (const auto& frame : frames) {
		estimator.AddSample(SampleFromFrame(frame));
	}

	const auto estimate = estimator.Estimate(0.20);
	EXPECT_LT(estimate.settled, 1.0);
	EXPECT_LE(estimate.stable_sample_count, 30u);
}
