#include <gtest/gtest.h>

#include "PhantomMetrics.h"
#include "PhantomTrajectory.h"

using namespace phantom;
using namespace phantom::analysis;

TEST(PhantomTrajectory, GeneratesRequestedFrameCountAndCoreRoles)
{
	PhantomTrajectoryOptions options;
	options.motion = PhantomMotion::ForwardWalk;
	options.frame_count = 120;

	const auto frames = GenerateTrajectory(options);
	ASSERT_EQ(frames.size(), 120u);
	const auto& first = frames.front();
	EXPECT_TRUE(first.role_valid[static_cast<size_t>(BodyRole::Waist)]);
	EXPECT_TRUE(first.role_valid[static_cast<size_t>(BodyRole::LeftFoot)]);
	EXPECT_TRUE(first.role_valid[static_cast<size_t>(BodyRole::RightElbow)]);
	EXPECT_NEAR(first.hmd.position[1], options.height_m, 1e-9);
	EXPECT_GT(frames.back().hmd.position[2], frames.front().hmd.position[2]);
}

TEST(PhantomTrajectory, CrouchAndSitLowerTheHead)
{
	PhantomTrajectoryOptions options;
	options.motion = PhantomMotion::Crouch;
	options.frame_count = 181;
	const auto crouch = GenerateTrajectory(options);

	options.motion = PhantomMotion::SitStand;
	const auto sit = GenerateTrajectory(options);

	const auto middle = crouch.size() / 2;
	EXPECT_LT(crouch[middle].hmd.position[1], crouch.front().hmd.position[1] - 0.40);
	EXPECT_LT(sit[middle].hmd.position[1], sit.front().hmd.position[1] - 0.50);
}

TEST(PhantomTrajectory, WalkHasAlternatingPlantedFeet)
{
	PhantomTrajectoryOptions options;
	options.motion = PhantomMotion::WalkInPlace;
	options.frame_count = 180;
	const auto frames = GenerateTrajectory(options);

	uint32_t left_planted = 0;
	uint32_t right_planted = 0;
	for (const auto& frame : frames) {
		if (frame.planted[static_cast<size_t>(BodyRole::LeftFoot)]) ++left_planted;
		if (frame.planted[static_cast<size_t>(BodyRole::RightFoot)]) ++right_planted;
	}
	EXPECT_GT(left_planted, 0u);
	EXPECT_GT(right_planted, 0u);
	EXPECT_LT(left_planted, frames.size());
	EXPECT_LT(right_planted, frames.size());
}

TEST(PhantomTrajectory, FootSkateMetricIsZeroWhenFootIsLocked)
{
	PhantomTrajectoryOptions options;
	options.motion = PhantomMotion::IdleStand;
	options.frame_count = 60;
	const auto frames = GenerateTrajectory(options);
	std::vector<BodyCompletionPose> left_foot;
	for (const auto& frame : frames) {
		left_foot.push_back(frame.roles[static_cast<size_t>(BodyRole::LeftFoot)]);
	}

	const auto stats = ComputeFootSkateStats(left_foot, options.floor_y_m, 0.05, 0.01);
	EXPECT_GT(stats.planted_frames, 0u);
	EXPECT_EQ(stats.skating_frames, 0u);
	EXPECT_NEAR(stats.total_slide_m, 0.0, 1e-9);
}

TEST(PhantomTrajectory, CanBuildBodyCompletionInputsWithMeasuredRoles)
{
	PhantomTrajectoryOptions options;
	options.motion = PhantomMotion::ControllerReach;
	options.frame_count = 20;
	const auto frames = GenerateTrajectory(options);
	const auto enabled = AllVirtualTrackerRolesEnabled();
	auto measured = RolesEnabled({BodyRole::Waist, BodyRole::LeftFoot, BodyRole::RightFoot});

	const auto input = MakeBodyCompletionInput(frames.back(), DefaultTrajectoryPriors(options), enabled, measured);
	EXPECT_TRUE(input.hmd.valid);
	EXPECT_TRUE(input.left_controller.valid);
	EXPECT_TRUE(input.real_roles[static_cast<size_t>(BodyRole::Waist)].valid);
	EXPECT_TRUE(input.real_roles[static_cast<size_t>(BodyRole::LeftFoot)].valid);
	EXPECT_FALSE(input.real_roles[static_cast<size_t>(BodyRole::Chest)].valid);
}
