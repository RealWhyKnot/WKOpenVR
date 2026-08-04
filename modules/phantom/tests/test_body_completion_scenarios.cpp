#include <gtest/gtest.h>

#include "BodyCompletionSolver.h"
#include "PhantomMetrics.h"
#include "PhantomTrajectory.h"

#include <array>
#include <vector>

using namespace phantom;
using namespace phantom::analysis;

namespace {

std::vector<BodyCompletionPose> SolveRoleSequence(BodyCompletionSolver& solver,
                                                  const std::vector<PhantomTrajectoryFrame>& frames,
                                                  const BodyCompletionPriors& priors,
                                                  const std::array<bool, kBodyRoleCount>& enabled,
                                                  const std::array<bool, kBodyRoleCount>& measured, BodyRole role,
                                                  bool left_controller = true, bool right_controller = true)
{
	std::vector<BodyCompletionPose> out;
	out.reserve(frames.size());
	for (const auto& frame : frames) {
		const auto input = MakeBodyCompletionInput(frame, priors, enabled, measured, left_controller, right_controller);
		const auto result = solver.Solve(input);
		const auto& role_out = result.roles[static_cast<size_t>(role)];
		if (role_out.valid) {
			out.push_back(role_out.pose);
		}
	}
	return out;
}

} // namespace

TEST(BodyCompletionScenarios, PartialTrackersKeepMeasuredRolesAndFillMissingChest)
{
	PhantomTrajectoryOptions options;
	options.motion = PhantomMotion::ForwardWalk;
	options.frame_count = 90;
	const auto frames = GenerateTrajectory(options);
	const auto priors = DefaultTrajectoryPriors(options);
	const auto enabled = AllVirtualTrackerRolesEnabled();
	const auto measured = RolesEnabled({BodyRole::Waist, BodyRole::LeftFoot, BodyRole::RightFoot});

	BodyCompletionSolver solver;
	uint32_t measured_feet = 0;
	uint32_t filled_chest = 0;
	for (const auto& frame : frames) {
		const auto input = MakeBodyCompletionInput(frame, priors, enabled, measured);
		const auto result = solver.Solve(input);
		const auto& left_foot = result.roles[static_cast<size_t>(BodyRole::LeftFoot)];
		const auto& chest = result.roles[static_cast<size_t>(BodyRole::Chest)];

		ASSERT_TRUE(left_foot.valid);
		ASSERT_TRUE(chest.valid);
		if (left_foot.mode == BodyCompletionMode::Measured &&
		    DistanceM(left_foot.pose.position, frame.roles[static_cast<size_t>(BodyRole::LeftFoot)].position) < 1e-9) {
			++measured_feet;
		}
		if (chest.mode == BodyCompletionMode::HmdRoot && (chest.source_mask & kBodySourcePredicted) != 0) {
			++filled_chest;
		}
	}

	EXPECT_EQ(measured_feet, frames.size());
	EXPECT_EQ(filled_chest, frames.size());
}

TEST(BodyCompletionScenarios, NoFbtControllerEvidenceIsMoreConfidentThanLegGuesswork)
{
	PhantomTrajectoryOptions options;
	options.motion = PhantomMotion::ControllerReach;
	options.frame_count = 120;
	const auto frames = GenerateTrajectory(options);
	const auto priors = DefaultTrajectoryPriors(options);
	const auto enabled = AllVirtualTrackerRolesEnabled();
	std::array<bool, kBodyRoleCount> measured{};

	BodyCompletionSolver solver;
	for (const auto& frame : frames) {
		const auto input = MakeBodyCompletionInput(frame, priors, enabled, measured);
		const auto result = solver.Solve(input);
		const auto& left_elbow = result.roles[static_cast<size_t>(BodyRole::LeftElbow)];
		const auto& left_foot = result.roles[static_cast<size_t>(BodyRole::LeftFoot)];
		ASSERT_TRUE(left_elbow.valid);
		ASSERT_TRUE(left_foot.valid);
		EXPECT_EQ(left_elbow.mode, BodyCompletionMode::ControllerIk);
		EXPECT_GT(left_elbow.confidence, left_foot.confidence);
		EXPECT_LT(result.global_confidence, 0.70f);
	}
}

TEST(BodyCompletionScenarios, HmdOnlyAllAbsentStaysBelowHighConfidence)
{
	PhantomTrajectoryOptions options;
	options.motion = PhantomMotion::BendLean;
	options.frame_count = 120;
	const auto frames = GenerateTrajectory(options);
	auto priors = BodyCompletionPriors{};
	priors.virtual_min_confidence = 0.50;
	const auto enabled = AllVirtualTrackerRolesEnabled();
	std::array<bool, kBodyRoleCount> measured{};

	BodyCompletionSolver solver;
	for (const auto& frame : frames) {
		const auto input = MakeBodyCompletionInput(frame, priors, enabled, measured, false, false);
		const auto result = solver.Solve(input);
		ASSERT_GT(result.global_confidence, 0.0f);
		EXPECT_LT(result.global_confidence, 0.65f);
		for (uint8_t i = 0; i < kBodyRoleCount; ++i) {
			const auto role = static_cast<BodyRole>(i);
			if (role == BodyRole::Hmd || role == BodyRole::None) continue;
			const auto& out = result.roles[i];
			if (!out.valid) continue;
			EXPECT_LT(out.confidence, 0.65f) << BodyRoleToKey(role);
		}
		EXPECT_FALSE(
		    OutputWouldPublish(result.roles[static_cast<size_t>(BodyRole::LeftElbow)], priors.virtual_min_confidence));
		EXPECT_FALSE(
		    OutputWouldPublish(result.roles[static_cast<size_t>(BodyRole::RightElbow)], priors.virtual_min_confidence));
	}
}

TEST(BodyCompletionScenarios, ForwardWalkFeetStepInsteadOfGliding)
{
	PhantomTrajectoryOptions options;
	options.motion = PhantomMotion::ForwardWalk;
	options.frame_count = 360; // 4 s at 90 Hz, walking at 0.8 m/s
	const auto frames = GenerateTrajectory(options);
	const auto priors = DefaultTrajectoryPriors(options);
	const auto enabled = AllVirtualTrackerRolesEnabled();
	std::array<bool, kBodyRoleCount> measured{};

	BodyCompletionSolver solver;
	const auto left_foot = SolveRoleSequence(solver, frames, priors, enabled, measured, BodyRole::LeftFoot);
	ASSERT_EQ(left_foot.size(), frames.size());

	// Planted frames must mostly hold still. Without the gait the feet glide
	// with the pelvis at walk speed, skating on essentially every planted frame.
	const auto skate = ComputeFootSkateStats(left_foot, options.floor_y_m, 0.05, 0.01);
	EXPECT_GT(skate.planted_frames, 60u);
	EXPECT_LT(skate.skating_frames, skate.planted_frames / 4);

	// Stepping still has to keep up with the body instead of being left behind.
	const double body_z = frames.back().hmd.position[2];
	EXPECT_GT(left_foot.back().position[2], body_z - 1.0);

	// No teleports: every frame-to-frame move stays within a fast swing's reach.
	const auto continuity = ComputeContinuityStats(left_foot, 0.06, 25.0);
	EXPECT_EQ(continuity.teleport_count, 0u);
}

TEST(BodyCompletionScenarios, WalkThenStopKeepsFeetWhereTheyLanded)
{
	PhantomTrajectoryOptions walk_options;
	walk_options.motion = PhantomMotion::ForwardWalk;
	walk_options.frame_count = 270;
	const auto walk_frames = GenerateTrajectory(walk_options);
	const auto priors = DefaultTrajectoryPriors(walk_options);
	const auto enabled = AllVirtualTrackerRolesEnabled();
	std::array<bool, kBodyRoleCount> measured{};

	BodyCompletionSolver solver;
	std::vector<BodyCompletionPose> left_foot;
	for (const auto& frame : walk_frames) {
		const auto input = MakeBodyCompletionInput(frame, priors, enabled, measured);
		const auto result = solver.Solve(input);
		left_foot.push_back(result.roles[static_cast<size_t>(BodyRole::LeftFoot)].pose);
	}

	// Freeze the body at the last walking pose for two seconds of standing.
	PhantomTrajectoryFrame still = walk_frames.back();
	still.hmd.velocity[0] = still.hmd.velocity[1] = still.hmd.velocity[2] = 0.0;
	for (int i = 0; i < 180; ++i) {
		const auto input = MakeBodyCompletionInput(still, priors, enabled, measured);
		const auto result = solver.Solve(input);
		left_foot.push_back(result.roles[static_cast<size_t>(BodyRole::LeftFoot)].pose);
	}

	// After stopping, the foot settles and stops moving entirely: no teleports
	// anywhere, and the final second of standing shows no planar creep.
	const auto continuity = ComputeContinuityStats(left_foot, 0.08, 25.0);
	EXPECT_EQ(continuity.teleport_count, 0u);
	const auto tail = std::vector<BodyCompletionPose>(left_foot.end() - 90, left_foot.end());
	const auto tail_skate = ComputeFootSkateStats(tail, walk_options.floor_y_m, 0.05, 0.005);
	EXPECT_EQ(tail_skate.skating_frames, 0u);
	EXPECT_LT(tail_skate.total_slide_m, 0.01);
}

TEST(BodyCompletionScenarios, StandingNoFbtFeetDoNotSkateWhilePlanted)
{
	PhantomTrajectoryOptions options;
	options.motion = PhantomMotion::IdleStand;
	options.frame_count = 90;
	const auto frames = GenerateTrajectory(options);
	const auto priors = DefaultTrajectoryPriors(options);
	const auto enabled = AllVirtualTrackerRolesEnabled();
	std::array<bool, kBodyRoleCount> measured{};

	BodyCompletionSolver solver;
	const auto left_foot = SolveRoleSequence(solver, frames, priors, enabled, measured, BodyRole::LeftFoot);
	const auto stats = ComputeFootSkateStats(left_foot, options.floor_y_m, 0.05, 0.01);

	EXPECT_GT(stats.planted_frames, 0u);
	EXPECT_EQ(stats.skating_frames, 0u);
	EXPECT_LT(stats.total_slide_m, 0.02);
}
