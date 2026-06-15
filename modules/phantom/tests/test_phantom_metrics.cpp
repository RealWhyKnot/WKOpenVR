#include <gtest/gtest.h>

#include "PhantomMetrics.h"

#include <cmath>
#include <vector>

using namespace phantom;
using namespace phantom::analysis;

namespace {

BodyCompletionPose PoseAt(double x, double y, double z)
{
	BodyCompletionPose pose;
	pose.position[0] = x;
	pose.position[1] = y;
	pose.position[2] = z;
	pose.rotation[0] = 1.0;
	return pose;
}

} // namespace

TEST(PhantomMetrics, PoseErrorSummarizesPositionAndOrientation)
{
	std::vector<BodyCompletionPose> truth = {PoseAt(0.0, 0.0, 0.0), PoseAt(1.0, 0.0, 0.0)};
	std::vector<BodyCompletionPose> estimated = {PoseAt(0.0, 0.0, 0.0), PoseAt(1.3, 0.4, 0.0)};

	const auto stats = ComputePoseErrorStats(estimated, truth);
	EXPECT_EQ(stats.position_m.count, 2u);
	EXPECT_NEAR(stats.position_m.rms, std::sqrt((0.0 + 0.25) / 2.0), 1e-9);
	EXPECT_NEAR(stats.position_m.max, 0.5, 1e-9);
	EXPECT_NEAR(stats.orientation_deg.max, 0.0, 1e-9);
}

TEST(PhantomMetrics, QuaternionErrorUsesShortestArc)
{
	const double identity[4] = {1.0, 0.0, 0.0, 0.0};
	const double neg_identity[4] = {-1.0, 0.0, 0.0, 0.0};
	const double yaw180[4] = {0.0, 0.0, 1.0, 0.0};

	EXPECT_NEAR(QuaternionAngularErrorDeg(identity, neg_identity), 0.0, 1e-9);
	EXPECT_NEAR(QuaternionAngularErrorDeg(identity, yaw180), 180.0, 1e-9);
}

TEST(PhantomMetrics, ContinuityFlagsTeleportsAndInvalidSamples)
{
	std::vector<BodyCompletionPose> samples = {PoseAt(0.0, 0.0, 0.0), PoseAt(0.02, 0.0, 0.0), PoseAt(2.0, 0.0, 0.0),
	                                           PoseAt(2.01, 0.0, 0.0)};
	samples[3].rotation[0] = std::numeric_limits<double>::quiet_NaN();

	const auto stats = ComputeContinuityStats(samples, 0.5, 45.0);
	EXPECT_EQ(stats.sample_count, 4u);
	EXPECT_EQ(stats.invalid_count, 1u);
	EXPECT_EQ(stats.teleport_count, 1u);
	EXPECT_GT(stats.max_step_m, 1.0);
}

TEST(PhantomMetrics, FootSkateCountsOnlyPlantedHorizontalSlide)
{
	std::vector<BodyCompletionPose> foot = {PoseAt(0.0, 0.04, 0.0), PoseAt(0.01, 0.04, 0.0), PoseAt(0.20, 0.04, 0.0),
	                                        PoseAt(0.30, 0.20, 0.0)};

	const auto stats = ComputeFootSkateStats(foot, 0.0, 0.05, 0.05);
	EXPECT_EQ(stats.planted_frames, 2u);
	EXPECT_EQ(stats.skating_frames, 1u);
	EXPECT_NEAR(stats.total_slide_m, 0.20, 1e-9);
	EXPECT_NEAR(stats.max_frame_slide_m, 0.19, 1e-9);
}

TEST(PhantomMetrics, ConfidenceCalibrationCountsMonotonicViolations)
{
	const std::vector<float> confidence = {0.05f, 0.25f, 0.45f, 0.65f, 0.85f};
	const std::vector<double> error = {0.50, 0.40, 0.30, 0.35, 0.20};

	const auto stats = ComputeConfidenceCalibration(confidence, error);
	EXPECT_EQ(stats.counts[0], 1u);
	EXPECT_EQ(stats.counts[4], 1u);
	EXPECT_EQ(stats.monotonic_violations, 1u);
}

TEST(PhantomMetrics, VirtualPublishGateUsesConfidenceThreshold)
{
	BodyCompletionRoleOutput out;
	out.valid = true;
	out.confidence = 0.19f;
	EXPECT_FALSE(OutputWouldPublish(out, 0.20));
	out.confidence = 0.20f;
	EXPECT_TRUE(OutputWouldPublish(out, 0.20));
}
