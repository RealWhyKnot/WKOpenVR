#include <gtest/gtest.h>

#include "DeadReckoner.h"
#include "DropoutState.h"
#include "PhantomMetrics.h"
#include "PhantomTrajectory.h"
#include "PoseHistory.h"

#include <openvr_driver.h>

#include <cmath>

using namespace phantom;
using namespace phantom::analysis;

namespace {

constexpr int64_t kQpcFreq = 10000000;

int64_t Ms(int64_t ms)
{
	return ms * (kQpcFreq / 1000);
}

vr::DriverPose_t DriverPoseFromBodyPose(const BodyCompletionPose& pose)
{
	vr::DriverPose_t out{};
	out.qWorldFromDriverRotation = {1, 0, 0, 0};
	out.qDriverFromHeadRotation = {1, 0, 0, 0};
	out.qRotation = {pose.rotation[0], pose.rotation[1], pose.rotation[2], pose.rotation[3]};
	for (int i = 0; i < 3; ++i) {
		out.vecPosition[i] = pose.position[i];
		out.vecVelocity[i] = pose.velocity[i];
	}
	out.poseIsValid = true;
	out.deviceIsConnected = true;
	out.result = vr::TrackingResult_Running_OK;
	return out;
}

vr::DriverPose_t RealPoseAt(double x)
{
	auto pose = DriverPoseFromBodyPose(Pose(x, 1.0, 0.0));
	pose.vecVelocity[0] = 1.0;
	return pose;
}

} // namespace

TEST(DropoutScenarios, IntermittentReconnectStartsDistinctDropoutCycles)
{
	PoseHistory history;
	DropoutState state;
	auto timings = LadderTimings::Defaults();
	state.SetTimings(timings);

	auto first = RealPoseAt(0.0);
	history.Push(Ms(0), first);
	state.OnRealPoseObserved(Ms(0), history, first);

	state.Tick(Ms(timings.dropout_silence_ms + timings.blend_out_ms + 5), kQpcFreq);
	ASSERT_EQ(state.state(), TrackerState::SYNTH_RECKON);
	ASSERT_EQ(state.dropout_count(), 1u);

	auto recovered = RealPoseAt(0.2);
	history.Push(Ms(220), recovered);
	state.OnRealPoseObserved(Ms(220), history, recovered);
	EXPECT_EQ(state.state(), TrackerState::BLEND_IN);
	state.Tick(Ms(220 + timings.blend_in_ms + 5), kQpcFreq);
	ASSERT_EQ(state.state(), TrackerState::REAL);

	state.Tick(Ms(220 + timings.blend_in_ms + timings.dropout_silence_ms + timings.blend_out_ms + 20), kQpcFreq);
	EXPECT_EQ(state.state(), TrackerState::SYNTH_RECKON);
	EXPECT_EQ(state.dropout_count(), 2u);
}

TEST(DropoutScenarios, LongSilenceStopsPublishingAndReportsOutOfRangeFirst)
{
	PoseHistory history;
	DropoutState state;
	auto timings = LadderTimings::Defaults();
	state.SetTimings(timings);
	const auto pose = RealPoseAt(0.0);
	history.Push(Ms(0), pose);
	state.OnRealPoseObserved(Ms(0), history, pose);

	state.Tick(Ms(timings.synth_hold_ms + 1), kQpcFreq);
	EXPECT_EQ(state.state(), TrackerState::OUT_OF_RANGE);
	EXPECT_EQ(state.tracking_result_override(), vr::TrackingResult_Running_OutOfRange);
	EXPECT_TRUE(state.should_publish());

	state.Tick(Ms(timings.lost_hold_ms + 1), kQpcFreq);
	EXPECT_EQ(state.state(), TrackerState::LOST);
	EXPECT_FALSE(state.should_publish());
}

TEST(DropoutScenarios, DeadReckonerTracksShortForwardWalkDropout)
{
	PhantomTrajectoryOptions options;
	options.motion = PhantomMotion::ForwardWalk;
	options.frame_count = 30;
	const auto frames = GenerateTrajectory(options);
	constexpr uint32_t source_frame = 10;
	constexpr uint32_t target_frame = 15;

	PoseHistory history;
	const auto source_pose = DriverPoseFromBodyPose(frames[source_frame].roles[static_cast<size_t>(BodyRole::Waist)]);
	const int64_t source_qpc =
	    static_cast<int64_t>((frames[source_frame].time_ms / 1000.0) * static_cast<double>(kQpcFreq));
	const int64_t target_qpc =
	    static_cast<int64_t>((frames[target_frame].time_ms / 1000.0) * static_cast<double>(kQpcFreq));
	history.Push(source_qpc, source_pose);

	DeadReckoner reckoner;
	vr::DriverPose_t projected{};
	ASSERT_TRUE(reckoner.Project(history, kQpcFreq, target_qpc, projected));

	const auto& truth = frames[target_frame].roles[static_cast<size_t>(BodyRole::Waist)];
	EXPECT_LT(DistanceM(projected.vecPosition, truth.position), 0.10);
	EXPECT_EQ(projected.result, vr::TrackingResult_Running_OK);
}

TEST(DropoutScenarios, DeadReckonerRejectsInvalidSyntheticInputs)
{
	PoseHistory history;
	auto invalid = RealPoseAt(0.0);
	invalid.vecPosition[0] = std::numeric_limits<double>::quiet_NaN();
	history.Push(Ms(0), invalid);

	DeadReckoner reckoner;
	vr::DriverPose_t out{};
	EXPECT_FALSE(reckoner.Project(history, kQpcFreq, Ms(50), out));
}
