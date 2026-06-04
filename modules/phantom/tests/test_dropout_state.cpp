#include <gtest/gtest.h>

#include "DropoutState.h"
#include "PhantomTypes.h"
#include "PoseHistory.h"

#include <openvr_driver.h>

namespace {

constexpr int64_t kQpcFreq = 10000000; // 10 MHz, so 1 ms = 10000 ticks.

vr::DriverPose_t MakeRealPose()
{
	vr::DriverPose_t p{};
	p.qWorldFromDriverRotation = {1, 0, 0, 0};
	p.qDriverFromHeadRotation = {1, 0, 0, 0};
	p.qRotation = {1, 0, 0, 0};
	p.poseIsValid = true;
	p.deviceIsConnected = true;
	p.result = vr::TrackingResult_Running_OK;
	return p;
}

int64_t Ms(int64_t ms)
{
	return ms * (kQpcFreq / 1000);
}

} // namespace

TEST(DropoutStateTest, StartsInRealAndStaysWhilePosesArrive)
{
	phantom::PoseHistory hist;
	phantom::DropoutState s;
	s.SetTimings(phantom::LadderTimings::Defaults());

	const auto pose = MakeRealPose();
	s.OnRealPoseObserved(Ms(0), hist, pose);
	s.Tick(Ms(0), kQpcFreq);
	EXPECT_EQ(s.state(), phantom::TrackerState::REAL);

	s.OnRealPoseObserved(Ms(11), hist, pose);
	s.Tick(Ms(11), kQpcFreq);
	EXPECT_EQ(s.state(), phantom::TrackerState::REAL);
}

TEST(DropoutStateTest, EnterBlendOutAfterSilenceThreshold)
{
	phantom::PoseHistory hist;
	phantom::DropoutState s;
	auto t = phantom::LadderTimings::Defaults();
	s.SetTimings(t);
	const auto pose = MakeRealPose();
	s.OnRealPoseObserved(Ms(0), hist, pose);

	s.Tick(Ms(t.dropout_silence_ms - 1), kQpcFreq);
	EXPECT_EQ(s.state(), phantom::TrackerState::REAL);

	s.Tick(Ms(t.dropout_silence_ms + 1), kQpcFreq);
	EXPECT_EQ(s.state(), phantom::TrackerState::BLEND_OUT);
	EXPECT_EQ(s.dropout_count(), 1u);
}

TEST(DropoutStateTest, EscalatesThroughLadderOnContinuedSilence)
{
	phantom::PoseHistory hist;
	phantom::DropoutState s;
	auto t = phantom::LadderTimings::Defaults();
	s.SetTimings(t);
	s.OnRealPoseObserved(Ms(0), hist, MakeRealPose());

	s.Tick(Ms(t.dropout_silence_ms + 1), kQpcFreq);
	ASSERT_EQ(s.state(), phantom::TrackerState::BLEND_OUT);

	s.Tick(Ms(t.dropout_silence_ms + t.blend_out_ms + 1), kQpcFreq);
	EXPECT_EQ(s.state(), phantom::TrackerState::SYNTH_RECKON);

	s.Tick(Ms(t.dropout_silence_ms + t.synth_hold_ms + 1), kQpcFreq);
	EXPECT_EQ(s.state(), phantom::TrackerState::OUT_OF_RANGE);
	EXPECT_EQ(s.tracking_result_override(), vr::TrackingResult_Running_OutOfRange);

	s.Tick(Ms(t.dropout_silence_ms + t.lost_hold_ms + 1), kQpcFreq);
	EXPECT_EQ(s.state(), phantom::TrackerState::LOST);
	EXPECT_FALSE(s.should_publish());
}

TEST(DropoutStateTest, BlendInOnRecovery)
{
	phantom::PoseHistory hist;
	phantom::DropoutState s;
	auto t = phantom::LadderTimings::Defaults();
	s.SetTimings(t);
	s.OnRealPoseObserved(Ms(0), hist, MakeRealPose());
	s.Tick(Ms(t.dropout_silence_ms + t.blend_out_ms + 5), kQpcFreq);
	ASSERT_EQ(s.state(), phantom::TrackerState::SYNTH_RECKON);

	s.OnRealPoseObserved(Ms(t.dropout_silence_ms + t.blend_out_ms + 6), hist, MakeRealPose());
	EXPECT_EQ(s.state(), phantom::TrackerState::BLEND_IN);

	s.Tick(Ms(t.dropout_silence_ms + t.blend_out_ms + t.blend_in_ms + 100), kQpcFreq);
	EXPECT_EQ(s.state(), phantom::TrackerState::REAL);
}
