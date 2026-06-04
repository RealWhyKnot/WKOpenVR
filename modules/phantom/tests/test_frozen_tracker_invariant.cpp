#include <gtest/gtest.h>

#include "DropoutState.h"
#include "PhantomTypes.h"
#include "PoseHistory.h"

#include <openvr_driver.h>

// The load-bearing invariant for the entire phantom module: after T seconds
// of silence on any tracker, the published ETrackingResult MUST NOT be
// Running_OK. The whole reason this module exists is to prevent the
// "frozen pose, still Running_OK, avatar limb stuck mid-air" wedge.
// This test gates every release.
//
// "T seconds" here is min(synth_hold_ms, lost_hold_ms). Past synth_hold the
// override flips to OutOfRange; past lost_hold the publish is suppressed
// outright. Either way the result is not Running_OK.

namespace {

constexpr int64_t kQpcFreq = 10000000;
int64_t Ms(int64_t ms)
{
	return ms * (kQpcFreq / 1000);
}

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

} // namespace

TEST(FrozenTrackerInvariant, ResultIsNotOkPastSynthHold)
{
	phantom::PoseHistory hist;
	phantom::DropoutState s;
	auto t = phantom::LadderTimings::Defaults();
	s.SetTimings(t);

	// One real pose, then complete silence.
	s.OnRealPoseObserved(0, hist, MakeRealPose());

	// Just past synth_hold_ms: result override must be OutOfRange.
	s.Tick(Ms((int64_t)t.synth_hold_ms + 50), kQpcFreq);
	EXPECT_NE(s.tracking_result_override(), vr::TrackingResult_Running_OK)
	    << "After synth_hold_ms of silence, the published ETrackingResult "
	       "must not still be Running_OK -- that would surface the "
	       "frozen-pose wedge to downstream consumers.";

	// Just past lost_hold_ms: should_publish must be false (stop publishing).
	s.Tick(Ms((int64_t)t.lost_hold_ms + 50), kQpcFreq);
	EXPECT_FALSE(s.should_publish()) << "After lost_hold_ms of silence, the module must stop publishing "
	                                    "for this device so SteamVR treats it as disconnected.";
}

TEST(FrozenTrackerInvariant, HoldsAcrossArbitraryTimingConfigurations)
{
	const phantom::LadderTimings variations[] = {
	    phantom::LadderTimings::Defaults(),
	    // Aggressive: very short synth_hold, modest lost_hold.
	    {40, 80, 150, 250, 500, 1500},
	    // Conservative: long synth_hold, long lost_hold.
	    {40, 80, 150, 250, 5000, 15000},
	    // Tiny windows for stress.
	    {40, 80, 150, 250, 100, 200},
	};
	for (const auto& t : variations) {
		phantom::PoseHistory hist;
		phantom::DropoutState s;
		s.SetTimings(t);
		s.OnRealPoseObserved(0, hist, MakeRealPose());
		s.Tick(Ms((int64_t)t.synth_hold_ms + 50), kQpcFreq);
		EXPECT_NE(s.tracking_result_override(), vr::TrackingResult_Running_OK);
		s.Tick(Ms((int64_t)t.lost_hold_ms + 50), kQpcFreq);
		EXPECT_FALSE(s.should_publish());
	}
}
