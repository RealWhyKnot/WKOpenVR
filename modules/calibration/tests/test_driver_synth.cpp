// Unit tests for the driver-level HMD synthesis pure helpers in
// DriverSynthCompose.h. The helper is intentionally strict: when DriverSynth
// is active, the synthesized HMD pose is driven by the selected tracker
// snapshot and does not depend on the upstream headset pose.

#include "DriverSynthCompose.h"

#include <gtest/gtest.h>
#include <chrono>

using namespace driver_synth;
using clk = std::chrono::steady_clock;

namespace {

vr::DriverPose_t MakePose(double x, double y, double z, bool poseIsValid = true)
{
	vr::DriverPose_t p{};
	p.poseIsValid = poseIsValid;
	p.result = vr::TrackingResult_Running_OK;
	p.vecPosition[0] = x;
	p.vecPosition[1] = y;
	p.vecPosition[2] = z;
	p.qRotation.w = 1.0;
	p.qWorldFromDriverRotation.w = 1.0;
	p.qDriverFromHeadRotation.w = 1.0;
	p.deviceIsConnected = true;
	return p;
}

SynthState MakeState(int mode = 3, int32_t deviceId = 2, bool offsetCalibrated = true, double tx = 0.0, double ty = 0.0,
                     double tz = 0.0)
{
	SynthState s{};
	s.mode = mode;
	s.deviceId = deviceId;
	s.offsetCalibrated = offsetCalibrated;
	s.headFromTrackerTrans[0] = tx;
	s.headFromTrackerTrans[1] = ty;
	s.headFromTrackerTrans[2] = tz;
	s.headFromTrackerRot[0] = 0.0;
	s.headFromTrackerRot[1] = 0.0;
	s.headFromTrackerRot[2] = 0.0;
	s.headFromTrackerRot[3] = 1.0;
	return s;
}

TrackerSnapshot FreshSnap(vr::DriverPose_t pose, clk::time_point now, int32_t deviceId = 2)
{
	TrackerSnapshot snap{};
	snap.pose = pose;
	snap.capturedAt = now;
	snap.capturedForDeviceId = deviceId;
	snap.valid = true;
	return snap;
}

TrackerSnapshot StaleSnap(vr::DriverPose_t pose, clk::time_point now, int32_t deviceId = 2)
{
	using ms = std::chrono::milliseconds;
	TrackerSnapshot snap{};
	snap.pose = pose;
	snap.capturedAt = now - ms(kStaleLimitMs + 5);
	snap.capturedForDeviceId = deviceId;
	snap.valid = true;
	return snap;
}

} // namespace

TEST(DriverSynth, Valid_tracker_offsetCalibrated)
{
	const auto now = clk::now();
	const SynthState state = MakeState();

	vr::DriverPose_t trackerPose = MakePose(0.1, 1.7, 0.2);
	trackerPose.vecVelocity[0] = 0.5;
	trackerPose.vecWorldFromDriverTranslation[0] = 5.0;
	trackerPose.vecWorldFromDriverTranslation[1] = 1.0;
	trackerPose.vecWorldFromDriverTranslation[2] = 2.0;

	const TrackerSnapshot trackerSnap = FreshSnap(trackerPose, now);

	vr::DriverPose_t out{};
	EXPECT_TRUE(Compose(state, trackerSnap, now, out));

	EXPECT_NEAR(out.vecPosition[0], 0.1, 1e-9);
	EXPECT_NEAR(out.vecPosition[1], 1.7, 1e-9);
	EXPECT_NEAR(out.vecPosition[2], 0.2, 1e-9);
	EXPECT_NEAR(out.vecVelocity[0], 0.5, 1e-9);

	EXPECT_NEAR(out.vecWorldFromDriverTranslation[0], 5.0, 1e-9);
	EXPECT_NEAR(out.vecWorldFromDriverTranslation[1], 1.0, 1e-9);
	EXPECT_NEAR(out.vecWorldFromDriverTranslation[2], 2.0, 1e-9);

	EXPECT_NEAR(out.vecDriverFromHeadTranslation[0], 0.0, 1e-9);
	EXPECT_NEAR(out.vecDriverFromHeadTranslation[1], 0.0, 1e-9);
	EXPECT_NEAR(out.vecDriverFromHeadTranslation[2], 0.0, 1e-9);
	EXPECT_NEAR(out.qDriverFromHeadRotation.w, 1.0, 1e-9);
	EXPECT_NEAR(out.qDriverFromHeadRotation.x, 0.0, 1e-9);
	EXPECT_NEAR(out.qDriverFromHeadRotation.y, 0.0, 1e-9);
	EXPECT_NEAR(out.qDriverFromHeadRotation.z, 0.0, 1e-9);
}

TEST(DriverSynth, Tracker_invalid)
{
	const auto now = clk::now();
	const SynthState state = MakeState();
	const TrackerSnapshot trackerSnap = FreshSnap(MakePose(0.0, 1.7, 0.0, false), now);

	vr::DriverPose_t out{};
	EXPECT_FALSE(Compose(state, trackerSnap, now, out));
}

TEST(DriverSynth, Tracker_stale)
{
	const auto now = clk::now();
	const SynthState state = MakeState();
	const TrackerSnapshot staleTracker = StaleSnap(MakePose(0.0, 1.7, 0.0), now);

	vr::DriverPose_t out{};
	EXPECT_FALSE(Compose(state, staleTracker, now, out));
}

TEST(DriverSynth, CustomStaleLimitAcceptsLongerTrackerGap)
{
	using ms = std::chrono::milliseconds;
	const auto now = clk::now();
	const SynthState state = MakeState();
	TrackerSnapshot trackerSnap = FreshSnap(MakePose(0.0, 1.7, 0.0), now);
	trackerSnap.capturedAt = now - ms(kStaleLimitMs + 20);

	SourceBlendConfig cfg{};
	cfg.staleLimitMs = static_cast<int>(kStaleLimitMs + 50);

	vr::DriverPose_t out{};
	EXPECT_TRUE(Compose(state, trackerSnap, now, out, cfg));
}

TEST(DriverSynth, Upstream_hmd_staleness_does_not_block_tracker_synth)
{
	const auto now = clk::now();
	const SynthState state = MakeState();
	const TrackerSnapshot freshTracker = FreshSnap(MakePose(0.0, 1.7, 0.0), now);

	vr::DriverPose_t out{};
	EXPECT_TRUE(Compose(state, freshTracker, now, out));
}

TEST(DriverSynth, Tracker_position_is_not_quest_sanity_gated)
{
	const auto now = clk::now();
	const SynthState state = MakeState();
	const TrackerSnapshot trackerSnap = FreshSnap(MakePose(2.0, 1.7, 0.0), now);

	vr::DriverPose_t out{};
	EXPECT_TRUE(Compose(state, trackerSnap, now, out));
	EXPECT_NEAR(out.vecPosition[0], 2.0, 1e-9);
}

TEST(DriverSynth, Mode_off)
{
	const auto now = clk::now();
	const SynthState state = MakeState(0);
	const TrackerSnapshot trackerSnap = FreshSnap(MakePose(0.0, 1.7, 0.0), now);

	vr::DriverPose_t out{};
	EXPECT_FALSE(Compose(state, trackerSnap, now, out));
}

TEST(DriverSynth, Mode_autopaired)
{
	const auto now = clk::now();
	const SynthState state = MakeState(1);
	const TrackerSnapshot trackerSnap = FreshSnap(MakePose(0.0, 1.7, 0.0), now);

	vr::DriverPose_t out{};
	EXPECT_FALSE(Compose(state, trackerSnap, now, out));
}

TEST(DriverSynth, DeviceId_unresolved)
{
	const auto now = clk::now();
	const SynthState state = MakeState(3, -1);
	const TrackerSnapshot trackerSnap = FreshSnap(MakePose(0.0, 1.7, 0.0), now);

	vr::DriverPose_t out{};
	EXPECT_FALSE(Compose(state, trackerSnap, now, out));
}

TEST(DriverSynth, OffsetCalibrated_false)
{
	const auto now = clk::now();
	const SynthState state = MakeState(3, 2, false);
	const TrackerSnapshot trackerSnap = FreshSnap(MakePose(0.0, 1.7, 0.0), now);

	vr::DriverPose_t out{};
	EXPECT_FALSE(Compose(state, trackerSnap, now, out));
}

TEST(DriverSynth, HeadFromTracker_offset_stored)
{
	const auto now = clk::now();
	const SynthState state = MakeState(3, 2, true, 0.01, -0.05, 0.02);
	const TrackerSnapshot trackerSnap = FreshSnap(MakePose(0.0, 1.7, 0.0), now);

	vr::DriverPose_t out{};
	ASSERT_TRUE(Compose(state, trackerSnap, now, out));

	EXPECT_NEAR(out.vecDriverFromHeadTranslation[0], 0.01, 1e-9);
	EXPECT_NEAR(out.vecDriverFromHeadTranslation[1], -0.05, 1e-9);
	EXPECT_NEAR(out.vecDriverFromHeadTranslation[2], 0.02, 1e-9);
}

TEST(DriverSynth, Tracker_worldFromDriver_is_preserved)
{
	const auto now = clk::now();
	const SynthState state = MakeState();

	vr::DriverPose_t tracker = MakePose(1.0, 0.0, 0.0);
	tracker.qWorldFromDriverRotation.w = 0.0;
	tracker.qWorldFromDriverRotation.x = 0.0;
	tracker.qWorldFromDriverRotation.y = 1.0;
	tracker.qWorldFromDriverRotation.z = 0.0;
	tracker.vecWorldFromDriverTranslation[0] = -1.0;
	tracker.vecWorldFromDriverTranslation[1] = 0.5;
	tracker.vecWorldFromDriverTranslation[2] = 3.0;

	const TrackerSnapshot trackerSnap = FreshSnap(tracker, now);

	vr::DriverPose_t out{};
	ASSERT_TRUE(Compose(state, trackerSnap, now, out));

	EXPECT_NEAR(out.qWorldFromDriverRotation.w, 0.0, 1e-9);
	EXPECT_NEAR(out.qWorldFromDriverRotation.y, 1.0, 1e-9);
	EXPECT_NEAR(out.vecWorldFromDriverTranslation[0], -1.0, 1e-9);
	EXPECT_NEAR(out.vecWorldFromDriverTranslation[1], 0.5, 1e-9);
	EXPECT_NEAR(out.vecWorldFromDriverTranslation[2], 3.0, 1e-9);
}

TEST(DriverSynth, Tracker_snap_for_different_deviceId_rejected)
{
	const auto now = clk::now();
	const SynthState state = MakeState(3, 2);

	const TrackerSnapshot trackerSnap = FreshSnap(MakePose(0.0, 1.7, 0.0), now, 7);

	vr::DriverPose_t out{};
	EXPECT_FALSE(Compose(state, trackerSnap, now, out));
}

TEST(DriverSynthBlend, BriefStaleTrackerHoldsLastSynthPose)
{
	using ms = std::chrono::milliseconds;
	const auto t0 = clk::now();
	SourceBlendState blend{};
	vr::DriverPose_t out{};

	const vr::DriverPose_t fallback = MakePose(10.0, 1.7, 0.0);
	const vr::DriverPose_t synth = MakePose(0.0, 1.7, 0.0);

	auto r = StepSourceBlend(blend, fallback, &synth, true, t0, out);
	EXPECT_EQ(r.phase, SourceBlendPhase::SynthStable);
	EXPECT_NEAR(out.vecPosition[0], 0.0, 1e-9);

	r = StepSourceBlend(blend, fallback, nullptr, false, t0 + ms(50), out);
	EXPECT_EQ(r.phase, SourceBlendPhase::GraceHold);
	EXPECT_NEAR(out.vecPosition[0], 0.0, 1e-9);
}

TEST(DriverSynthBlend, CustomGraceHoldMasksLongerTrackerDropout)
{
	using ms = std::chrono::milliseconds;
	const auto t0 = clk::now();
	SourceBlendState blend{};
	SourceBlendConfig cfg{};
	cfg.graceHoldMs = static_cast<int>(kGraceHoldMs + 500);
	vr::DriverPose_t out{};

	const vr::DriverPose_t fallback = MakePose(10.0, 1.7, 0.0);
	const vr::DriverPose_t synth = MakePose(0.0, 1.7, 0.0);

	StepSourceBlend(blend, fallback, &synth, true, t0, out, cfg);
	auto r = StepSourceBlend(blend, fallback, nullptr, false, t0 + ms(kGraceHoldMs + 250), out, cfg);

	EXPECT_EQ(r.phase, SourceBlendPhase::GraceHold);
	EXPECT_NEAR(out.vecPosition[0], 0.0, 1e-9);
}

TEST(DriverSynthBlend, PersistentTrackerLossBlendsTowardFallback)
{
	using ms = std::chrono::milliseconds;
	const auto t0 = clk::now();
	SourceBlendState blend{};
	vr::DriverPose_t out{};

	const vr::DriverPose_t fallback = MakePose(10.0, 1.7, 0.0);
	const vr::DriverPose_t synth = MakePose(0.0, 1.7, 0.0);

	StepSourceBlend(blend, fallback, &synth, true, t0, out);
	StepSourceBlend(blend, fallback, nullptr, false, t0 + ms(1), out);
	StepSourceBlend(blend, fallback, nullptr, false, t0 + ms(kGraceHoldMs + 1), out);
	auto r = StepSourceBlend(blend, fallback, nullptr, false, t0 + ms(kGraceHoldMs + 1 + kBlendToFallbackMs / 2), out);

	EXPECT_EQ(r.phase, SourceBlendPhase::BlendingToFallback);
	EXPECT_GT(out.vecPosition[0], 0.0);
	EXPECT_LT(out.vecPosition[0], 10.0);
}

TEST(DriverSynthBlend, RecoveredTrackerWaitsThenBlendsBackToSynth)
{
	using ms = std::chrono::milliseconds;
	const auto t0 = clk::now();
	SourceBlendState blend{};
	vr::DriverPose_t out{};

	const vr::DriverPose_t fallback = MakePose(10.0, 1.7, 0.0);
	const vr::DriverPose_t synthA = MakePose(0.0, 1.7, 0.0);
	const vr::DriverPose_t synthB = MakePose(2.0, 1.7, 0.0);

	StepSourceBlend(blend, fallback, &synthA, true, t0, out);
	StepSourceBlend(blend, fallback, nullptr, false, t0 + ms(1), out);
	StepSourceBlend(blend, fallback, nullptr, false, t0 + ms(kGraceHoldMs + 1), out);
	StepSourceBlend(blend, fallback, nullptr, false, t0 + ms(kGraceHoldMs + 1 + kBlendToFallbackMs + 1), out);
	EXPECT_EQ(blend.phase, SourceBlendPhase::FallbackStable);
	EXPECT_NEAR(out.vecPosition[0], 10.0, 1e-9);

	const auto recovered = t0 + ms(kGraceHoldMs + kBlendToFallbackMs + 20);
	auto r = StepSourceBlend(blend, fallback, &synthB, true, recovered, out);
	EXPECT_EQ(r.phase, SourceBlendPhase::WaitingForStableSynth);
	EXPECT_NEAR(out.vecPosition[0], 10.0, 1e-9);

	StepSourceBlend(blend, fallback, &synthB, true, recovered + ms(kStableBeforeSynthMs + 1), out);
	r = StepSourceBlend(blend, fallback, &synthB, true, recovered + ms(kStableBeforeSynthMs + 1 + kBlendToSynthMs / 2),
	                    out);
	EXPECT_EQ(r.phase, SourceBlendPhase::BlendingToSynth);
	EXPECT_GT(out.vecPosition[0], 2.0);
	EXPECT_LT(out.vecPosition[0], 10.0);
}

TEST(DriverSynthBlend, CustomStableBeforeSynthDelaysReturn)
{
	using ms = std::chrono::milliseconds;
	const auto t0 = clk::now();
	SourceBlendState blend{};
	SourceBlendConfig cfg{};
	cfg.stableBeforeSynthMs = static_cast<int>(kStableBeforeSynthMs + 500);
	vr::DriverPose_t out{};

	const vr::DriverPose_t fallback = MakePose(10.0, 1.7, 0.0);
	const vr::DriverPose_t synthA = MakePose(0.0, 1.7, 0.0);
	const vr::DriverPose_t synthB = MakePose(2.0, 1.7, 0.0);

	StepSourceBlend(blend, fallback, &synthA, true, t0, out, cfg);
	StepSourceBlend(blend, fallback, nullptr, false, t0 + ms(1), out, cfg);
	StepSourceBlend(blend, fallback, nullptr, false, t0 + ms(kGraceHoldMs + 2), out, cfg);
	StepSourceBlend(blend, fallback, nullptr, false, t0 + ms(kGraceHoldMs + kBlendToFallbackMs + 4), out, cfg);
	ASSERT_EQ(blend.phase, SourceBlendPhase::FallbackStable);

	const auto recovered = t0 + ms(kGraceHoldMs + kBlendToFallbackMs + 20);
	StepSourceBlend(blend, fallback, &synthB, true, recovered, out, cfg);
	auto r = StepSourceBlend(blend, fallback, &synthB, true, recovered + ms(kStableBeforeSynthMs + 250), out, cfg);

	EXPECT_EQ(r.phase, SourceBlendPhase::WaitingForStableSynth);
	EXPECT_NEAR(out.vecPosition[0], 10.0, 1e-9);
}

TEST(DriverSynthBlend, BlendPoseNormalizesRotations)
{
	auto a = MakePose(0.0, 1.7, 0.0);
	auto b = MakePose(1.0, 1.7, 0.0);
	b.qRotation = {0.0, 0.0, 1.0, 0.0};

	vr::DriverPose_t out{};
	BlendPose(a, b, 0.5, out);

	const double n = out.qRotation.w * out.qRotation.w + out.qRotation.x * out.qRotation.x +
	                 out.qRotation.y * out.qRotation.y + out.qRotation.z * out.qRotation.z;
	EXPECT_NEAR(n, 1.0, 1e-6);
}
