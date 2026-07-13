#include "DynamicResolutionLogic.h"

#include <gtest/gtest.h>

namespace dynamicres = wkopenvr::dynamicres;

namespace {

// Fluent per-tick fixture: per-frame distributions plus harm counts, budget fixed at 10 ms,
// 90 frames per tick unless overridden.
struct TickSpec
{
	dynamicres::DynamicResolutionTiming t;

	TickSpec& GpuHarm(int n)
	{
		t.framesWithGpuReproj = n;
		return *this;
	}
	TickSpec& CpuHarm(int n)
	{
		t.framesWithCpuReproj = n;
		return *this;
	}
	TickSpec& Motion(int n)
	{
		t.framesWithMotion = n;
		return *this;
	}
	TickSpec& Throttled(int n)
	{
		t.framesThrottled = n;
		return *this;
	}
	TickSpec& Drops(int n)
	{
		t.framesWithDrops = n;
		return *this;
	}
	TickSpec& Mispresent(int n)
	{
		t.framesMispresented = n;
		return *this;
	}
	TickSpec& MultiPresent(int n)
	{
		t.framesMultiPresented = n;
		return *this;
	}
	TickSpec& Over(int n)
	{
		t.framesOverBudget = n;
		return *this;
	}
	TickSpec& Interval(double ms)
	{
		t.clientFrameIntervalMs = ms;
		return *this;
	}
	TickSpec& Frames(int n)
	{
		t.framesConsidered = n;
		return *this;
	}
	operator dynamicres::DynamicResolutionTiming() const { return t; }
};

TickSpec Tick(double p50, double p95 = -1.0, double maxMs = -1.0)
{
	TickSpec spec;
	spec.t.frameBudgetMs = 10.0;
	spec.t.framesConsidered = 90;
	spec.t.appGpuP50Ms = p50;
	spec.t.appGpuP95Ms = p95 < 0.0 ? p50 : p95;
	spec.t.appGpuMaxMs = maxMs < 0.0 ? spec.t.appGpuP95Ms : maxMs;
	spec.t.clientFrameIntervalMs = 10.0;
	return spec;
}

dynamicres::DynamicResolutionControllerInput Input(const dynamicres::DynamicResolutionTiming& timing,
                                                   double currentScale = 1.0, double baselineScale = 1.0)
{
	dynamicres::DynamicResolutionControllerInput input;
	input.timing = timing;
	input.baselineScale = baselineScale;
	input.currentScale = currentScale;
	input.sceneRunning = true;
	return input;
}

dynamicres::DynamicResolutionSettings FastSettings()
{
	dynamicres::DynamicResolutionSettings settings;
	settings.windowSize = 4;
	settings.lowerRequiredTicks = 2;
	settings.raiseRequiredTicks = 2;
	settings.cpuReleaseTicks = 4;
	settings.settleTicks = 0;
	settings.noEffectLimit = 2;
	settings.stepFraction = 0.15;
	return settings;
}

} // namespace

TEST(DynamicResolutionLogic, PerFrameAppGpuUsesPreAndPostSubmit)
{
	EXPECT_DOUBLE_EQ(dynamicres::PerFrameAppGpuMs(6.0, 1.5, 20.0, 3.0), 7.5);
}

TEST(DynamicResolutionLogic, PerFrameAppGpuFallsBackToTotalMinusCompositor)
{
	EXPECT_DOUBLE_EQ(dynamicres::PerFrameAppGpuMs(0.0, 0.0, 14.0, 3.5), 10.5);
}

TEST(DynamicResolutionLogic, PercentileSortedPicksExactRank)
{
	const std::vector<double> values{5.0, 1.0, 9.0, 3.0, 7.0, 2.0, 8.0, 4.0, 10.0, 6.0};
	EXPECT_DOUBLE_EQ(dynamicres::PercentileSorted(values, 0.5), 5.0);
	EXPECT_DOUBLE_EQ(dynamicres::PercentileSorted(values, 0.95), 10.0);
	EXPECT_DOUBLE_EQ(dynamicres::PercentileSorted({}, 0.95), 0.0);
}

TEST(DynamicResolutionLogic, ClassifiesGpuBoundAndLowersAfterDwell)
{
	dynamicres::DynamicResolutionController controller;
	const auto settings = FastSettings();
	dynamicres::DynamicResolutionControllerOutput out;

	for (int i = 0; i < 5; ++i) {
		out = controller.Evaluate(Input(Tick(9.0, 9.5).GpuHarm(5)), settings);
	}

	EXPECT_EQ(out.classification.pressure, dynamicres::ResolutionPressure::GpuBound);
	EXPECT_EQ(out.action, dynamicres::ResolutionAction::Lower);
	EXPECT_LT(out.targetScale, 1.0);
	EXPECT_GE(out.targetScale, settings.minScaleFraction);
}

TEST(DynamicResolutionLogic, SingleFrameHitchNeverLowers)
{
	dynamicres::DynamicResolutionController controller;
	const auto settings = FastSettings();

	for (int i = 0; i < 3; ++i) {
		controller.Evaluate(Input(Tick(6.0)), settings);
	}
	auto out = controller.Evaluate(Input(Tick(6.0, 9.5).GpuHarm(2)), settings);
	EXPECT_NE(out.action, dynamicres::ResolutionAction::Lower);
	for (int i = 0; i < 4; ++i) {
		out = controller.Evaluate(Input(Tick(6.0)), settings);
		EXPECT_NE(out.action, dynamicres::ResolutionAction::Lower);
	}
}

TEST(DynamicResolutionLogic, TwoConsecutiveHarmedTicksLower)
{
	dynamicres::DynamicResolutionController controller;
	const auto settings = FastSettings();

	for (int i = 0; i < 3; ++i) {
		controller.Evaluate(Input(Tick(6.0)), settings);
	}
	auto out = controller.Evaluate(Input(Tick(9.0, 9.8).GpuHarm(2)), settings);
	EXPECT_NE(out.action, dynamicres::ResolutionAction::Lower);
	out = controller.Evaluate(Input(Tick(9.0, 9.8).GpuHarm(2)), settings);
	EXPECT_EQ(out.action, dynamicres::ResolutionAction::Lower);
}

TEST(DynamicResolutionLogic, HonorsCustomLowerDwell)
{
	dynamicres::DynamicResolutionController controller;
	auto settings = FastSettings();
	settings.lowerRequiredTicks = 10; // longer than the window; the counter is controller state

	dynamicres::DynamicResolutionControllerOutput out;
	for (int i = 0; i < 9; ++i) {
		out = controller.Evaluate(Input(Tick(9.0, 9.5).GpuHarm(5)), settings);
		EXPECT_NE(out.action, dynamicres::ResolutionAction::Lower) << "tick " << i;
	}
	out = controller.Evaluate(Input(Tick(9.0, 9.5).GpuHarm(5)), settings);
	EXPECT_EQ(out.action, dynamicres::ResolutionAction::Lower);
}

TEST(DynamicResolutionLogic, LowerSolveSizesCut)
{
	dynamicres::DynamicResolutionController controller;
	const auto settings = FastSettings();
	dynamicres::DynamicResolutionControllerOutput out;

	// p95 12 ms vs a 9 ms fit target wants 0.75x, but the per-write cap (15%) binds first.
	for (int i = 0; i < 4; ++i) {
		out = controller.Evaluate(Input(Tick(11.5, 12.0).GpuHarm(5)), settings);
	}

	ASSERT_EQ(out.action, dynamicres::ResolutionAction::Lower);
	EXPECT_NEAR(out.targetScale, 0.85, 1e-9);
}

TEST(DynamicResolutionLogic, LowerStepScalesWithSeverity)
{
	auto settings = FastSettings();
	settings.stepFraction = 0.25;
	dynamicres::DynamicResolutionController mild;
	dynamicres::DynamicResolutionController severe;
	dynamicres::DynamicResolutionControllerOutput mildOut;
	dynamicres::DynamicResolutionControllerOutput severeOut;

	for (int i = 0; i < 4; ++i) {
		mildOut = mild.Evaluate(Input(Tick(10.5, 11.0).GpuHarm(5)), settings);
		severeOut = severe.Evaluate(Input(Tick(13.0, 14.0).GpuHarm(5)), settings);
	}

	ASSERT_EQ(mildOut.action, dynamicres::ResolutionAction::Lower);
	ASSERT_EQ(severeOut.action, dynamicres::ResolutionAction::Lower);
	EXPECT_LT(severeOut.targetScale, mildOut.targetScale); // worse overload cuts deeper
}

TEST(DynamicResolutionLogic, HighAppGpuAloneDoesNotLower)
{
	dynamicres::DynamicResolutionController controller;
	const auto settings = FastSettings();
	dynamicres::DynamicResolutionControllerOutput out;

	// Sustained high app GPU work but every frame presented on time: a game may safely spend most
	// of the budget, so hold -- do not lower.
	for (int i = 0; i < 6; ++i) {
		out = controller.Evaluate(Input(Tick(9.5, 9.8)), settings);
	}

	EXPECT_NE(out.classification.pressure, dynamicres::ResolutionPressure::GpuBound);
	EXPECT_EQ(out.action, dynamicres::ResolutionAction::None);
}

TEST(DynamicResolutionLogic, PeakSpikeWithoutDropsDoesNotLower)
{
	dynamicres::DynamicResolutionController controller;
	const auto settings = FastSettings();
	dynamicres::DynamicResolutionControllerOutput out;

	for (int i = 0; i < 4; ++i) {
		out = controller.Evaluate(Input(Tick(6.0, 6.5, 9.9)), settings);
	}

	EXPECT_NE(out.classification.pressure, dynamicres::ResolutionPressure::GpuBound);
	EXPECT_NE(out.action, dynamicres::ResolutionAction::Lower);
}

TEST(DynamicResolutionLogic, OverBudgetShareAloneDoesNotLower)
{
	dynamicres::DynamicResolutionController controller;
	const auto settings = FastSettings();
	dynamicres::DynamicResolutionControllerOutput out;

	for (int i = 0; i < 4; ++i) {
		out = controller.Evaluate(Input(Tick(7.5, 8.0).Over(20)), settings);
	}

	EXPECT_NE(out.classification.pressure, dynamicres::ResolutionPressure::GpuBound);
	EXPECT_NE(out.action, dynamicres::ResolutionAction::Lower);
}

TEST(DynamicResolutionLogic, HarmRateBoundary)
{
	const auto settings = FastSettings();

	// One GPU-harmed frame of 90 (1.1%) sits under the 1.5% tick-harm threshold: never lowers.
	dynamicres::DynamicResolutionController atBaseline;
	dynamicres::DynamicResolutionControllerOutput out;
	for (int i = 0; i < 6; ++i) {
		out = atBaseline.Evaluate(Input(Tick(9.0, 9.4).GpuHarm(1)), settings);
		EXPECT_NE(out.action, dynamicres::ResolutionAction::Lower);
	}

	// The same trickle still exceeds the raise tolerance (0.2%): recovery holds, no thrash.
	dynamicres::DynamicResolutionController lowered;
	for (int i = 0; i < 6; ++i) {
		out = lowered.Evaluate(Input(Tick(9.0, 9.4).GpuHarm(1), 0.8), settings);
	}
	EXPECT_EQ(out.action, dynamicres::ResolutionAction::None);
	EXPECT_STREQ(out.withheldGate, "raise");
	EXPECT_STREQ(out.withheldCause, "harm_rate");
}

TEST(DynamicResolutionLogic, ThrottledFramesAreGpuHarm)
{
	dynamicres::DynamicResolutionController controller;
	const auto settings = FastSettings();

	for (int i = 0; i < 3; ++i) {
		controller.Evaluate(Input(Tick(6.0)), settings);
	}
	controller.Evaluate(Input(Tick(9.0, 9.5).Throttled(10)), settings);
	const auto out = controller.Evaluate(Input(Tick(9.0, 9.5).Throttled(10)), settings);

	EXPECT_EQ(out.classification.pressure, dynamicres::ResolutionPressure::GpuBound);
	EXPECT_EQ(out.action, dynamicres::ResolutionAction::Lower);
}

TEST(DynamicResolutionLogic, DropsAttributedByGpuTail)
{
	const auto settings = FastSettings();

	// Drops with the GPU tail at budget and no CPU bits attribute to the GPU: lower.
	dynamicres::DynamicResolutionController gpuTail;
	dynamicres::DynamicResolutionControllerOutput out;
	for (int i = 0; i < 3; ++i) {
		gpuTail.Evaluate(Input(Tick(6.0)), settings);
	}
	gpuTail.Evaluate(Input(Tick(9.2, 9.6).Drops(3)), settings);
	out = gpuTail.Evaluate(Input(Tick(9.2, 9.6).Drops(3)), settings);
	EXPECT_EQ(out.action, dynamicres::ResolutionAction::Lower);

	// The same drops with an idle GPU are someone else's problem: hold.
	dynamicres::DynamicResolutionController idleGpu;
	for (int i = 0; i < 6; ++i) {
		out = idleGpu.Evaluate(Input(Tick(4.5, 5.0).Drops(3)), settings);
		EXPECT_NE(out.action, dynamicres::ResolutionAction::Lower);
	}
	EXPECT_NE(out.classification.pressure, dynamicres::ResolutionPressure::GpuBound);
}

TEST(DynamicResolutionLogic, CpuBoundMissesDoNotLowerResolution)
{
	dynamicres::DynamicResolutionController controller;
	const auto settings = FastSettings();
	dynamicres::DynamicResolutionControllerOutput out;

	for (int i = 0; i < 6; ++i) {
		out = controller.Evaluate(Input(Tick(4.0).CpuHarm(5)), settings);
	}

	EXPECT_EQ(out.classification.pressure, dynamicres::ResolutionPressure::CpuBound);
	EXPECT_TRUE(out.classification.gpuHasHeadroom);
	EXPECT_EQ(out.action, dynamicres::ResolutionAction::None);
}

TEST(DynamicResolutionLogic, CpuBoundWithGpuHeadroomRaisesTowardBaseline)
{
	dynamicres::DynamicResolutionController controller;
	const auto settings = FastSettings();

	dynamicres::DynamicResolutionControllerOutput out;
	for (int i = 0; i < 6; ++i) {
		out = controller.Evaluate(Input(Tick(4.0).CpuHarm(5), 0.8), settings);
	}

	EXPECT_EQ(out.classification.pressure, dynamicres::ResolutionPressure::CpuBound);
	EXPECT_TRUE(out.classification.gpuHasHeadroom);
	EXPECT_EQ(out.action, dynamicres::ResolutionAction::Raise);
	EXPECT_GT(out.targetScale, 0.8);
	EXPECT_LE(out.targetScale, 1.0);
}

TEST(DynamicResolutionLogic, CpuBoundWithoutGpuHeadroomHoldsScale)
{
	dynamicres::DynamicResolutionController controller;
	const auto settings = FastSettings();

	dynamicres::DynamicResolutionControllerOutput out;
	for (int i = 0; i < 6; ++i) {
		out = controller.Evaluate(Input(Tick(9.3, 9.5).CpuHarm(5), 0.8), settings);
	}

	EXPECT_EQ(out.classification.pressure, dynamicres::ResolutionPressure::CpuBound);
	EXPECT_FALSE(out.classification.gpuHasHeadroom);
	EXPECT_EQ(out.action, dynamicres::ResolutionAction::None);
}

TEST(DynamicResolutionLogic, CpuBoundReleaseCanBeDisabled)
{
	dynamicres::DynamicResolutionController controller;
	auto settings = FastSettings();
	settings.releaseOnCpuBound = false;

	dynamicres::DynamicResolutionControllerOutput out;
	for (int i = 0; i < 6; ++i) {
		out = controller.Evaluate(Input(Tick(4.0).CpuHarm(5), 0.8), settings);
	}

	EXPECT_EQ(out.classification.pressure, dynamicres::ResolutionPressure::CpuBound);
	EXPECT_TRUE(out.classification.gpuHasHeadroom);
	EXPECT_EQ(out.action, dynamicres::ResolutionAction::None);
}

TEST(DynamicResolutionLogic, MeasuredCpuStallDoesNotLower)
{
	dynamicres::DynamicResolutionController controller;
	const auto settings = FastSettings();

	// GPU idle (4 ms of 10) but the CPU cadence is slow (13 ms): a measured CPU stall. Lowering
	// resolution cannot help, so hold scale.
	dynamicres::DynamicResolutionControllerOutput out;
	for (int i = 0; i < 6; ++i) {
		out = controller.Evaluate(Input(Tick(4.0).Interval(13.0)), settings);
	}

	EXPECT_TRUE(out.classification.cpuStalled);
	EXPECT_EQ(out.classification.pressure, dynamicres::ResolutionPressure::CpuBound);
	EXPECT_EQ(out.action, dynamicres::ResolutionAction::None);
}

TEST(DynamicResolutionLogic, MotionSmoothingWithHighGpuLowersToRecoverFullRate)
{
	dynamicres::DynamicResolutionController controller;
	const auto settings = FastSettings();

	// Motion smoothing engaged (half-rate) with the GPU at budget: lower to climb back to full rate.
	dynamicres::DynamicResolutionControllerOutput out;
	for (int i = 0; i < 6; ++i) {
		out = controller.Evaluate(Input(Tick(9.0, 9.5).Motion(90).Interval(22.0)), settings);
	}

	EXPECT_TRUE(out.classification.motionSmoothingEngaged);
	EXPECT_EQ(out.classification.pressure, dynamicres::ResolutionPressure::GpuBound);
	EXPECT_EQ(out.action, dynamicres::ResolutionAction::Lower);
	EXPECT_LT(out.targetScale, 1.0);
}

TEST(DynamicResolutionLogic, MotionSmoothingWithLowGpuDoesNotLower)
{
	dynamicres::DynamicResolutionController controller;
	const auto settings = FastSettings();

	// Motion smoothing active but the GPU is idle: the limiter is not the GPU, so hold scale --
	// and do not read the half-rate cadence as a CPU stall.
	dynamicres::DynamicResolutionControllerOutput out;
	for (int i = 0; i < 6; ++i) {
		out = controller.Evaluate(Input(Tick(4.0).Motion(90).Interval(22.0)), settings);
	}

	EXPECT_TRUE(out.classification.motionSmoothingEngaged);
	EXPECT_NE(out.classification.pressure, dynamicres::ResolutionPressure::GpuBound);
	EXPECT_NE(out.classification.pressure, dynamicres::ResolutionPressure::CpuBound);
	EXPECT_EQ(out.action, dynamicres::ResolutionAction::None);
}

TEST(DynamicResolutionLogic, MotionHalfRateLowGpuRecovers)
{
	dynamicres::DynamicResolutionController controller;
	const auto settings = FastSettings();

	// Locked in motion-smoothing half-rate below baseline with plenty of GPU headroom: the scale
	// must climb back instead of deadlocking at low resolution.
	dynamicres::DynamicResolutionControllerOutput out;
	for (int i = 0; i < 6; ++i) {
		out = controller.Evaluate(Input(Tick(5.0).Motion(90).Interval(22.0), 0.7), settings);
	}

	EXPECT_EQ(out.action, dynamicres::ResolutionAction::Raise);
	EXPECT_GT(out.targetScale, 0.7);
}

TEST(DynamicResolutionLogic, OneMotionFrameDoesNotBlockRaise)
{
	dynamicres::DynamicResolutionController controller;
	const auto settings = FastSettings();

	controller.Evaluate(Input(Tick(4.0).Motion(1), 0.8), settings);
	dynamicres::DynamicResolutionControllerOutput out;
	for (int i = 0; i < 4; ++i) {
		out = controller.Evaluate(Input(Tick(4.0), 0.8), settings);
	}

	EXPECT_EQ(out.action, dynamicres::ResolutionAction::Raise);
}

TEST(DynamicResolutionLogic, HeadroomRaisesOnlyTowardBaseline)
{
	dynamicres::DynamicResolutionController controller;
	const auto settings = FastSettings();
	dynamicres::DynamicResolutionControllerOutput out;

	for (int i = 0; i < 5; ++i) {
		out = controller.Evaluate(Input(Tick(4.0), 0.8), settings);
	}

	EXPECT_EQ(out.classification.pressure, dynamicres::ResolutionPressure::Headroom);
	EXPECT_EQ(out.action, dynamicres::ResolutionAction::Raise);
	EXPECT_GT(out.targetScale, 0.8);
	EXPECT_LE(out.targetScale, 1.0);
}

TEST(DynamicResolutionLogic, RaiseStepSolvesTowardFit)
{
	auto settings = FastSettings();
	settings.raiseRequiredTicks = 1;

	dynamicres::DynamicResolutionController deepHeadroom;
	dynamicres::DynamicResolutionController shallowHeadroom;
	dynamicres::DynamicResolutionControllerOutput deepOut;
	dynamicres::DynamicResolutionControllerOutput shallowOut;

	for (int i = 0; i < 4; ++i) {
		deepOut = deepHeadroom.Evaluate(Input(Tick(3.0), 0.70), settings);
		shallowOut = shallowHeadroom.Evaluate(Input(Tick(8.2, 8.5), 0.70), settings);
	}

	ASSERT_EQ(deepOut.action, dynamicres::ResolutionAction::Raise);
	ASSERT_EQ(shallowOut.action, dynamicres::ResolutionAction::Raise);
	EXPECT_GT(deepOut.targetScale - 0.70, shallowOut.targetScale - 0.70);
}

TEST(DynamicResolutionLogic, RaisePredictedFitWithholds)
{
	dynamicres::DynamicResolutionController controller;
	const auto settings = FastSettings();
	dynamicres::DynamicResolutionControllerOutput out;

	// Clean window, but p95 already sits at 8.9 of a 9.0 ms fit budget: the model proves the next
	// step would not fit, so recovery holds instead of stepping into reprojection.
	for (int i = 0; i < 6; ++i) {
		out = controller.Evaluate(Input(Tick(8.5, 8.9), 0.8), settings);
	}

	EXPECT_EQ(out.action, dynamicres::ResolutionAction::None);
	EXPECT_STREQ(out.withheldGate, "raise");
	EXPECT_STREQ(out.withheldCause, "predicted_fit");
	EXPECT_GT(out.classification.predictedRaiseP95Ms, 0.0);
}

TEST(DynamicResolutionLogic, RaiseToleratesOccasionalNonGpuHitch)
{
	dynamicres::DynamicResolutionController controller;
	auto settings = FastSettings();
	settings.windowSize = 6; // one hitch frame in ~540 sits inside the raise tolerance

	controller.Evaluate(Input(Tick(4.0), 0.8), settings);
	controller.Evaluate(Input(Tick(4.0).CpuHarm(1), 0.8), settings);
	dynamicres::DynamicResolutionControllerOutput out;
	for (int i = 0; i < 4; ++i) {
		out = controller.Evaluate(Input(Tick(4.0), 0.8), settings);
	}

	EXPECT_EQ(out.action, dynamicres::ResolutionAction::Raise);
}

TEST(DynamicResolutionLogic, RaiseWaitsForGpuDropToClear)
{
	dynamicres::DynamicResolutionController controller;
	const auto settings = FastSettings();

	for (int i = 0; i < 3; ++i) {
		controller.Evaluate(Input(Tick(4.0), 0.8), settings);
	}
	// A GPU-harmed tick lands: recovery must pause until the harm ages out of the window.
	auto out = controller.Evaluate(Input(Tick(7.0, 9.6).GpuHarm(2), 0.8), settings);
	EXPECT_NE(out.action, dynamicres::ResolutionAction::Raise);
	out = controller.Evaluate(Input(Tick(4.0), 0.8), settings);
	EXPECT_NE(out.action, dynamicres::ResolutionAction::Raise);

	bool raised = false;
	for (int i = 0; i < 8; ++i) {
		out = controller.Evaluate(Input(Tick(4.0), 0.8), settings);
		if (out.action == dynamicres::ResolutionAction::Raise) raised = true;
	}
	EXPECT_TRUE(raised);
}

TEST(DynamicResolutionLogic, RaiseRecoversToBaselineWithinTickBudget)
{
	dynamicres::DynamicResolutionController controller;
	const auto settings = FastSettings();
	double currentScale = 0.60;

	for (int tick = 0; tick < 20 && currentScale < 0.995; ++tick) {
		const auto out = controller.Evaluate(Input(Tick(5.0), currentScale), settings);
		if (out.action == dynamicres::ResolutionAction::Raise) {
			currentScale = out.targetScale;
			controller.NoteWrite(out.action, currentScale, out.classification, settings);
		}
	}

	EXPECT_NEAR(currentScale, 1.0, 0.005);
}

TEST(DynamicResolutionLogic, RecoversAfterReprojClears)
{
	dynamicres::DynamicResolutionController controller;
	const auto settings = FastSettings();
	dynamicres::DynamicResolutionControllerOutput out;

	// Proven GPU harm lowers the scale.
	for (int i = 0; i < 4; ++i) {
		out = controller.Evaluate(Input(Tick(9.5, 10.0).GpuHarm(5)), settings);
	}
	ASSERT_EQ(out.action, dynamicres::ResolutionAction::Lower);
	double scale = out.targetScale;
	controller.NoteWrite(out.action, scale, out.classification, settings);
	ASSERT_LT(scale, 1.0);

	// The drops stop and the scene runs clean: the scale climbs back to baseline (no ratchet).
	bool raised = false;
	for (int tick = 0; tick < 20 && scale < 0.995; ++tick) {
		out = controller.Evaluate(Input(Tick(4.0), scale), settings);
		if (out.action == dynamicres::ResolutionAction::Raise) {
			raised = true;
			scale = out.targetScale;
			controller.NoteWrite(out.action, scale, out.classification, settings);
		}
	}

	EXPECT_TRUE(raised);
	EXPECT_NEAR(scale, 1.0, 0.005);
}

TEST(DynamicResolutionLogic, BetaLearnedFromWrite)
{
	dynamicres::DynamicResolutionController controller;
	const auto settings = FastSettings();
	dynamicres::DynamicResolutionControllerOutput out;

	for (int i = 0; i < 4; ++i) {
		out = controller.Evaluate(Input(Tick(11.5, 12.0).GpuHarm(5)), settings);
	}
	ASSERT_EQ(out.action, dynamicres::ResolutionAction::Lower);
	ASSERT_NEAR(out.targetScale, 0.85, 1e-9);
	controller.NoteWrite(out.action, out.targetScale, out.classification, settings);

	// Post-write p95 lands at 10.86 ms: observed sensitivity ~0.615, EMA pulls beta to ~0.81.
	for (int i = 0; i < 3; ++i) {
		out = controller.Evaluate(Input(Tick(10.4, 10.86), 0.85), settings);
	}
	ASSERT_TRUE(out.effectCheck.ran);
	EXPECT_STREQ(out.effectCheck.verdict, "effective");
	EXPECT_NEAR(out.effectCheck.betaObserved, 0.615, 0.01);

	out = controller.Evaluate(Input(Tick(10.4, 10.86), 0.85), settings);
	EXPECT_NEAR(out.classification.costBeta, 0.807, 0.01);
}

TEST(DynamicResolutionLogic, NoEffectLoweringRequestsRestore)
{
	dynamicres::DynamicResolutionController controller;
	auto settings = FastSettings();
	settings.noEffectLimit = 1;

	dynamicres::DynamicResolutionControllerOutput out;
	for (int i = 0; i < 4; ++i) {
		out = controller.Evaluate(Input(Tick(11.5, 12.0).GpuHarm(5)), settings);
	}
	ASSERT_EQ(out.action, dynamicres::ResolutionAction::Lower);
	controller.NoteWrite(out.action, out.targetScale, out.classification, settings);

	// The cut moved p95 not at all: lowering demonstrably does nothing here, so restore.
	for (int i = 0; i < 3; ++i) {
		out = controller.Evaluate(Input(Tick(11.5, 12.0).GpuHarm(5), 0.85), settings);
	}

	EXPECT_EQ(out.action, dynamicres::ResolutionAction::NoEffect);
	EXPECT_DOUBLE_EQ(out.targetScale, 1.0);
}

TEST(DynamicResolutionLogic, RaiseRegressionBurnsScale)
{
	dynamicres::DynamicResolutionController controller;
	auto settings = FastSettings();
	settings.burnedDecayTicks = 10;
	dynamicres::DynamicResolutionControllerOutput out;

	// Clean climb from 0.8: the fit model allows a raise to 0.9.
	for (int i = 0; i < 3; ++i) {
		out = controller.Evaluate(Input(Tick(7.6, 8.0), 0.8), settings);
	}
	ASSERT_EQ(out.action, dynamicres::ResolutionAction::Raise);
	ASSERT_NEAR(out.targetScale, 0.9, 1e-9);
	controller.NoteWrite(out.action, out.targetScale, out.classification, settings);

	// The raise brings harm back: exact revert, and 0.9 is burned.
	for (int i = 0; i < 3; ++i) {
		out = controller.Evaluate(Input(Tick(9.2, 9.9).GpuHarm(3), 0.9), settings);
	}
	ASSERT_EQ(out.action, dynamicres::ResolutionAction::Lower);
	EXPECT_DOUBLE_EQ(out.targetScale, 0.8);
	EXPECT_EQ(out.reason, "raise regressed");
	controller.NoteWrite(out.action, out.targetScale, out.classification, settings);

	// While burned, recovery stops short of the scale that regressed.
	for (int i = 0; i < 3; ++i) {
		out = controller.Evaluate(Input(Tick(7.6, 8.0), 0.8), settings);
	}
	ASSERT_EQ(out.action, dynamicres::ResolutionAction::Raise);
	EXPECT_LE(out.targetScale, 0.9 * 0.98 + 1e-9);
	controller.NoteWrite(out.action, out.targetScale, out.classification, settings);

	// After the burn decays, the fit may target past 0.9 again.
	double scale = out.targetScale;
	bool passedBurn = false;
	for (int tick = 0; tick < 24 && !passedBurn; ++tick) {
		out = controller.Evaluate(Input(Tick(7.0, 7.4), scale), settings);
		if (out.action == dynamicres::ResolutionAction::Raise) {
			scale = out.targetScale;
			passedBurn = scale > 0.9;
			controller.NoteWrite(out.action, scale, out.classification, settings);
		}
	}
	EXPECT_TRUE(passedBurn);
}

TEST(DynamicResolutionLogic, AppPacedHoldsAndNeverSupersamples)
{
	const auto settings = FastSettings();

	// An app pacing itself below refresh (fps cap): no harm, slow cadence, re-presented frames.
	// Refresh-budget fit is meaningless here, so never lower and never supersample.
	dynamicres::DynamicResolutionController atBaseline;
	dynamicres::DynamicResolutionControllerOutput out;
	for (int i = 0; i < 12; ++i) {
		out = atBaseline.Evaluate(Input(Tick(4.0).MultiPresent(90).Interval(22.0)), settings);
		EXPECT_EQ(out.action, dynamicres::ResolutionAction::None);
	}
	EXPECT_TRUE(out.classification.appPaced);
	EXPECT_STREQ(out.withheldGate, "supersample");
	EXPECT_STREQ(out.withheldCause, "app_paced");

	// Below baseline the same state still recovers toward the user's scale.
	dynamicres::DynamicResolutionController lowered;
	for (int i = 0; i < 5; ++i) {
		out = lowered.Evaluate(Input(Tick(4.0).MultiPresent(90).Interval(22.0), 0.8), settings);
	}
	EXPECT_EQ(out.action, dynamicres::ResolutionAction::Raise);
}

TEST(DynamicResolutionLogic, SupersampleGateReachable)
{
	dynamicres::DynamicResolutionController controller;
	const auto settings = FastSettings();
	dynamicres::DynamicResolutionControllerOutput out;

	// A realistic clean run (p95 at 60% of budget, zero harm) must actually reach the
	// above-baseline raise within its dwell.
	bool supersampled = false;
	for (int i = 0; i < 12 && !supersampled; ++i) {
		out = controller.Evaluate(Input(Tick(5.5, 6.0)), settings);
		supersampled = out.action == dynamicres::ResolutionAction::Raise && out.targetScale > 1.0;
	}

	EXPECT_TRUE(supersampled);
	EXPECT_LE(out.targetScale, dynamicres::CeilingScale(settings, 1.0));
}

TEST(DynamicResolutionLogic, SupersamplePredictedFitWithholds)
{
	dynamicres::DynamicResolutionController controller;
	const auto settings = FastSettings();
	dynamicres::DynamicResolutionControllerOutput out;

	// p95 6.4 passes the 6.5 ms gate, but the model predicts the next step would not fit.
	for (int i = 0; i < 12; ++i) {
		out = controller.Evaluate(Input(Tick(6.0, 6.4)), settings);
		EXPECT_EQ(out.action, dynamicres::ResolutionAction::None);
	}
	EXPECT_STREQ(out.withheldGate, "supersample");
	EXPECT_STREQ(out.withheldCause, "predicted_fit");
}

TEST(DynamicResolutionLogic, SupersampleToleratesTraceOverBudget)
{
	dynamicres::DynamicResolutionController controller;
	const auto settings = FastSettings();
	dynamicres::DynamicResolutionControllerOutput out;

	// One over-budget frame ages out of the window; a trace does not pin the gate shut.
	controller.Evaluate(Input(Tick(5.5, 6.0).Over(1)), settings);
	bool supersampled = false;
	for (int i = 0; i < 12 && !supersampled; ++i) {
		out = controller.Evaluate(Input(Tick(5.5, 6.0)), settings);
		supersampled = out.action == dynamicres::ResolutionAction::Raise && out.targetScale > 1.0;
	}
	EXPECT_TRUE(supersampled);
}

TEST(DynamicResolutionLogic, SupersampleBlockedBySteadyOverBudget)
{
	dynamicres::DynamicResolutionController controller;
	const auto settings = FastSettings();
	dynamicres::DynamicResolutionControllerOutput out;

	// An over-budget frame every tick (1.1% > 0.5%) keeps the tail too hot to supersample.
	for (int i = 0; i < 12; ++i) {
		out = controller.Evaluate(Input(Tick(5.5, 6.0).Over(1)), settings);
		EXPECT_EQ(out.action, dynamicres::ResolutionAction::None);
	}
	EXPECT_STREQ(out.withheldGate, "supersample");
}

TEST(DynamicResolutionLogic, BacksOffFromAboveBaselineOnGpuPressure)
{
	dynamicres::DynamicResolutionController controller;
	const auto settings = FastSettings();

	for (int i = 0; i < 3; ++i) {
		controller.Evaluate(Input(Tick(6.0), 1.30), settings);
	}
	controller.Evaluate(Input(Tick(9.5, 10.5).GpuHarm(5), 1.30), settings);
	const auto out = controller.Evaluate(Input(Tick(9.5, 10.5).GpuHarm(5), 1.30), settings);

	EXPECT_EQ(out.classification.pressure, dynamicres::ResolutionPressure::GpuBound);
	EXPECT_EQ(out.action, dynamicres::ResolutionAction::Lower);
	EXPECT_LT(out.targetScale, 1.30);
}

TEST(DynamicResolutionLogic, WriteClearsSampleWindow)
{
	dynamicres::DynamicResolutionController controller;
	const auto settings = FastSettings();
	dynamicres::DynamicResolutionControllerOutput out;

	for (int i = 0; i < 4; ++i) {
		out = controller.Evaluate(Input(Tick(9.0, 9.5).GpuHarm(5)), settings);
	}
	ASSERT_EQ(out.action, dynamicres::ResolutionAction::Lower);
	controller.NoteWrite(out.action, out.targetScale, out.classification, settings);

	out = controller.Evaluate(Input(Tick(4.0), out.targetScale), settings);

	EXPECT_EQ(out.classification.tickCount, 1);
	EXPECT_EQ(out.classification.pressure, dynamicres::ResolutionPressure::Waiting);
	EXPECT_EQ(out.action, dynamicres::ResolutionAction::None);
}

TEST(DynamicResolutionLogic, ExternalOverrideBacksOffWithoutWriting)
{
	dynamicres::DynamicResolutionController controller;
	const auto settings = FastSettings();
	auto input = Input(Tick(9.5, 10.0).GpuHarm(5), 0.9);
	input.externalOverride = true;

	const auto out = controller.Evaluate(input, settings);

	EXPECT_EQ(out.action, dynamicres::ResolutionAction::ExternalOverride);
	EXPECT_DOUBLE_EQ(out.targetScale, 0.9);
}

TEST(DynamicResolutionLogic, SceneStopRequestsRestore)
{
	dynamicres::DynamicResolutionController controller;
	const auto settings = FastSettings();
	auto input = Input(Tick(4.0), 0.8);
	input.sceneRunning = false;

	const auto out = controller.Evaluate(input, settings);

	EXPECT_EQ(out.action, dynamicres::ResolutionAction::Restore);
	EXPECT_DOUBLE_EQ(out.targetScale, 1.0);
}

TEST(DynamicResolutionLogic, ApplyQualityPresetSeedsKnobs)
{
	dynamicres::DynamicResolutionSettings settings;
	dynamicres::ApplyQualityPreset(dynamicres::QualityPreset::MaxFps, settings);
	EXPECT_EQ(settings.qualityPreset, dynamicres::QualityPreset::MaxFps);
	EXPECT_DOUBLE_EQ(settings.minScaleFraction, 0.50);
	EXPECT_DOUBLE_EQ(settings.maxScaleFraction, 1.00);
	EXPECT_DOUBLE_EQ(settings.gpuHarmRateFraction, 0.010);
	EXPECT_DOUBLE_EQ(settings.raiseSafetyFraction, 0.85);
	EXPECT_EQ(settings.lowerRequiredTicks, 2);
	EXPECT_EQ(settings.settleTicks, 1);

	dynamicres::ApplyQualityPreset(dynamicres::QualityPreset::Quality, settings);
	EXPECT_EQ(settings.qualityPreset, dynamicres::QualityPreset::Quality);
	EXPECT_DOUBLE_EQ(settings.minScaleFraction, 0.80);
	EXPECT_DOUBLE_EQ(settings.gpuHarmRateFraction, 0.030);
	EXPECT_DOUBLE_EQ(settings.lowerTargetFraction, 0.94);
	EXPECT_EQ(settings.burnedDecayTicks, 90);
	EXPECT_EQ(settings.settleTicks, 3);
}

TEST(DynamicResolutionLogic, ReconcilePresetRederivesStaleNamedPresetKnobs)
{
	// A profile saved by an older build pins the Quality preset but stale metric knobs.
	dynamicres::DynamicResolutionSettings settings;
	settings.qualityPreset = dynamicres::QualityPreset::Quality;
	settings.gpuHarmRateFraction = 0.10;
	settings.settleTicks = 0;

	dynamicres::ReconcilePresetSettings(settings);

	EXPECT_DOUBLE_EQ(settings.gpuHarmRateFraction, 0.030);
	EXPECT_EQ(settings.settleTicks, 3);
}

TEST(DynamicResolutionLogic, ReconcilePresetLeavesCustomUntouched)
{
	dynamicres::DynamicResolutionSettings settings;
	settings.qualityPreset = dynamicres::QualityPreset::Custom;
	settings.gpuHarmRateFraction = 0.08;
	settings.settleTicks = 0;

	dynamicres::ReconcilePresetSettings(settings);

	EXPECT_DOUBLE_EQ(settings.gpuHarmRateFraction, 0.08);
	EXPECT_EQ(settings.settleTicks, 0);
}

TEST(DynamicResolutionLogic, CeilingScaleAllowsSupersampleButClampsHard)
{
	dynamicres::DynamicResolutionSettings settings;
	settings.maxScaleFraction = 1.50;
	EXPECT_DOUBLE_EQ(dynamicres::CeilingScale(settings, 1.0), 1.5);
	EXPECT_DOUBLE_EQ(dynamicres::CeilingScale(settings, 1.5), 1.5); // hard clamp at 1.5

	settings.maxScaleFraction = 1.00;
	EXPECT_DOUBLE_EQ(dynamicres::CeilingScale(settings, 0.9), 0.9); // no supersampling
}
