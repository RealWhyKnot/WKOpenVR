#include "DynamicResolutionLogic.h"

#include <gtest/gtest.h>

namespace dynamicres = wkopenvr::dynamicres;

namespace {

constexpr uint32_t kReprojectionReasonCpu = 0x01;
constexpr uint32_t kReprojectionReasonGpu = 0x02;
constexpr uint32_t kReprojectionAsync = 0x04;
constexpr uint32_t kReprojectionMotion = 0x08;

dynamicres::DynamicResolutionTiming Timing(double appGpuMs, bool unstable, uint32_t reprojectionFlags = 0,
                                           double peakMs = -1.0, double intervalMs = 0.0, int framesOver = 0,
                                           int framesConsidered = 0)
{
	dynamicres::DynamicResolutionTiming t;
	t.frameBudgetMs = 10.0;
	t.preSubmitGpuMs = appGpuMs;
	t.totalRenderGpuMs = appGpuMs + 1.0;
	t.compositorRenderGpuMs = 1.0;
	t.peakAppGpuMs = peakMs >= 0.0 ? peakMs : appGpuMs;
	t.clientFrameIntervalMs = intervalMs;
	t.framesOverBudget = framesOver;
	t.framesConsidered = framesConsidered;
	t.framePresents = unstable ? 2u : 1u;
	t.droppedFrames = unstable ? 1u : 0u;
	t.reprojectionFlags = reprojectionFlags;
	return t;
}

dynamicres::DynamicResolutionControllerInput Input(double appGpuMs, bool unstable, double currentScale = 1.0,
                                                   uint32_t reprojectionFlags = 0, double peakMs = -1.0,
                                                   double intervalMs = 0.0, int framesOver = 0,
                                                   int framesConsidered = 0)
{
	dynamicres::DynamicResolutionControllerInput input;
	input.timing = Timing(appGpuMs, unstable, reprojectionFlags, peakMs, intervalMs, framesOver, framesConsidered);
	input.baselineScale = 1.0;
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

TEST(DynamicResolutionLogic, UsesPreAndPostSubmitAppGpuWork)
{
	dynamicres::DynamicResolutionTiming timing;
	timing.preSubmitGpuMs = 6.0;
	timing.postSubmitGpuMs = 1.5;
	timing.totalRenderGpuMs = 20.0;
	timing.compositorRenderGpuMs = 3.0;

	EXPECT_DOUBLE_EQ(dynamicres::ComputeAppGpuMs(timing), 7.5);
}

TEST(DynamicResolutionLogic, FallsBackToTotalMinusCompositorWhenSubmitTimingMissing)
{
	dynamicres::DynamicResolutionTiming timing;
	timing.totalRenderGpuMs = 14.0;
	timing.compositorRenderGpuMs = 3.5;

	EXPECT_DOUBLE_EQ(dynamicres::ComputeAppGpuMs(timing), 10.5);
}

TEST(DynamicResolutionLogic, ClassifiesGpuBoundAndLowersAfterDwell)
{
	dynamicres::DynamicResolutionController controller;
	const auto settings = FastSettings();
	dynamicres::DynamicResolutionControllerOutput out;

	for (int i = 0; i < 5; ++i) {
		out = controller.Evaluate(Input(9.5, true, 1.0, kReprojectionReasonGpu), settings);
	}

	EXPECT_EQ(out.classification.pressure, dynamicres::ResolutionPressure::GpuBound);
	EXPECT_EQ(out.action, dynamicres::ResolutionAction::Lower);
	EXPECT_LT(out.targetScale, 1.0);
	EXPECT_GE(out.targetScale, settings.minScaleFraction);
}

TEST(DynamicResolutionLogic, CpuBoundMissesDoNotLowerResolution)
{
	dynamicres::DynamicResolutionController controller;
	const auto settings = FastSettings();
	dynamicres::DynamicResolutionControllerOutput out;

	for (int i = 0; i < 6; ++i) {
		out = controller.Evaluate(Input(4.0, true, 1.0, kReprojectionReasonCpu), settings);
	}

	EXPECT_EQ(out.classification.pressure, dynamicres::ResolutionPressure::CpuBound);
	EXPECT_EQ(out.action, dynamicres::ResolutionAction::None);
}

TEST(DynamicResolutionLogic, CpuReasonWithInflatedTotalSpanDoesNotLower)
{
	dynamicres::DynamicResolutionController controller;
	const auto settings = FastSettings();
	dynamicres::DynamicResolutionControllerOutput out;

	for (int i = 0; i < 6; ++i) {
		auto input = Input(4.0, true, 1.0, kReprojectionReasonCpu);
		input.timing.totalRenderGpuMs = 14.0;
		input.timing.compositorRenderGpuMs = 1.0;
		out = controller.Evaluate(input, settings);
	}

	EXPECT_EQ(out.classification.pressure, dynamicres::ResolutionPressure::CpuBound);
	EXPECT_EQ(out.action, dynamicres::ResolutionAction::None);
	EXPECT_LT(out.classification.medianAppGpuMs, 5.0);
}

TEST(DynamicResolutionLogic, CpuBoundAtBaselineDoesNotChangeScale)
{
	dynamicres::DynamicResolutionController controller;
	const auto settings = FastSettings();
	dynamicres::DynamicResolutionControllerOutput out;

	for (int i = 0; i < 6; ++i) {
		out = controller.Evaluate(Input(4.0, true, 1.0, kReprojectionReasonCpu), settings);
	}

	EXPECT_EQ(out.classification.pressure, dynamicres::ResolutionPressure::CpuBound);
	EXPECT_TRUE(out.classification.gpuHasHeadroom);
	EXPECT_EQ(out.action, dynamicres::ResolutionAction::None);
}

TEST(DynamicResolutionLogic, AsyncReprojectionModeAloneDoesNotCountAsUnstable)
{
	dynamicres::DynamicResolutionController controller;
	const auto settings = FastSettings();
	dynamicres::DynamicResolutionControllerOutput out;

	// Mid-budget GPU (between the 8.0 ms headroom and 8.5 ms lower thresholds) with async mode on:
	// async never marks instability, and mid-budget app GPU is neither GPU-bound nor headroom.
	for (int i = 0; i < 4; ++i) {
		out = controller.Evaluate(Input(8.2, false, 1.0, kReprojectionAsync), settings);
	}

	EXPECT_EQ(out.classification.unstableSamples, 0);
	EXPECT_EQ(out.classification.pressure, dynamicres::ResolutionPressure::Waiting);
	EXPECT_EQ(out.action, dynamicres::ResolutionAction::None);
}

TEST(DynamicResolutionLogic, AsyncReprojectionModeDoesNotBlockRaiseBack)
{
	dynamicres::DynamicResolutionController controller;
	const auto settings = FastSettings();
	dynamicres::DynamicResolutionControllerOutput out;

	for (int i = 0; i < 5; ++i) {
		out = controller.Evaluate(Input(4.0, false, 0.8, kReprojectionAsync), settings);
	}

	EXPECT_EQ(out.classification.pressure, dynamicres::ResolutionPressure::Headroom);
	EXPECT_EQ(out.action, dynamicres::ResolutionAction::Raise);
}

TEST(DynamicResolutionLogic, HeadroomRaisesOnlyTowardBaseline)
{
	dynamicres::DynamicResolutionController controller;
	const auto settings = FastSettings();
	dynamicres::DynamicResolutionControllerOutput out;

	for (int i = 0; i < 5; ++i) {
		out = controller.Evaluate(Input(4.0, false, 0.8), settings);
	}

	EXPECT_EQ(out.classification.pressure, dynamicres::ResolutionPressure::Headroom);
	EXPECT_EQ(out.action, dynamicres::ResolutionAction::Raise);
	EXPECT_GT(out.targetScale, 0.8);
	EXPECT_LE(out.targetScale, 1.0);
}

TEST(DynamicResolutionLogic, RaiseStepScalesWithHeadroom)
{
	auto settings = FastSettings();
	settings.raiseRequiredTicks = 1;

	dynamicres::DynamicResolutionController deepHeadroom;
	dynamicres::DynamicResolutionController shallowHeadroom;
	dynamicres::DynamicResolutionControllerOutput deepOut;
	dynamicres::DynamicResolutionControllerOutput shallowOut;

	for (int i = 0; i < 4; ++i) {
		deepOut = deepHeadroom.Evaluate(Input(3.0, false, 0.70), settings);
		shallowOut = shallowHeadroom.Evaluate(Input(6.8, false, 0.70), settings);
	}

	ASSERT_EQ(deepOut.action, dynamicres::ResolutionAction::Raise);
	ASSERT_EQ(shallowOut.action, dynamicres::ResolutionAction::Raise);
	EXPECT_GT(deepOut.targetScale - 0.70, shallowOut.targetScale - 0.70);
}

TEST(DynamicResolutionLogic, RaiseToleratesOccasionalNonGpuHitch)
{
	dynamicres::DynamicResolutionController controller;
	const auto settings = FastSettings();
	dynamicres::DynamicResolutionControllerOutput out;

	out = controller.Evaluate(Input(4.0, false, 0.8), settings);
	out = controller.Evaluate(Input(4.0, true, 0.8, kReprojectionReasonCpu), settings);
	out = controller.Evaluate(Input(4.0, false, 0.8), settings);
	out = controller.Evaluate(Input(4.0, false, 0.8), settings);

	EXPECT_EQ(out.classification.pressure, dynamicres::ResolutionPressure::Headroom);
	EXPECT_EQ(out.action, dynamicres::ResolutionAction::Raise);
}

TEST(DynamicResolutionLogic, CpuBoundWithGpuHeadroomRaisesTowardBaseline)
{
	dynamicres::DynamicResolutionController controller;
	const auto settings = FastSettings();

	dynamicres::DynamicResolutionControllerOutput out;
	for (int i = 0; i < 6; ++i) {
		out = controller.Evaluate(Input(4.0, true, 0.8, kReprojectionReasonCpu), settings);
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
		out = controller.Evaluate(Input(9.0, true, 0.8, kReprojectionReasonCpu), settings);
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
		out = controller.Evaluate(Input(4.0, true, 0.8, kReprojectionReasonCpu), settings);
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
		out = controller.Evaluate(Input(4.0, false, 1.0, 0, -1.0, 13.0), settings);
	}

	EXPECT_TRUE(out.classification.cpuStalled);
	EXPECT_EQ(out.classification.pressure, dynamicres::ResolutionPressure::CpuBound);
	EXPECT_EQ(out.action, dynamicres::ResolutionAction::None);
}

TEST(DynamicResolutionLogic, HighAppGpuAloneDoesNotLower)
{
	dynamicres::DynamicResolutionController controller;
	const auto settings = FastSettings();

	// Sustained high app GPU work but the runtime is holding refresh (no reprojection, no dropped
	// frames): a game may safely spend most of the budget, so hold -- do not lower.
	dynamicres::DynamicResolutionControllerOutput out;
	for (int i = 0; i < 6; ++i) {
		out = controller.Evaluate(Input(9.5, false, 1.0), settings);
	}

	EXPECT_NE(out.classification.pressure, dynamicres::ResolutionPressure::GpuBound);
	EXPECT_EQ(out.action, dynamicres::ResolutionAction::None);
}

TEST(DynamicResolutionLogic, MotionSmoothingWithHighGpuLowersToRecoverFullRate)
{
	dynamicres::DynamicResolutionController controller;
	const auto settings = FastSettings();

	// Motion smoothing engaged (half-rate) with high app GPU: lower to climb back to full rate.
	dynamicres::DynamicResolutionControllerOutput out;
	for (int i = 0; i < 6; ++i) {
		out = controller.Evaluate(Input(9.5, true, 1.0, kReprojectionMotion), settings);
	}

	EXPECT_TRUE(out.classification.motionSmoothingActive);
	EXPECT_EQ(out.classification.pressure, dynamicres::ResolutionPressure::GpuBound);
	EXPECT_EQ(out.action, dynamicres::ResolutionAction::Lower);
	EXPECT_LT(out.targetScale, 1.0);
}

TEST(DynamicResolutionLogic, MotionSmoothingWithLowGpuDoesNotLower)
{
	dynamicres::DynamicResolutionController controller;
	const auto settings = FastSettings();

	// Motion smoothing active but GPU is idle: the limiter is not the GPU, so hold scale.
	dynamicres::DynamicResolutionControllerOutput out;
	for (int i = 0; i < 6; ++i) {
		out = controller.Evaluate(Input(4.0, true, 1.0, kReprojectionMotion), settings);
	}

	EXPECT_TRUE(out.classification.motionSmoothingActive);
	EXPECT_NE(out.classification.pressure, dynamicres::ResolutionPressure::GpuBound);
	EXPECT_EQ(out.action, dynamicres::ResolutionAction::None);
}

TEST(DynamicResolutionLogic, RaiseRecoversToBaselineWithinTickBudget)
{
	dynamicres::DynamicResolutionController controller;
	const auto settings = FastSettings();
	double currentScale = 0.60;

	for (int tick = 0; tick < 16 && currentScale < 0.995; ++tick) {
		const auto out = controller.Evaluate(Input(4.0, false, currentScale), settings);
		if (out.action == dynamicres::ResolutionAction::Raise) {
			currentScale = out.targetScale;
			controller.NoteWrite(out.action, currentScale, out.classification, settings);
		}
	}

	EXPECT_NEAR(currentScale, 1.0, 0.005);
}

TEST(DynamicResolutionLogic, WriteClearsSampleWindow)
{
	dynamicres::DynamicResolutionController controller;
	const auto settings = FastSettings();
	dynamicres::DynamicResolutionControllerOutput out;

	for (int i = 0; i < 5; ++i) {
		out = controller.Evaluate(Input(9.5, true, 1.0, kReprojectionReasonGpu), settings);
	}
	ASSERT_EQ(out.action, dynamicres::ResolutionAction::Lower);
	controller.NoteWrite(out.action, out.targetScale, out.classification, settings);

	out = controller.Evaluate(Input(4.0, false, out.targetScale), settings);

	EXPECT_EQ(out.classification.sampleCount, 1);
	EXPECT_EQ(out.classification.pressure, dynamicres::ResolutionPressure::Waiting);
	EXPECT_EQ(out.action, dynamicres::ResolutionAction::None);
}

TEST(DynamicResolutionLogic, ExternalOverrideBacksOffWithoutWriting)
{
	dynamicres::DynamicResolutionController controller;
	const auto settings = FastSettings();
	auto input = Input(9.5, true, 0.9);
	input.externalOverride = true;

	const auto out = controller.Evaluate(input, settings);

	EXPECT_EQ(out.action, dynamicres::ResolutionAction::ExternalOverride);
	EXPECT_DOUBLE_EQ(out.targetScale, 0.9);
}

TEST(DynamicResolutionLogic, NoEffectLoweringRequestsRestore)
{
	dynamicres::DynamicResolutionController controller;
	auto settings = FastSettings();
	settings.noEffectLimit = 1;

	dynamicres::DynamicResolutionControllerOutput out;
	for (int i = 0; i < 5; ++i) {
		out = controller.Evaluate(Input(9.5, true, 1.0, kReprojectionReasonGpu), settings);
	}
	ASSERT_EQ(out.action, dynamicres::ResolutionAction::Lower);
	controller.NoteWrite(out.action, out.targetScale, out.classification, settings);

	for (int i = 0; i < 3; ++i) {
		out = controller.Evaluate(Input(9.5, true, 0.9, kReprojectionReasonGpu), settings);
	}

	EXPECT_EQ(out.action, dynamicres::ResolutionAction::NoEffect);
	EXPECT_DOUBLE_EQ(out.targetScale, 1.0);
}

TEST(DynamicResolutionLogic, SceneStopRequestsRestore)
{
	dynamicres::DynamicResolutionController controller;
	const auto settings = FastSettings();
	auto input = Input(4.0, false, 0.8);
	input.sceneRunning = false;

	const auto out = controller.Evaluate(input, settings);

	EXPECT_EQ(out.action, dynamicres::ResolutionAction::Restore);
	EXPECT_DOUBLE_EQ(out.targetScale, 1.0);
}

TEST(DynamicResolutionLogic, PeakSpikeWithoutDropsDoesNotLower)
{
	dynamicres::DynamicResolutionController controller;
	const auto settings = FastSettings();
	dynamicres::DynamicResolutionControllerOutput out;

	// A single frame peaks past the safety margin, but the runtime still presented every frame (no
	// reprojection, no dropped frames). A stray tail spike must not lower on its own.
	for (int i = 0; i < 4; ++i) {
		out = controller.Evaluate(Input(6.0, false, 1.0, 0, 9.5), settings);
	}

	EXPECT_NE(out.classification.pressure, dynamicres::ResolutionPressure::GpuBound);
	EXPECT_NE(out.action, dynamicres::ResolutionAction::Lower);
}

TEST(DynamicResolutionLogic, OverBudgetShareAloneDoesNotLower)
{
	dynamicres::DynamicResolutionController controller;
	const auto settings = FastSettings();
	dynamicres::DynamicResolutionControllerOutput out;

	// A share of frames render over the budget estimate, but the runtime still held refresh (no
	// reprojection, no dropped frames). Predicted over-budget alone is not a real drop -- do not lower.
	for (int i = 0; i < 4; ++i) {
		out = controller.Evaluate(Input(7.5, false, 1.0, 0, 7.5, 0.0, 20, 90), settings);
	}

	EXPECT_NE(out.classification.pressure, dynamicres::ResolutionPressure::GpuBound);
	EXPECT_NE(out.action, dynamicres::ResolutionAction::Lower);
}

TEST(DynamicResolutionLogic, LowerStepEscalatesWithPeak)
{
	const auto settings = FastSettings();
	dynamicres::DynamicResolutionController flat;
	dynamicres::DynamicResolutionController spiky;
	dynamicres::DynamicResolutionControllerOutput flatOut;
	dynamicres::DynamicResolutionControllerOutput spikyOut;

	// Both are GPU-bound (reprojecting) at the same median; only the tail peak differs.
	for (int i = 0; i < 4; ++i) {
		flatOut = flat.Evaluate(Input(9.0, true, 1.0, kReprojectionReasonGpu, 9.0), settings);
		spikyOut = spiky.Evaluate(Input(9.0, true, 1.0, kReprojectionReasonGpu, 9.9), settings);
	}

	ASSERT_EQ(flatOut.action, dynamicres::ResolutionAction::Lower);
	ASSERT_EQ(spikyOut.action, dynamicres::ResolutionAction::Lower);
	EXPECT_LT(spikyOut.targetScale, flatOut.targetScale); // a worse tail cuts harder
}

TEST(DynamicResolutionLogic, RaiseRecoversDespiteOccasionalOverBudget)
{
	dynamicres::DynamicResolutionController controller;
	const auto settings = FastSettings();
	dynamicres::DynamicResolutionControllerOutput out;

	// Headroom by the median metric with a stray over-budget frame but no reprojection/dropped
	// frames: a predicted over-budget frame must no longer pin the resolution down (the old ratchet).
	for (int i = 0; i < 6; ++i) {
		out = controller.Evaluate(Input(4.0, false, 0.8, 0, 4.0, 0.0, 1, 90), settings);
	}

	EXPECT_EQ(out.classification.pressure, dynamicres::ResolutionPressure::Headroom);
	EXPECT_GT(out.classification.framesOverBudgetTotal, 0);
	EXPECT_EQ(out.action, dynamicres::ResolutionAction::Raise);
	EXPECT_GT(out.targetScale, 0.8);
}

TEST(DynamicResolutionLogic, RaiseWaitsForGpuDropToClear)
{
	dynamicres::DynamicResolutionController controller;
	const auto settings = FastSettings();
	dynamicres::DynamicResolutionControllerOutput out;

	// Below baseline with low median GPU, but a GPU-reason reprojection is still in the window: the
	// GPU is actively failing to hold refresh, so recovery must wait -- it does not raise.
	for (int i = 0; i < 6; ++i) {
		out = controller.Evaluate(Input(7.0, true, 0.8, kReprojectionReasonGpu), settings);
	}

	EXPECT_GT(out.classification.gpuReasonSamples, 0);
	EXPECT_NE(out.action, dynamicres::ResolutionAction::Raise);
}

TEST(DynamicResolutionLogic, RecoversAfterReprojClears)
{
	dynamicres::DynamicResolutionController controller;
	const auto settings = FastSettings();
	dynamicres::DynamicResolutionControllerOutput out;

	// A real GPU-reason reprojection lowers the scale.
	for (int i = 0; i < 5; ++i) {
		out = controller.Evaluate(Input(9.5, true, 1.0, kReprojectionReasonGpu), settings);
	}
	ASSERT_EQ(out.action, dynamicres::ResolutionAction::Lower);
	double scale = out.targetScale;
	controller.NoteWrite(out.action, scale, out.classification, settings);
	ASSERT_LT(scale, 1.0);

	// The drops stop and the scene runs clean: the scale climbs back to baseline (no ratchet).
	bool raised = false;
	for (int tick = 0; tick < 20 && scale < 0.995; ++tick) {
		out = controller.Evaluate(Input(4.0, false, scale), settings);
		if (out.action == dynamicres::ResolutionAction::Raise) {
			raised = true;
			scale = out.targetScale;
			controller.NoteWrite(out.action, scale, out.classification, settings);
		}
	}

	EXPECT_TRUE(raised);
	EXPECT_NEAR(scale, 1.0, 0.005);
}

TEST(DynamicResolutionLogic, SupersamplesAboveBaselineOnDeepHeadroom)
{
	dynamicres::DynamicResolutionController controller;
	const auto settings = FastSettings();
	dynamicres::DynamicResolutionControllerOutput out;

	// Sitting at baseline with deep, clean, sustained headroom: spend it on quality above baseline.
	for (int i = 0; i < 12; ++i) {
		out = controller.Evaluate(Input(3.0, false, 1.0), settings);
	}

	EXPECT_TRUE(out.classification.deepHeadroom);
	EXPECT_EQ(out.action, dynamicres::ResolutionAction::Raise);
	EXPECT_GT(out.targetScale, 1.0);
	EXPECT_LE(out.targetScale, dynamicres::CeilingScale(settings, 1.0));
}

TEST(DynamicResolutionLogic, DoesNotSupersampleWhenAnyFrameOverBudget)
{
	dynamicres::DynamicResolutionController controller;
	const auto settings = FastSettings();
	dynamicres::DynamicResolutionControllerOutput out;

	// Deep median headroom but the tail still clips budget: never raise above baseline.
	for (int i = 0; i < 12; ++i) {
		out = controller.Evaluate(Input(3.0, false, 1.0, 0, 3.0, 0.0, 1, 90), settings);
	}

	EXPECT_GT(out.classification.framesOverBudgetTotal, 0);
	EXPECT_EQ(out.action, dynamicres::ResolutionAction::None);
}

TEST(DynamicResolutionLogic, BacksOffFromAboveBaselineOnGpuPressure)
{
	dynamicres::DynamicResolutionController controller;
	const auto settings = FastSettings();
	dynamicres::DynamicResolutionControllerOutput out;

	// Currently supersampled above baseline (1.30) when GPU load returns: lower immediately.
	for (int i = 0; i < 4; ++i) {
		out = controller.Evaluate(Input(9.5, true, 1.30, kReprojectionReasonGpu), settings);
	}

	EXPECT_EQ(out.classification.pressure, dynamicres::ResolutionPressure::GpuBound);
	EXPECT_EQ(out.action, dynamicres::ResolutionAction::Lower);
	EXPECT_LT(out.targetScale, 1.30);
}

TEST(DynamicResolutionLogic, ApplyQualityPresetSeedsKnobs)
{
	dynamicres::DynamicResolutionSettings settings;
	dynamicres::ApplyQualityPreset(dynamicres::QualityPreset::MaxFps, settings);
	EXPECT_EQ(settings.qualityPreset, dynamicres::QualityPreset::MaxFps);
	EXPECT_DOUBLE_EQ(settings.minScaleFraction, 0.50);
	EXPECT_DOUBLE_EQ(settings.maxScaleFraction, 1.00);
	EXPECT_EQ(settings.lowerRequiredTicks, 1);
	EXPECT_DOUBLE_EQ(settings.headroomGpuBudgetFraction, 0.75);
	EXPECT_EQ(settings.settleTicks, 1);

	dynamicres::ApplyQualityPreset(dynamicres::QualityPreset::Quality, settings);
	EXPECT_EQ(settings.qualityPreset, dynamicres::QualityPreset::Quality);
	EXPECT_DOUBLE_EQ(settings.minScaleFraction, 0.80);
	EXPECT_DOUBLE_EQ(settings.maxScaleFraction, 1.50);
	EXPECT_DOUBLE_EQ(settings.headroomGpuBudgetFraction, 0.85);
	EXPECT_EQ(settings.settleTicks, 3);
}

TEST(DynamicResolutionLogic, ReconcilePresetRederivesStaleNamedPresetKnobs)
{
	// A profile saved by an older build pins the Quality preset but stale recovery knobs.
	dynamicres::DynamicResolutionSettings settings;
	settings.qualityPreset = dynamicres::QualityPreset::Quality;
	settings.headroomGpuBudgetFraction = 0.70;
	settings.settleTicks = 0;

	dynamicres::ReconcilePresetSettings(settings);

	EXPECT_DOUBLE_EQ(settings.headroomGpuBudgetFraction, 0.85);
	EXPECT_EQ(settings.settleTicks, 3);
}

TEST(DynamicResolutionLogic, ReconcilePresetLeavesCustomUntouched)
{
	dynamicres::DynamicResolutionSettings settings;
	settings.qualityPreset = dynamicres::QualityPreset::Custom;
	settings.headroomGpuBudgetFraction = 0.66;
	settings.settleTicks = 0;

	dynamicres::ReconcilePresetSettings(settings);

	EXPECT_DOUBLE_EQ(settings.headroomGpuBudgetFraction, 0.66);
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
