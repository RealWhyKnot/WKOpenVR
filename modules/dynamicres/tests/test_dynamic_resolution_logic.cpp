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

	// Mid-budget GPU (between headroom and lower thresholds) with async mode on: async never marks
	// instability, and mid-budget app GPU is neither GPU-bound nor headroom.
	for (int i = 0; i < 4; ++i) {
		out = controller.Evaluate(Input(7.5, false, 1.0, kReprojectionAsync), settings);
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
		out = controller.Evaluate(Input(8.0, true, 0.8, kReprojectionReasonCpu), settings);
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

TEST(DynamicResolutionLogic, HighAppGpuLowersWithoutGpuReprojFlag)
{
	dynamicres::DynamicResolutionController controller;
	const auto settings = FastSettings();

	// Sustained high app GPU work, no reprojection flag yet: lower proactively before frames drop.
	dynamicres::DynamicResolutionControllerOutput out;
	for (int i = 0; i < 6; ++i) {
		out = controller.Evaluate(Input(9.5, false, 1.0), settings);
	}

	EXPECT_EQ(out.classification.pressure, dynamicres::ResolutionPressure::GpuBound);
	EXPECT_EQ(out.action, dynamicres::ResolutionAction::Lower);
	EXPECT_LT(out.targetScale, 1.0);
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

TEST(DynamicResolutionLogic, PeakSpikeOverMarginLowersFast)
{
	dynamicres::DynamicResolutionController controller;
	const auto settings = FastSettings();
	dynamicres::DynamicResolutionControllerOutput out;

	// Median app GPU sits below the lower threshold, but a frame peaks past the safety margin.
	// The tail signal lowers within a single full window (no multi-tick dwell).
	for (int i = 0; i < 3; ++i) {
		out = controller.Evaluate(Input(8.0, false, 1.0, 0, 9.5), settings);
	}

	EXPECT_TRUE(out.classification.gpuOverMargin);
	EXPECT_EQ(out.classification.pressure, dynamicres::ResolutionPressure::GpuBound);
	EXPECT_EQ(out.action, dynamicres::ResolutionAction::Lower);
}

TEST(DynamicResolutionLogic, OverBudgetFrameFractionTriggersLower)
{
	dynamicres::DynamicResolutionController controller;
	const auto settings = FastSettings();
	dynamicres::DynamicResolutionControllerOutput out;

	// Median and peak look fine, but a sustained share of frames render over budget.
	for (int i = 0; i < 4; ++i) {
		out = controller.Evaluate(Input(7.5, false, 1.0, 0, 7.5, 0.0, 20, 90), settings);
	}

	EXPECT_TRUE(out.classification.gpuOverMargin);
	EXPECT_EQ(out.classification.pressure, dynamicres::ResolutionPressure::GpuBound);
	EXPECT_EQ(out.action, dynamicres::ResolutionAction::Lower);
}

TEST(DynamicResolutionLogic, LowerStepEscalatesWithPeak)
{
	const auto settings = FastSettings();
	dynamicres::DynamicResolutionController flat;
	dynamicres::DynamicResolutionController spiky;
	dynamicres::DynamicResolutionControllerOutput flatOut;
	dynamicres::DynamicResolutionControllerOutput spikyOut;

	for (int i = 0; i < 4; ++i) {
		flatOut = flat.Evaluate(Input(9.0, false, 1.0, 0, 9.0), settings);
		spikyOut = spiky.Evaluate(Input(9.0, false, 1.0, 0, 9.9), settings);
	}

	ASSERT_EQ(flatOut.action, dynamicres::ResolutionAction::Lower);
	ASSERT_EQ(spikyOut.action, dynamicres::ResolutionAction::Lower);
	EXPECT_LT(spikyOut.targetScale, flatOut.targetScale); // a worse tail cuts harder
}

TEST(DynamicResolutionLogic, RaiseBlockedByRecentOverBudget)
{
	dynamicres::DynamicResolutionController controller;
	const auto settings = FastSettings();
	dynamicres::DynamicResolutionControllerOutput out;

	// Headroom by the median metric, but a few frames went over budget: do not raise yet.
	for (int i = 0; i < 6; ++i) {
		out = controller.Evaluate(Input(4.0, false, 0.8, 0, 4.0, 0.0, 1, 90), settings);
	}

	EXPECT_EQ(out.classification.pressure, dynamicres::ResolutionPressure::Headroom);
	EXPECT_GT(out.classification.framesOverBudgetTotal, 0);
	EXPECT_EQ(out.action, dynamicres::ResolutionAction::None);
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

	dynamicres::ApplyQualityPreset(dynamicres::QualityPreset::Quality, settings);
	EXPECT_EQ(settings.qualityPreset, dynamicres::QualityPreset::Quality);
	EXPECT_DOUBLE_EQ(settings.minScaleFraction, 0.80);
	EXPECT_DOUBLE_EQ(settings.maxScaleFraction, 1.50);
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
