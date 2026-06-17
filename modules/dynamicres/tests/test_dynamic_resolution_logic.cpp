#include "DynamicResolutionLogic.h"

#include <gtest/gtest.h>

namespace dynamicres = wkopenvr::dynamicres;

namespace {

constexpr uint32_t kReprojectionReasonCpu = 0x01;
constexpr uint32_t kReprojectionReasonGpu = 0x02;
constexpr uint32_t kReprojectionAsync = 0x04;

dynamicres::DynamicResolutionTiming Timing(double appGpuMs, bool unstable, uint32_t reprojectionFlags = 0)
{
	dynamicres::DynamicResolutionTiming t;
	t.frameBudgetMs = 10.0;
	t.preSubmitGpuMs = appGpuMs;
	t.totalRenderGpuMs = appGpuMs + 1.0;
	t.compositorRenderGpuMs = 1.0;
	t.framePresents = unstable ? 2u : 1u;
	t.droppedFrames = unstable ? 1u : 0u;
	t.reprojectionFlags = reprojectionFlags;
	return t;
}

dynamicres::DynamicResolutionControllerInput Input(double appGpuMs, bool unstable, double currentScale = 1.0,
                                                   uint32_t reprojectionFlags = 0)
{
	dynamicres::DynamicResolutionControllerInput input;
	input.timing = Timing(appGpuMs, unstable, reprojectionFlags);
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
	settings.settleTicks = 0;
	settings.noEffectLimit = 2;
	settings.stepFraction = 0.10;
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
	EXPECT_NEAR(out.targetScale, 0.9, 1e-6);
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

TEST(DynamicResolutionLogic, AsyncReprojectionModeAloneDoesNotCountAsUnstable)
{
	dynamicres::DynamicResolutionController controller;
	const auto settings = FastSettings();
	dynamicres::DynamicResolutionControllerOutput out;

	for (int i = 0; i < 4; ++i) {
		out = controller.Evaluate(Input(9.5, false, 1.0, kReprojectionAsync), settings);
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

TEST(DynamicResolutionLogic, StreamingConservativeModeUsesHigherFloorAndSmallerStep)
{
	dynamicres::DynamicResolutionController controller;
	auto settings = FastSettings();
	settings.actUnderStreaming = true;
	settings.minScaleFraction = 0.50;
	settings.streamingMinScaleFraction = 0.80;
	settings.conservativeStreaming = true;

	dynamicres::DynamicResolutionControllerInput input = Input(9.5, true, 0.82);
	input.streamingDetected = true;

	dynamicres::DynamicResolutionControllerOutput out;
	for (int i = 0; i < 5; ++i) {
		input.timing.reprojectionFlags = kReprojectionReasonGpu;
		out = controller.Evaluate(input, settings);
	}

	EXPECT_EQ(out.action, dynamicres::ResolutionAction::Lower);
	EXPECT_NEAR(out.targetScale, 0.80, 1e-6);
}

TEST(DynamicResolutionLogic, StreamingActionIsBlockedByDefault)
{
	dynamicres::DynamicResolutionController controller;
	auto settings = FastSettings();
	settings.actUnderStreaming = false;

	auto input = Input(9.5, true, 1.0, kReprojectionReasonGpu);
	input.streamingDetected = true;

	dynamicres::DynamicResolutionControllerOutput out;
	for (int i = 0; i < 4; ++i) {
		out = controller.Evaluate(input, settings);
	}

	EXPECT_TRUE(out.actingBlocked);
	EXPECT_EQ(out.action, dynamicres::ResolutionAction::None);
	EXPECT_EQ(out.reason, "streaming headset");
}

TEST(DynamicResolutionLogic, StreamingCodecAllowlistCanActWhenStreamingDefaultIsOff)
{
	dynamicres::DynamicResolutionController controller;
	auto settings = FastSettings();
	settings.actUnderStreaming = false;

	auto input = Input(9.5, true, 1.0, kReprojectionReasonGpu);
	input.streamingDetected = true;
	input.streamingCodecAllowsAction = true;

	dynamicres::DynamicResolutionControllerOutput out;
	for (int i = 0; i < 5; ++i) {
		out = controller.Evaluate(input, settings);
	}

	EXPECT_FALSE(out.actingBlocked);
	EXPECT_EQ(out.action, dynamicres::ResolutionAction::Lower);
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

	for (int i = 0; i < 1; ++i) {
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
