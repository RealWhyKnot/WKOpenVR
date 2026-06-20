// Unit tests for facetracking::FaceSignalProcessor.
//
// Coverage:
//   - Mapped upstream VRCFT slots are overwritten from the internal 63-slot
//     frame after calibration/correction.
//   - JawOpen is reduced by MouthClosed consistently in internal and upstream
//     representations.
//   - Existing gaze/openness smoothing sliders affect the driver hot path with
//     frame-time-aware behaviour.

#include "FaceSignalProcessor.h"
#include "Profiles.h"
#include "Protocol.h"
#include "facetracking/UpstreamShapeMap.h"

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>

namespace {

constexpr uint32_t kOursJawOpen = 26;
constexpr uint32_t kOursMouthClose = 40;
constexpr uint32_t kOursBrowLowererLeft = 12;
constexpr uint32_t kOursBrowInnerUpLeft = 14;
constexpr uint32_t kOursBrowOuterUpLeft = 16;
constexpr uint32_t kOursBrowPinchLeft = 18;
constexpr uint32_t kUpstreamJawOpen = 22;
constexpr uint32_t kUpstreamMouthClosed = 29;
constexpr uint32_t kUpstreamMouthCornerPullLeft = 57;
constexpr uint32_t kOursMouthSmileLeft = 45;
constexpr uint32_t kOursMouthSmileRight = 46;

uint16_t PackStrengths(uint8_t mouth, uint8_t brow)
{
	return static_cast<uint16_t>((static_cast<uint16_t>(brow) << 8) | mouth);
}

uint64_t QpcNow()
{
	LARGE_INTEGER now{};
	QueryPerformanceCounter(&now);
	return static_cast<uint64_t>(now.QuadPart);
}

uint64_t QpcAfterMs(uint64_t base, double ms)
{
	LARGE_INTEGER freq{};
	QueryPerformanceFrequency(&freq);
	return base + static_cast<uint64_t>(static_cast<double>(freq.QuadPart) * ms / 1000.0);
}

protocol::FaceTrackingConfig MakeConfig(uint8_t gaze_smoothing = 0, uint8_t openness_smoothing = 0)
{
	protocol::FaceTrackingConfig cfg{};
	cfg.master_enabled = 1;
	cfg.gaze_smoothing = gaze_smoothing;
	cfg.openness_smoothing = openness_smoothing;
	return cfg;
}

protocol::FaceTrackingFrameBody MakeExpressionFrame()
{
	protocol::FaceTrackingFrameBody frame{};
	frame.qpc_sample_time = QpcNow();
	frame.flags = 0x2;
	return frame;
}

protocol::FaceTrackingFrameBody MakeEyeFrame(uint64_t qpc, float open)
{
	protocol::FaceTrackingFrameBody frame{};
	frame.qpc_sample_time = qpc;
	frame.flags = 0x1;
	frame.eye_openness_l = open;
	frame.eye_openness_r = open;
	frame.eye_gaze_l[0] = 0.0f;
	frame.eye_gaze_l[1] = 0.0f;
	frame.eye_gaze_l[2] = -1.0f;
	frame.eye_gaze_r[0] = 0.0f;
	frame.eye_gaze_r[1] = 0.0f;
	frame.eye_gaze_r[2] = -1.0f;
	return frame;
}

std::array<protocol::FaceShapeTuningParams, protocol::FACETRACKING_EXPRESSION_COUNT> MakeDefaultShapeTuning()
{
	std::array<protocol::FaceShapeTuningParams, protocol::FACETRACKING_EXPRESSION_COUNT> tuning{};
	for (auto& shape : tuning) {
		shape.scale_percent = protocol::FACETRACKING_SHAPE_TUNING_DEFAULT_PERCENT;
		shape.min_percent = protocol::FACETRACKING_SHAPE_TUNING_DEFAULT_MIN_PERCENT;
		shape.max_percent = protocol::FACETRACKING_SHAPE_TUNING_DEFAULT_MAX_PERCENT;
	}
	return tuning;
}

float Length3(const float v[3])
{
	return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

} // namespace

TEST(FaceSignalProcessor, MappedInternalValuesOverwriteRawUpstreamSlots)
{
	facetracking::FaceSignalProcessor processor;
	protocol::FaceTrackingConfig cfg = MakeConfig();
	protocol::FaceTrackingFrameBody frame = MakeExpressionFrame();

	frame.expressions[kOursJawOpen] = 0.25f;
	frame.expressions[kOursMouthClose] = 0.0f;
	frame.expressions[kOursMouthSmileLeft] = 0.40f;
	frame.upstream_expressions[kUpstreamJawOpen] = 0.90f;
	frame.upstream_expressions[kUpstreamMouthCornerPullLeft] = 0.10f;

	processor.Apply(frame, cfg);

	EXPECT_NEAR(frame.upstream_expressions[kUpstreamJawOpen], 0.25f, 1e-6f);
	EXPECT_NEAR(frame.upstream_expressions[kUpstreamMouthCornerPullLeft], 0.40f, 1e-6f);
}

TEST(FaceSignalProcessor, ShapeTuningUnderextendsInternalAndUpstreamSlots)
{
	facetracking::FaceSignalProcessor processor;
	protocol::FaceTrackingConfig cfg = MakeConfig();
	protocol::FaceTrackingFrameBody frame = MakeExpressionFrame();
	auto tuning = MakeDefaultShapeTuning();
	tuning[kOursMouthSmileLeft].scale_percent = 60;

	frame.expressions[kOursMouthSmileLeft] = 1.0f;
	frame.upstream_expressions[kUpstreamMouthCornerPullLeft] = 1.0f;

	processor.Apply(frame, cfg, tuning.data());

	EXPECT_NEAR(frame.expressions[kOursMouthSmileLeft], 0.60f, 1e-6f);
	EXPECT_NEAR(frame.upstream_expressions[kUpstreamMouthCornerPullLeft], 0.60f, 1e-6f);
}

TEST(FaceSignalProcessor, ShapeTuningBoostsInternalAndUpstreamSlotsWithinOutputRange)
{
	facetracking::FaceSignalProcessor processor;
	protocol::FaceTrackingConfig cfg = MakeConfig();
	protocol::FaceTrackingFrameBody frame = MakeExpressionFrame();
	auto tuning = MakeDefaultShapeTuning();
	tuning[kOursMouthSmileLeft].scale_percent = 150;

	frame.expressions[kOursMouthSmileLeft] = 0.40f;
	frame.upstream_expressions[kUpstreamMouthCornerPullLeft] = 0.40f;

	processor.Apply(frame, cfg, tuning.data());

	EXPECT_NEAR(frame.expressions[kOursMouthSmileLeft], 0.60f, 1e-6f);
	EXPECT_NEAR(frame.upstream_expressions[kUpstreamMouthCornerPullLeft], 0.60f, 1e-6f);
}

TEST(FaceSignalProcessor, ShapeTuningClampsOutputAtValidSignalMaximum)
{
	facetracking::FaceSignalProcessor processor;
	protocol::FaceTrackingConfig cfg = MakeConfig();
	protocol::FaceTrackingFrameBody frame = MakeExpressionFrame();
	auto tuning = MakeDefaultShapeTuning();
	tuning[kOursMouthSmileLeft].scale_percent = 500;

	frame.expressions[kOursMouthSmileLeft] = 1.0f;

	processor.Apply(frame, cfg, tuning.data());

	EXPECT_NEAR(frame.expressions[kOursMouthSmileLeft], 1.0f, 1e-6f);
	EXPECT_NEAR(frame.upstream_expressions[kUpstreamMouthCornerPullLeft], 1.0f, 1e-6f);
}

TEST(FaceSignalProcessor, ShapeTuningCapsOutputAtConfiguredMaximum)
{
	facetracking::FaceSignalProcessor processor;
	protocol::FaceTrackingConfig cfg = MakeConfig();
	protocol::FaceTrackingFrameBody frame = MakeExpressionFrame();
	auto tuning = MakeDefaultShapeTuning();
	tuning[kOursMouthSmileLeft].scale_percent = 200;
	tuning[kOursMouthSmileLeft].max_percent = 70;

	frame.expressions[kOursMouthSmileLeft] = 1.0f;
	frame.upstream_expressions[kUpstreamMouthCornerPullLeft] = 1.0f;

	processor.Apply(frame, cfg, tuning.data());

	EXPECT_NEAR(frame.expressions[kOursMouthSmileLeft], 0.70f, 1e-6f);
	EXPECT_NEAR(frame.upstream_expressions[kUpstreamMouthCornerPullLeft], 0.70f, 1e-6f);
}

TEST(FaceSignalProcessor, ShapeTuningRaisesOutputAtConfiguredMinimum)
{
	facetracking::FaceSignalProcessor processor;
	protocol::FaceTrackingConfig cfg = MakeConfig();
	protocol::FaceTrackingFrameBody frame = MakeExpressionFrame();
	auto tuning = MakeDefaultShapeTuning();
	tuning[kOursMouthSmileLeft].scale_percent = 0;
	tuning[kOursMouthSmileLeft].min_percent = 25;
	tuning[kOursMouthSmileLeft].max_percent = 80;

	frame.expressions[kOursMouthSmileLeft] = 1.0f;

	processor.Apply(frame, cfg, tuning.data());

	EXPECT_NEAR(frame.expressions[kOursMouthSmileLeft], 0.25f, 1e-6f);
	EXPECT_NEAR(frame.upstream_expressions[kUpstreamMouthCornerPullLeft], 0.25f, 1e-6f);
}

TEST(FaceSignalProcessor, ShapeTuningHighConfiguredBoundsClampToValidSignalMaximum)
{
	facetracking::FaceSignalProcessor processor;
	protocol::FaceTrackingConfig cfg = MakeConfig();
	protocol::FaceTrackingFrameBody frame = MakeExpressionFrame();
	auto tuning = MakeDefaultShapeTuning();
	tuning[kOursMouthSmileLeft].scale_percent = 200;
	tuning[kOursMouthSmileLeft].min_percent = 200;
	tuning[kOursMouthSmileLeft].max_percent = 200;

	frame.expressions[kOursMouthSmileLeft] = 0.25f;

	processor.Apply(frame, cfg, tuning.data());

	EXPECT_NEAR(frame.expressions[kOursMouthSmileLeft], 1.0f, 1e-6f);
	EXPECT_NEAR(frame.upstream_expressions[kUpstreamMouthCornerPullLeft], 1.0f, 1e-6f);
}

TEST(FaceSignalProcessor, ClampsAllExpressionOutputsBeforePublish)
{
	facetracking::FaceSignalProcessor processor;
	protocol::FaceTrackingConfig cfg = MakeConfig();
	protocol::FaceTrackingFrameBody frame = MakeExpressionFrame();

	frame.expressions[kOursJawOpen] = 2.5f;
	frame.expressions[kOursMouthSmileLeft] = -0.5f;
	frame.upstream_expressions[12] = 2.5f; // Unmapped upstream slot.

	processor.Apply(frame, cfg);

	EXPECT_FLOAT_EQ(frame.expressions[kOursJawOpen], 1.0f);
	EXPECT_FLOAT_EQ(frame.expressions[kOursMouthSmileLeft], 0.0f);
	EXPECT_FLOAT_EQ(frame.upstream_expressions[kUpstreamJawOpen], 1.0f);
	EXPECT_FLOAT_EQ(frame.upstream_expressions[kUpstreamMouthCornerPullLeft], 0.0f);
	EXPECT_FLOAT_EQ(frame.upstream_expressions[12], 1.0f);
}

TEST(FaceSignalProcessor, MouthCloseCompensationIsDisabledByDefault)
{
	facetracking::FaceSignalProcessor processor;
	protocol::FaceTrackingConfig cfg = MakeConfig();
	protocol::FaceTrackingFrameBody frame = MakeExpressionFrame();

	frame.expressions[kOursJawOpen] = 0.80f;
	frame.expressions[kOursMouthClose] = 0.50f;
	frame.upstream_expressions[kUpstreamJawOpen] = 0.10f;

	processor.Apply(frame, cfg);

	EXPECT_NEAR(frame.expressions[kOursJawOpen], 0.80f, 1e-6f);
	EXPECT_NEAR(frame.upstream_expressions[kUpstreamJawOpen], 0.80f, 1e-6f);
}

TEST(FaceSignalProcessor, MouthCloseCompensationUpdatesInternalAndUpstream)
{
	facetracking::FaceSignalProcessor processor;
	protocol::FaceTrackingConfig cfg = MakeConfig();
	cfg.expression_correction_flags = protocol::FACETRACKING_EXPR_CORRECT_MOUTH_CLOSE;
	protocol::FaceTrackingFrameBody frame = MakeExpressionFrame();

	frame.expressions[kOursJawOpen] = 0.80f;
	frame.expressions[kOursMouthClose] = 0.50f;
	frame.upstream_expressions[kUpstreamJawOpen] = 0.80f;
	frame.upstream_expressions[kUpstreamMouthClosed] = 0.50f;

	processor.Apply(frame, cfg);

	EXPECT_NEAR(frame.expressions[kOursJawOpen], 0.50f, 1e-6f);
	EXPECT_NEAR(frame.upstream_expressions[kUpstreamJawOpen], 0.50f, 1e-6f);
	EXPECT_NEAR(frame.upstream_expressions[kUpstreamMouthClosed], 0.50f, 1e-6f);
}

TEST(FaceSignalProcessor, SmileMouthOpenAssistIsOptIn)
{
	facetracking::FaceSignalProcessor processor;
	protocol::FaceTrackingConfig cfg = MakeConfig();
	protocol::FaceTrackingFrameBody frame = MakeExpressionFrame();

	frame.expressions[kOursJawOpen] = 0.01f;
	frame.expressions[kOursMouthSmileLeft] = 1.0f;
	frame.expressions[kOursMouthSmileRight] = 1.0f;

	processor.Apply(frame, cfg);
	EXPECT_NEAR(frame.expressions[kOursJawOpen], 0.01f, 1e-6f);

	cfg.expression_correction_flags = protocol::FACETRACKING_EXPR_CORRECT_SMILE_OPEN;
	cfg.expression_correction_strengths = PackStrengths(100, 0);
	frame = MakeExpressionFrame();
	frame.expressions[kOursJawOpen] = 0.01f;
	frame.expressions[kOursMouthSmileLeft] = 1.0f;
	frame.expressions[kOursMouthSmileRight] = 1.0f;

	processor.Apply(frame, cfg);
	EXPECT_NEAR(frame.expressions[kOursJawOpen], 0.18f, 1e-5f);
	EXPECT_NEAR(frame.upstream_expressions[kUpstreamJawOpen], 0.18f, 1e-5f);
}

TEST(FaceSignalProcessor, IdleMouthAutoCloseRequiresOptInAndHoldTime)
{
	facetracking::FaceSignalProcessor processor;
	protocol::FaceTrackingConfig cfg = MakeConfig();
	cfg.expression_correction_flags = protocol::FACETRACKING_EXPR_CORRECT_IDLE_CLOSE;
	cfg.expression_correction_strengths = PackStrengths(100, 0);
	const uint64_t base = QpcNow();

	protocol::FaceTrackingFrameBody frame = MakeExpressionFrame();
	frame.qpc_sample_time = base;
	frame.expressions[kOursJawOpen] = 0.16f;
	processor.Apply(frame, cfg);
	EXPECT_NEAR(frame.expressions[kOursJawOpen], 0.16f, 1e-6f);

	frame = MakeExpressionFrame();
	frame.qpc_sample_time = QpcAfterMs(base, 1300.0);
	frame.expressions[kOursJawOpen] = 0.16f;
	processor.Apply(frame, cfg);

	EXPECT_NEAR(frame.expressions[kOursJawOpen], 0.0f, 1e-6f);
	EXPECT_NEAR(frame.upstream_expressions[kUpstreamJawOpen], 0.0f, 1e-6f);
}

TEST(FaceSignalProcessor, EyelidBrowSyncRequiresOptIn)
{
	facetracking::FaceSignalProcessor processor;
	protocol::FaceTrackingConfig cfg = MakeConfig();
	protocol::FaceTrackingFrameBody frame = MakeExpressionFrame();
	frame.flags = 0x3;
	frame.eye_openness_l = 0.0f;
	frame.expressions[kOursBrowInnerUpLeft] = 0.80f;
	frame.expressions[kOursBrowOuterUpLeft] = 0.70f;

	processor.Apply(frame, cfg);
	EXPECT_NEAR(frame.expressions[kOursBrowInnerUpLeft], 0.80f, 1e-6f);

	cfg.expression_correction_flags = protocol::FACETRACKING_EXPR_CORRECT_BROW_SYNC;
	cfg.expression_correction_strengths = PackStrengths(0, 100);
	frame = MakeExpressionFrame();
	frame.flags = 0x3;
	frame.eye_openness_l = 0.0f;
	frame.expressions[kOursBrowInnerUpLeft] = 0.80f;
	frame.expressions[kOursBrowOuterUpLeft] = 0.70f;

	processor.Apply(frame, cfg);

	EXPECT_NEAR(frame.expressions[kOursBrowInnerUpLeft], 0.48f, 1e-6f);
	EXPECT_NEAR(frame.expressions[kOursBrowOuterUpLeft], 0.42f, 1e-6f);
	EXPECT_NEAR(frame.expressions[kOursBrowLowererLeft], 0.12f, 1e-6f);
	EXPECT_NEAR(frame.expressions[kOursBrowPinchLeft], 0.06f, 1e-6f);
}

TEST(FaceSignalProcessor, UnmappedUpstreamShapesRemainRaw)
{
	facetracking::FaceSignalProcessor processor;
	protocol::FaceTrackingConfig cfg = MakeConfig();
	protocol::FaceTrackingFrameBody frame = MakeExpressionFrame();

	frame.upstream_expressions[12] = 0.72f; // NasalDilationRight, unmapped.

	processor.Apply(frame, cfg);

	EXPECT_NEAR(frame.upstream_expressions[12], 0.72f, 1e-6f);
}

TEST(FaceSignalProcessor, OpennessSmoothingStrengthZeroTracksRaw)
{
	facetracking::FaceSignalProcessor processor;
	protocol::FaceTrackingConfig cfg = MakeConfig(0, 0);
	const uint64_t base = QpcNow();

	protocol::FaceTrackingFrameBody frame = MakeEyeFrame(base, 0.0f);
	processor.Apply(frame, cfg);
	EXPECT_FLOAT_EQ(frame.eye_openness_l, 0.0f);

	frame = MakeEyeFrame(QpcAfterMs(base, 8.0), 1.0f);
	processor.Apply(frame, cfg);
	EXPECT_FLOAT_EQ(frame.eye_openness_l, 1.0f);
	EXPECT_FLOAT_EQ(frame.eye_openness_r, 1.0f);
}

TEST(FaceSignalProcessor, OpennessSmoothingConvergesToRawTarget)
{
	facetracking::FaceSignalProcessor processor;
	protocol::FaceTrackingConfig cfg = MakeConfig(0, 100);
	const uint64_t base = QpcNow();

	protocol::FaceTrackingFrameBody frame = MakeEyeFrame(base, 0.0f);
	processor.Apply(frame, cfg);

	for (int i = 1; i <= 240; ++i) {
		frame = MakeEyeFrame(QpcAfterMs(base, i * 8.0), 1.0f);
		processor.Apply(frame, cfg);
	}

	EXPECT_GT(frame.eye_openness_l, 0.99f);
	EXPECT_GT(frame.eye_openness_r, 0.99f);
}

TEST(FaceSignalProcessor, OpennessSmoothingUsesFrameDelta)
{
	protocol::FaceTrackingConfig cfg = MakeConfig(0, 100);
	const uint64_t base = QpcNow();

	facetracking::FaceSignalProcessor shortStep;
	protocol::FaceTrackingFrameBody shortFrame = MakeEyeFrame(base, 0.0f);
	shortStep.Apply(shortFrame, cfg);
	shortFrame = MakeEyeFrame(QpcAfterMs(base, 8.0), 1.0f);
	shortStep.Apply(shortFrame, cfg);

	facetracking::FaceSignalProcessor longStep;
	protocol::FaceTrackingFrameBody longFrame = MakeEyeFrame(base, 0.0f);
	longStep.Apply(longFrame, cfg);
	longFrame = MakeEyeFrame(QpcAfterMs(base, 33.0), 1.0f);
	longStep.Apply(longFrame, cfg);

	EXPECT_GT(longFrame.eye_openness_l, shortFrame.eye_openness_l);
	EXPECT_LT(shortFrame.eye_openness_l, 1.0f);
}

TEST(FaceSignalProcessor, GazeSmoothingKeepsUnitLength)
{
	facetracking::FaceSignalProcessor processor;
	protocol::FaceTrackingConfig cfg = MakeConfig(100, 0);
	const uint64_t base = QpcNow();

	protocol::FaceTrackingFrameBody frame = MakeEyeFrame(base, 1.0f);
	processor.Apply(frame, cfg);

	frame = MakeEyeFrame(QpcAfterMs(base, 8.0), 1.0f);
	frame.eye_gaze_l[0] = 1.0f;
	frame.eye_gaze_l[1] = 0.0f;
	frame.eye_gaze_l[2] = 0.0f;
	processor.Apply(frame, cfg);

	EXPECT_NEAR(Length3(frame.eye_gaze_l), 1.0f, 1e-5f);
}

TEST(FaceTrackingProfileDefaults, PreferenceCorrectionsAreOptIn)
{
	FacetrackingProfile profile;

	EXPECT_FALSE(profile.eyelid_sync_enabled);
	EXPECT_EQ(profile.eyelid_sync_mode, protocol::FACETRACKING_EYELID_SYNC_MOST_CLOSED);
	EXPECT_EQ(profile.gaze_smoothing, 0);
	EXPECT_EQ(profile.openness_smoothing, 0);
	EXPECT_FALSE(profile.mouth_close_compensation_enabled);
	EXPECT_FALSE(profile.smile_mouth_open_assist_enabled);
	EXPECT_FALSE(profile.idle_mouth_auto_close_enabled);
	EXPECT_FALSE(profile.eyelid_brow_sync_enabled);
}
