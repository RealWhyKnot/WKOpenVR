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
#include "Protocol.h"
#include "facetracking/UpstreamShapeMap.h"

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>

namespace {

constexpr uint32_t kOursJawOpen = 26;
constexpr uint32_t kOursMouthClose = 40;
constexpr uint32_t kUpstreamJawOpen = 22;
constexpr uint32_t kUpstreamMouthClosed = 29;
constexpr uint32_t kUpstreamMouthCornerPullLeft = 57;
constexpr uint32_t kOursMouthSmileLeft = 45;

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
    return base + static_cast<uint64_t>(
        static_cast<double>(freq.QuadPart) * ms / 1000.0);
}

protocol::FaceTrackingConfig MakeConfig(uint8_t gaze_smoothing = 0,
                                        uint8_t openness_smoothing = 0)
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

TEST(FaceSignalProcessor, MouthCloseCompensationUpdatesInternalAndUpstream)
{
    facetracking::FaceSignalProcessor processor;
    protocol::FaceTrackingConfig cfg = MakeConfig();
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
