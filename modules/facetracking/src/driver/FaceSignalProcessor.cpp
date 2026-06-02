#include "FaceSignalProcessor.h"

#include "facetracking/UpstreamShapeMap.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace facetracking {
namespace {

static constexpr uint32_t kOursJawOpen = 26;
static constexpr uint32_t kOursMouthClose = 40;
static constexpr float kMouthCloseJawScale = 0.60f;
static constexpr float kDefaultFrameDtSec = 1.0f / 120.0f;
static constexpr float kMaxSmoothingGapSec = 0.250f;

float Clamp01(float v)
{
    if (!std::isfinite(v)) return 0.0f;
    return std::max(0.0f, std::min(1.0f, v));
}

float ClampFinite(float v)
{
    return std::isfinite(v) ? v : 0.0f;
}

float SmoothAlpha(uint8_t strength, float dt_sec)
{
    if (strength == 0) return 1.0f;

    float s = static_cast<float>(strength);
    s = std::max(0.0f, std::min(100.0f, s)) / 100.0f;
    dt_sec = std::max(0.0f, std::min(kMaxSmoothingGapSec, dt_sec));

    // Higher slider values mean a longer low-pass time constant. Squaring the
    // slider keeps the lower half useful for light cleanup while preserving a
    // high end for very noisy modules.
    const float tau_sec = (5.0f + 145.0f * s * s) / 1000.0f;
    const float alpha = 1.0f - std::exp(-dt_sec / tau_sec);
    return std::max(0.0f, std::min(1.0f, alpha));
}

float VecLength(const float v[3])
{
    return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

void NormalizeVec3(float v[3])
{
    const float len = VecLength(v);
    if (len < 1e-6f) return;
    const float inv = 1.0f / len;
    v[0] *= inv;
    v[1] *= inv;
    v[2] *= inv;
}

void ApplyMouthCloseCompensation(protocol::FaceTrackingFrameBody &frame)
{
    if ((frame.flags & 0x2u) == 0) return;

    const float jaw = Clamp01(frame.expressions[kOursJawOpen]);
    const float close = Clamp01(frame.expressions[kOursMouthClose]);
    frame.expressions[kOursJawOpen] =
        std::max(0.0f, jaw - kMouthCloseJawScale * close);
}

void CopyMappedInternalShapesToUpstream(protocol::FaceTrackingFrameBody &frame)
{
    if ((frame.flags & 0x2u) == 0) return;

    for (int upstream = 0; upstream < kUpstreamShapeCount; ++upstream) {
        const int ours = kUpstreamToOurs[upstream];
        if (ours < 0 ||
            ours >= static_cast<int>(protocol::FACETRACKING_EXPRESSION_COUNT)) {
            continue;
        }
        frame.upstream_expressions[upstream] = Clamp01(frame.expressions[ours]);
    }
}

} // namespace

LARGE_INTEGER FaceSignalProcessor::QpcFreq() const
{
    if (qpc_freq_.QuadPart == 0) {
        QueryPerformanceFrequency(&qpc_freq_);
    }
    return qpc_freq_;
}

float FaceSignalProcessor::FrameDeltaSeconds(
    const protocol::FaceTrackingFrameBody &frame)
{
    if (last_source_hash_ != frame.source_module_uuid_hash) {
        Reset();
        last_source_hash_ = frame.source_module_uuid_hash;
    }

    float dt_sec = kDefaultFrameDtSec;
    if (frame.qpc_sample_time != 0 && last_qpc_sample_time_ != 0 &&
        frame.qpc_sample_time > last_qpc_sample_time_) {
        const LARGE_INTEGER freq = QpcFreq();
        if (freq.QuadPart > 0) {
            dt_sec = static_cast<float>(frame.qpc_sample_time - last_qpc_sample_time_) /
                     static_cast<float>(freq.QuadPart);
        }
    }
    last_qpc_sample_time_ = frame.qpc_sample_time;
    return std::max(0.0f, std::min(kMaxSmoothingGapSec, dt_sec));
}

void FaceSignalProcessor::SmoothScalar(float &value,
                                       ScalarFilter &state,
                                       uint8_t strength,
                                       float dt_sec)
{
    value = Clamp01(value);
    if (strength == 0 || dt_sec >= kMaxSmoothingGapSec) {
        state.value = value;
        state.initialized = true;
        return;
    }
    if (!state.initialized) {
        state.value = value;
        state.initialized = true;
        return;
    }

    const float alpha = SmoothAlpha(strength, dt_sec);
    state.value += alpha * (value - state.value);
    value = Clamp01(state.value);
}

void FaceSignalProcessor::SmoothVec3(float value[3],
                                     Vec3Filter &state,
                                     uint8_t strength,
                                     float dt_sec)
{
    value[0] = ClampFinite(value[0]);
    value[1] = ClampFinite(value[1]);
    value[2] = ClampFinite(value[2]);
    NormalizeVec3(value);

    if (strength == 0 || dt_sec >= kMaxSmoothingGapSec) {
        std::memcpy(state.value, value, sizeof(state.value));
        state.initialized = true;
        return;
    }
    if (!state.initialized) {
        std::memcpy(state.value, value, sizeof(state.value));
        state.initialized = true;
        return;
    }

    const float alpha = SmoothAlpha(strength, dt_sec);
    for (int i = 0; i < 3; ++i) {
        state.value[i] += alpha * (value[i] - state.value[i]);
        value[i] = state.value[i];
    }
    NormalizeVec3(value);
    std::memcpy(state.value, value, sizeof(state.value));
}

void FaceSignalProcessor::Apply(protocol::FaceTrackingFrameBody &frame,
                                const protocol::FaceTrackingConfig &config)
{
    const float dt_sec = FrameDeltaSeconds(frame);

    if ((frame.flags & 0x1u) != 0) {
        SmoothVec3(frame.eye_gaze_l, gaze_l_, config.gaze_smoothing, dt_sec);
        SmoothVec3(frame.eye_gaze_r, gaze_r_, config.gaze_smoothing, dt_sec);
        SmoothScalar(frame.eye_openness_l, openness_l_,
                     config.openness_smoothing, dt_sec);
        SmoothScalar(frame.eye_openness_r, openness_r_,
                     config.openness_smoothing, dt_sec);
    }

    ApplyMouthCloseCompensation(frame);
    CopyMappedInternalShapesToUpstream(frame);
}

void FaceSignalProcessor::Reset()
{
    openness_l_ = ScalarFilter{};
    openness_r_ = ScalarFilter{};
    gaze_l_ = Vec3Filter{};
    gaze_r_ = Vec3Filter{};
    last_qpc_sample_time_ = 0;
}

} // namespace facetracking
