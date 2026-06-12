#include "FaceSignalProcessor.h"

#include "facetracking/UpstreamShapeMap.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace facetracking {
namespace {

static constexpr uint32_t kOursJawOpen = 26;
static constexpr uint32_t kOursMouthClose = 40;
static constexpr uint32_t kOursBrowLowererLeft = 12;
static constexpr uint32_t kOursBrowLowererRight = 13;
static constexpr uint32_t kOursBrowInnerUpLeft = 14;
static constexpr uint32_t kOursBrowInnerUpRight = 15;
static constexpr uint32_t kOursBrowOuterUpLeft = 16;
static constexpr uint32_t kOursBrowOuterUpRight = 17;
static constexpr uint32_t kOursBrowPinchLeft = 18;
static constexpr uint32_t kOursBrowPinchRight = 19;
static constexpr uint32_t kOursLipFunnelUpperLeft = 34;
static constexpr uint32_t kOursLipFunnelUpperRight = 35;
static constexpr uint32_t kOursLipFunnelLowerLeft = 36;
static constexpr uint32_t kOursLipFunnelLowerRight = 37;
static constexpr uint32_t kOursLipPuckerUpperLeft = 38;
static constexpr uint32_t kOursLipPuckerUpperRight = 39;
static constexpr uint32_t kOursMouthSmileLeft = 45;
static constexpr uint32_t kOursMouthSmileRight = 46;
static constexpr uint32_t kOursMouthSadLeft = 47;
static constexpr uint32_t kOursMouthSadRight = 48;
static constexpr uint32_t kOursMouthStretchLeft = 49;
static constexpr uint32_t kOursMouthStretchRight = 50;
static constexpr float kMouthCloseJawScale = 0.60f;
static constexpr float kSmileAssistThreshold = 0.35f;
static constexpr float kSmileAssistMaxJaw = 0.18f;
static constexpr float kIdleMouthJawMin = 0.08f;
static constexpr float kIdleMouthJawMax = 0.28f;
static constexpr float kIdleMouthActivityMax = 0.20f;
static constexpr float kIdleMouthDelaySec = 1.20f;
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

float Strength01(uint8_t strength)
{
	return std::max(0.0f, std::min(100.0f, static_cast<float>(strength))) / 100.0f;
}

float SmoothStep(float edge0, float edge1, float x)
{
	if (edge1 <= edge0) return x >= edge1 ? 1.0f : 0.0f;
	x = std::max(0.0f, std::min(1.0f, (x - edge0) / (edge1 - edge0)));
	return x * x * (3.0f - 2.0f * x);
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

bool HasCorrection(const protocol::FaceTrackingConfig& config, uint8_t flag)
{
	return (config.expression_correction_flags & flag) != 0;
}

uint8_t MouthCorrectionStrength(const protocol::FaceTrackingConfig& config)
{
	return static_cast<uint8_t>(config.expression_correction_strengths & 0xFFu);
}

uint8_t BrowCorrectionStrength(const protocol::FaceTrackingConfig& config)
{
	return static_cast<uint8_t>((config.expression_correction_strengths >> 8) & 0xFFu);
}

float MaxExpr(const protocol::FaceTrackingFrameBody& frame, uint32_t a, uint32_t b)
{
	return std::max(Clamp01(frame.expressions[a]), Clamp01(frame.expressions[b]));
}

void ApplyMouthCloseCompensation(protocol::FaceTrackingFrameBody& frame)
{
	const float jaw = Clamp01(frame.expressions[kOursJawOpen]);
	const float close = Clamp01(frame.expressions[kOursMouthClose]);
	frame.expressions[kOursJawOpen] = std::max(0.0f, jaw - kMouthCloseJawScale * close);
}

void ApplySmileMouthOpenAssist(protocol::FaceTrackingFrameBody& frame, uint8_t strength)
{
	const float amount = Strength01(strength);
	if (amount <= 0.0f) return;

	const float smile = MaxExpr(frame, kOursMouthSmileLeft, kOursMouthSmileRight);
	const float assist = SmoothStep(kSmileAssistThreshold, 1.0f, smile) * kSmileAssistMaxJaw * amount;
	frame.expressions[kOursJawOpen] = std::max(Clamp01(frame.expressions[kOursJawOpen]), assist);
}

float MouthActivity(const protocol::FaceTrackingFrameBody& frame)
{
	float activity = 0.0f;
	activity = std::max(activity, MaxExpr(frame, kOursMouthSmileLeft, kOursMouthSmileRight));
	activity = std::max(activity, MaxExpr(frame, kOursMouthSadLeft, kOursMouthSadRight));
	activity = std::max(activity, MaxExpr(frame, kOursMouthStretchLeft, kOursMouthStretchRight));
	activity = std::max(activity, MaxExpr(frame, kOursLipFunnelUpperLeft, kOursLipFunnelUpperRight));
	activity = std::max(activity, MaxExpr(frame, kOursLipFunnelLowerLeft, kOursLipFunnelLowerRight));
	activity = std::max(activity, MaxExpr(frame, kOursLipPuckerUpperLeft, kOursLipPuckerUpperRight));
	return activity;
}

bool IsIdleMouthOpenCandidate(const protocol::FaceTrackingFrameBody& frame)
{
	const float jaw = Clamp01(frame.expressions[kOursJawOpen]);
	if (jaw < kIdleMouthJawMin || jaw > kIdleMouthJawMax) return false;
	if (Clamp01(frame.expressions[kOursMouthClose]) > kIdleMouthActivityMax) return false;
	return MouthActivity(frame) <= kIdleMouthActivityMax;
}

void ApplyBrowSync(protocol::FaceTrackingFrameBody& frame, uint8_t strength)
{
	if ((frame.flags & 0x1u) == 0) return;

	const float amount = Strength01(strength);
	if (amount <= 0.0f) return;

	auto applySide = [&](float eye_openness, uint32_t lower, uint32_t inner_up, uint32_t outer_up, uint32_t pinch) {
		const float closed = 1.0f - Clamp01(eye_openness);
		const float influence = amount * closed;
		if (influence <= 0.0f) return;

		const float reduce_up = 1.0f - 0.40f * influence;
		frame.expressions[inner_up] = Clamp01(frame.expressions[inner_up] * reduce_up);
		frame.expressions[outer_up] = Clamp01(frame.expressions[outer_up] * reduce_up);

		const float lower_floor = 0.12f * influence;
		const float pinch_floor = 0.06f * influence;
		frame.expressions[lower] = std::max(Clamp01(frame.expressions[lower]), lower_floor);
		frame.expressions[pinch] = std::max(Clamp01(frame.expressions[pinch]), pinch_floor);
	};

	applySide(frame.eye_openness_l, kOursBrowLowererLeft, kOursBrowInnerUpLeft, kOursBrowOuterUpLeft,
	          kOursBrowPinchLeft);
	applySide(frame.eye_openness_r, kOursBrowLowererRight, kOursBrowInnerUpRight, kOursBrowOuterUpRight,
	          kOursBrowPinchRight);
}

void CopyMappedInternalShapesToUpstream(protocol::FaceTrackingFrameBody& frame)
{
	if ((frame.flags & 0x2u) == 0) return;

	for (int upstream = 0; upstream < kUpstreamShapeCount; ++upstream) {
		const int ours = kUpstreamToOurs[upstream];
		if (ours < 0 || ours >= static_cast<int>(protocol::FACETRACKING_EXPRESSION_COUNT)) {
			continue;
		}
		frame.upstream_expressions[upstream] = ClampExpressionOutputSignal(frame.expressions[ours]);
	}
}

void ApplyShapeTuning(protocol::FaceTrackingFrameBody& frame, const protocol::FaceShapeTuningParams* shape_tuning)
{
	if (!shape_tuning) return;

	for (uint32_t i = 0; i < protocol::FACETRACKING_EXPRESSION_COUNT; ++i) {
		const uint16_t percent =
		    std::min<uint16_t>(shape_tuning[i].scale_percent, protocol::FACETRACKING_SHAPE_TUNING_MAX_PERCENT);
		const uint16_t minPercent =
		    std::min<uint16_t>(shape_tuning[i].min_percent, protocol::FACETRACKING_SHAPE_TUNING_MAX_PERCENT);
		const uint16_t maxPercent =
		    std::min<uint16_t>(shape_tuning[i].max_percent, protocol::FACETRACKING_SHAPE_TUNING_MAX_PERCENT);
		const uint16_t lo = std::min(minPercent, maxPercent);
		const uint16_t hi = std::max(minPercent, maxPercent);
		const float scale = static_cast<float>(percent) / 100.0f;
		const float minValue = static_cast<float>(lo) / 100.0f;
		const float maxValue = static_cast<float>(hi) / 100.0f;
		frame.expressions[i] =
		    std::clamp(ClampExpressionOutputSignal(frame.expressions[i] * scale), minValue, maxValue);
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

float FaceSignalProcessor::FrameDeltaSeconds(const protocol::FaceTrackingFrameBody& frame)
{
	if (last_source_hash_ != frame.source_module_uuid_hash) {
		Reset();
		last_source_hash_ = frame.source_module_uuid_hash;
	}

	float dt_sec = kDefaultFrameDtSec;
	if (frame.qpc_sample_time != 0 && last_qpc_sample_time_ != 0 && frame.qpc_sample_time > last_qpc_sample_time_) {
		const LARGE_INTEGER freq = QpcFreq();
		if (freq.QuadPart > 0) {
			dt_sec =
			    static_cast<float>(frame.qpc_sample_time - last_qpc_sample_time_) / static_cast<float>(freq.QuadPart);
		}
	}
	last_qpc_sample_time_ = frame.qpc_sample_time;
	return std::max(0.0f, std::min(kMaxSmoothingGapSec, dt_sec));
}

void FaceSignalProcessor::SmoothScalar(float& value, ScalarFilter& state, uint8_t strength, float dt_sec)
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

void FaceSignalProcessor::SmoothVec3(float value[3], Vec3Filter& state, uint8_t strength, float dt_sec)
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

void FaceSignalProcessor::Apply(protocol::FaceTrackingFrameBody& frame, const protocol::FaceTrackingConfig& config,
                                const protocol::FaceShapeTuningParams* shape_tuning, float* pre_tuning_expressions)
{
	const float dt_sec = FrameDeltaSeconds(frame);

	if ((frame.flags & 0x1u) != 0) {
		SmoothVec3(frame.eye_gaze_l, gaze_l_, config.gaze_smoothing, dt_sec);
		SmoothVec3(frame.eye_gaze_r, gaze_r_, config.gaze_smoothing, dt_sec);
		SmoothScalar(frame.eye_openness_l, openness_l_, config.openness_smoothing, dt_sec);
		SmoothScalar(frame.eye_openness_r, openness_r_, config.openness_smoothing, dt_sec);
	}

	if ((frame.flags & 0x2u) != 0) {
		if (HasCorrection(config, protocol::FACETRACKING_EXPR_CORRECT_MOUTH_CLOSE)) {
			ApplyMouthCloseCompensation(frame);
		}

		if (HasCorrection(config, protocol::FACETRACKING_EXPR_CORRECT_SMILE_OPEN)) {
			ApplySmileMouthOpenAssist(frame, MouthCorrectionStrength(config));
		}

		if (HasCorrection(config, protocol::FACETRACKING_EXPR_CORRECT_IDLE_CLOSE) && frame.qpc_sample_time != 0 &&
		    IsIdleMouthOpenCandidate(frame)) {
			if (idle_mouth_open_since_qpc_ == 0) {
				idle_mouth_open_since_qpc_ = frame.qpc_sample_time;
			}
			const LARGE_INTEGER freq = QpcFreq();
			const float held_sec = (freq.QuadPart > 0 && frame.qpc_sample_time > idle_mouth_open_since_qpc_)
			                           ? static_cast<float>(frame.qpc_sample_time - idle_mouth_open_since_qpc_) /
			                                 static_cast<float>(freq.QuadPart)
			                           : 0.0f;
			if (held_sec >= kIdleMouthDelaySec) {
				const float keep = 1.0f - Strength01(MouthCorrectionStrength(config));
				frame.expressions[kOursJawOpen] = Clamp01(frame.expressions[kOursJawOpen]) * keep;
			}
		}
		else {
			idle_mouth_open_since_qpc_ = 0;
		}

		if (HasCorrection(config, protocol::FACETRACKING_EXPR_CORRECT_BROW_SYNC)) {
			ApplyBrowSync(frame, BrowCorrectionStrength(config));
		}

		if (pre_tuning_expressions) {
			std::memcpy(pre_tuning_expressions, frame.expressions,
			            sizeof(float) * protocol::FACETRACKING_EXPRESSION_COUNT);
		}
		ApplyShapeTuning(frame, shape_tuning);
	}
	else {
		idle_mouth_open_since_qpc_ = 0;
	}

	CopyMappedInternalShapesToUpstream(frame);
}

void FaceSignalProcessor::Reset()
{
	openness_l_ = ScalarFilter{};
	openness_r_ = ScalarFilter{};
	gaze_l_ = Vec3Filter{};
	gaze_r_ = Vec3Filter{};
	last_qpc_sample_time_ = 0;
	idle_mouth_open_since_qpc_ = 0;
}

} // namespace facetracking
