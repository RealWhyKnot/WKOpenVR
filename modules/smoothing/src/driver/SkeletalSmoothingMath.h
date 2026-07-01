#pragma once

#include <cstdint>

#include <openvr_driver.h>

namespace skeletal::math {

constexpr uint32_t kFingerBoneCount = 31;
constexpr int kFingersPerHand = 5;
constexpr int kMotionRangeCount = 2;

inline int MotionRangeIndex(int motionRange)
{
	return motionRange == static_cast<int>(vr::VRSkeletalMotionRange_WithoutController) ? 1 : 0;
}

inline double ComputeRateHz(uint64_t count, double elapsedSec)
{
	if (elapsedSec <= 0.0) return 0.0;
	return static_cast<double>(count) / elapsedSec;
}

struct FingerFrameState
{
	bool initialized = false;
	bool reseed_pending[kFingersPerHand] = {};
	vr::VRBoneTransform_t previous[kFingerBoneCount] = {};
};

struct FingerFrameResult
{
	bool seeded = false;
	bool reseeded = false;
	bool appliedSmoothing = false;
	float maxPosDelta = 0.0f;
	int maxPosDeltaBone = -1;
	float minQuatDot = 1.0f;
	int minQuatDotBone = -1;
};

// Strongest smoothing (smallest EMA alpha) at smoothness=100. Kept as a named
// constant so both the default overload and any live-tuned caller share it.
constexpr float kDefaultMaxSmoothStrength = 0.95f;

float Lerpf(float a, float b, float t);
vr::HmdQuaternionf_t SlerpQuat(const vr::HmdQuaternionf_t& a, vr::HmdQuaternionf_t b, float t);
int FingerIndexForBone(uint32_t bone);
float SmoothnessToAlpha(uint8_t smoothness);
// Same mapping with a caller-supplied max strength (fraction of full smoothing
// applied at smoothness=100, clamped to [0,1]). maxStrength = 0.95 reproduces
// the single-argument overload exactly.
float SmoothnessToAlpha(uint8_t smoothness, float maxStrength);
FingerFrameResult SmoothFingerFrame(FingerFrameState& state, const vr::VRBoneTransform_t* input, uint32_t count,
                                    int handBase, uint16_t fingerMask, const float alphaPerFinger[kFingersPerHand],
                                    vr::VRBoneTransform_t* output);

} // namespace skeletal::math
