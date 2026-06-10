#pragma once

#include <cstdint>

#include <openvr_driver.h>

namespace skeletal::math {

constexpr uint32_t kFingerBoneCount = 31;
constexpr int kFingersPerHand = 5;
constexpr int kMotionRangeCount = 2;

// WithController and WithoutController submissions interleave on the same
// component handle. A frame observer that treats them as one stream reads
// the pose gap between the two ranges as per-frame motion, so observer
// state must be indexed per range.
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

struct DashboardFrameState
{
	bool initialized = false;
	vr::VRBoneTransform_t previous[kFingerBoneCount] = {};
};

struct DashboardFrameObservation
{
	bool active = false;
	bool seeded = false;
	bool liveFrame = false;
	float maxPosDelta = 0.0f;
	int maxPosDeltaBone = -1;
	float minQuatDot = 1.0f;
};

float Lerpf(float a, float b, float t);
vr::HmdQuaternionf_t SlerpQuat(const vr::HmdQuaternionf_t& a, vr::HmdQuaternionf_t b, float t);
int FingerIndexForBone(uint32_t bone);
float SmoothnessToAlpha(uint8_t smoothness);
FingerFrameResult SmoothFingerFrame(FingerFrameState& state, const vr::VRBoneTransform_t* input, uint32_t count,
                                    int handBase, uint16_t fingerMask, const float alphaPerFinger[kFingersPerHand],
                                    vr::VRBoneTransform_t* output);
DashboardFrameObservation ObserveDashboardFrame(DashboardFrameState& state, bool dashboardActive,
                                                const vr::VRBoneTransform_t* input, uint32_t count);

} // namespace skeletal::math
