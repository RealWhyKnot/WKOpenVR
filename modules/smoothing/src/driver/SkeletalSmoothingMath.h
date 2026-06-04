#pragma once

#include <cstdint>

#include <openvr_driver.h>

namespace skeletal::math {

constexpr uint32_t kFingerBoneCount = 31;
constexpr int kFingersPerHand = 5;

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

float Lerpf(float a, float b, float t);
vr::HmdQuaternionf_t SlerpQuat(const vr::HmdQuaternionf_t& a, vr::HmdQuaternionf_t b, float t);
int FingerIndexForBone(uint32_t bone);
float SmoothnessToAlpha(uint8_t smoothness);
FingerFrameResult SmoothFingerFrame(FingerFrameState& state, const vr::VRBoneTransform_t* input, uint32_t count,
                                    int handBase, uint16_t fingerMask, const float alphaPerFinger[kFingersPerHand],
                                    vr::VRBoneTransform_t* output);

} // namespace skeletal::math
