#include "SkeletalSmoothingMath.h"
#include "ServerTrackedDeviceProviderConfigPacking.h"

#include <gtest/gtest.h>

#include <cstdint>

namespace {

using skeletal::math::FingerFrameState;
using skeletal::math::kFingerBoneCount;
using skeletal::math::kFingersPerHand;

constexpr float kEpsilon = 1.0e-5f;

vr::VRBoneTransform_t MakeBone(float x, float y, float z)
{
	vr::VRBoneTransform_t bone{};
	bone.position.v[0] = x;
	bone.position.v[1] = y;
	bone.position.v[2] = z;
	bone.position.v[3] = 1.0f;
	bone.orientation.w = 1.0f;
	bone.orientation.x = 0.0f;
	bone.orientation.y = 0.0f;
	bone.orientation.z = 0.0f;
	return bone;
}

void MakeFrame(vr::VRBoneTransform_t (&frame)[kFingerBoneCount], float base)
{
	for (uint32_t i = 0; i < kFingerBoneCount; ++i) {
		const float offset = static_cast<float>(i) * 0.001f;
		frame[i] = MakeBone(base + offset, base + offset * 2.0f, base + offset * 3.0f);
	}
}

void ExpectBoneNear(const vr::VRBoneTransform_t& actual, const vr::VRBoneTransform_t& expected)
{
	EXPECT_NEAR(actual.position.v[0], expected.position.v[0], kEpsilon);
	EXPECT_NEAR(actual.position.v[1], expected.position.v[1], kEpsilon);
	EXPECT_NEAR(actual.position.v[2], expected.position.v[2], kEpsilon);
	EXPECT_NEAR(actual.position.v[3], expected.position.v[3], kEpsilon);
	EXPECT_NEAR(actual.orientation.w, expected.orientation.w, kEpsilon);
	EXPECT_NEAR(actual.orientation.x, expected.orientation.x, kEpsilon);
	EXPECT_NEAR(actual.orientation.y, expected.orientation.y, kEpsilon);
	EXPECT_NEAR(actual.orientation.z, expected.orientation.z, kEpsilon);
}

void FillAlpha(float (&alpha)[kFingersPerHand], float value)
{
	for (float& a : alpha) {
		a = value;
	}
}

uint16_t AllButFinger(int bit)
{
	return static_cast<uint16_t>(protocol::kAllFingersMask & ~(1u << bit));
}

} // namespace

TEST(SmoothnessToAlpha, MapsRangeAndStaysMonotonic)
{
	EXPECT_FLOAT_EQ(skeletal::math::SmoothnessToAlpha(0), 1.0f);
	EXPECT_NEAR(skeletal::math::SmoothnessToAlpha(100), 0.05f, kEpsilon);

	const float low = skeletal::math::SmoothnessToAlpha(25);
	const float mid = skeletal::math::SmoothnessToAlpha(50);
	const float high = skeletal::math::SmoothnessToAlpha(75);
	EXPECT_GT(low, mid);
	EXPECT_GT(mid, high);
	EXPECT_GT(mid, skeletal::math::SmoothnessToAlpha(100));
	EXPECT_LT(mid, skeletal::math::SmoothnessToAlpha(0));
}

TEST(FingerIndexForBone, MapsOpenVrHandBones)
{
	EXPECT_EQ(skeletal::math::FingerIndexForBone(0), -1);
	EXPECT_EQ(skeletal::math::FingerIndexForBone(1), -1);
	EXPECT_EQ(skeletal::math::FingerIndexForBone(2), 0);
	EXPECT_EQ(skeletal::math::FingerIndexForBone(5), 0);
	EXPECT_EQ(skeletal::math::FingerIndexForBone(6), 1);
	EXPECT_EQ(skeletal::math::FingerIndexForBone(10), 1);
	EXPECT_EQ(skeletal::math::FingerIndexForBone(11), 2);
	EXPECT_EQ(skeletal::math::FingerIndexForBone(15), 2);
	EXPECT_EQ(skeletal::math::FingerIndexForBone(16), 3);
	EXPECT_EQ(skeletal::math::FingerIndexForBone(20), 3);
	EXPECT_EQ(skeletal::math::FingerIndexForBone(21), 4);
	EXPECT_EQ(skeletal::math::FingerIndexForBone(25), 4);
	EXPECT_EQ(skeletal::math::FingerIndexForBone(26), -1);
	EXPECT_EQ(skeletal::math::FingerIndexForBone(30), -1);
}

TEST(FingerSmoothingConfigPacking, RoundTripsHeaderAndPerFingerValues)
{
	protocol::FingerSmoothingConfig cfg{};
	cfg.master_enabled = true;
	cfg.smoothness = 43;
	cfg.finger_mask = 0x02A5;
	for (uint8_t i = 0; i < 10; ++i) {
		cfg.per_finger_smoothness[i] = static_cast<uint8_t>(10 + i);
	}
	cfg._reserved = 99;
	cfg._reserved2[0] = 98;
	cfg._reserved2[1] = 97;

	const auto unpacked =
	    pairdriver::UnpackFingerSmoothing(pairdriver::PackFingerHeader(cfg), pairdriver::PackFingerLow(cfg));

	EXPECT_TRUE(unpacked.master_enabled);
	EXPECT_EQ(unpacked.smoothness, 43);
	EXPECT_EQ(unpacked.finger_mask, 0x02A5);
	for (uint8_t i = 0; i < 10; ++i) {
		EXPECT_EQ(unpacked.per_finger_smoothness[i], static_cast<uint8_t>(10 + i));
	}
	EXPECT_EQ(unpacked._reserved, 0);
	EXPECT_EQ(unpacked._reserved2[0], 0);
	EXPECT_EQ(unpacked._reserved2[1], 0);
}

TEST(FingerSmoothingConfigPacking, ComputesReseedBitsOnlyWhenFingerBecomesSmoothed)
{
	protocol::FingerSmoothingConfig prev{};
	protocol::FingerSmoothingConfig next{};
	next.master_enabled = true;
	next.smoothness = 50;
	next.finger_mask = protocol::kAllFingersMask;
	EXPECT_EQ(pairdriver::ComputeFingerSmoothingReseedBits(prev, next), protocol::kAllFingersMask);

	prev = next;
	next.smoothness = 0;
	EXPECT_EQ(pairdriver::ComputeFingerSmoothingReseedBits(prev, next), 0);

	prev = {};
	next = {};
	next.master_enabled = true;
	next.smoothness = 0;
	next.finger_mask = protocol::kAllFingersMask;
	next.per_finger_smoothness[6] = 80;
	EXPECT_EQ(pairdriver::ComputeFingerSmoothingReseedBits(prev, next), static_cast<uint16_t>(1u << 6));

	next.finger_mask = AllButFinger(6);
	EXPECT_EQ(pairdriver::ComputeFingerSmoothingReseedBits(prev, next), 0);
}

TEST(SmoothFingerFrame, GlobalZeroPassesThroughAfterSeed)
{
	vr::VRBoneTransform_t first[kFingerBoneCount];
	vr::VRBoneTransform_t second[kFingerBoneCount];
	vr::VRBoneTransform_t output[kFingerBoneCount];
	MakeFrame(first, 0.0f);
	MakeFrame(second, 1.0f);
	float alpha[kFingersPerHand];
	FillAlpha(alpha, 1.0f);

	FingerFrameState state{};
	const auto seed =
	    skeletal::math::SmoothFingerFrame(state, first, kFingerBoneCount, 0, protocol::kAllFingersMask, alpha, output);
	EXPECT_TRUE(seed.seeded);
	EXPECT_TRUE(state.initialized);

	const auto result =
	    skeletal::math::SmoothFingerFrame(state, second, kFingerBoneCount, 0, protocol::kAllFingersMask, alpha, output);
	EXPECT_FALSE(result.appliedSmoothing);
	EXPECT_FALSE(result.reseeded);
	for (uint32_t i = 0; i < kFingerBoneCount; ++i) {
		ExpectBoneNear(output[i], second[i]);
		ExpectBoneNear(state.previous[i], second[i]);
	}
}

TEST(SmoothFingerFrame, GlobalStrengthSmoothsEnabledFingerOnly)
{
	vr::VRBoneTransform_t first[kFingerBoneCount];
	vr::VRBoneTransform_t second[kFingerBoneCount];
	vr::VRBoneTransform_t output[kFingerBoneCount];
	MakeFrame(first, 0.0f);
	MakeFrame(second, 0.0f);
	second[6].position.v[0] += 1.0f;
	second[6].position.v[1] += 2.0f;
	second[6].position.v[2] += 3.0f;
	first[26] = MakeBone(2.0f, 2.0f, 2.0f);
	second[26] = MakeBone(3.0f, 3.0f, 3.0f);
	float alpha[kFingersPerHand];
	FillAlpha(alpha, 0.5f);

	FingerFrameState state{};
	ASSERT_TRUE(
	    skeletal::math::SmoothFingerFrame(state, first, kFingerBoneCount, 0, protocol::kAllFingersMask, alpha, output)
	        .seeded);

	const auto result =
	    skeletal::math::SmoothFingerFrame(state, second, kFingerBoneCount, 0, protocol::kAllFingersMask, alpha, output);
	EXPECT_TRUE(result.appliedSmoothing);
	EXPECT_EQ(result.maxPosDeltaBone, 6);
	EXPECT_NEAR(output[6].position.v[0], (first[6].position.v[0] + second[6].position.v[0]) * 0.5f, kEpsilon);
	EXPECT_NEAR(output[6].position.v[1], (first[6].position.v[1] + second[6].position.v[1]) * 0.5f, kEpsilon);
	EXPECT_NEAR(output[6].position.v[2], (first[6].position.v[2] + second[6].position.v[2]) * 0.5f, kEpsilon);
	ExpectBoneNear(output[1], second[1]);
	ExpectBoneNear(output[26], second[26]);
}

TEST(SmoothFingerFrame, MaskedFingerPassesThrough)
{
	vr::VRBoneTransform_t first[kFingerBoneCount];
	vr::VRBoneTransform_t second[kFingerBoneCount];
	vr::VRBoneTransform_t output[kFingerBoneCount];
	MakeFrame(first, 0.0f);
	MakeFrame(second, 0.0f);
	second[6].position.v[0] += 1.0f;
	float alpha[kFingersPerHand];
	FillAlpha(alpha, 0.5f);

	FingerFrameState state{};
	ASSERT_TRUE(
	    skeletal::math::SmoothFingerFrame(state, first, kFingerBoneCount, 0, protocol::kAllFingersMask, alpha, output)
	        .seeded);

	const uint16_t maskWithoutLeftIndex = AllButFinger(1);
	const auto result =
	    skeletal::math::SmoothFingerFrame(state, second, kFingerBoneCount, 0, maskWithoutLeftIndex, alpha, output);
	EXPECT_TRUE(result.appliedSmoothing);
	ExpectBoneNear(output[6], second[6]);
	ExpectBoneNear(state.previous[6], second[6]);
}

TEST(SmoothFingerFrame, EmptyMaskPassesAllBonesThrough)
{
	vr::VRBoneTransform_t first[kFingerBoneCount];
	vr::VRBoneTransform_t second[kFingerBoneCount];
	vr::VRBoneTransform_t output[kFingerBoneCount];
	MakeFrame(first, 0.0f);
	MakeFrame(second, 1.0f);
	float alpha[kFingersPerHand];
	FillAlpha(alpha, 0.5f);

	FingerFrameState state{};
	ASSERT_TRUE(
	    skeletal::math::SmoothFingerFrame(state, first, kFingerBoneCount, 0, protocol::kAllFingersMask, alpha, output)
	        .seeded);

	const auto result = skeletal::math::SmoothFingerFrame(state, second, kFingerBoneCount, 0, 0, alpha, output);
	EXPECT_FALSE(result.appliedSmoothing);
	for (uint32_t i = 0; i < kFingerBoneCount; ++i) {
		ExpectBoneNear(output[i], second[i]);
		ExpectBoneNear(state.previous[i], second[i]);
	}
}

TEST(SmoothFingerFrame, ReseedPassesRawForOneFrameThenSmooths)
{
	vr::VRBoneTransform_t first[kFingerBoneCount];
	vr::VRBoneTransform_t second[kFingerBoneCount];
	vr::VRBoneTransform_t third[kFingerBoneCount];
	vr::VRBoneTransform_t output[kFingerBoneCount];
	MakeFrame(first, 0.0f);
	MakeFrame(second, 0.0f);
	MakeFrame(third, 0.0f);
	second[6].position.v[0] += 2.0f;
	third[6].position.v[0] += 4.0f;
	float alpha[kFingersPerHand];
	FillAlpha(alpha, 0.5f);

	FingerFrameState state{};
	ASSERT_TRUE(
	    skeletal::math::SmoothFingerFrame(state, first, kFingerBoneCount, 0, protocol::kAllFingersMask, alpha, output)
	        .seeded);
	state.reseed_pending[1] = true;
	const uint16_t leftIndexMask = static_cast<uint16_t>(1u << 1);

	const auto reseed =
	    skeletal::math::SmoothFingerFrame(state, second, kFingerBoneCount, 0, leftIndexMask, alpha, output);
	EXPECT_TRUE(reseed.reseeded);
	EXPECT_FALSE(reseed.appliedSmoothing);
	ExpectBoneNear(output[6], second[6]);
	EXPECT_FALSE(state.reseed_pending[1]);

	const auto smoothed =
	    skeletal::math::SmoothFingerFrame(state, third, kFingerBoneCount, 0, leftIndexMask, alpha, output);
	EXPECT_TRUE(smoothed.appliedSmoothing);
	EXPECT_NEAR(output[6].position.v[0], (second[6].position.v[0] + third[6].position.v[0]) * 0.5f, kEpsilon);
}

TEST(MotionRangeIndex, MapsBothSkeletalRanges)
{
	EXPECT_EQ(skeletal::math::MotionRangeIndex(static_cast<int>(vr::VRSkeletalMotionRange_WithController)), 0);
	EXPECT_EQ(skeletal::math::MotionRangeIndex(static_cast<int>(vr::VRSkeletalMotionRange_WithoutController)), 1);
	EXPECT_GE(skeletal::math::MotionRangeIndex(static_cast<int>(vr::VRSkeletalMotionRange_WithController)), 0);
	EXPECT_LT(skeletal::math::MotionRangeIndex(static_cast<int>(vr::VRSkeletalMotionRange_WithoutController)),
	          skeletal::math::kMotionRangeCount);
}

TEST(ComputeRateHz, GuardsZeroElapsedAndComputesCumulativeRate)
{
	EXPECT_DOUBLE_EQ(skeletal::math::ComputeRateHz(1000, 0.0), 0.0);
	EXPECT_DOUBLE_EQ(skeletal::math::ComputeRateHz(1000, -5.0), 0.0);
	EXPECT_DOUBLE_EQ(skeletal::math::ComputeRateHz(0, 10.0), 0.0);
	// 4,387,892 calls over a 23,041.5s session is ~190 Hz; dividing the
	// cumulative total by one 60s window misreports it as ~73 kHz.
	EXPECT_NEAR(skeletal::math::ComputeRateHz(4387892, 23041.5), 190.4, 0.1);
	EXPECT_NEAR(skeletal::math::ComputeRateHz(4387892, 60.0), 73131.5, 0.1);
}
