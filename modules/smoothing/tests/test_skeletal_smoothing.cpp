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

TEST(DashboardHandTrackingStatePacking, RoundTripsCompactState)
{
	protocol::DashboardHandTrackingState state{};
	state.enabled = 1;
	state.dashboard_visible = 1;
	state.primary_hand = protocol::DashboardHandTrackingHandRight;
	state._reserved = 99;
	state.update_mono_ms = 0x123456789ABCu;

	const uint64_t packed = pairdriver::PackDashboardHandTrackingState(state);
	const auto unpacked = pairdriver::UnpackDashboardHandTrackingState(packed);

	EXPECT_EQ(unpacked.enabled, 1);
	EXPECT_EQ(unpacked.dashboard_visible, 1);
	EXPECT_EQ(unpacked.primary_hand, protocol::DashboardHandTrackingHandRight);
	EXPECT_EQ(unpacked._reserved, 0);
	EXPECT_EQ(unpacked.update_mono_ms, state.update_mono_ms);

	state.primary_hand = 200;
	const auto normalized =
	    pairdriver::UnpackDashboardHandTrackingState(pairdriver::PackDashboardHandTrackingState(state));
	EXPECT_EQ(normalized.primary_hand, protocol::DashboardHandTrackingHandUnknown);
}

TEST(DashboardHandTrackingStatePacking, StaleVisibleStateBecomesInactive)
{
	protocol::DashboardHandTrackingState state{};
	state.enabled = 1;
	state.dashboard_visible = 1;
	state.primary_hand = protocol::DashboardHandTrackingHandLeft;
	state.update_mono_ms = 1000;

	const uint64_t packed = pairdriver::PackDashboardHandTrackingState(state);
	const auto fresh = pairdriver::DecodeDashboardHandTrackingState(packed, 1500, 1500);
	EXPECT_TRUE(fresh.active);
	EXPECT_FALSE(fresh.stale);
	EXPECT_EQ(fresh.ageMs, 500);
	EXPECT_EQ(fresh.primaryHand, protocol::DashboardHandTrackingHandLeft);

	const auto stale = pairdriver::DecodeDashboardHandTrackingState(packed, 2601, 1500);
	EXPECT_FALSE(stale.active);
	EXPECT_TRUE(stale.stale);
	EXPECT_EQ(stale.ageMs, 1601);

	state.enabled = 0;
	const auto disabled =
	    pairdriver::DecodeDashboardHandTrackingState(pairdriver::PackDashboardHandTrackingState(state), 2601, 1500);
	EXPECT_FALSE(disabled.active);
	EXPECT_FALSE(disabled.stale);
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

TEST(DashboardFrameObserver, LiveDashboardFramesUpdateRawState)
{
	vr::VRBoneTransform_t first[kFingerBoneCount];
	vr::VRBoneTransform_t second[kFingerBoneCount];
	MakeFrame(first, 0.0f);
	MakeFrame(second, 0.0f);
	second[6].position.v[0] += 0.01f;

	skeletal::math::DashboardFrameState state{};
	const auto seeded = skeletal::math::ObserveDashboardFrame(state, true, first, kFingerBoneCount);
	EXPECT_TRUE(seeded.active);
	EXPECT_TRUE(seeded.seeded);
	EXPECT_FALSE(seeded.liveFrame);
	EXPECT_TRUE(state.initialized);

	const auto live = skeletal::math::ObserveDashboardFrame(state, true, second, kFingerBoneCount);
	EXPECT_TRUE(live.active);
	EXPECT_FALSE(live.seeded);
	EXPECT_TRUE(live.liveFrame);
	EXPECT_EQ(live.maxPosDeltaBone, 6);
	ExpectBoneNear(state.previous[6], second[6]);

	const auto unchanged = skeletal::math::ObserveDashboardFrame(state, true, second, kFingerBoneCount);
	EXPECT_TRUE(unchanged.active);
	EXPECT_FALSE(unchanged.liveFrame);

	const auto inactive = skeletal::math::ObserveDashboardFrame(state, false, second, kFingerBoneCount);
	EXPECT_FALSE(inactive.active);
	EXPECT_FALSE(state.initialized);
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

TEST(DashboardHandTrackingHysteresis, ActiveStateSurvivesUntilStaleThreshold)
{
	protocol::DashboardHandTrackingState state{};
	state.enabled = 1;
	state.dashboard_visible = 1;
	state.update_mono_ms = 1000;
	const uint64_t packed = pairdriver::PackDashboardHandTrackingState(state);

	// wasActive=true: stays active right up to staleAfterMs, drops after.
	const auto held = pairdriver::DecodeDashboardHandTrackingStateWithHysteresis(packed, 3500, 3000, 750, true);
	EXPECT_TRUE(held.active);
	const auto dropped = pairdriver::DecodeDashboardHandTrackingStateWithHysteresis(packed, 4001, 3000, 750, true);
	EXPECT_FALSE(dropped.active);
	EXPECT_TRUE(dropped.stale);
}

TEST(DashboardHandTrackingHysteresis, ReactivationRequiresFreshUpdate)
{
	protocol::DashboardHandTrackingState state{};
	state.enabled = 1;
	state.dashboard_visible = 1;
	state.update_mono_ms = 1000;
	const uint64_t packed = pairdriver::PackDashboardHandTrackingState(state);

	// wasActive=false: an update older than freshAfterMs is not enough,
	// even though it is inside the staleAfterMs window.
	const auto tooOld = pairdriver::DecodeDashboardHandTrackingStateWithHysteresis(packed, 2500, 3000, 750, false);
	EXPECT_FALSE(tooOld.active);
	EXPECT_TRUE(tooOld.stale);
	const auto fresh = pairdriver::DecodeDashboardHandTrackingStateWithHysteresis(packed, 1500, 3000, 750, false);
	EXPECT_TRUE(fresh.active);
}

TEST(DashboardHandTrackingHysteresis, LateRefreshStreamSettlesInsteadOfFlapping)
{
	// Refresh pushes arriving every 1600ms with a symmetric 1500ms cutoff
	// used to flap active<->stale once per push. With staleAfter=3000 and
	// the active state held across the gap, the decode stays active at
	// every point of the late-refresh cycle.
	protocol::DashboardHandTrackingState state{};
	state.enabled = 1;
	state.dashboard_visible = 1;

	bool wasActive = false;
	uint64_t updateMs = 1000;
	state.update_mono_ms = updateMs;
	auto first = pairdriver::DecodeDashboardHandTrackingStateWithHysteresis(
	    pairdriver::PackDashboardHandTrackingState(state), updateMs + 100, 3000, 750, wasActive);
	wasActive = first.active;
	EXPECT_TRUE(wasActive);

	int transitions = 0;
	for (int cycle = 0; cycle < 10; ++cycle) {
		updateMs += 1600;
		state.update_mono_ms = updateMs;
		const uint64_t packed = pairdriver::PackDashboardHandTrackingState(state);
		// Sample just before and just after each late refresh lands.
		for (const uint64_t now : {updateMs + 1599, updateMs + 1600}) {
			const auto snap =
			    pairdriver::DecodeDashboardHandTrackingStateWithHysteresis(packed, now, 3000, 750, wasActive);
			if (snap.active != wasActive) ++transitions;
			wasActive = snap.active;
		}
	}
	EXPECT_EQ(transitions, 0);
	EXPECT_TRUE(wasActive);
}

TEST(DashboardHandTrackingMeaningfulChange, IgnoresRefreshTimestamp)
{
	protocol::DashboardHandTrackingState a{};
	a.enabled = 1;
	a.dashboard_visible = 1;
	a.primary_hand = protocol::DashboardHandTrackingHandLeft;
	a.update_mono_ms = 1000;
	protocol::DashboardHandTrackingState b = a;
	b.update_mono_ms = 1250;
	EXPECT_FALSE(pairdriver::DashboardHandTrackingMeaningfulChange(a, b));

	b.dashboard_visible = 0;
	EXPECT_TRUE(pairdriver::DashboardHandTrackingMeaningfulChange(a, b));
	b = a;
	b.enabled = 0;
	EXPECT_TRUE(pairdriver::DashboardHandTrackingMeaningfulChange(a, b));
	b = a;
	b.primary_hand = protocol::DashboardHandTrackingHandRight;
	EXPECT_TRUE(pairdriver::DashboardHandTrackingMeaningfulChange(a, b));
	// Both out-of-range hands normalize to unknown -- not a change.
	a.primary_hand = 200;
	b.primary_hand = 250;
	EXPECT_FALSE(pairdriver::DashboardHandTrackingMeaningfulChange(a, b));
}

TEST(MotionRangeIndex, MapsBothSkeletalRanges)
{
	EXPECT_EQ(skeletal::math::MotionRangeIndex(static_cast<int>(vr::VRSkeletalMotionRange_WithController)), 0);
	EXPECT_EQ(skeletal::math::MotionRangeIndex(static_cast<int>(vr::VRSkeletalMotionRange_WithoutController)), 1);
	EXPECT_GE(skeletal::math::MotionRangeIndex(static_cast<int>(vr::VRSkeletalMotionRange_WithController)), 0);
	EXPECT_LT(skeletal::math::MotionRangeIndex(static_cast<int>(vr::VRSkeletalMotionRange_WithoutController)),
	          skeletal::math::kMotionRangeCount);
}

TEST(DashboardFrameObserver, PerRangeStatesIgnoreCrossRangePoseGap)
{
	// WithController and WithoutController submissions interleave on the
	// same handle but describe different poses. A single observer reads
	// that gap as huge per-frame motion; one observer per range sees two
	// still streams.
	vr::VRBoneTransform_t withController[kFingerBoneCount];
	vr::VRBoneTransform_t withoutController[kFingerBoneCount];
	MakeFrame(withController, 0.0f);
	MakeFrame(withoutController, 0.5f); // 0.5m apart -- an obvious pose gap

	skeletal::math::DashboardFrameState perRange[skeletal::math::kMotionRangeCount] = {};
	float maxDelta = 0.0f;
	for (int i = 0; i < 6; ++i) {
		const bool without = (i % 2) != 0;
		const auto obs = skeletal::math::ObserveDashboardFrame(
		    perRange[skeletal::math::MotionRangeIndex(static_cast<int>(
		        without ? vr::VRSkeletalMotionRange_WithoutController : vr::VRSkeletalMotionRange_WithController))],
		    true, without ? withoutController : withController, kFingerBoneCount);
		if (obs.maxPosDelta > maxDelta) maxDelta = obs.maxPosDelta;
	}
	EXPECT_NEAR(maxDelta, 0.0f, kEpsilon);

	// The old single-state behavior reports the inter-stream gap instead.
	skeletal::math::DashboardFrameState shared{};
	skeletal::math::ObserveDashboardFrame(shared, true, withController, kFingerBoneCount);
	const auto crossRange = skeletal::math::ObserveDashboardFrame(shared, true, withoutController, kFingerBoneCount);
	EXPECT_GT(crossRange.maxPosDelta, 0.4f);
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
