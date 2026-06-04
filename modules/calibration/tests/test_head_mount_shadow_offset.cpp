#include "HeadMountShadowOffset.h"

#include <gtest/gtest.h>

namespace hm = spacecal::headmount;

namespace {

hm::ShadowGateInput ReadyGate()
{
	hm::ShadowGateInput in;
	in.toggleEnabled = true;
	in.windowSolved = true;
	in.posesFresh = true;
	in.targetMatches = true;
	in.profileHealthy = true;
	in.residualOk = true;
	in.sourceSettled = true;
	in.fallbackQuiet = true;
	in.mismatchPlausible = true;
	in.mismatchMeaningful = true;
	in.candidateStable = true;
	in.stableWindowCount = hm::kShadowRequiredStableWindows;
	return in;
}

} // namespace

TEST(HeadMountShadowOffset, DeltaClassifiesMeaningfulAndPlausible)
{
	Eigen::AffineCompact3d saved = Eigen::AffineCompact3d::Identity();
	Eigen::AffineCompact3d candidate = Eigen::AffineCompact3d::Identity();
	candidate.translation() = Eigen::Vector3d(0.04, 0.0, 0.0);

	const hm::OffsetDelta delta = hm::ComputeOffsetDelta(saved, candidate);

	EXPECT_TRUE(hm::IsMeaningfulOffsetDelta(delta));
	EXPECT_TRUE(hm::IsPlausibleOffsetDelta(delta));
	EXPECT_FALSE(hm::IsStableOffsetDelta(delta));
}

TEST(HeadMountShadowOffset, EnabledGateAppliesAfterStableWindows)
{
	const hm::ShadowGateResult result = hm::EvaluateShadowGate(ReadyGate());

	EXPECT_TRUE(result.readyToApply);
	EXPECT_TRUE(result.wouldApply);
	EXPECT_STREQ("ready", result.reason);
}

TEST(HeadMountShadowOffset, DisabledGateOnlyWouldApply)
{
	hm::ShadowGateInput in = ReadyGate();
	in.toggleEnabled = false;

	const hm::ShadowGateResult result = hm::EvaluateShadowGate(in);

	EXPECT_FALSE(result.readyToApply);
	EXPECT_TRUE(result.wouldApply);
	EXPECT_STREQ("toggle_disabled", result.reason);
}

TEST(HeadMountShadowOffset, NoisyCandidateIsBlocked)
{
	hm::ShadowGateInput in = ReadyGate();
	in.candidateStable = false;

	const hm::ShadowGateResult result = hm::EvaluateShadowGate(in);

	EXPECT_FALSE(result.readyToApply);
	EXPECT_FALSE(result.wouldApply);
	EXPECT_STREQ("offset_unstable", result.reason);
}

TEST(HeadMountShadowOffset, InsufficientMotionIsBlockedBeforeApply)
{
	hm::ShadowGateInput in = ReadyGate();
	in.windowSolved = false;

	const hm::ShadowGateResult result = hm::EvaluateShadowGate(in);

	EXPECT_FALSE(result.readyToApply);
	EXPECT_FALSE(result.wouldApply);
	EXPECT_STREQ("window_not_solved", result.reason);
}

TEST(HeadMountShadowOffset, StalePoseIsBlocked)
{
	hm::ShadowGateInput in = ReadyGate();
	in.posesFresh = false;

	const hm::ShadowGateResult result = hm::EvaluateShadowGate(in);

	EXPECT_FALSE(result.readyToApply);
	EXPECT_FALSE(result.wouldApply);
	EXPECT_STREQ("stale_pose", result.reason);
}

TEST(HeadMountShadowOffset, DriverSynthFallbackIsBlocked)
{
	hm::ShadowGateInput in = ReadyGate();
	in.fallbackQuiet = false;

	const hm::ShadowGateResult result = hm::EvaluateShadowGate(in);

	EXPECT_FALSE(result.readyToApply);
	EXPECT_FALSE(result.wouldApply);
	EXPECT_STREQ("driver_synth_fallback_active", result.reason);
}
