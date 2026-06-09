#include "FeatureFlags.h"

#include "DashboardInputRuntimeGate.h"

#include <gtest/gtest.h>

namespace {

TEST(FeatureFlags, FaceTrackingImpliesOscRouter)
{
	const uint32_t flags = pairdriver::ComposeFeatureFlags(false, false, false, false, true, false, false, false);

	EXPECT_NE(flags & pairdriver::kFeatureFaceTracking, 0u);
	EXPECT_NE(flags & pairdriver::kFeatureOscRouter, 0u);
}

TEST(FeatureFlags, CaptionsImpliesOscRouter)
{
	const uint32_t flags = pairdriver::ComposeFeatureFlags(false, false, false, false, false, false, true, false);

	EXPECT_NE(flags & pairdriver::kFeatureCaptions, 0u);
	EXPECT_NE(flags & pairdriver::kFeatureOscRouter, 0u);
}

TEST(FeatureFlags, OscRouterFlagStillWorksAlone)
{
	const uint32_t flags = pairdriver::ComposeFeatureFlags(false, false, false, false, false, true, false, false);

	EXPECT_EQ(flags, pairdriver::kFeatureOscRouter);
}

TEST(FeatureFlags, DashboardInputIsIndependent)
{
	const uint32_t flags = pairdriver::ComposeFeatureFlags(false, false, true, false, false, false, false, false);

	EXPECT_EQ(flags, pairdriver::kFeatureDashboardInput);
}

TEST(FeatureFlags, DashboardInputRuntimeGateRequiresOptIn)
{
	EXPECT_FALSE(openvr_pair::common::dashboardinput::RuntimeEnabled(true, false));
	EXPECT_TRUE(openvr_pair::common::dashboardinput::RuntimeEnabled(true, true));
}

TEST(FeatureFlags, EmptyMaskStaysInert)
{
	const uint32_t flags = pairdriver::ComposeFeatureFlags(false, false, false, false, false, false, false, false);

	EXPECT_EQ(flags, 0u);
}

} // namespace
