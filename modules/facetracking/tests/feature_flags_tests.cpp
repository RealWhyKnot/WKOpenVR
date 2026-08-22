#include "FeatureFlags.h"

#include <gtest/gtest.h>

namespace {

TEST(FeatureFlags, FaceTrackingImpliesOscRouter)
{
	const uint32_t flags = pairdriver::ComposeFeatureFlags(false, false, false, true, false, false, false);

	EXPECT_NE(flags & pairdriver::kFeatureFaceTracking, 0u);
	EXPECT_NE(flags & pairdriver::kFeatureOscRouter, 0u);
}

TEST(FeatureFlags, CaptionsImpliesOscRouter)
{
	const uint32_t flags = pairdriver::ComposeFeatureFlags(false, false, false, false, false, true, false);

	EXPECT_NE(flags & pairdriver::kFeatureCaptions, 0u);
	EXPECT_NE(flags & pairdriver::kFeatureOscRouter, 0u);
}

TEST(FeatureFlags, OscRouterFlagStillWorksAlone)
{
	const uint32_t flags = pairdriver::ComposeFeatureFlags(false, false, false, false, true, false, false);

	EXPECT_EQ(flags, pairdriver::kFeatureOscRouter);
}

TEST(FeatureFlags, EmptyMaskStaysInert)
{
	const uint32_t flags = pairdriver::ComposeFeatureFlags(false, false, false, false, false, false, false);

	EXPECT_EQ(flags, 0u);
}

// The app composes the desktop host's mask from the installed module set rather than through
// ComposeFeatureFlags, so the router implication has to hold on a raw mask too.
TEST(FeatureFlags, ImplicationsHoldOnARawMask)
{
	EXPECT_NE(pairdriver::ApplyFeatureImplications(pairdriver::kFeatureFaceTracking) & pairdriver::kFeatureOscRouter,
	          0u);
	EXPECT_NE(pairdriver::ApplyFeatureImplications(pairdriver::kFeatureCaptions) & pairdriver::kFeatureOscRouter, 0u);
	EXPECT_EQ(pairdriver::ApplyFeatureImplications(pairdriver::kFeatureCalibration), pairdriver::kFeatureCalibration);
	EXPECT_EQ(pairdriver::ApplyFeatureImplications(0u), 0u);
}

} // namespace
