#include "SmoothingPredictionLogic.h"

#include <gtest/gtest.h>

TEST(SmoothingPredictionUiLogic, HeadsetSynthesisTrackerUsesLockedHeadsetValue)
{
	SmoothingConfig cfg;
	cfg.trackerSmoothness["LHR-head"] = 25;

	EXPECT_EQ(wkopenvr::smoothing_prediction::VisiblePredictionRowSmoothness(
	              cfg, "LHR-head", /*isLocked*/ false, /*isHeadsetSynthesisTracker*/ true,
	              /*haveLockedHeadsetSmoothing*/ true, /*lockedHeadsetSmoothing*/ 49),
	          49);
}

TEST(SmoothingPredictionUiLogic, NormalTrackerUsesSavedTrackerSmoothness)
{
	SmoothingConfig cfg;
	cfg.trackerSmoothness["LHR-waist"] = 70;

	EXPECT_EQ(wkopenvr::smoothing_prediction::VisiblePredictionRowSmoothness(
	              cfg, "LHR-waist", /*isLocked*/ false, /*isHeadsetSynthesisTracker*/ false,
	              /*haveLockedHeadsetSmoothing*/ false, /*lockedHeadsetSmoothing*/ 0),
	          70);
}

TEST(SmoothingPredictionUiLogic, LockedNormalTrackerDisplaysZeroWithoutDeletingSavedValue)
{
	SmoothingConfig cfg;
	cfg.trackerSmoothness["LHR-waist"] = 70;

	EXPECT_EQ(wkopenvr::smoothing_prediction::VisiblePredictionRowSmoothness(
	              cfg, "LHR-waist", /*isLocked*/ true, /*isHeadsetSynthesisTracker*/ false,
	              /*haveLockedHeadsetSmoothing*/ false, /*lockedHeadsetSmoothing*/ 0),
	          0);
	EXPECT_EQ(cfg.trackerSmoothness["LHR-waist"], 70);
}

TEST(SmoothingPredictionUiLogic, CleanupRemovesOnlyHeadsetSynthesisTrackerEntry)
{
	SmoothingConfig cfg;
	cfg.trackerSmoothness["LHR-head"] = 25;
	cfg.trackerSmoothness["LHR-waist"] = 70;

	EXPECT_TRUE(wkopenvr::smoothing_prediction::RemoveHeadsetSynthesisTrackerSmoothness(cfg, "LHR-head"));
	EXPECT_EQ(cfg.trackerSmoothness.count("LHR-head"), 0u);
	EXPECT_EQ(cfg.trackerSmoothness["LHR-waist"], 70);
	EXPECT_FALSE(wkopenvr::smoothing_prediction::RemoveHeadsetSynthesisTrackerSmoothness(cfg, "LHR-head"));
}
