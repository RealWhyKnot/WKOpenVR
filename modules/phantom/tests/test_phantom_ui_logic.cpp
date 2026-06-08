#include <gtest/gtest.h>

#include "PhantomUiLogic.h"

TEST(PhantomUiLogic, DoesNotAttemptDriverConnectionBeforeSteamVrConnects)
{
	EXPECT_FALSE(phantom::ui::ShouldAttemptDriverConnection(false));
}

TEST(PhantomUiLogic, AttemptsDriverConnectionAfterSteamVrConnects)
{
	EXPECT_TRUE(phantom::ui::ShouldAttemptDriverConnection(true));
}

TEST(PhantomUiLogic, HidesDriverErrorsBeforeSteamVrConnects)
{
	EXPECT_FALSE(phantom::ui::ShouldShowDriverError(false, true));
	EXPECT_FALSE(phantom::ui::ShouldShowDriverError(false, false));
}

TEST(PhantomUiLogic, ShowsDriverErrorsAfterSteamVrConnects)
{
	EXPECT_TRUE(phantom::ui::ShouldShowDriverError(true, true));
	EXPECT_FALSE(phantom::ui::ShouldShowDriverError(true, false));
}

TEST(PhantomUiLogic, VirtualRoleTiersReflectRisk)
{
	EXPECT_EQ(phantom::ui::GetVirtualRoleTier(phantom::BodyRole::Waist), phantom::ui::VirtualRoleTier::Safer);
	EXPECT_EQ(phantom::ui::GetVirtualRoleTier(phantom::BodyRole::Chest), phantom::ui::VirtualRoleTier::Safer);
	EXPECT_EQ(phantom::ui::GetVirtualRoleTier(phantom::BodyRole::LeftKnee), phantom::ui::VirtualRoleTier::Beta);
	EXPECT_EQ(phantom::ui::GetVirtualRoleTier(phantom::BodyRole::RightElbow), phantom::ui::VirtualRoleTier::Beta);
	EXPECT_EQ(phantom::ui::GetVirtualRoleTier(phantom::BodyRole::LeftFoot), phantom::ui::VirtualRoleTier::Experimental);
}

TEST(PhantomUiLogic, VirtualRoleEnableRequiresCalibration)
{
	const auto readiness = phantom::ui::EvaluateVirtualRoleReadiness(false, false);
	EXPECT_FALSE(readiness.canEnable);
	ASSERT_NE(readiness.reason, nullptr);
}

TEST(PhantomUiLogic, VirtualRoleEnableBlocksPhysicalRoleConflict)
{
	const auto readiness = phantom::ui::EvaluateVirtualRoleReadiness(true, true);
	EXPECT_FALSE(readiness.canEnable);
	ASSERT_NE(readiness.reason, nullptr);
}

TEST(PhantomUiLogic, VirtualRoleEnableAllowedWhenCalibratedAndUnclaimed)
{
	const auto readiness = phantom::ui::EvaluateVirtualRoleReadiness(true, false);
	EXPECT_TRUE(readiness.canEnable);
	EXPECT_EQ(readiness.reason, nullptr);
}

TEST(PhantomUiLogic, DropoutTimingDefaultsClassifyAsBalanced)
{
	const auto balanced = phantom::ui::ValuesForDropoutTimingPreset(phantom::ui::DropoutTimingPreset::Balanced);
	EXPECT_EQ(phantom::ui::ClassifyDropoutTiming(balanced), phantom::ui::DropoutTimingPreset::Balanced);
	EXPECT_EQ(balanced.synth_hold_ms, phantom::DefaultTimings::kSynthHoldMs);
	EXPECT_EQ(balanced.lost_hold_ms, phantom::DefaultTimings::kLostHoldMs);
}

TEST(PhantomUiLogic, DropoutTimingPresetsHaveDistinctBridgeLengths)
{
	const auto conservative = phantom::ui::ValuesForDropoutTimingPreset(phantom::ui::DropoutTimingPreset::Conservative);
	const auto balanced = phantom::ui::ValuesForDropoutTimingPreset(phantom::ui::DropoutTimingPreset::Balanced);
	const auto extended = phantom::ui::ValuesForDropoutTimingPreset(phantom::ui::DropoutTimingPreset::Extended);

	EXPECT_LT(conservative.synth_hold_ms, balanced.synth_hold_ms);
	EXPECT_LT(balanced.synth_hold_ms, extended.synth_hold_ms);
	EXPECT_LT(conservative.lost_hold_ms, balanced.lost_hold_ms);
	EXPECT_LT(balanced.lost_hold_ms, extended.lost_hold_ms);
}

TEST(PhantomUiLogic, NonPresetDropoutTimingClassifiesAsCustom)
{
	auto values = phantom::ui::ValuesForDropoutTimingPreset(phantom::ui::DropoutTimingPreset::Balanced);
	++values.synth_hold_ms;
	EXPECT_EQ(phantom::ui::ClassifyDropoutTiming(values), phantom::ui::DropoutTimingPreset::Custom);
}

TEST(PhantomUiLogic, TrackerStateToneWalksTheDropoutLadder)
{
	EXPECT_EQ(phantom::ui::TrackerStateTone(phantom::TrackerState::REAL), phantom::ui::PhantomTone::Ok);
	EXPECT_EQ(phantom::ui::TrackerStateTone(phantom::TrackerState::BLEND_OUT), phantom::ui::PhantomTone::Pending);
	EXPECT_EQ(phantom::ui::TrackerStateTone(phantom::TrackerState::BLEND_IN), phantom::ui::PhantomTone::Pending);
	EXPECT_EQ(phantom::ui::TrackerStateTone(phantom::TrackerState::SYNTH_RECKON), phantom::ui::PhantomTone::Warn);
	EXPECT_EQ(phantom::ui::TrackerStateTone(phantom::TrackerState::SYNTH_IK), phantom::ui::PhantomTone::Warn);
	EXPECT_EQ(phantom::ui::TrackerStateTone(phantom::TrackerState::SYNTH_ML), phantom::ui::PhantomTone::Warn);
	EXPECT_EQ(phantom::ui::TrackerStateTone(phantom::TrackerState::OUT_OF_RANGE), phantom::ui::PhantomTone::Warn);
	EXPECT_EQ(phantom::ui::TrackerStateTone(phantom::TrackerState::LOST), phantom::ui::PhantomTone::Error);
}

TEST(PhantomUiLogic, EveryTrackerStateHasAConcreteTone)
{
	// No live tracker state should fall through to the Idle/default tone -- a
	// row always shows a meaningful green/amber/orange/red badge.
	const phantom::TrackerState states[] = {
	    phantom::TrackerState::REAL,         phantom::TrackerState::BLEND_OUT,
	    phantom::TrackerState::SYNTH_RECKON, phantom::TrackerState::SYNTH_IK,
	    phantom::TrackerState::SYNTH_ML,     phantom::TrackerState::BLEND_IN,
	    phantom::TrackerState::OUT_OF_RANGE, phantom::TrackerState::LOST,
	};
	for (const auto s : states) {
		EXPECT_NE(phantom::ui::TrackerStateTone(s), phantom::ui::PhantomTone::Idle);
	}
}

TEST(PhantomUiLogic, SolverModeToneSeparatesMeasuredFromInferred)
{
	EXPECT_EQ(phantom::ui::SolverModeTone(0), phantom::ui::PhantomTone::Idle);
	EXPECT_EQ(phantom::ui::SolverModeTone(1), phantom::ui::PhantomTone::Ok);
	EXPECT_EQ(phantom::ui::SolverModeTone(2), phantom::ui::PhantomTone::Warn);
	EXPECT_EQ(phantom::ui::SolverModeTone(3), phantom::ui::PhantomTone::Warn);
	EXPECT_EQ(phantom::ui::SolverModeTone(4), phantom::ui::PhantomTone::Warn);
	EXPECT_EQ(phantom::ui::SolverModeTone(5), phantom::ui::PhantomTone::Warn);
	EXPECT_EQ(phantom::ui::SolverModeTone(6), phantom::ui::PhantomTone::Error);
}
