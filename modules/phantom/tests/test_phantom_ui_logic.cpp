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
