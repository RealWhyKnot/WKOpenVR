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
