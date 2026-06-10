#include <gtest/gtest.h>

#include "DashboardInputRuntimeGate.h"
#include "DashboardInputSafeOverlayLogic.h"

TEST(DashboardInputRuntimeGate, RequiresModuleAndRuntimeOptInFlags)
{
	EXPECT_FALSE(openvr_pair::common::dashboardinput::RuntimeEnabled(false, false));
	EXPECT_FALSE(openvr_pair::common::dashboardinput::RuntimeEnabled(true, false));
	EXPECT_FALSE(openvr_pair::common::dashboardinput::RuntimeEnabled(false, true));
	EXPECT_TRUE(openvr_pair::common::dashboardinput::RuntimeEnabled(true, true));
}

TEST(DashboardInputSafeOverlayLogic, UsesSteamVROverlayGlobalPriorityRange)
{
	EXPECT_GE(openvr_pair::overlay::DashboardInputSafeOverlayPriority(), vr::k_nActionSetOverlayGlobalPriorityMin);
	EXPECT_LE(openvr_pair::overlay::DashboardInputSafeOverlayPriority(), vr::k_nActionSetOverlayGlobalPriorityMax);
}

TEST(DashboardInputSafeOverlayLogic, KeepsToggleActiveWhilePointerInputIsHidden)
{
	EXPECT_TRUE(openvr_pair::overlay::DashboardInputSafeOverlayToggleActive(true, true));
	EXPECT_FALSE(openvr_pair::overlay::DashboardInputSafeOverlayPointerActive(true, true, false));
	EXPECT_EQ(2u, openvr_pair::overlay::DashboardInputSafeOverlayActionSetCount(true, true, false));
}

TEST(DashboardInputSafeOverlayLogic, EnablesPointerInputOnlyForVisibleSafeOverlay)
{
	EXPECT_TRUE(openvr_pair::overlay::DashboardInputSafeOverlayPointerActive(true, true, true));
	EXPECT_EQ(4u, openvr_pair::overlay::DashboardInputSafeOverlayActionSetCount(true, true, true));
}

TEST(DashboardInputSafeOverlayLogic, DisablesAllActionSetsWhenModuleOrInputIsUnavailable)
{
	EXPECT_FALSE(openvr_pair::overlay::DashboardInputSafeOverlayToggleActive(false, true));
	EXPECT_FALSE(openvr_pair::overlay::DashboardInputSafeOverlayToggleActive(true, false));
	EXPECT_EQ(0u, openvr_pair::overlay::DashboardInputSafeOverlayActionSetCount(false, true, true));
	EXPECT_EQ(0u, openvr_pair::overlay::DashboardInputSafeOverlayActionSetCount(true, false, true));
}
