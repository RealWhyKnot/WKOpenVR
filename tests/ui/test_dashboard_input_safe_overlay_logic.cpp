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

TEST(DashboardInputRuntimeGate, ReportsActionableGateState)
{
	namespace dashboardinput = openvr_pair::common::dashboardinput;
	EXPECT_EQ(dashboardinput::RuntimeGateState::ModuleOff, dashboardinput::RuntimeState(false, false));
	EXPECT_EQ(dashboardinput::RuntimeGateState::MissingRuntimeOptIn, dashboardinput::RuntimeState(true, false));
	EXPECT_EQ(dashboardinput::RuntimeGateState::ModuleOff, dashboardinput::RuntimeState(false, true));
	EXPECT_EQ(dashboardinput::RuntimeGateState::Enabled, dashboardinput::RuntimeState(true, true));
	EXPECT_STREQ("Waiting for opt-in",
	             dashboardinput::RuntimeStateLabel(dashboardinput::RuntimeGateState::MissingRuntimeOptIn));
}

TEST(DashboardInputSafeOverlayLogic, VisibleOnlyWhenEnabledAndRequested)
{
	EXPECT_TRUE(openvr_pair::overlay::DashboardInputSafeOverlayShouldBeVisible(true, true, false));
	EXPECT_FALSE(openvr_pair::overlay::DashboardInputSafeOverlayShouldBeVisible(true, false, false));
	EXPECT_FALSE(openvr_pair::overlay::DashboardInputSafeOverlayShouldBeVisible(false, true, false));
	EXPECT_FALSE(openvr_pair::overlay::DashboardInputSafeOverlayShouldBeVisible(false, false, false));
}

TEST(DashboardInputSafeOverlayLogic, YieldsWhileDashboardIsOpen)
{
	EXPECT_FALSE(openvr_pair::overlay::DashboardInputSafeOverlayShouldBeVisible(true, true, true));
	// Returns as soon as the dashboard closes; the user request persists.
	EXPECT_TRUE(openvr_pair::overlay::DashboardInputSafeOverlayShouldBeVisible(true, true, false));
}

TEST(DashboardInputSafeOverlayLogic, DisabledFeatureIgnoresRequestAndDashboardState)
{
	EXPECT_FALSE(openvr_pair::overlay::DashboardInputSafeOverlayShouldBeVisible(false, true, true));
	EXPECT_FALSE(openvr_pair::overlay::DashboardInputSafeOverlayShouldBeVisible(false, true, false));
}
