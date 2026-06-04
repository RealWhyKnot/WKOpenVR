#include <gtest/gtest.h>

#include "OscRouterUiLogic.h"

TEST(OscRouterUiLogic, ShowsWaitingBeforeSteamVrConnects)
{
	EXPECT_EQ(oscrouter::ui::ResolveDriverPanelState(false, false, false),
	          oscrouter::ui::DriverPanelState::WaitingForSteamVr);
	EXPECT_FALSE(oscrouter::ui::ShouldAttemptLiveDriverIpc(false));
}

TEST(OscRouterUiLogic, ShowsActiveWhenStatsAreOpen)
{
	EXPECT_EQ(oscrouter::ui::ResolveDriverPanelState(true, true, false), oscrouter::ui::DriverPanelState::Active);
}

TEST(OscRouterUiLogic, WaitsForDriverBeforeSurfacingProblem)
{
	EXPECT_EQ(oscrouter::ui::ResolveDriverPanelState(true, false, false),
	          oscrouter::ui::DriverPanelState::WaitingForDriver);
	EXPECT_EQ(oscrouter::ui::ResolveDriverPanelState(true, false, true), oscrouter::ui::DriverPanelState::Problem);
}

TEST(OscRouterUiLogic, RetriesLiveIpcAfterUnavailablePipe)
{
	EXPECT_FALSE(oscrouter::ui::ShouldRetryLiveDriverIpc(false, false, true));
	EXPECT_FALSE(oscrouter::ui::ShouldRetryLiveDriverIpc(true, true, true));
	EXPECT_FALSE(oscrouter::ui::ShouldRetryLiveDriverIpc(true, false, false));
	EXPECT_TRUE(oscrouter::ui::ShouldRetryLiveDriverIpc(true, false, true));
}
