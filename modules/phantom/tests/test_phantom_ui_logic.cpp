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
