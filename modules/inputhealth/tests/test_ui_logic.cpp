#include <gtest/gtest.h>

#include "InputHealthUiLogic.h"

TEST(InputHealthUiLogic, HidesDriverProblemBannerForDriverWaitError)
{
	EXPECT_FALSE(inputhealth::ui::ShouldShowDriverProblemBanner(true, true));
}

TEST(InputHealthUiLogic, ShowsDriverProblemBannerForActionableError)
{
	EXPECT_TRUE(inputhealth::ui::ShouldShowDriverProblemBanner(true, false));
}

TEST(InputHealthUiLogic, HidesDriverProblemBannerWithoutError)
{
	EXPECT_FALSE(inputhealth::ui::ShouldShowDriverProblemBanner(false, false));
	EXPECT_FALSE(inputhealth::ui::ShouldShowDriverProblemBanner(false, true));
}

TEST(InputHealthUiLogic, HidesShmemProblemTextUntilRuntimeIsConnected)
{
	EXPECT_FALSE(inputhealth::ui::ShouldShowShmemProblemText(false, false, true, false));
	EXPECT_FALSE(inputhealth::ui::ShouldShowShmemProblemText(true, true, true, false));
	EXPECT_FALSE(inputhealth::ui::ShouldShowShmemProblemText(true, false, false, false));
	EXPECT_TRUE(inputhealth::ui::ShouldShowShmemProblemText(true, false, true, false));
}

TEST(InputHealthUiLogic, ShowsShmemVersionMismatchBeforeRuntimeConnects)
{
	EXPECT_TRUE(inputhealth::ui::ShouldShowShmemProblemText(false, false, true, true));
}
