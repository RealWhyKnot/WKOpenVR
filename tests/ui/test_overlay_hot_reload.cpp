#include "OverlayHotReload.h"

#include <gtest/gtest.h>

namespace {

using openvr_pair::overlay::DeriveOverlayReloadPaths;
using openvr_pair::overlay::LooksLikePeImage;
using openvr_pair::overlay::RelaunchNeedsRollback;

TEST(OverlayHotReload, DerivesSiblingPaths)
{
	const auto p = DeriveOverlayReloadPaths(LR"(C:\Program Files\WKOpenVR\WKOpenVR.exe)");
	EXPECT_EQ(p.canonical, LR"(C:\Program Files\WKOpenVR\WKOpenVR.exe)");
	EXPECT_EQ(p.staged, LR"(C:\Program Files\WKOpenVR\WKOpenVR.new.exe)");
	EXPECT_EQ(p.backup, LR"(C:\Program Files\WKOpenVR\WKOpenVR.old.exe)");
}

TEST(OverlayHotReload, HandlesForwardSlashes)
{
	const auto p = DeriveOverlayReloadPaths(L"D:/build/artifacts/Release/WKOpenVR.exe");
	EXPECT_EQ(p.canonical, L"D:/build/artifacts/Release\\WKOpenVR.exe");
	EXPECT_EQ(p.staged, L"D:/build/artifacts/Release\\WKOpenVR.new.exe");
}

TEST(OverlayHotReload, EmptyWhenNoDirectory)
{
	const auto p = DeriveOverlayReloadPaths(L"WKOpenVR.exe");
	EXPECT_TRUE(p.canonical.empty());
	EXPECT_TRUE(p.staged.empty());
	EXPECT_TRUE(p.backup.empty());
}

TEST(OverlayHotReload, HealthyRelaunchNeedsNoRollback)
{
	EXPECT_FALSE(RelaunchNeedsRollback(/*launchSucceeded=*/true, /*aliveAfterGrace=*/true));
}

TEST(OverlayHotReload, FailedLaunchNeedsRollback)
{
	EXPECT_TRUE(RelaunchNeedsRollback(/*launchSucceeded=*/false, /*aliveAfterGrace=*/false));
}

TEST(OverlayHotReload, EarlyExitNeedsRollback)
{
	// The launch itself worked, but the new build died within the grace window.
	EXPECT_TRUE(RelaunchNeedsRollback(/*launchSucceeded=*/true, /*aliveAfterGrace=*/false));
}

TEST(OverlayHotReload, AcceptsPeImageHeader)
{
	const unsigned char mz[] = {'M', 'Z', 0x90, 0x00};
	EXPECT_TRUE(LooksLikePeImage(mz, sizeof(mz)));
}

TEST(OverlayHotReload, RejectsNonPeContent)
{
	const unsigned char text[] = {'n', 'o', 't', ' ', 'a', 'n', ' ', 'e', 'x', 'e'};
	EXPECT_FALSE(LooksLikePeImage(text, sizeof(text)));
	EXPECT_FALSE(LooksLikePeImage(text, 0));
	const unsigned char shortMz[] = {'M'};
	EXPECT_FALSE(LooksLikePeImage(shortMz, sizeof(shortMz)));
}

} // namespace
