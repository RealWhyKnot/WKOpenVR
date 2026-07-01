#include "OverlayHotReload.h"

#include <gtest/gtest.h>

namespace {

using openvr_pair::overlay::DeriveOverlayReloadPaths;

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

} // namespace
