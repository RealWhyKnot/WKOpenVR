#include "HostSupervisorBase.h"

#include <gtest/gtest.h>

namespace {

using openvr_pair::common::ShouldHotReloadHost;
using openvr_pair::common::ShouldUnhaltForNewHostExe;

TEST(HostHotReload, ReloadsWhenSpawnedExeChanged)
{
	// spawned by us, have handle, baseline known, disk mtime differs -> reload.
	EXPECT_TRUE(ShouldHotReloadHost(/*attached=*/false, /*haveHandle=*/true, /*baseline=*/100, /*disk=*/200));
}

TEST(HostHotReload, NoReloadWhenUnchanged)
{
	EXPECT_FALSE(ShouldHotReloadHost(false, true, 100, 100));
}

TEST(HostHotReload, NeverReloadsAttachedHost)
{
	// A host from a prior session that we merely attached to isn't ours to swap.
	EXPECT_FALSE(ShouldHotReloadHost(/*attached=*/true, true, 100, 200));
}

TEST(HostHotReload, NoReloadWithoutHandle)
{
	EXPECT_FALSE(ShouldHotReloadHost(false, /*haveHandle=*/false, 100, 200));
}

TEST(HostHotReload, NoReloadWhenBaselineOrDiskUnknown)
{
	EXPECT_FALSE(ShouldHotReloadHost(false, true, /*baseline=*/0, 200));
	EXPECT_FALSE(ShouldHotReloadHost(false, true, 100, /*disk=*/0));
}

TEST(HostHotReload, HaltClearsWhenExeChanged)
{
	// Crash-loop halt + a rebuilt exe on disk -> the fixed build takes over.
	EXPECT_TRUE(ShouldUnhaltForNewHostExe(/*attached=*/false, /*baseline=*/100, /*disk=*/200));
}

TEST(HostHotReload, HaltPersistsWhenExeUnchanged)
{
	EXPECT_FALSE(ShouldUnhaltForNewHostExe(false, 100, 100));
}

TEST(HostHotReload, HaltPersistsForAttachedOrUnknown)
{
	EXPECT_FALSE(ShouldUnhaltForNewHostExe(/*attached=*/true, 100, 200));
	EXPECT_FALSE(ShouldUnhaltForNewHostExe(false, /*baseline=*/0, 200));
	EXPECT_FALSE(ShouldUnhaltForNewHostExe(false, 100, /*disk=*/0));
}

} // namespace
