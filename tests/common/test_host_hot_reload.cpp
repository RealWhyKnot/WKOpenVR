#include "HostSupervisorBase.h"

#include <gtest/gtest.h>

namespace {

using openvr_pair::common::ShouldHotReloadHost;

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

} // namespace
