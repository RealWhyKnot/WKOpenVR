#include <gtest/gtest.h>

#include "FreezeHoldLogic.h"

namespace freeze = wkopenvr::freeze;

namespace {

vr::DriverPose_t MakeCachedPose()
{
	vr::DriverPose_t p{};
	p.vecPosition[0] = 1.0;
	p.vecPosition[1] = 2.0;
	p.vecPosition[2] = 3.0;
	p.qRotation = {0.7071, 0.0, 0.7071, 0.0};
	// Nonzero motion that a proper hold must scrub so the compositor can't
	// extrapolate the "frozen" device.
	p.vecVelocity[0] = 5.0;
	p.vecVelocity[1] = -5.0;
	p.vecVelocity[2] = 5.0;
	p.vecAngularVelocity[0] = 1.0;
	p.vecAngularVelocity[1] = 1.0;
	p.vecAngularVelocity[2] = 1.0;
	p.poseIsValid = false;
	p.result = vr::TrackingResult_Running_OutOfRange;
	p.deviceIsConnected = false;
	return p;
}

} // namespace

// --- ShouldHold gating -----------------------------------------------------

TEST(FreezeHoldLogic, NotFrozenNeverHolds)
{
	EXPECT_FALSE(freeze::ShouldHold(/*frozen=*/false, /*includeHmd=*/true, /*openVRID=*/5, /*haveCached=*/true));
}

TEST(FreezeHoldLogic, FrozenWithoutCacheDoesNotHold)
{
	EXPECT_FALSE(freeze::ShouldHold(/*frozen=*/true, /*includeHmd=*/true, /*openVRID=*/5, /*haveCached=*/false));
}

TEST(FreezeHoldLogic, FrozenTrackerHolds)
{
	EXPECT_TRUE(freeze::ShouldHold(/*frozen=*/true, /*includeHmd=*/false, /*openVRID=*/3, /*haveCached=*/true));
}

TEST(FreezeHoldLogic, HmdExcludedWhenNotIncluded)
{
	// Device 0 is the HMD; without opt-in it stays live so the user can look around.
	EXPECT_FALSE(freeze::ShouldHold(/*frozen=*/true, /*includeHmd=*/false, /*openVRID=*/0, /*haveCached=*/true));
}

TEST(FreezeHoldLogic, HmdHeldWhenIncluded)
{
	EXPECT_TRUE(freeze::ShouldHold(/*frozen=*/true, /*includeHmd=*/true, /*openVRID=*/0, /*haveCached=*/true));
}

// --- MakeHeldPose rewrite --------------------------------------------------

TEST(FreezeHoldLogic, HeldPosePreservesPositionAndRotation)
{
	const vr::DriverPose_t cached = MakeCachedPose();
	vr::DriverPose_t live{};
	freeze::MakeHeldPose(live, cached);

	EXPECT_DOUBLE_EQ(live.vecPosition[0], 1.0);
	EXPECT_DOUBLE_EQ(live.vecPosition[1], 2.0);
	EXPECT_DOUBLE_EQ(live.vecPosition[2], 3.0);
	EXPECT_DOUBLE_EQ(live.qRotation.w, cached.qRotation.w);
	EXPECT_DOUBLE_EQ(live.qRotation.x, cached.qRotation.x);
	EXPECT_DOUBLE_EQ(live.qRotation.y, cached.qRotation.y);
	EXPECT_DOUBLE_EQ(live.qRotation.z, cached.qRotation.z);
}

TEST(FreezeHoldLogic, HeldPoseZeroesVelocities)
{
	const vr::DriverPose_t cached = MakeCachedPose();
	vr::DriverPose_t live{};
	freeze::MakeHeldPose(live, cached);

	for (int i = 0; i < 3; ++i) {
		EXPECT_DOUBLE_EQ(live.vecVelocity[i], 0.0);
		EXPECT_DOUBLE_EQ(live.vecAngularVelocity[i], 0.0);
	}
}

TEST(FreezeHoldLogic, HeldPoseIsValidAndConnected)
{
	const vr::DriverPose_t cached = MakeCachedPose();
	vr::DriverPose_t live{};
	freeze::MakeHeldPose(live, cached);

	EXPECT_TRUE(live.poseIsValid);
	EXPECT_EQ(live.result, vr::TrackingResult_Running_OK);
	EXPECT_TRUE(live.deviceIsConnected);
}
