#include <gtest/gtest.h>

#include "TrackingStyle.h"

TEST(TrackingStyleTest, PresetsWriteOnlyExplicitRelativeLockModes)
{
	const TrackingStyle styles[] = {TrackingStyle::Manual, TrackingStyle::Continuous, TrackingStyle::LockedWithRecovery,
	                                TrackingStyle::HardTrackerLock};

	for (const TrackingStyle style : styles) {
		CalibrationContext ctx;
		ApplyTrackingStylePreset(ctx, style);

		EXPECT_NE(ctx.lockRelativePositionMode, CalibrationContext::LockMode::AUTO);
		EXPECT_EQ(ctx.trackingStyle, style);
	}
}

TEST(TrackingStyleTest, PresetMappingMatchesSupportedStyles)
{
	CalibrationContext ctx;

	ApplyTrackingStylePreset(ctx, TrackingStyle::Manual);
	EXPECT_EQ(ctx.headMount.mode, HeadMountMode::Off);
	EXPECT_TRUE(ctx.headMount.allowRawHmdFallback);
	EXPECT_EQ(ctx.lockRelativePositionMode, CalibrationContext::LockMode::OFF);
	EXPECT_FALSE(TrackingStyleRunsContinuous(ctx.trackingStyle));

	ApplyTrackingStylePreset(ctx, TrackingStyle::Continuous);
	EXPECT_EQ(ctx.headMount.mode, HeadMountMode::Off);
	EXPECT_TRUE(ctx.headMount.allowRawHmdFallback);
	EXPECT_EQ(ctx.lockRelativePositionMode, CalibrationContext::LockMode::OFF);
	EXPECT_TRUE(TrackingStyleRunsContinuous(ctx.trackingStyle));

	ApplyTrackingStylePreset(ctx, TrackingStyle::LockedWithRecovery);
	EXPECT_EQ(ctx.headMount.mode, HeadMountMode::DriverSynth);
	EXPECT_TRUE(ctx.headMount.allowRawHmdFallback);
	EXPECT_EQ(ctx.lockRelativePositionMode, CalibrationContext::LockMode::ON);
	EXPECT_TRUE(TrackingStyleRunsContinuous(ctx.trackingStyle));
	EXPECT_TRUE(TrackingStyleShowsBoundarySetup(ctx.trackingStyle));

	ApplyTrackingStylePreset(ctx, TrackingStyle::HardTrackerLock);
	EXPECT_EQ(ctx.headMount.mode, HeadMountMode::DriverSynth);
	EXPECT_FALSE(ctx.headMount.allowRawHmdFallback);
	EXPECT_EQ(ctx.lockRelativePositionMode, CalibrationContext::LockMode::ON);
	EXPECT_FALSE(TrackingStyleRunsContinuous(ctx.trackingStyle));
	EXPECT_FALSE(TrackingStyleShowsBoundarySetup(ctx.trackingStyle));
}

TEST(TrackingStyleTest, HmdPoseEventRecoveryIsPlainContinuousOnly)
{
	EXPECT_TRUE(HmdPoseEventRecoveryEligible(CalibrationState::Continuous, TrackingStyle::Continuous));
	EXPECT_TRUE(HmdPoseEventRecoveryEligible(CalibrationState::ContinuousStandby, TrackingStyle::Continuous));

	EXPECT_FALSE(HmdPoseEventRecoveryEligible(CalibrationState::None, TrackingStyle::Continuous));
	EXPECT_FALSE(HmdPoseEventRecoveryEligible(CalibrationState::Continuous, TrackingStyle::Manual));
	EXPECT_FALSE(HmdPoseEventRecoveryEligible(CalibrationState::ContinuousStandby, TrackingStyle::Manual));

	EXPECT_TRUE(TrackingStyleRunsContinuous(TrackingStyle::LockedWithRecovery));
	EXPECT_FALSE(HmdPoseEventRecoveryEligible(CalibrationState::Continuous, TrackingStyle::LockedWithRecovery));
	EXPECT_FALSE(HmdPoseEventRecoveryEligible(CalibrationState::ContinuousStandby, TrackingStyle::LockedWithRecovery));

	EXPECT_FALSE(TrackingStyleRunsContinuous(TrackingStyle::HardTrackerLock));
	EXPECT_FALSE(HmdPoseEventRecoveryEligible(CalibrationState::Continuous, TrackingStyle::HardTrackerLock));
	EXPECT_FALSE(HmdPoseEventRecoveryEligible(CalibrationState::ContinuousStandby, TrackingStyle::HardTrackerLock));
}

TEST(TrackingStyleTest, LegacyAutoLockRawMapsToOff)
{
	EXPECT_EQ(ExplicitLockModeFromRaw((int)CalibrationContext::LockMode::AUTO), CalibrationContext::LockMode::OFF);
	EXPECT_EQ(ExplicitLockModeFromRaw((int)CalibrationContext::LockMode::OFF), CalibrationContext::LockMode::OFF);
	EXPECT_EQ(ExplicitLockModeFromRaw((int)CalibrationContext::LockMode::ON), CalibrationContext::LockMode::ON);
}
