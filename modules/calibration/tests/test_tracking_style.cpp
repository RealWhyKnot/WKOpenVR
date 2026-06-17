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
	EXPECT_FALSE(TrackingStyleShowsBoundarySetup(ctx.trackingStyle));

	ApplyTrackingStylePreset(ctx, TrackingStyle::HardTrackerLock);
	EXPECT_EQ(ctx.headMount.mode, HeadMountMode::DriverSynth);
	EXPECT_FALSE(ctx.headMount.allowRawHmdFallback);
	EXPECT_EQ(ctx.lockRelativePositionMode, CalibrationContext::LockMode::ON);
	EXPECT_FALSE(TrackingStyleRunsContinuous(ctx.trackingStyle));
	EXPECT_FALSE(TrackingStyleShowsBoundarySetup(ctx.trackingStyle));
}

TEST(TrackingStyleTest, LockRelativeModeHelpersUseExplicitOnOffOnly)
{
	EXPECT_EQ(LockRelativeModeFromEnabled(false), CalibrationContext::LockMode::OFF);
	EXPECT_EQ(LockRelativeModeFromEnabled(true), CalibrationContext::LockMode::ON);

	EXPECT_FALSE(LockRelativeModeEnabled(CalibrationContext::LockMode::OFF));
	EXPECT_TRUE(LockRelativeModeEnabled(CalibrationContext::LockMode::ON));
	EXPECT_FALSE(LockRelativeModeEnabled(CalibrationContext::LockMode::AUTO));

	EXPECT_FALSE(ResolveLockRelativePositionValue(CalibrationContext::LockMode::OFF, false));
	EXPECT_TRUE(ResolveLockRelativePositionValue(CalibrationContext::LockMode::ON, false));
	EXPECT_FALSE(ResolveLockRelativePositionValue(CalibrationContext::LockMode::AUTO, false));
	EXPECT_TRUE(ResolveLockRelativePositionValue(CalibrationContext::LockMode::OFF, true));
}

TEST(TrackingStyleTest, PreservingPresetKeepsManualLockRelativeOverride)
{
	CalibrationContext ctx;
	ctx.lockRelativePositionMode = CalibrationContext::LockMode::ON;

	ApplyTrackingStylePresetPreservingLockMode(ctx, TrackingStyle::Continuous);

	EXPECT_EQ(ctx.trackingStyle, TrackingStyle::Continuous);
	EXPECT_EQ(ctx.headMount.mode, HeadMountMode::Off);
	EXPECT_EQ(ctx.lockRelativePositionMode, CalibrationContext::LockMode::ON);

	ApplyTrackingStylePreset(ctx, TrackingStyle::Continuous);
	EXPECT_EQ(ctx.lockRelativePositionMode, CalibrationContext::LockMode::OFF);
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

TEST(TrackingStyleTest, CalibrationDeviceLocksArePlainContinuousOnly)
{
	EXPECT_TRUE(TrackingStylePublishesCalibrationDeviceLocks(TrackingStyle::Continuous));
	EXPECT_FALSE(TrackingStylePublishesCalibrationDeviceLocks(TrackingStyle::Manual));
	EXPECT_FALSE(TrackingStylePublishesCalibrationDeviceLocks(TrackingStyle::LockedWithRecovery));
	EXPECT_FALSE(TrackingStylePublishesCalibrationDeviceLocks(TrackingStyle::HardTrackerLock));
}

TEST(TrackingStyleTest, LegacyAutoLockRawMapsToOff)
{
	EXPECT_EQ(ExplicitLockModeFromRaw((int)CalibrationContext::LockMode::AUTO), CalibrationContext::LockMode::OFF);
	EXPECT_EQ(ExplicitLockModeFromRaw((int)CalibrationContext::LockMode::OFF), CalibrationContext::LockMode::OFF);
	EXPECT_EQ(ExplicitLockModeFromRaw((int)CalibrationContext::LockMode::ON), CalibrationContext::LockMode::ON);
}
