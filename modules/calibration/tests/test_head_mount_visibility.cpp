#include "HeadMountVisibility.h"

#include <gtest/gtest.h>

namespace hm = wkopenvr::headmount;

TEST(HeadMountVisibility, HideTrackerMatchesPersistedSerialOutsideContinuous)
{
    CalibrationContext ctx;
    ctx.state = CalibrationState::None;
    ctx.headMount.hideTracker = true;
    ctx.headMount.trackerSerial = "LHR-HEAD";
    ctx.headMount.trackerTrackingSystem = "lighthouse";
    ctx.headMount.deviceID = -1;

    EXPECT_TRUE(hm::ShouldHideHeadMountTracker(
        ctx, 9u, "LHR-HEAD", "lighthouse"));
    EXPECT_TRUE(hm::ShouldQuashPublishedTrackerPose(
        ctx, 9u, "LHR-HEAD", "lighthouse"));
}

TEST(HeadMountVisibility, HideTrackerDoesNotRequireHeadMountMode)
{
    CalibrationContext ctx;
    ctx.state = CalibrationState::ContinuousStandby;
    ctx.headMount.mode = HeadMountMode::Off;
    ctx.headMount.hideTracker = true;
    ctx.headMount.trackerSerial = "LHR-HEAD";
    ctx.headMount.trackerTrackingSystem = "lighthouse";

    EXPECT_TRUE(hm::ShouldHideHeadMountTracker(
        ctx, 9u, "LHR-HEAD", "lighthouse"));
}

TEST(HeadMountVisibility, HideTrackerRejectsHmdSlot)
{
    CalibrationContext ctx;
    ctx.headMount.hideTracker = true;
    ctx.headMount.trackerSerial = "LHR-HEAD";
    ctx.headMount.trackerTrackingSystem = "lighthouse";

    EXPECT_FALSE(hm::ShouldHideHeadMountTracker(
        ctx, vr::k_unTrackedDeviceIndex_Hmd, "LHR-HEAD", "lighthouse"));
}

TEST(HeadMountVisibility, HideTrackerRejectsDifferentSerialOrSystem)
{
    CalibrationContext ctx;
    ctx.headMount.hideTracker = true;
    ctx.headMount.trackerSerial = "LHR-HEAD";
    ctx.headMount.trackerTrackingSystem = "lighthouse";

    EXPECT_FALSE(hm::ShouldHideHeadMountTracker(
        ctx, 9u, "LHR-WAIST", "lighthouse"));
    EXPECT_FALSE(hm::ShouldHideHeadMountTracker(
        ctx, 9u, "LHR-HEAD", "oculus"));
}

TEST(HeadMountVisibility, ContinuousTargetHideStillUsesTargetToggle)
{
    CalibrationContext ctx;
    ctx.state = CalibrationState::Continuous;
    ctx.targetID = 12;
    ctx.quashTargetInContinuous = true;
    ctx.headMount.hideTracker = false;

    EXPECT_TRUE(hm::ShouldQuashPublishedTrackerPose(
        ctx, 12u, "LHR-TARGET", "lighthouse"));
}

TEST(HeadMountVisibility, ContinuousTargetHideOnlyRunsInContinuousState)
{
    CalibrationContext ctx;
    ctx.state = CalibrationState::ContinuousStandby;
    ctx.targetID = 12;
    ctx.quashTargetInContinuous = true;
    ctx.headMount.hideTracker = false;

    EXPECT_FALSE(hm::ShouldHideContinuousTarget(ctx, 12u));
    EXPECT_FALSE(hm::ShouldQuashPublishedTrackerPose(
        ctx, 12u, "LHR-TARGET", "lighthouse"));
}
