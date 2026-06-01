// Unit tests for the pure logic backing the head-mount group panel in
// UserInterfaceTabsBasic.cpp.
//
// No ImGui or OpenVR runtime required -- all logic under test is extracted
// as pure functions below (mirroring what the panel uses).

#include <gtest/gtest.h>
#include "Calibration.h"  // HeadMountMode, HeadMountConfig
#include "HeadMountOffsetPreflight.h"
#include "HeadMountTargetBinding.h"

// ---------------------------------------------------------------------------
// Pure helpers mirroring the panel's radio-enable gate
// ---------------------------------------------------------------------------

namespace {

// Returns true when the Corroborate radio item should be enabled.
// Mirrors: const bool canCorroborate = hm.offsetCalibrated;
bool CorroborateRadioEnabled(bool offsetCalibrated) {
    return offsetCalibrated;
}

// Returns true when the DriverSynth radio item should be enabled.
// Mirrors the same gate as Corroborate (both require offsetCalibrated).
bool DriverSynthRadioEnabled(bool offsetCalibrated) {
    return offsetCalibrated;
}

// Returns true when mode m requires offsetCalibrated to be meaningful.
bool ModeRequiresOffset(HeadMountMode m) {
    return wkopenvr::headmount::HeadMountModeUsesOffsetInContinuous(m);
}

CalibrationContext ReadyOffsetCalibrationContext() {
    CalibrationContext ctx;
    ctx.state = CalibrationState::Continuous;
    ctx.validProfile = true;
    ctx.relativePosCalibrated = true;
    ctx.targetID = 7;
    ctx.targetStandby.trackingSystem = "lighthouse";
    ctx.targetStandby.model = "VIVE Tracker 3.0";
    ctx.targetStandby.serial = "LHR-HEAD";
    wkopenvr::headmount::BindHeadMountToContinuousTarget(ctx);
    ctx.headMount.mode = HeadMountMode::Off;
    return ctx;
}

} // namespace

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

TEST(HeadMountPanelLogic, CorroborateRequiresOffset) {
    EXPECT_FALSE(CorroborateRadioEnabled(false));
    EXPECT_TRUE(CorroborateRadioEnabled(true));
}

TEST(HeadMountPanelLogic, DriverSynthRequiresOffset) {
    EXPECT_FALSE(DriverSynthRadioEnabled(false));
    EXPECT_TRUE(DriverSynthRadioEnabled(true));
}

TEST(HeadMountPanelLogic, OnlyOffDoesNotRequireOffset) {
    EXPECT_FALSE(ModeRequiresOffset(HeadMountMode::Off));
    EXPECT_TRUE(ModeRequiresOffset(HeadMountMode::AutoPaired));
}

TEST(HeadMountPanelLogic, CorroborateAndDriverSynthRequireOffset) {
    EXPECT_TRUE(ModeRequiresOffset(HeadMountMode::Corroborate));
    EXPECT_TRUE(ModeRequiresOffset(HeadMountMode::DriverSynth));
}

TEST(HeadMountPanelLogic, ModeEnumOrderIsStable) {
    // The driver encodes mode as uint32_t; the numeric values must not change.
    EXPECT_EQ(0u, static_cast<uint32_t>(HeadMountMode::Off));
    EXPECT_EQ(1u, static_cast<uint32_t>(HeadMountMode::AutoPaired));
    EXPECT_EQ(2u, static_cast<uint32_t>(HeadMountMode::Corroborate));
    EXPECT_EQ(3u, static_cast<uint32_t>(HeadMountMode::DriverSynth));
}

TEST(HeadMountPanelLogic, DefaultConfigHasOffModeAndNoOffset) {
    HeadMountConfig hm;
    EXPECT_EQ(HeadMountMode::Off, hm.mode);
    EXPECT_FALSE(hm.offsetCalibrated);
    EXPECT_TRUE(hm.autoCorrectOffset);
    EXPECT_TRUE(hm.trackerSerial.empty());
    EXPECT_EQ(-1, hm.deviceID);
    EXPECT_TRUE(wkopenvr::headmount::DriverSynthTimingIsDefault(
        hm.driverSynthTiming));
}

TEST(HeadMountPanelLogic, OffsetPreflightAllowsStableContinuousWithModeOff) {
    CalibrationContext ctx = ReadyOffsetCalibrationContext();

    const auto result = wkopenvr::headmount::EvaluateOffsetCalibrationPreflight(ctx);

    EXPECT_TRUE(result.ready);
    EXPECT_STREQ("ready", result.reason);
}

TEST(HeadMountPanelLogic, OffsetPreflightRequiresContinuousState) {
    CalibrationContext ctx = ReadyOffsetCalibrationContext();
    ctx.state = CalibrationState::ContinuousStandby;

    const auto result = wkopenvr::headmount::EvaluateOffsetCalibrationPreflight(ctx);

    EXPECT_FALSE(result.ready);
    EXPECT_STREQ("continuous_not_running", result.reason);
}

TEST(HeadMountPanelLogic, OffsetPreflightRequiresValidProfile) {
    CalibrationContext ctx = ReadyOffsetCalibrationContext();
    ctx.validProfile = false;

    const auto result = wkopenvr::headmount::EvaluateOffsetCalibrationPreflight(ctx);

    EXPECT_FALSE(result.ready);
    EXPECT_STREQ("profile_not_ready", result.reason);
}

TEST(HeadMountPanelLogic, OffsetPreflightRequiresRelativePoseLock) {
    CalibrationContext ctx = ReadyOffsetCalibrationContext();
    ctx.relativePosCalibrated = false;

    const auto result = wkopenvr::headmount::EvaluateOffsetCalibrationPreflight(ctx);

    EXPECT_FALSE(result.ready);
    EXPECT_STREQ("relative_pose_not_ready", result.reason);
}

TEST(HeadMountPanelLogic, OffsetPreflightBlocksModesThatUseSolvedOffset) {
    CalibrationContext ctx = ReadyOffsetCalibrationContext();

    ctx.headMount.mode = HeadMountMode::AutoPaired;
    auto result = wkopenvr::headmount::EvaluateOffsetCalibrationPreflight(ctx);
    EXPECT_FALSE(result.ready);
    EXPECT_STREQ("head_mount_mode_active", result.reason);

    ctx.headMount.mode = HeadMountMode::Corroborate;
    result = wkopenvr::headmount::EvaluateOffsetCalibrationPreflight(ctx);
    EXPECT_FALSE(result.ready);
    EXPECT_STREQ("head_mount_mode_active", result.reason);

    ctx.headMount.mode = HeadMountMode::DriverSynth;
    result = wkopenvr::headmount::EvaluateOffsetCalibrationPreflight(ctx);
    EXPECT_FALSE(result.ready);
    EXPECT_STREQ("head_mount_mode_active", result.reason);
}

TEST(HeadMountPanelLogic, CorroborateWithoutOffsetShouldBeDisabled) {
    HeadMountConfig hm;
    hm.mode = HeadMountMode::Corroborate;
    hm.offsetCalibrated = false;
    // The UI gate: Corroborate is visually disabled when !offsetCalibrated.
    EXPECT_FALSE(CorroborateRadioEnabled(hm.offsetCalibrated));
}

TEST(HeadMountPanelLogic, CorroborateWithOffsetShouldBeEnabled) {
    HeadMountConfig hm;
    hm.mode = HeadMountMode::Corroborate;
    hm.offsetCalibrated = true;
    EXPECT_TRUE(CorroborateRadioEnabled(hm.offsetCalibrated));
}

TEST(HeadMountPanelLogic, AutoPairedRequiresOffset) {
    HeadMountConfig hm;
    hm.mode = HeadMountMode::AutoPaired;
    hm.offsetCalibrated = false;
    EXPECT_TRUE(ModeRequiresOffset(hm.mode));
}

TEST(HeadMountPanelLogic, ContinuousBindingUsesTargetIdentity) {
    CalibrationContext ctx;
    ctx.state = CalibrationState::Continuous;
    ctx.targetID = 7;
    ctx.targetStandby.trackingSystem = "lighthouse";
    ctx.targetStandby.model = "VIVE Tracker 3.0";
    ctx.targetStandby.serial = "LHR-HEAD";

    EXPECT_TRUE(wkopenvr::headmount::BindHeadMountToContinuousTarget(ctx));
    EXPECT_EQ(ctx.headMount.trackerTrackingSystem, "lighthouse");
    EXPECT_EQ(ctx.headMount.trackerModel, "VIVE Tracker 3.0");
    EXPECT_EQ(ctx.headMount.trackerSerial, "LHR-HEAD");
    EXPECT_EQ(ctx.headMount.deviceID, 7);
}

TEST(HeadMountPanelLogic, NoneStateBindingMirrorsTargetIdentity) {
    // Binding must run in idle state so the Headset-tab status line reflects
    // the picked continuous target before the user clicks Start. Prior to
    // the idle-state fix this assertion was inverted -- the test locked in
    // the bug that left hm.deviceID at -1 with the status line red.
    CalibrationContext ctx;
    ctx.state = CalibrationState::None;
    ctx.targetID = 7;
    ctx.targetStandby.trackingSystem = "lighthouse";
    ctx.targetStandby.model = "VIVE Tracker 3.0";
    ctx.targetStandby.serial = "LHR-HEAD";

    EXPECT_TRUE(wkopenvr::headmount::BindHeadMountToContinuousTarget(ctx));
    EXPECT_EQ(ctx.headMount.trackerTrackingSystem, "lighthouse");
    EXPECT_EQ(ctx.headMount.trackerModel, "VIVE Tracker 3.0");
    EXPECT_EQ(ctx.headMount.trackerSerial, "LHR-HEAD");
    EXPECT_EQ(ctx.headMount.deviceID, 7);
}

TEST(HeadMountPanelLogic, BindingWithoutContinuousTargetIdentityStillNoOps) {
    CalibrationContext ctx;
    ctx.state = CalibrationState::None;
    ctx.targetID = 7;
    // targetStandby left empty -- HasContinuousTargetIdentity returns false.

    EXPECT_FALSE(wkopenvr::headmount::BindHeadMountToContinuousTarget(ctx));
    EXPECT_TRUE(ctx.headMount.trackerSerial.empty());
    EXPECT_EQ(ctx.headMount.deviceID, -1);
}

TEST(HeadMountPanelLogic, BindingFollowsTargetIDInvalidation) {
    CalibrationContext ctx;
    ctx.state = CalibrationState::None;
    ctx.targetID = 5;
    ctx.targetStandby.trackingSystem = "lighthouse";
    ctx.targetStandby.model = "VIVE Tracker 3.0";
    ctx.targetStandby.serial = "LHR-HEAD";

    EXPECT_TRUE(wkopenvr::headmount::BindHeadMountToContinuousTarget(ctx));
    EXPECT_EQ(ctx.headMount.deviceID, 5);

    ctx.targetID = -1;
    wkopenvr::headmount::BindHeadMountToContinuousTarget(ctx);
    EXPECT_EQ(ctx.headMount.deviceID, -1);
}

TEST(HeadMountPanelLogic, RebindingDifferentTargetClearsOffset) {
    CalibrationContext ctx;
    ctx.state = CalibrationState::ContinuousStandby;
    ctx.targetID = 9;
    ctx.targetStandby.trackingSystem = "lighthouse";
    ctx.targetStandby.model = "VIVE Tracker 3.0";
    ctx.targetStandby.serial = "LHR-NEW";
    ctx.headMount.trackerTrackingSystem = "lighthouse";
    ctx.headMount.trackerModel = "VIVE Tracker 3.0";
    ctx.headMount.trackerSerial = "LHR-OLD";
    ctx.headMount.offsetCalibrated = true;
    ctx.headMount.headFromTracker.translation() = Eigen::Vector3d(0.10, 0.20, 0.30);
    const uint32_t beforeVersion = ctx.headMountOffsetVersion;

    EXPECT_TRUE(wkopenvr::headmount::BindHeadMountToContinuousTarget(ctx));
    EXPECT_EQ(ctx.headMount.trackerSerial, "LHR-NEW");
    EXPECT_FALSE(ctx.headMount.offsetCalibrated);
    EXPECT_TRUE(ctx.headMount.headFromTracker.isApprox(Eigen::AffineCompact3d::Identity()));
    EXPECT_GT(ctx.headMountOffsetVersion, beforeVersion);
}

TEST(HeadMountPanelLogic, RebindingSameTargetPreservesOffset) {
    CalibrationContext ctx;
    ctx.state = CalibrationState::Continuous;
    ctx.targetID = 9;
    ctx.targetStandby.trackingSystem = "lighthouse";
    ctx.targetStandby.model = "VIVE Tracker 3.0";
    ctx.targetStandby.serial = "LHR-HEAD";
    ctx.headMount.trackerTrackingSystem = "lighthouse";
    ctx.headMount.trackerModel = "VIVE Tracker 3.0";
    ctx.headMount.trackerSerial = "LHR-HEAD";
    ctx.headMount.offsetCalibrated = true;
    ctx.headMount.headFromTracker.translation() = Eigen::Vector3d(0.10, 0.20, 0.30);
    const uint32_t beforeVersion = ctx.headMountOffsetVersion;

    EXPECT_TRUE(wkopenvr::headmount::BindHeadMountToContinuousTarget(ctx));
    EXPECT_TRUE(ctx.headMount.offsetCalibrated);
    EXPECT_NEAR(ctx.headMount.headFromTracker.translation().x(), 0.10, 1e-9);
    EXPECT_NEAR(ctx.headMount.headFromTracker.translation().y(), 0.20, 1e-9);
    EXPECT_NEAR(ctx.headMount.headFromTracker.translation().z(), 0.30, 1e-9);
    EXPECT_EQ(ctx.headMountOffsetVersion, beforeVersion);
}
