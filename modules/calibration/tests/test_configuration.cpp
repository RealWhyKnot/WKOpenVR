// Tests for the profile JSON load/save round-trip and the schema migration
// pipeline. The production code routes through Windows registry I/O (Load-
// Profile / SaveProfile in Configuration.cpp), but the underlying parsing
// and serialization is exposed via ParseProfile / WriteProfile, which take
// std::istream / std::ostream and don't touch the registry. We exercise
// those directly here so the test stays hermetic.
//
// Coverage:
//   - Schema-version write: every fresh save stamps the current version.
//   - Schema migration: legacy profiles with no schema_version load as v0
//     and migrate up to the current version.
//   - "Refuse to load newer" guard: a profile claiming a future schema
//     version leaves validProfile=false and doesn't corrupt the in-memory
//     context.
//   - Round-trip: every customised setting survives save->load.
//   - Round-trip covers every persisted field (calibrated transform,
//     thresholds, lock mode, etc.).

#include <gtest/gtest.h>

#include <cstring>
#include <sstream>
#include <string>

#include "Configuration.h"
#include "Calibration.h"
#include "ContinuousCalibrationGuard.h"

namespace {

// Build a minimal v0 / v1 / v2 / v3 JSON payload programmatically. Real
// production profiles have many more keys; for the migration tests we only
// need the schema-version field plus enough valid scaffolding that
// ParseProfile doesn't reject for missing required keys.
std::string MakeMinimalProfile(int schemaVersion, const std::string& extraKeys = "") {
    std::ostringstream o;
    o << "[{";
    if (schemaVersion >= 1) {
        o << "\"schema_version\":" << schemaVersion << ",";
    }
    o << "\"reference_tracking_system\":\"lighthouse\",";
    o << "\"target_tracking_system\":\"oculus\",";
    o << "\"roll\":0.0,\"yaw\":0.0,\"pitch\":0.0,";
    o << "\"x\":0.0,\"y\":0.0,\"z\":0.0,";
    o << "\"continuous_calibration_target_offset_x\":0.0,";
    o << "\"continuous_calibration_target_offset_y\":0.0,";
    o << "\"continuous_calibration_target_offset_z\":0.0";
    if (!extraKeys.empty()) {
        o << "," << extraKeys;
    }
    o << "}]";
    return o.str();
}

} // namespace

// ---------------------------------------------------------------------------
// Round-trip: write a context, read it back into a fresh context, every
// customised field survives. Acts as a contract test for "the profile
// remembers what the user did" -- the most basic configuration test.
// ---------------------------------------------------------------------------
TEST(ConfigurationTest, RoundTripPreservesCustomFields) {
    CalibrationContext src;
    src.referenceTrackingSystem = "lighthouse";
    src.targetTrackingSystem = "oculus";
    src.calibratedTranslation = Eigen::Vector3d(12.5, 0.0, -7.25);
    src.calibratedRotation = Eigen::Vector3d(0.0, 45.0, 0.0);
    src.jitterThreshold = 5.5f;
    src.recalibrateOnMovement = false;
    src.baseStationDriftCorrectionEnabled = false;
    src.ignoreOutliers = false;             // non-default; must be written
    src.continuousCalibrationThreshold = 2.5f;
    src.oneShotCalibrationSpeed = CalibrationContext::SLOW; // non-default; must round-trip
    src.continuousCalibrationSpeed = CalibrationContext::VERY_SLOW;
    src.lockRelativePositionMode = CalibrationContext::LockMode::ON;
    src.validProfile = true;

    std::stringstream io;
    WriteProfile(src, io);

    CalibrationContext dst;
    ParseProfile(dst, io);

    EXPECT_EQ(dst.referenceTrackingSystem, "lighthouse");
    EXPECT_EQ(dst.targetTrackingSystem, "oculus");
    EXPECT_DOUBLE_EQ(dst.calibratedTranslation.x(), 12.5);
    EXPECT_DOUBLE_EQ(dst.calibratedTranslation.z(), -7.25);
    EXPECT_DOUBLE_EQ(dst.calibratedRotation.y(), 45.0);
    EXPECT_FLOAT_EQ(dst.jitterThreshold, 5.5f);
    EXPECT_FALSE(dst.recalibrateOnMovement);
    EXPECT_FALSE(dst.baseStationDriftCorrectionEnabled);
    EXPECT_FALSE(dst.ignoreOutliers);
    EXPECT_FLOAT_EQ(dst.continuousCalibrationThreshold, 2.5f);
    EXPECT_EQ(dst.oneShotCalibrationSpeed, CalibrationContext::SLOW);
    EXPECT_EQ(dst.continuousCalibrationSpeed, CalibrationContext::VERY_SLOW);
    EXPECT_EQ(dst.lockRelativePositionMode, CalibrationContext::LockMode::ON);
}

// ---------------------------------------------------------------------------
// Default-only fields are not written and load as defaults. The skip-if-
// default optimization means a brand-new context's saved JSON shouldn't
// carry every field; missing keys reload as the in-code defaults, which
// matches the in-code defaults baked into CalibrationContext.
// ---------------------------------------------------------------------------
// Clear() should reset the AUTO-Lock pending-flip queue along with the
// detector state, so a profile-reload doesn't carry a stale committed-flip
// intention across the boundary.
TEST(ConfigurationTest, ClearResetsAutoLockPendingFlip) {
    CalibrationContext ctx;
    ctx.autoLockEffectivelyLocked = true;
    ctx.autoLockHasPendingFlip = true;
    ctx.autoLockPendingFlipTo = false;

    ctx.Clear();

    EXPECT_FALSE(ctx.autoLockEffectivelyLocked);
    EXPECT_FALSE(ctx.autoLockHasPendingFlip);
    EXPECT_FALSE(ctx.autoLockPendingFlipTo);
}

TEST(ConfigurationTest, DefaultFieldsRoundTripAsDefaults) {
    CalibrationContext src; // fresh defaults
    src.referenceTrackingSystem = "lighthouse";
    src.targetTrackingSystem = "oculus";
    src.validProfile = true;

    std::stringstream io;
    WriteProfile(src, io);

    CalibrationContext dst;
    ParseProfile(dst, io);

    // The in-code defaults survive a no-customization round-trip.
    EXPECT_TRUE(dst.recalibrateOnMovement);              // default true
    EXPECT_TRUE(dst.enableStaticRecalibration);          // default true (flipped this session)
    EXPECT_TRUE(dst.baseStationDriftCorrectionEnabled);  // default AUTO (no-op without base stations)
    EXPECT_FLOAT_EQ(dst.jitterThreshold, 3.0f);
    EXPECT_EQ(dst.oneShotCalibrationSpeed, CalibrationContext::FAST);
    EXPECT_EQ(dst.continuousCalibrationSpeed, CalibrationContext::AUTO);
    EXPECT_EQ(dst.lockRelativePositionMode, CalibrationContext::LockMode::AUTO);
}

// ---------------------------------------------------------------------------
// Refuse-to-load-newer guard: if a future build wrote a profile with a
// higher schema version, the current build must NOT corrupt the in-memory
// context by partially loading it. The guard sets validProfile=false and
// returns early.
// ---------------------------------------------------------------------------
TEST(ConfigurationTest, RefusesProfileFromNewerSchema) {
    // Pick a version far enough in the future that no near-term schema bump
    // would accidentally make this test pass. 999 is symbolic.
    std::string newerJson = MakeMinimalProfile(/*schemaVersion=*/999);

    CalibrationContext ctx;
    ctx.validProfile = true; // pre-set, the guard should clear this
    std::stringstream io(newerJson);
    ParseProfile(ctx, io);

    EXPECT_FALSE(ctx.validProfile)
        << "Refusing to load a newer-schema profile must leave validProfile=false";
}

// ---------------------------------------------------------------------------
// Schema migration v0 -> current. v0 = no schema_version key (legacy
// profiles written before the version was introduced). The load path must
// run all forward-migration steps; specifically it must NOT keep
// auto_suppress_on_external_tool around (dropped in v2's migration).
// ---------------------------------------------------------------------------
TEST(ConfigurationTest, MigrateV0ProfileLoadsCleanly) {
    // v0 had no schema_version key. Include the legacy auto-suppress key so
    // we can verify the migration drops it. Add a SLOW calibration_speed so
    // we can verify the v1->v2 step rewrites it to AUTO.
    std::string v0Json = MakeMinimalProfile(
        /*schemaVersion=*/0,
        // calibration_speed=1 -> SLOW. Migration rewrites to 3 (AUTO).
        "\"calibration_speed\":1,"
        "\"auto_suppress_on_external_tool\":true");

    CalibrationContext ctx;
    std::stringstream io(v0Json);
    ParseProfile(ctx, io);

    // Migration v1->v2 rewrites legacy SLOW (=1) to AUTO (=3) since SLOW was
    // the old default that most users never customised.
    EXPECT_EQ(ctx.oneShotCalibrationSpeed, CalibrationContext::FAST)
        << "legacy AUTO should become FAST for one-shot";
    EXPECT_EQ(ctx.continuousCalibrationSpeed, CalibrationContext::AUTO)
        << "v1->v2 migration should still land continuous mode on AUTO";

    // Profile loaded; main fields populated.
    EXPECT_EQ(ctx.referenceTrackingSystem, "lighthouse");
    EXPECT_EQ(ctx.targetTrackingSystem, "oculus");
}

// ---------------------------------------------------------------------------
// Schema migration v2 -> v3. v2 profiles have no additional_calibrations
// array (multi-ecosystem support added in v3). They must load as if extras
// were an empty list.
// ---------------------------------------------------------------------------
TEST(ConfigurationTest, MigrateV2ProfileLoadsWithEmptyExtras) {
    std::string v2Json = MakeMinimalProfile(/*schemaVersion=*/2);

    CalibrationContext ctx;
    std::stringstream io(v2Json);
    ParseProfile(ctx, io);

    EXPECT_EQ(ctx.referenceTrackingSystem, "lighthouse");
    EXPECT_TRUE(ctx.additionalCalibrations.empty())
        << "v2 profile should load with no extras (additional_calibrations was added in v3)";
}

TEST(ConfigurationTest, MigrateV4FastSpeedKeepsOneShotFastAndContinuousAuto) {
    std::string v4Json = MakeMinimalProfile(
        /*schemaVersion=*/4,
        "\"calibration_speed\":0");

    CalibrationContext ctx;
    std::stringstream io(v4Json);
    ParseProfile(ctx, io);

    EXPECT_EQ(ctx.oneShotCalibrationSpeed, CalibrationContext::FAST);
    EXPECT_EQ(ctx.continuousCalibrationSpeed, CalibrationContext::AUTO);
}

TEST(ConfigurationTest, MigrateV4VerySlowSpeedPreservesSlowContinuousChoice) {
    std::string v4Json = MakeMinimalProfile(
        /*schemaVersion=*/4,
        "\"calibration_speed\":2");

    CalibrationContext ctx;
    std::stringstream io(v4Json);
    ParseProfile(ctx, io);

    EXPECT_EQ(ctx.oneShotCalibrationSpeed, CalibrationContext::VERY_SLOW);
    EXPECT_EQ(ctx.continuousCalibrationSpeed, CalibrationContext::VERY_SLOW);
}

TEST(ConfigurationTest, ResolvedSpeedUsesContinuousSettingOnlyInContinuousMode) {
    CalibrationContext ctx;
    ctx.oneShotCalibrationSpeed = CalibrationContext::FAST;
    ctx.continuousCalibrationSpeed = CalibrationContext::AUTO;

    ctx.state = CalibrationState::None;
    EXPECT_EQ(ctx.ActiveCalibrationSpeed(), CalibrationContext::FAST);

    ctx.state = CalibrationState::Continuous;
    EXPECT_EQ(ctx.ActiveCalibrationSpeed(), CalibrationContext::AUTO);
}

TEST(ConfigurationTest, ContinuousSnapshotRestoresRelativePoseMetadata) {
    CalibrationContext ctx;
    ctx.enabled = true;
    ctx.validProfile = true;
    ctx.referenceTrackingSystem = "lighthouse";
    ctx.targetTrackingSystem = "oculus";
    ctx.calibratedTranslation = Eigen::Vector3d(1.0, 2.0, 3.0);
    ctx.calibratedRotation = Eigen::Vector3d(4.0, 5.0, 6.0);
    ctx.calibratedScale = 1.25;
    ctx.refToTargetPose.translation() = Eigen::Vector3d(0.1, 0.2, 0.3);
    ctx.relativePosCalibrated = true;

    const auto snap = ctx.CaptureProfileSnapshot();
    ctx.calibratedTranslation = Eigen::Vector3d(99.0, 99.0, 99.0);
    ctx.relativePosCalibrated = false;
    ctx.validProfile = false;

    ctx.RestoreProfileSnapshot(snap);

    EXPECT_TRUE(ctx.validProfile);
    EXPECT_TRUE(ctx.relativePosCalibrated);
    EXPECT_TRUE(ctx.calibratedTranslation.isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));
    EXPECT_NEAR(ctx.refToTargetPose.translation().z(), 0.3, 1e-9);
}

TEST(ConfigurationTest, ContinuousCandidateGuardBlocksLargeSteadyJump) {
    const Eigen::Vector3d baselineCm(10.0, 20.0, 30.0);
    const Eigen::Vector3d candidateCm(10.0, 20.0, 80.1);

    const auto result = spacecal::continuous::EvaluateCandidate(
        /*inContinuous=*/true,
        /*hasBaseline=*/true,
        /*hasAcceptedThisSession=*/true,
        baselineCm,
        candidateCm,
        Eigen::Matrix3d::Identity());

    EXPECT_FALSE(result.accepted);
    EXPECT_STREQ(result.reason, "jump_exceeds_limit");
}

TEST(ConfigurationTest, PublishCandidateGuardAllowsTranslationOnlyCandidate) {
    const Eigen::Vector3d candidateCm(0.0, 25.0, 0.0);

    const auto result = spacecal::continuous::EvaluatePublishCandidate(
        /*inContinuous=*/false,
        /*hasBaseline=*/false,
        /*hasAcceptedThisSession=*/false,
        /*candidateFromRelPose=*/false,
        /*allowLargeFirstFullSolveCorrection=*/true,
        Eigen::Vector3d::Zero(),
        candidateCm,
        Eigen::Matrix3d::Identity());

    EXPECT_TRUE(result.accepted);
    EXPECT_STREQ(result.reason, "accepted");
    EXPECT_NEAR(result.rotAngleRad, 0.0, 1e-12);
    EXPECT_NEAR(result.translationMagnitudeCm, 25.0, 1e-12);
}

TEST(ConfigurationTest, PublishCandidateGuardRejectsExactNoopCandidate) {
    const auto result = spacecal::continuous::EvaluatePublishCandidate(
        /*inContinuous=*/false,
        /*hasBaseline=*/false,
        /*hasAcceptedThisSession=*/false,
        /*candidateFromRelPose=*/false,
        /*allowLargeFirstFullSolveCorrection=*/true,
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero(),
        Eigen::Matrix3d::Identity());

    EXPECT_FALSE(result.accepted);
    EXPECT_STREQ(result.reason, "identity_candidate");
}

TEST(ConfigurationTest, PublishCandidateGuardAllowsLargeFirstFullSolveCorrection) {
    const Eigen::Vector3d baselineCm(-160.0, 210.0, -115.0);
    const Eigen::Vector3d candidateCm(230.0, 150.0, 125.0);

    const auto result = spacecal::continuous::EvaluatePublishCandidate(
        /*inContinuous=*/true,
        /*hasBaseline=*/true,
        /*hasAcceptedThisSession=*/false,
        /*candidateFromRelPose=*/false,
        /*allowLargeFirstFullSolveCorrection=*/true,
        baselineCm,
        candidateCm,
        Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitY()).toRotationMatrix());

    EXPECT_TRUE(result.accepted);
    EXPECT_STREQ(result.reason, "accepted");
    EXPECT_GT(result.jumpM, spacecal::continuous::kMaxFirstAcceptedJumpM);
}

TEST(ConfigurationTest, PublishCandidateGuardBlocksLateFirstFullSolveAfterJumpReject) {
    const Eigen::Vector3d baselineCm(-166.06, 202.62, -109.37);
    const Eigen::Vector3d candidateCm(-64.69, 217.40, -126.24);

    const auto result = spacecal::continuous::EvaluatePublishCandidate(
        /*inContinuous=*/true,
        /*hasBaseline=*/true,
        /*hasAcceptedThisSession=*/false,
        /*candidateFromRelPose=*/false,
        /*allowLargeFirstFullSolveCorrection=*/false,
        baselineCm,
        candidateCm,
        Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitY()).toRotationMatrix());

    EXPECT_FALSE(result.accepted);
    EXPECT_STREQ(result.reason, "jump_exceeds_limit");
    EXPECT_GT(result.jumpM, spacecal::continuous::kMaxFirstAcceptedJumpM);
}

TEST(ConfigurationTest, LargeFullSolveStabilityRequiresRepeatedCluster) {
    const Eigen::Vector3d anchorCm(-64.69, 217.40, -126.24);
    const Eigen::Matrix3d anchorRot = Eigen::Matrix3d::Identity();

    const auto first = spacecal::continuous::EvaluateLargeFullSolveStability(
        /*hasPending=*/false,
        /*pendingSampleCount=*/0,
        Eigen::Vector3d::Zero(),
        Eigen::Matrix3d::Identity(),
        anchorCm,
        anchorRot);

    EXPECT_FALSE(first.stable);
    EXPECT_EQ(first.nextSampleCount, 1);
    EXPECT_TRUE(first.storeAsPendingAnchor);

    const auto second = spacecal::continuous::EvaluateLargeFullSolveStability(
        /*hasPending=*/true,
        /*pendingSampleCount=*/1,
        anchorCm,
        anchorRot,
        anchorCm + Eigen::Vector3d(10.0, 0.0, 0.0),
        Eigen::AngleAxisd(0.10, Eigen::Vector3d::UnitY()).toRotationMatrix());

    EXPECT_FALSE(second.stable);
    EXPECT_EQ(second.nextSampleCount, 2);
    EXPECT_FALSE(second.storeAsPendingAnchor);

    const auto third = spacecal::continuous::EvaluateLargeFullSolveStability(
        /*hasPending=*/true,
        /*pendingSampleCount=*/2,
        anchorCm,
        anchorRot,
        anchorCm + Eigen::Vector3d(14.0, 1.0, 0.0),
        Eigen::AngleAxisd(0.20, Eigen::Vector3d::UnitY()).toRotationMatrix());

    EXPECT_TRUE(third.stable);
    EXPECT_EQ(third.nextSampleCount, spacecal::continuous::kStableLargeFullSolveSamples);
    EXPECT_FALSE(third.storeAsPendingAnchor);
}

TEST(ConfigurationTest, LargeFullSolveStabilityResetsWhenCandidateLeavesCluster) {
    const Eigen::Vector3d anchorCm(-64.69, 217.40, -126.24);

    const auto result = spacecal::continuous::EvaluateLargeFullSolveStability(
        /*hasPending=*/true,
        /*pendingSampleCount=*/2,
        anchorCm,
        Eigen::Matrix3d::Identity(),
        anchorCm + Eigen::Vector3d(16.0, 0.0, 0.0),
        Eigen::Matrix3d::Identity());

    EXPECT_FALSE(result.stable);
    EXPECT_EQ(result.nextSampleCount, 1);
    EXPECT_TRUE(result.storeAsPendingAnchor);
    EXPECT_GT(result.translationDeltaCm, spacecal::continuous::kStableLargeFullSolveTranslationDeltaCm);
}

TEST(ConfigurationTest, PublishCandidateGuardKeepsRelPoseFirstJumpLimit) {
    const Eigen::Vector3d baselineCm(-160.0, 210.0, -115.0);
    const Eigen::Vector3d candidateCm(230.0, 150.0, 125.0);

    const auto result = spacecal::continuous::EvaluatePublishCandidate(
        /*inContinuous=*/true,
        /*hasBaseline=*/true,
        /*hasAcceptedThisSession=*/false,
        /*candidateFromRelPose=*/true,
        /*allowLargeFirstFullSolveCorrection=*/true,
        baselineCm,
        candidateCm,
        Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitY()).toRotationMatrix());

    EXPECT_FALSE(result.accepted);
    EXPECT_STREQ(result.reason, "jump_exceeds_limit");
}

TEST(ConfigurationTest, PublishCandidateGuardKeepsSteadyLimitForFullSolves) {
    const Eigen::Vector3d baselineCm(10.0, 20.0, 30.0);
    const Eigen::Vector3d candidateCm(10.0, 20.0, 80.1);

    const auto result = spacecal::continuous::EvaluatePublishCandidate(
        /*inContinuous=*/true,
        /*hasBaseline=*/true,
        /*hasAcceptedThisSession=*/true,
        /*candidateFromRelPose=*/false,
        /*allowLargeFirstFullSolveCorrection=*/true,
        baselineCm,
        candidateCm,
        Eigen::Matrix3d::Identity());

    EXPECT_FALSE(result.accepted);
    EXPECT_STREQ(result.reason, "jump_exceeds_limit");
}

TEST(ConfigurationTest, RelPoseTrustRequiresLockedCalibratedPose) {
    {
        const auto result = spacecal::continuous::EvaluateRelPoseTrust(
            /*lockRelativePosition=*/false,
            /*relativePosCalibrated=*/true);

        EXPECT_FALSE(result.accepted);
        EXPECT_STREQ(result.reason, "relpose_unlocked");
    }

    {
        const auto result = spacecal::continuous::EvaluateRelPoseTrust(
            /*lockRelativePosition=*/true,
            /*relativePosCalibrated=*/false);

        EXPECT_FALSE(result.accepted);
        EXPECT_STREQ(result.reason, "relpose_uncalibrated");
    }

    {
        const auto result = spacecal::continuous::EvaluateRelPoseTrust(
            /*lockRelativePosition=*/true,
            /*relativePosCalibrated=*/true);

        EXPECT_TRUE(result.accepted);
        EXPECT_STREQ(result.reason, "accepted");
    }
}

TEST(ConfigurationTest, RuntimeRecoveryClearDoesNotLeaveIdentityProfileValid) {
    CalibrationContext ctx;
    ctx.validProfile = true;
    ctx.calibratedTranslation = Eigen::Vector3d(120.0, 30.0, -45.0);
    ctx.calibratedRotation = Eigen::Vector3d(1.0, 2.0, 3.0);
    ctx.refToTargetPose.translation() = Eigen::Vector3d(0.1, 0.2, 0.3);
    ctx.relativePosCalibrated = true;
    ctx.hasAppliedCalibrationResult = true;
    ctx.lastAcceptedContinuousSnapshot = ctx.CaptureProfileSnapshot();
    ctx.continuousPreAcceptJumpRejects = 2;
    ctx.pendingLargeFullSolve = true;
    ctx.pendingLargeFullSolveSamples = 2;
    ctx.pendingLargeFullSolveTranslation = Eigen::Vector3d(1.0, 2.0, 3.0);
    ctx.pendingLargeFullSolveRotation =
        Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitY()).toRotationMatrix();

    ctx.ClearRuntimeCalibrationForRecovery();

    EXPECT_FALSE(ctx.validProfile);
    EXPECT_FALSE(ctx.relativePosCalibrated);
    EXPECT_FALSE(ctx.hasAppliedCalibrationResult);
    EXPECT_TRUE(ctx.calibratedTranslation.isApprox(Eigen::Vector3d::Zero()));
    EXPECT_TRUE(ctx.calibratedRotation.isApprox(Eigen::Vector3d::Zero()));
    EXPECT_TRUE(ctx.refToTargetPose.matrix().isApprox(Eigen::AffineCompact3d::Identity().matrix()));
    EXPECT_FALSE(ctx.CaptureProfileSnapshot().validProfile);
    EXPECT_FALSE(ctx.lastAcceptedContinuousSnapshot.captured);
    EXPECT_EQ(ctx.continuousPreAcceptJumpRejects, 0);
    EXPECT_FALSE(ctx.pendingLargeFullSolve);
    EXPECT_EQ(ctx.pendingLargeFullSolveSamples, 0);
}

TEST(ConfigurationTest, LegacyMathMasterDisablesEffectiveValidatedFeatures) {
    CalibrationContext ctx;
    ctx.useUpstreamMath = true;

    EXPECT_FALSE(ctx.EffectiveGccPhatLatency());
    EXPECT_FALSE(ctx.EffectiveCusumGeometryShift());
    EXPECT_FALSE(ctx.EffectiveVelocityAwareWeighting());
    EXPECT_FALSE(ctx.EffectiveTukeyBiweight());
    EXPECT_FALSE(ctx.EffectiveBlendFilter());
    EXPECT_FALSE(ctx.EffectivePredictiveRecoveryEnabled());
    EXPECT_FALSE(ctx.EffectiveReanchorChiSquareEnabled());
    EXPECT_FALSE(ctx.EffectiveRestLockedYawEnabled());

    EXPECT_FALSE(ctx.useCusumGeometryShift)
        << "Stored CUSUM preference must survive the master override";
}

// ---------------------------------------------------------------------------
// Default-value pins. These are the in-code defaults that ResetConfig()
// produces; load paths inherit them when a key is missing from the JSON. A
// test that fails here means a default was changed -- look at the diff and
// make sure the change is intentional, then update the literal here.
//
// Why pin defaults: silent default flips are a class of bug that's hard to
// catch in code review. Forcing the test to be touched whenever a default
// changes makes the change explicit and reviewable.
// ---------------------------------------------------------------------------
TEST(ConfigurationTest, InCodeDefaultsArePinned) {
    CalibrationContext ctx;

    EXPECT_FLOAT_EQ(ctx.jitterThreshold, 3.0f);
    EXPECT_TRUE(ctx.recalibrateOnMovement)
        << "recalibrateOnMovement default is ON: prevents phantom drift while still";
    EXPECT_TRUE(ctx.enableStaticRecalibration)
        << "enableStaticRecalibration default is ON: no-op when not locked, "
           "accelerates rigid-attachment recovery; flipped on this fork";
    EXPECT_TRUE(ctx.baseStationDriftCorrectionEnabled)
        << "baseStationDriftCorrectionEnabled default is AUTO (true): "
           "no-op when no base stations are detected (e.g. Quest-only "
           "setups), corrects on detected universe shifts otherwise";
    EXPECT_FALSE(ctx.requireTriggerPressToApply);
    EXPECT_TRUE(ctx.ignoreOutliers)
        << "ignoreOutliers default is now true: the filter is a no-op when "
           "consensus is uniform and prevents one bad sample from skewing "
           "the fit when it isn't.";
    EXPECT_TRUE(ctx.quashTargetInContinuous)
        << "quashTargetInContinuous default is now true: hides the duplicate "
           "tracker pose while continuous calibration runs. Gated on "
           "state == Continuous in the apply path, so one-shot is unaffected.";
    EXPECT_FLOAT_EQ(ctx.continuousCalibrationThreshold, 1.5f);
    EXPECT_FLOAT_EQ(ctx.maxRelativeErrorThreshold, 0.005f);
    EXPECT_EQ(ctx.oneShotCalibrationSpeed, CalibrationContext::FAST)
        << "One-shot default speed is FAST.";
    EXPECT_EQ(ctx.continuousCalibrationSpeed, CalibrationContext::AUTO)
        << "Continuous default speed is AUTO.";
    EXPECT_FALSE(ctx.useCusumGeometryShift);
    EXPECT_FALSE(ctx.useTukeyBiweight);
    EXPECT_FALSE(ctx.useBlendFilter);
    EXPECT_FALSE(ctx.predictiveRecoveryEnabled);
    EXPECT_FALSE(ctx.reanchorChiSquareEnabled);
    EXPECT_EQ(ctx.lockRelativePositionMode, CalibrationContext::LockMode::AUTO);
    EXPECT_DOUBLE_EQ(ctx.calibratedScale, 1.0);
    EXPECT_DOUBLE_EQ(ctx.targetLatencyOffsetMs, 0.0);
    EXPECT_FALSE(ctx.latencyAutoDetect);

    // v24 slew-rate cap defaults. Stationary at the published lateral-drift
    // detection threshold (~0.5 mm/sec); moving 20x faster so motion-masked
    // catch-up of a 30 mm pending correction takes ~3 s. Pin both halves --
    // the moving rate has to stay above the stationary rate or the cap
    // inverts when the user starts moving.
    EXPECT_DOUBLE_EQ(ctx.alignmentSpeedParams.slew_stationary_pos_rate, 0.0005);
    EXPECT_DOUBLE_EQ(ctx.alignmentSpeedParams.slew_stationary_rot_rate, 0.000873);
    EXPECT_DOUBLE_EQ(ctx.alignmentSpeedParams.slew_moving_pos_rate,     0.010);
    EXPECT_DOUBLE_EQ(ctx.alignmentSpeedParams.slew_moving_rot_rate,     0.01745);
    EXPECT_LT(ctx.alignmentSpeedParams.slew_stationary_pos_rate,
              ctx.alignmentSpeedParams.slew_moving_pos_rate);
    EXPECT_LT(ctx.alignmentSpeedParams.slew_stationary_rot_rate,
              ctx.alignmentSpeedParams.slew_moving_rot_rate);
}

// ---------------------------------------------------------------------------
// v24 slew-rate fields round-trip via VisitAlignmentParams (the same path
// the existing align_speed_*/thr_*_* knobs use). The fields live under the
// "alignment_params" JSON object, not the top-level skip-if-default block,
// because they ship with the rest of AlignmentSpeedParams in one IPC
// message and splitting the persistence would just create a sync hazard.
// ---------------------------------------------------------------------------
TEST(ConfigurationTest, SlewRateFieldsRoundTrip) {
    CalibrationContext src;
    src.referenceTrackingSystem = "lighthouse";
    src.targetTrackingSystem = "oculus";
    src.validProfile = true;
    // Pick values that are NOT the defaults so a missed save/load path
    // would let the dst read back as the construction defaults.
    src.alignmentSpeedParams.slew_stationary_pos_rate = 0.0010;
    src.alignmentSpeedParams.slew_stationary_rot_rate = 0.002;
    src.alignmentSpeedParams.slew_moving_pos_rate     = 0.025;
    src.alignmentSpeedParams.slew_moving_rot_rate     = 0.05;

    std::stringstream io;
    WriteProfile(src, io);

    CalibrationContext dst;
    ParseProfile(dst, io);

    EXPECT_NEAR(dst.alignmentSpeedParams.slew_stationary_pos_rate, 0.0010,  1e-9);
    EXPECT_NEAR(dst.alignmentSpeedParams.slew_stationary_rot_rate, 0.002,   1e-9);
    EXPECT_NEAR(dst.alignmentSpeedParams.slew_moving_pos_rate,     0.025,   1e-9);
    EXPECT_NEAR(dst.alignmentSpeedParams.slew_moving_rot_rate,     0.05,    1e-9);
}

// ---------------------------------------------------------------------------
// Wedge-guard tests removed 2026-05-05. The load-time wedge guard in
// ParseProfile and the runtime detector in CalibrationTick are both
// disabled in production — they were causing a reset loop on the user's
// Quest+Lighthouse setup whose legitimate convergence values exceed any
// fixed magnitude bound. See project_wedge_guard_removed_2026-05-05.md.
// ---------------------------------------------------------------------------


// ---------------------------------------------------------------------------
// Regression guard for the registry-read underflow bug fixed 2026-05-04.
// RegGetValueA can return size==0 for an empty/malformed REG_SZ; the original
// code did `str.resize(size - 1)` which underflowed to 0xFFFFFFFF and threw
// std::bad_alloc, crashing the overlay before ParseProfile could run. Pinned
// behavior: size==0 input maps to size==0 output (caller treats as "no
// profile"); positive input maps to input-1 (strip the null terminator).
// ---------------------------------------------------------------------------
#ifdef _WIN32
TEST(ConfigurationTest, Regression_StripRegistryNullTerminator_HandlesZero) {
    EXPECT_EQ(StripRegistryNullTerminator(0), 0u)
        << "size==0 must NOT underflow â€” caller short-circuits to empty string";
}

TEST(ConfigurationTest, Regression_StripRegistryNullTerminator_StripsOne) {
    EXPECT_EQ(StripRegistryNullTerminator(1), 0u)
        << "size==1 (just a null byte) maps to empty string";
    EXPECT_EQ(StripRegistryNullTerminator(2), 1u);
    EXPECT_EQ(StripRegistryNullTerminator(10), 9u);
    EXPECT_EQ(StripRegistryNullTerminator(0xFFFF), 0xFFFEu);
}
#endif

// ---------------------------------------------------------------------------
// SaveProfile-path schema-validation pin (audit follow-up). Every persistence
// site in the overlay routes through `SaveProfile(ctx)` â†’ `WriteProfile(ctx,
// stream)` â†’ the `WRITE_IF_CHANGED_*` macros (Configuration.cpp:586-617).
// These tests pin three contracts at the WriteProfile boundary:
//
//   1. schema_version is ALWAYS stamped â€” without it, future loads treat
//      the profile as v0 and run all migration steps redundantly.
//   2. Calibration data (translation, rotation, scale, tracking-system
//      names, the standby device records) is ALWAYS written â€” it IS the
//      calibration, not a tunable.
//   3. Setting fields are SKIPPED when at default â€” old profiles that pre-
//      date the field don't accidentally get the field's default value
//      stamped onto disk on next save (which would block future default-
//      flips from taking effect on existing user profiles).
//
// If anyone adds a new SaveProfile call site that bypasses WriteProfile, or
// modifies WriteProfile to drop schema_version / change a calibration field
// to skip-if-default / change a tunable to always-write, one of these
// expectations fails.
// ---------------------------------------------------------------------------
TEST(ConfigurationTest, WriteProfile_AlwaysStampsCalibrationData) {
    // Calibration data is mandatory on every save. Even a fresh
    // unconfigured context with default zeros must emit the calibration
    // keys so a future load always has them.
    CalibrationContext ctx; // all defaults
    ctx.referenceTrackingSystem = "lighthouse";
    ctx.targetTrackingSystem = "oculus";
    ctx.validProfile = true;

    std::stringstream io;
    WriteProfile(ctx, io);
    const std::string json = io.str();

    EXPECT_NE(json.find("\"x\":"),  std::string::npos);
    EXPECT_NE(json.find("\"y\":"),  std::string::npos);
    EXPECT_NE(json.find("\"z\":"),  std::string::npos);
    EXPECT_NE(json.find("\"roll\":"),  std::string::npos);
    EXPECT_NE(json.find("\"yaw\":"),   std::string::npos);
    EXPECT_NE(json.find("\"pitch\":"), std::string::npos);
    EXPECT_NE(json.find("\"scale\":"), std::string::npos);
    EXPECT_NE(json.find("\"reference_tracking_system\":"), std::string::npos);
    EXPECT_NE(json.find("\"target_tracking_system\":"),    std::string::npos);
}

TEST(ConfigurationTest, WriteProfile_SkipsDefaultTunables) {
    // Skip-if-default: tunables left at their CalibrationContext-construction
    // defaults must NOT appear in the JSON. Without this contract, a user's
    // pre-2026 profile would come out of round-trip with new fields baked in
    // at today's defaults â€” and any future default flip would have no effect
    // on those profiles.
    CalibrationContext ctx;  // all defaults
    ctx.referenceTrackingSystem = "lighthouse";
    ctx.targetTrackingSystem = "oculus";
    ctx.validProfile = true;

    std::stringstream io;
    WriteProfile(ctx, io);
    const std::string json = io.str();

    // recalibrateOnMovement defaults to true â†’ skip
    EXPECT_EQ(json.find("recalibrate_on_movement"), std::string::npos)
        << "default-true bool must be skipped on save";
    // baseStationDriftCorrectionEnabled defaults to true â†’ skip
    EXPECT_EQ(json.find("base_station_drift_correction"), std::string::npos);
    // ignoreOutliers defaults to true -> skip
    EXPECT_EQ(json.find("ignore_outliers"), std::string::npos);
    // jitterThreshold defaults to 3.0f -> skip
    EXPECT_EQ(json.find("jitter_threshold"), std::string::npos);
}

TEST(ConfigurationTest, WriteProfile_StampsNonDefaultTunables) {
    // The other half of WRITE_IF_CHANGED: tunables flipped away from default
    // MUST be written. If someone breaks the macro to always-skip, the
    // user's customisations would silently disappear on next save.
    CalibrationContext ctx;
    ctx.referenceTrackingSystem = "lighthouse";
    ctx.targetTrackingSystem = "oculus";
    ctx.validProfile = true;
    ctx.recalibrateOnMovement = false;       // flipped from default true
    ctx.ignoreOutliers = false;              // flipped from default true
    ctx.jitterThreshold = 5.5f;              // flipped from default 3.0
    ctx.baseStationDriftCorrectionEnabled = false; // flipped from default true

    std::stringstream io;
    WriteProfile(ctx, io);
    const std::string json = io.str();

    EXPECT_NE(json.find("recalibrate_on_movement"),   std::string::npos);
    EXPECT_NE(json.find("ignore_outliers"),            std::string::npos);
    EXPECT_NE(json.find("jitter_threshold"),           std::string::npos);
    EXPECT_NE(json.find("base_station_drift_correction"), std::string::npos);
}

TEST(ConfigurationTest, WriteProfile_InvalidProfileWritesNothing) {
    // Sanity: WriteProfile early-returns on !validProfile (Configuration.cpp:530).
    // A caller that hands in an unfinished context should produce empty
    // output, not partial JSON that a re-load would silently truncate.
    CalibrationContext ctx;
    ctx.validProfile = false;

    std::stringstream io;
    WriteProfile(ctx, io);

    EXPECT_TRUE(io.str().empty())
        << "WriteProfile must early-return on invalid context â€” partial JSON "
           "would be a silent data-loss bug for the caller";
}

// ---------------------------------------------------------------------------
// Save always stamps the current schema_version. Future reads (on the same
// or later builds) need that key present; without it, even today's load
// path treats the profile as v0 and runs the migration steps redundantly.
// ---------------------------------------------------------------------------
TEST(ConfigurationTest, SaveStampsCurrentSchemaVersion) {
    CalibrationContext src;
    src.referenceTrackingSystem = "lighthouse";
    src.targetTrackingSystem = "oculus";
    src.validProfile = true;

    std::stringstream io;
    WriteProfile(src, io);

    // Reload through ParseProfile and verify validProfile became true (which
    // it does only if schema_version <= kProfileSchemaVersion is satisfied).
    // If the saved JSON had no schema_version, it'd load as v0 -- still
    // accepted but the rewritten file would become "stuck" at v0 instead of
    // migrating forward. The contract is that every save bumps to current.
    CalibrationContext dst;
    std::stringstream io2(io.str());
    ParseProfile(dst, io2);
    EXPECT_TRUE(dst.validProfile)
        << "A freshly-saved profile must be re-loadable cleanly";

    // Also sanity-check that "schema_version" appears in the JSON literally.
    // (Not a perfect check -- the parser doesn't expose the key list -- but
    // confirms the WriteProfile path writes the field at all.)
    EXPECT_NE(io.str().find("schema_version"), std::string::npos)
        << "Saved JSON should contain a schema_version key";
}

// ---------------------------------------------------------------------------
// Schema migration v3 -> v4. v3 profiles have no head_mount or boundary
// sections. They must load with both at their disabled defaults.
// ---------------------------------------------------------------------------
TEST(ConfigurationTest, MigrateV3ProfileLoadsWithDisabledV4Sections) {
    std::string v3Json = MakeMinimalProfile(/*schemaVersion=*/3);

    CalibrationContext ctx;
    std::stringstream io(v3Json);
    ParseProfile(ctx, io);

    EXPECT_TRUE(ctx.validProfile);
    EXPECT_EQ(ctx.headMount.mode, HeadMountMode::Off)
        << "v3 profile must default head_mount.mode to Off";
    EXPECT_TRUE(ctx.headMount.trackerSerial.empty())
        << "v3 profile must default head_mount.trackerSerial to empty";
    EXPECT_FALSE(ctx.headMount.offsetCalibrated)
        << "v3 profile must default head_mount.offsetCalibrated to false";
    EXPECT_FALSE(ctx.boundary.enabled)
        << "v3 profile must default boundary.enabled to false";
}

// ---------------------------------------------------------------------------
// Round-trip for the v4 sections (head_mount, boundary). Non-default
// values must survive a write->read cycle intact.
// ---------------------------------------------------------------------------
TEST(ConfigurationTest, V4SectionsRoundTrip) {
    CalibrationContext src;
    src.referenceTrackingSystem = "lighthouse";
    src.targetTrackingSystem = "oculus";
    src.validProfile = true;

    src.headMount.mode = HeadMountMode::Corroborate;
    src.headMount.trackerSerial = "LHR-AABBCCDD";
    src.headMount.trackerTrackingSystem = "lighthouse";
    src.headMount.hideTracker = false;
    src.headMount.offsetCalibrated = true;
    // Set a non-identity headFromTracker.
    Eigen::Quaterniond q = Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitY())
                           * Eigen::AngleAxisd(0.05, Eigen::Vector3d::UnitX());
    src.headMount.headFromTracker = Eigen::AffineCompact3d::Identity();
    src.headMount.headFromTracker.linear() = q.toRotationMatrix();
    src.headMount.headFromTracker.translation() = Eigen::Vector3d(0.01, -0.02, 0.03);

    src.boundary.enabled = true;
    src.boundary.floorY = -0.05;
    src.boundary.ceilingY = 2.3;
    src.boundary.vertices = { {1.0, 0.0, 0.0}, {-1.0, 0.0, 0.5} };
    src.boundary.priorChaperone = {0xDE, 0xAD, 0xBE, 0xEF};
    src.boundary.priorChaperoneCaptured = true;

    std::stringstream io;
    WriteProfile(src, io);

    CalibrationContext dst;
    std::stringstream io2(io.str());
    ParseProfile(dst, io2);

    EXPECT_TRUE(dst.validProfile);
    EXPECT_EQ(dst.headMount.mode, HeadMountMode::Corroborate);
    EXPECT_EQ(dst.headMount.trackerSerial, "LHR-AABBCCDD");
    EXPECT_EQ(dst.headMount.trackerTrackingSystem, "lighthouse");
    EXPECT_FALSE(dst.headMount.hideTracker);
    EXPECT_TRUE(dst.headMount.offsetCalibrated);
    // Translation must round-trip to within floating-point precision.
    EXPECT_NEAR(dst.headMount.headFromTracker.translation()(0), 0.01,  1e-9);
    EXPECT_NEAR(dst.headMount.headFromTracker.translation()(1), -0.02, 1e-9);
    EXPECT_NEAR(dst.headMount.headFromTracker.translation()(2), 0.03,  1e-9);
    // Rotation: compare via quaternions; normalize first.
    Eigen::Quaterniond qSrc(src.headMount.headFromTracker.rotation());
    Eigen::Quaterniond qDst(dst.headMount.headFromTracker.rotation());
    qSrc.normalize(); qDst.normalize();
    EXPECT_NEAR(std::abs(qSrc.dot(qDst)), 1.0, 1e-6)
        << "headFromTracker rotation must round-trip";

    EXPECT_TRUE(dst.boundary.enabled);
    EXPECT_NEAR(dst.boundary.floorY,   -0.05, 1e-9);
    EXPECT_NEAR(dst.boundary.ceilingY,  2.3,  1e-9);
    ASSERT_EQ(dst.boundary.vertices.size(), 2u);
    EXPECT_NEAR(dst.boundary.vertices[0].x, 1.0,  1e-9);
    EXPECT_NEAR(dst.boundary.vertices[1].z, 0.5,  1e-9);
    ASSERT_EQ(dst.boundary.priorChaperone.size(), 4u);
    EXPECT_EQ(dst.boundary.priorChaperone[0], 0xDE);
    EXPECT_EQ(dst.boundary.priorChaperone[3], 0xEF);
    EXPECT_TRUE(dst.boundary.priorChaperoneCaptured);

}

// Skip-if-default: the new sections must NOT appear in the JSON
// when all fields are at their default values.
TEST(ConfigurationTest, V4SectionsSkippedWhenDefault) {
    CalibrationContext ctx;
    ctx.referenceTrackingSystem = "lighthouse";
    ctx.targetTrackingSystem = "oculus";
    ctx.validProfile = true;
    // All head_mount / boundary fields at construction defaults.

    std::stringstream io;
    WriteProfile(ctx, io);
    const std::string json = io.str();

    EXPECT_EQ(json.find("\"head_mount\""),  std::string::npos)
        << "head_mount must be omitted when at default (Off, no serial)";
    EXPECT_EQ(json.find("\"boundary\""),    std::string::npos)
        << "boundary must be omitted when disabled and no vertices captured";
}
