// Pin tests for the cycle-level snap-vs-blend decision in
// spacecal::motiongate::ShouldBlendCycle. Used by the overlay's
// ScanAndApplyProfile to decide whether SetDeviceTransform.lerp ships true
// (smooth blend in the driver's BlendTransform) or false (driver assigns
// transform := target without blending).
//
// History: this file previously also pinned regime-based still-floor
// helpers (ClassifyCorrection / StillFloor) that gated the per-tick lerp
// progress while recalibrateOnMovement was on. Those were replaced on
// 2026-05-24 by an absolute mm/sec time-rate cap (SlewRateCap.h +
// test_slew_rate_cap.cpp); the cycle-level snap decision below is
// orthogonal and stayed.

#include <gtest/gtest.h>

#include "MotionGate.h"

using spacecal::motiongate::ShouldBlendCycle;
using spacecal::motiongate::ShouldSnapFirstContinuousCandidate;

// ---------------------------------------------------------------------------
// Auto-recovery snap (option-3 bundle, 2026-05-04). After
// RecoverFromWedgedCalibration fires, the next ScanAndApplyProfile cycle
// MUST snap so the recovery doesn't get smoothed through whatever stale
// steady-state the driver had cached.
// ---------------------------------------------------------------------------
TEST(AutoRecoverySnapTest, RecoveryCycleSnaps_OverridesEverything) {
    // snapThisCycle=true must override both the continuous-state and the
    // freshly-adopted gates. This is the post-RecoverFromWedgedCalibration
    // tick; we want a discontinuity, not a smooth ride.
    EXPECT_FALSE(ShouldBlendCycle(/*continuous=*/true,
                                  /*freshlyAdopted=*/false,
                                  /*snapThisCycle=*/true))
        << "Recovery snap must override the normal continuous-blend path "
           "-- otherwise the user's brand-new post-recovery cal gets slowly "
           "interpolated from the wedged steady-state, defeating the "
           "recovery.";
    EXPECT_FALSE(ShouldBlendCycle(true,  true,  true));
    EXPECT_FALSE(ShouldBlendCycle(false, false, true));
    EXPECT_FALSE(ShouldBlendCycle(false, true,  true));
}

TEST(AutoRecoverySnapTest, FreshAdoptionSnaps) {
    // Independent of recovery: a tracker that just connected has no
    // meaningful `transform` to blend FROM. Snap so it doesn't ramp in
    // from identity.
    EXPECT_FALSE(ShouldBlendCycle(/*continuous=*/true,
                                  /*freshlyAdopted=*/true,
                                  /*snapThisCycle=*/false));
}

TEST(AutoRecoverySnapTest, OneShotStateSnaps) {
    // Only continuous mode lerps; the one-shot finalisation should snap
    // because there's no continuous-update stream to smooth toward.
    EXPECT_FALSE(ShouldBlendCycle(/*continuous=*/false,
                                  /*freshlyAdopted=*/false,
                                  /*snapThisCycle=*/false));
}

TEST(AutoRecoverySnapTest, NormalCycleBlends) {
    // The healthy steady-state: continuous mode, established device, no
    // recovery in flight. This is the only combination that should blend.
    EXPECT_TRUE(ShouldBlendCycle(/*continuous=*/true,
                                 /*freshlyAdopted=*/false,
                                 /*snapThisCycle=*/false));
}

TEST(AutoRecoverySnapTest, FlagIsOneShot_NextCycleResumesBlend) {
    // Simulates the pattern in production: cycle N fires recovery and snaps
    // (snapThisCycle=true), cycle N+1 the flag is cleared and blending
    // resumes. The flag-management is in ScanAndApplyProfile (consume at
    // end); this test pins the per-cycle pure decision both halves use.
    bool snapThisCycle = true;  // set by RecoverFromWedgedCalibration
    EXPECT_FALSE(ShouldBlendCycle(true, false, snapThisCycle))
        << "Cycle N (recovery): must snap";
    snapThisCycle = false;  // cleared at the end of cycle N
    EXPECT_TRUE(ShouldBlendCycle(true, false, snapThisCycle))
        << "Cycle N+1: blending resumes";
}

TEST(AutoRecoverySnapTest, FirstMeaningfulContinuousCandidateSnaps) {
    EXPECT_TRUE(ShouldSnapFirstContinuousCandidate(
        /*continuous=*/true,
        /*hasAcceptedSnapshot=*/false,
        /*hasGuardBaseline=*/true,
        /*jumpCm=*/3.0));

    EXPECT_TRUE(ShouldSnapFirstContinuousCandidate(true, false, true, 12.5));
    EXPECT_TRUE(ShouldSnapFirstContinuousCandidate(true, false, true, 25.0));
}

TEST(AutoRecoverySnapTest, FirstContinuousCandidateDoesNotSnapForSmallOrUnsafeDeltas) {
    EXPECT_FALSE(ShouldSnapFirstContinuousCandidate(
        /*continuous=*/true,
        /*hasAcceptedSnapshot=*/false,
        /*hasGuardBaseline=*/true,
        /*jumpCm=*/2.99));
    EXPECT_FALSE(ShouldSnapFirstContinuousCandidate(
        /*continuous=*/false,
        /*hasAcceptedSnapshot=*/false,
        /*hasGuardBaseline=*/true,
        /*jumpCm=*/12.0));
    EXPECT_FALSE(ShouldSnapFirstContinuousCandidate(
        /*continuous=*/true,
        /*hasAcceptedSnapshot=*/true,
        /*hasGuardBaseline=*/true,
        /*jumpCm=*/12.0));
    EXPECT_FALSE(ShouldSnapFirstContinuousCandidate(
        /*continuous=*/true,
        /*hasAcceptedSnapshot=*/false,
        /*hasGuardBaseline=*/false,
        /*jumpCm=*/12.0));
    EXPECT_FALSE(ShouldSnapFirstContinuousCandidate(
        /*continuous=*/true,
        /*hasAcceptedSnapshot=*/false,
        /*hasGuardBaseline=*/true,
        /*jumpCm=*/25.01));
}

static_assert(!ShouldBlendCycle(true, false, true),
    "snap flag must override continuous-mode blend");
static_assert(ShouldBlendCycle(true, false, false),
    "the only blend-true case is continuous + established + no snap");
static_assert(!ShouldBlendCycle(false, false, false),
    "non-continuous state must snap");
static_assert(ShouldSnapFirstContinuousCandidate(true, false, true, 3.0),
    "first meaningful continuous correction must snap");
static_assert(!ShouldSnapFirstContinuousCandidate(true, false, true, 25.01),
    "meter-scale first continuous corrections must blend, not snap");
