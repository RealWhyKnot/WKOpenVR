// Pure-function pins for the slew-rate cap that replaces the regime-based
// still-floor in MotionGate.h. Contract: corrections converge at a bounded
// physical rate (mm/sec, deg/sec) when the user is stationary, and at a
// higher bounded rate when moving. No regime, no snap.
//
// User-feel intent (paraphrased 2026-05-24): the previous still-floor was
// "still very snappy" -- accumulated corrections after a long stationary
// stretch jumped on the first motion. The cap needs to keep that
// imperceptible regardless of how much pending correction has built up.

#include <gtest/gtest.h>

#include <cmath>

#include "SlewRateCap.h"

using spacecal::slew::ApplyCap;
using spacecal::slew::BlendRate;
using spacecal::slew::kConvergedPosM;
using spacecal::slew::kConvergedRotRad;

namespace {

// ---------------------------------------------------------------------------
// Stationary convergence. 30 mm pending correction at 0.0005 m/sec
// (0.5 mm/sec) should take ~60 seconds to converge when the user is fully
// still. The proposedLerp here is high (would normally close the gap fast)
// so the cap is the binding constraint, not the time-based rate.
// ---------------------------------------------------------------------------
TEST(SlewRateCapTest, StationaryConvergesAtCappedRate) {
    double pendingPosM = 0.030;            // 30 mm
    const double pendingRotRad = 0.0;
    const double dt = 1.0 / 90.0;          // typical SteamVR pose-update tick
    const double capPos = 0.0005;          // 0.5 mm/sec
    const double capRot = 0.000873;        // ~0.05 deg/sec
    const double rawLerp = 2.0 * dt;       // align_speed_large = 2/sec

    int frames = 0;
    while (pendingPosM > 0.0005 && frames < 90 * 120) {  // safety stop at 120 s
        const double effLerp = ApplyCap(rawLerp, pendingPosM, pendingRotRad,
                                        capPos, capRot, dt);
        pendingPosM *= (1.0 - effLerp);
        ++frames;
    }
    const double elapsedSec = frames * dt;
    EXPECT_GT(elapsedSec, 55.0)
        << "30 mm at 0.5 mm/sec should not converge in less than ~55 s when still";
    EXPECT_LT(elapsedSec, 70.0)
        << "30 mm at 0.5 mm/sec should converge in ~60 s, with some slack for"
           " the asymptote near zero";
}

// ---------------------------------------------------------------------------
// Moving convergence. The same 30 mm pending correction at 0.010 m/sec
// (10 mm/sec, the default moving cap) should converge in ~3 seconds. This is
// what the user sees when motion is hiding the catch-up.
// ---------------------------------------------------------------------------
TEST(SlewRateCapTest, MovingConvergesQuickly) {
    double pendingPosM = 0.030;
    const double pendingRotRad = 0.0;
    const double dt = 1.0 / 90.0;
    const double capPos = 0.010;           // 10 mm/sec
    const double capRot = 0.01745;         // ~1 deg/sec
    const double rawLerp = 2.0 * dt;

    int frames = 0;
    while (pendingPosM > 0.0005 && frames < 90 * 10) {
        const double effLerp = ApplyCap(rawLerp, pendingPosM, pendingRotRad,
                                        capPos, capRot, dt);
        pendingPosM *= (1.0 - effLerp);
        ++frames;
    }
    const double elapsedSec = frames * dt;
    EXPECT_GT(elapsedSec, 2.5)
        << "30 mm at 10 mm/sec should not converge in less than ~2.5 s";
    EXPECT_LT(elapsedSec, 4.0)
        << "30 mm at 10 mm/sec should converge in ~3 s, with slack for"
           " the asymptote";
}

// ---------------------------------------------------------------------------
// Near-convergence guard. When pending is below the convergence threshold,
// the cap can't be exceeded regardless of lerp; the function must return
// the input lerp unchanged with no divide blow-up. Without this guard, a
// 1e-12 pending value would produce a near-infinite scale factor.
// ---------------------------------------------------------------------------
TEST(SlewRateCapTest, NearConvergenceGuardPosition) {
    const double tinyPending = kConvergedPosM * 0.5;   // below the floor
    const double lerp = 0.99;
    const double out = ApplyCap(lerp,
                                tinyPending, 0.0,
                                /*capPos=*/0.0005, /*capRot=*/0.001,
                                /*dt=*/1.0 / 90.0);
    EXPECT_DOUBLE_EQ(out, lerp)
        << "Near-zero pending position must not divide-explode; returns"
           " proposed lerp unchanged";
}

TEST(SlewRateCapTest, NearConvergenceGuardRotation) {
    const double tinyPending = kConvergedRotRad * 0.5;
    const double lerp = 0.99;
    const double out = ApplyCap(lerp,
                                0.0, tinyPending,
                                /*capPos=*/0.0005, /*capRot=*/0.001,
                                /*dt=*/1.0 / 90.0);
    EXPECT_DOUBLE_EQ(out, lerp);
}

// ---------------------------------------------------------------------------
// Mixed pos+rot: the slowest axis dominates the scale. If rotation would
// overshoot but position would not, the lerp factor must be throttled by the
// rotation-dictated factor so the rotation step is exactly at the cap; the
// position step rides along at the same scale (it would otherwise be below
// its own cap, which is fine -- the slower axis is the binding constraint).
// ---------------------------------------------------------------------------
TEST(SlewRateCapTest, MixedAxesSlowestDominates) {
    const double dt = 1.0 / 90.0;
    const double pendingPosM   = 0.001;    // 1 mm
    const double pendingRotRad = 0.1745;   // 10 deg
    const double capPos = 1.0;             // very high pos cap -- not binding
    const double capRot = 0.01745;         // 1 deg/sec
    const double rawLerp = 0.5;            // would close half the gap

    const double effLerp = ApplyCap(rawLerp, pendingPosM, pendingRotRad,
                                    capPos, capRot, dt);

    // Rotation step must be at the cap.
    const double rotStep = pendingRotRad * effLerp;
    EXPECT_NEAR(rotStep, capRot * dt, 1e-12)
        << "Rotation step must hit the cap exactly when rotation is binding";

    // Position step rides the same scale -- below its own cap, as expected.
    const double posStep = pendingPosM * effLerp;
    EXPECT_LT(posStep, capPos * dt)
        << "Position step is below its own (loose) cap";
}

// ---------------------------------------------------------------------------
// Cap not binding: when both axes' would-steps are inside the caps, the
// function returns the input lerp unchanged.
// ---------------------------------------------------------------------------
TEST(SlewRateCapTest, CapNotBindingReturnsInputLerp) {
    const double dt = 1.0 / 90.0;
    // 1 mm pending, 0.5 mm/sec cap: 1 mm * lerp must be < 0.5 mm * dt.
    // 0.5 mm * (1/90) = ~0.0056 mm = 5.6 um. So lerp must be < 0.0056.
    // Pick a deliberately-small lerp.
    const double lerp = 0.001;
    const double out = ApplyCap(lerp,
                                0.001, 0.001,
                                /*capPos=*/0.0005, /*capRot=*/0.01745,
                                dt);
    EXPECT_DOUBLE_EQ(out, lerp)
        << "Proposed lerp inside both caps: return unchanged";
}

// ---------------------------------------------------------------------------
// dt = 0: must return 0 cleanly (no division by zero or NaN downstream).
// Same for non-positive proposedLerp.
// ---------------------------------------------------------------------------
TEST(SlewRateCapTest, ZeroDtReturnsZero) {
    EXPECT_DOUBLE_EQ(
        ApplyCap(0.5, 0.030, 0.1, 0.0005, 0.01745, 0.0),
        0.0);
}

TEST(SlewRateCapTest, NonPositiveLerpReturnsZero) {
    EXPECT_DOUBLE_EQ(
        ApplyCap(0.0, 0.030, 0.1, 0.0005, 0.01745, 1.0 / 90.0),
        0.0);
    EXPECT_DOUBLE_EQ(
        ApplyCap(-0.1, 0.030, 0.1, 0.0005, 0.01745, 1.0 / 90.0),
        0.0);
}

// ---------------------------------------------------------------------------
// BlendRate monotonicity in motionGate. Effective rate must increase
// monotonically as the user starts moving -- a faster cap should never
// engage at a lower gate.
// ---------------------------------------------------------------------------
TEST(SlewRateCapTest, BlendRateMonotonic) {
    const double stationary = 0.0005;
    const double moving = 0.010;

    const double r0   = BlendRate(stationary, moving, 0.0);
    const double r25  = BlendRate(stationary, moving, 0.25);
    const double r50  = BlendRate(stationary, moving, 0.5);
    const double r75  = BlendRate(stationary, moving, 0.75);
    const double r100 = BlendRate(stationary, moving, 1.0);

    EXPECT_DOUBLE_EQ(r0,   stationary);
    EXPECT_DOUBLE_EQ(r100, moving);
    EXPECT_LT(r0,   r25);
    EXPECT_LT(r25,  r50);
    EXPECT_LT(r50,  r75);
    EXPECT_LT(r75,  r100);
    // Linearity check at the midpoint.
    EXPECT_NEAR(r50, 0.5 * (stationary + moving), 1e-12);
}

// ---------------------------------------------------------------------------
// BlendRate clamps an out-of-range gate. A buggy caller passing > 1 should
// not be able to widen the effective rate beyond moving.
// ---------------------------------------------------------------------------
TEST(SlewRateCapTest, BlendRateClampsGate) {
    const double stationary = 0.0005;
    const double moving = 0.010;

    EXPECT_DOUBLE_EQ(BlendRate(stationary, moving,  2.0), moving);
    EXPECT_DOUBLE_EQ(BlendRate(stationary, moving, -1.0), stationary);
}

// ---------------------------------------------------------------------------
// Catastrophic-correction-when-still scenario the redesign exists to fix:
// 5 cm of accumulated correction, user fully still, with the default
// stationary rate. After 5 seconds the user-visible step is bounded -- no
// snap. Contrast: at the prior 90% Large still-floor, the same correction
// would have closed ~85% of the gap in 1 second.
// ---------------------------------------------------------------------------
TEST(SlewRateCapTest, AccumulatedCorrectionWhileStillStaysGentle) {
    double pendingPosM = 0.050;            // 50 mm accumulated
    const double pendingRotRad = 0.0;
    const double dt = 1.0 / 90.0;
    const double capPos = 0.0005;          // stationary default
    const double capRot = 0.000873;
    const double rawLerp = 2.0 * dt;

    // Simulate 5 seconds.
    const int frames = static_cast<int>(5.0 / dt);
    for (int i = 0; i < frames; ++i) {
        const double effLerp = ApplyCap(rawLerp, pendingPosM, pendingRotRad,
                                        capPos, capRot, dt);
        pendingPosM *= (1.0 - effLerp);
    }
    // At 0.5 mm/sec for 5 s, we expect ~2.5 mm closed out of 50 mm -- the
    // remaining pending is ~47.5 mm. (The cap is on PER-FRAME STEP, so the
    // total progress over 5 s is exactly cap*time when the cap is binding
    // throughout.)
    EXPECT_GT(pendingPosM, 0.045)
        << "Stationary catch-up of a large correction must remain bounded:"
           " no more than 5 mm closed in 5 seconds at 0.5 mm/sec";
    EXPECT_LT(pendingPosM, 0.0485)
        << "But progress IS made -- ~2.5 mm closed";
}

// ---------------------------------------------------------------------------
// constexpr pins. Catch contract changes at compile time too.
// ---------------------------------------------------------------------------
static_assert(ApplyCap(0.0, 0.03, 0.0, 0.0005, 0.001, 1.0/90.0) == 0.0,
    "non-positive lerp returns 0");
static_assert(ApplyCap(0.5, 0.03, 0.0, 0.0005, 0.001, 0.0) == 0.0,
    "zero dt returns 0");
static_assert(BlendRate(0.0005, 0.010, 0.0) == 0.0005,
    "gate 0 returns stationary");
static_assert(BlendRate(0.0005, 0.010, 1.0) == 0.010,
    "gate 1 returns moving");

}  // namespace
