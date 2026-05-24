// Pure-function pin tests for the geometry-shift fast watchdog. Covers the
// per-tick spike check + the sustained-firings gate. The decision is split so
// each half is unit-testable in isolation; the production caller in
// CalibrationTick owns the running counter.
//
// This is audit row #3 from project_upstream_regression_audit_2026-05-04.md
// — the fork-only geometry-shift detector at Calibration.cpp:2098-2142,
// added in commit 9d0ba0b. No active regression observed; tests exist to
// pin the contract so any future tuning surfaces deliberately.

#include <gtest/gtest.h>

#include "GeometryShiftDetector.h"

using spacecal::geometry_shift::IsCurrentErrorSpike;
using spacecal::geometry_shift::ShouldFireGeometryShiftRecovery;
using spacecal::geometry_shift::kSpikeRatio;
using spacecal::geometry_shift::kMedianFloor;
using spacecal::geometry_shift::kMinSustainedSpikes;

// ---------------------------------------------------------------------------
// IsCurrentErrorSpike: median floor. If the median is below kMedianFloor
// (essentially zero noise on the time series), the spike check is meaningless
// — return false. Without this floor, any noise spike against a near-zero
// median would trip 5× ratio trivially and the detector would fire on bootstrap.
// ---------------------------------------------------------------------------
TEST(GeometryShiftDetectorTest, IsCurrentErrorSpike_NearZeroMedianIsNotASpike) {
    EXPECT_FALSE(IsCurrentErrorSpike(/*current=*/100.0, /*median=*/0.0));
    EXPECT_FALSE(IsCurrentErrorSpike(100.0, 1e-10));  // below floor
    EXPECT_FALSE(IsCurrentErrorSpike(0.001, 1e-12));
}

// ---------------------------------------------------------------------------
// IsCurrentErrorSpike: 5× ratio. Anything > 5× the median fires; anything <=
// does not. Boundary at exactly 5× is *not* a spike (strict >).
// ---------------------------------------------------------------------------
TEST(GeometryShiftDetectorTest, IsCurrentErrorSpike_RatioBoundary) {
    EXPECT_FALSE(IsCurrentErrorSpike(/*current=*/4.99, /*median=*/1.0))
        << "Just under 5× should not fire";
    EXPECT_FALSE(IsCurrentErrorSpike(5.0, 1.0))
        << "Exactly 5× must NOT fire (strict-greater-than)";
    EXPECT_TRUE(IsCurrentErrorSpike(5.01, 1.0))
        << "Just over 5× must fire";
    EXPECT_TRUE(IsCurrentErrorSpike(50.0, 1.0));
    EXPECT_TRUE(IsCurrentErrorSpike(0.6, 0.1)) << "Scale-invariant";
}

// ---------------------------------------------------------------------------
// IsCurrentErrorSpike: realistic continuous-cal numbers. error_currentCal at
// 1-3 mm during healthy operation; a real geometry shift jumps to 30+ mm.
// Pin that the detector catches the genuine case and stays quiet on the
// normal noise band.
// ---------------------------------------------------------------------------
TEST(GeometryShiftDetectorTest, IsCurrentErrorSpike_HealthyHuntingNotFlagged) {
    // Healthy continuous-cal: error 1.5-3.5 mm, median 2.0 mm. Noise should
    // not flag.
    for (double current : {1.5, 1.8, 2.0, 2.4, 3.5}) {
        EXPECT_FALSE(IsCurrentErrorSpike(current, /*median=*/2.0))
            << "Healthy hunting at current=" << current << " mm flagged spuriously";
    }
}

TEST(GeometryShiftDetectorTest, IsCurrentErrorSpike_RealGeometryShiftFlagged) {
    // Lighthouse bumped: error jumps from 2 mm to 30+ mm. Must flag.
    EXPECT_TRUE(IsCurrentErrorSpike(/*current=*/30.0, /*median=*/2.0));
    EXPECT_TRUE(IsCurrentErrorSpike(50.0, 5.0));
}

// ---------------------------------------------------------------------------
// ShouldFireGeometryShiftRecovery: sustain count. Fires at exactly
// kMinSustainedSpikes, NOT before. Pin the boundary so the 3-tick delay
// (~100-300 ms) is never accidentally tightened or loosened in code review.
// ---------------------------------------------------------------------------
TEST(GeometryShiftDetectorTest, ShouldFireGeometryShiftRecovery_AtBoundary) {
    EXPECT_FALSE(ShouldFireGeometryShiftRecovery(0));
    EXPECT_FALSE(ShouldFireGeometryShiftRecovery(1));
    EXPECT_FALSE(ShouldFireGeometryShiftRecovery(kMinSustainedSpikes - 1))
        << "Just under threshold (" << (kMinSustainedSpikes - 1)
        << ") must not fire";
    EXPECT_TRUE(ShouldFireGeometryShiftRecovery(kMinSustainedSpikes))
        << "At threshold (" << kMinSustainedSpikes << ") must fire";
    EXPECT_TRUE(ShouldFireGeometryShiftRecovery(kMinSustainedSpikes + 1));
    EXPECT_TRUE(ShouldFireGeometryShiftRecovery(100));
}

// ---------------------------------------------------------------------------
// constexpr pins. Both functions evaluate at compile time; static_assert
// fails the build (not just the test) if the contract is broken.
// ---------------------------------------------------------------------------
static_assert(!IsCurrentErrorSpike(100.0, 0.0),
    "near-zero median must short-circuit the spike check");
static_assert(IsCurrentErrorSpike(5.01, 1.0),
    "5.01× the median must register as a spike");
static_assert(!IsCurrentErrorSpike(5.0, 1.0),
    "exactly 5× must NOT fire — strict greater-than");
static_assert(!ShouldFireGeometryShiftRecovery(2),
    "2 sustained spikes must not yet fire");
static_assert(ShouldFireGeometryShiftRecovery(3),
    "3 sustained spikes is the documented trigger");

// ---------------------------------------------------------------------------
// CUSUM (Page 1954) opt-in path. Same recovery action as the legacy detector;
// different per-tick decision. The pure helper UpdateCusumGeometryShift
// owns the increment + threshold logic so we can pin it without spinning up
// the calibration tick.
// ---------------------------------------------------------------------------
TEST(CusumGeometryShiftTest, NoiseAtBaselineDoesNotAccumulate) {
    using spacecal::geometry_shift::CusumState;
    using spacecal::geometry_shift::UpdateCusumGeometryShift;
    using spacecal::geometry_shift::kCusumDriftMm;
    using spacecal::geometry_shift::kCusumThreshold;
    CusumState s{};
    // Feed 100 ticks of "current = baseline" -- per-sample increment is
    // (0 - drift) = -kCusumDriftMm < 0, so S stays clamped at 0.
    for (int i = 0; i < 100; i++) {
        const bool fire = UpdateCusumGeometryShift(s, /*current=*/2.0, /*baseline=*/2.0);
        EXPECT_FALSE(fire);
        EXPECT_DOUBLE_EQ(s.S, 0.0);
    }
}

TEST(CusumGeometryShiftTest, BelowDriftDoesNotFire) {
    using spacecal::geometry_shift::CusumState;
    using spacecal::geometry_shift::UpdateCusumGeometryShift;
    using spacecal::geometry_shift::kCusumDriftMm;
    using spacecal::geometry_shift::kCusumThreshold;
    CusumState s{};
    // Per-sample excursion at exactly +0.4 mm (below the 0.5 mm drift). Each
    // tick contributes (0.4 - 0.5) = -0.1 to S, clamped at 0. No fire ever.
    for (int i = 0; i < 1000; i++) {
        const bool fire = UpdateCusumGeometryShift(s, /*current=*/2.4, /*baseline=*/2.0);
        EXPECT_FALSE(fire);
    }
    EXPECT_DOUBLE_EQ(s.S, 0.0);
}

TEST(CusumGeometryShiftTest, SustainedShiftFiresAfterSustainGate) {
    using spacecal::geometry_shift::CusumState;
    using spacecal::geometry_shift::UpdateCusumGeometryShift;
    using spacecal::geometry_shift::kCusumDriftMm;
    using spacecal::geometry_shift::kCusumThreshold;
    using spacecal::geometry_shift::kMinSustainedSpikes;
    CusumState s{};
    // 5 mm sustained shift: per-tick increment = (5 - 0) - 0.5 = 4.5 mm.
    // tick 1: S = 4.5, still below threshold (5.0), sustain counter stays 0.
    // tick 2: S = 9.0, above threshold, sustain = 1, no fire (need 3).
    // tick 3: S = 13.5, above threshold, sustain = 2, no fire.
    // tick 4: S = 18.0, above threshold, sustain = 3 -> FIRE; reset.
    EXPECT_FALSE(UpdateCusumGeometryShift(s, /*current=*/5.0, /*baseline=*/0.0));
    EXPECT_NEAR(s.S, 4.5, 1e-9);
    EXPECT_EQ(s.sustainedAboveThreshold, 0);

    EXPECT_FALSE(UpdateCusumGeometryShift(s, 5.0, 0.0));
    EXPECT_EQ(s.sustainedAboveThreshold, 1);

    EXPECT_FALSE(UpdateCusumGeometryShift(s, 5.0, 0.0));
    EXPECT_EQ(s.sustainedAboveThreshold, 2);

    EXPECT_TRUE(UpdateCusumGeometryShift(s, 5.0, 0.0));
    EXPECT_DOUBLE_EQ(s.S, 0.0) << "CUSUM must reset S to 0 after firing";
    EXPECT_EQ(s.sustainedAboveThreshold, 0)
        << "Sustain counter must reset along with S";
}

TEST(CusumGeometryShiftTest, SingleSpikeAboveThresholdDoesNotFire) {
    using spacecal::geometry_shift::CusumState;
    using spacecal::geometry_shift::UpdateCusumGeometryShift;
    CusumState s{};
    // One huge tick well above threshold. Old behaviour: fire immediately.
    // New behaviour: sustain counter = 1, no fire. This is the failure mode
    // the 2026-05-21 log surfaced -- the CUSUM detector fired on routine
    // transient excursions that legacy 5x-median would have ignored.
    EXPECT_FALSE(UpdateCusumGeometryShift(s, /*current=*/100.0, /*baseline=*/0.0));
    EXPECT_EQ(s.sustainedAboveThreshold, 1);
    EXPECT_GT(s.S, 5.0) << "S did cross threshold; sustain gate is what holds the fire";
}

TEST(CusumGeometryShiftTest, SpikeThenNormalResetsSustainCounter) {
    using spacecal::geometry_shift::CusumState;
    using spacecal::geometry_shift::UpdateCusumGeometryShift;
    CusumState s{};
    // Spike, spike, normal, spike, spike, spike (3 in a row at the end) -> fire.
    UpdateCusumGeometryShift(s, /*current=*/100.0, /*baseline=*/0.0);
    EXPECT_EQ(s.sustainedAboveThreshold, 1);
    UpdateCusumGeometryShift(s, 100.0, 0.0);
    EXPECT_EQ(s.sustainedAboveThreshold, 2);
    // Drop below baseline so S clamps back to 0 -- sustain counter must reset.
    UpdateCusumGeometryShift(s, /*current=*/-1000.0, /*baseline=*/0.0);
    EXPECT_DOUBLE_EQ(s.S, 0.0);
    EXPECT_EQ(s.sustainedAboveThreshold, 0)
        << "Single tick at or below threshold must reset the sustain counter";
}

TEST(CusumGeometryShiftTest, RecoversFromSpike_ResumesQuiet) {
    using spacecal::geometry_shift::CusumState;
    using spacecal::geometry_shift::UpdateCusumGeometryShift;
    CusumState s{};
    // Sustained shift fires after 4 ticks (1 to reach threshold + 3 sustain).
    UpdateCusumGeometryShift(s, 5.0, 0.0);
    UpdateCusumGeometryShift(s, 5.0, 0.0);
    UpdateCusumGeometryShift(s, 5.0, 0.0);
    UpdateCusumGeometryShift(s, 5.0, 0.0);
    EXPECT_DOUBLE_EQ(s.S, 0.0);
    EXPECT_EQ(s.sustainedAboveThreshold, 0);
    // Now post-fire: error returns to baseline. State is at 0; no further fires.
    for (int i = 0; i < 100; i++) {
        EXPECT_FALSE(UpdateCusumGeometryShift(s, 0.0, 0.0));
    }
    EXPECT_DOUBLE_EQ(s.S, 0.0);
}

TEST(CusumGeometryShiftTest, ManualThresholdTuningWorks) {
    using spacecal::geometry_shift::CusumState;
    using spacecal::geometry_shift::UpdateCusumGeometryShift;
    CusumState s{};
    // Tighter threshold (1.0 mm) crosses sooner on the same shift; still
    // requires the 3-tick sustain gate to fire.
    // Increment per tick = (3.0 - 0) - 0.5 = 2.5 mm.
    //   tick 1: S=2.5, > 1.0, sustain=1
    //   tick 2: S=5.0, > 1.0, sustain=2
    //   tick 3: S=7.5, > 1.0, sustain=3 -> FIRE
    EXPECT_FALSE(UpdateCusumGeometryShift(s, /*current=*/3.0, /*baseline=*/0.0,
                                          /*driftMm=*/0.5, /*threshold=*/1.0));
    EXPECT_FALSE(UpdateCusumGeometryShift(s, 3.0, 0.0, 0.5, 1.0));
    EXPECT_TRUE(UpdateCusumGeometryShift(s, 3.0, 0.0, 0.5, 1.0));
    EXPECT_DOUBLE_EQ(s.S, 0.0);
}

TEST(CusumGeometryShiftTest, OutSustainAtFireCapturesPreResetValue) {
    // Regression for the 2026-05-24 log audit: every CUSUM fire line printed
    // `sustained=0` because the caller read the state field AFTER the
    // in-function reset. The out-param exposes the pre-reset value so the
    // log can prove the sustain gate was satisfied (>= kMinSustainedSpikes).
    using spacecal::geometry_shift::CusumState;
    using spacecal::geometry_shift::UpdateCusumGeometryShift;
    using spacecal::geometry_shift::kMinSustainedSpikes;
    CusumState s{};

    int sustainAtFire = -1;
    double valueAtFire = -1.0;

    // Per SustainedShiftFiresAfterSustainGate above: with current=5/baseline=0
    // the increment is 4.5/tick, so tick 1 lands S=4.5 (below threshold; no
    // sustain bump), ticks 2-4 cross the threshold and tick 4 hits sustain=3.
    for (int i = 0; i < 3; ++i) {
        EXPECT_FALSE(UpdateCusumGeometryShift(s, 5.0, 0.0,
            spacecal::geometry_shift::kCusumDriftMm,
            spacecal::geometry_shift::kCusumThreshold,
            &valueAtFire, &sustainAtFire));
        EXPECT_EQ(sustainAtFire, -1) << "no fire => out-param untouched (tick " << i << ")";
        EXPECT_DOUBLE_EQ(valueAtFire, -1.0);
    }
    EXPECT_TRUE(UpdateCusumGeometryShift(s, 5.0, 0.0,
        spacecal::geometry_shift::kCusumDriftMm,
        spacecal::geometry_shift::kCusumThreshold,
        &valueAtFire, &sustainAtFire));
    EXPECT_GE(sustainAtFire, kMinSustainedSpikes)
        << "fire requires sustain >= kMinSustainedSpikes; out-param must "
           "expose that pre-reset value so the diagnostic log can confirm it";
    EXPECT_GT(valueAtFire, 0.0)
        << "value-at-fire out-param should mirror the pre-reset S";
    EXPECT_EQ(s.sustainedAboveThreshold, 0)
        << "post-fire, the state itself resets -- the out-param is the only "
           "way to recover the pre-reset count";
}

TEST(CusumGeometryShiftTest, NullOutParamsAreSafe) {
    // Defensive: passing nullptr for either out-param must not crash the
    // hot path. The function takes both as optional pointers.
    using spacecal::geometry_shift::CusumState;
    using spacecal::geometry_shift::UpdateCusumGeometryShift;
    CusumState s{};
    // Same 4-tick fire pattern as above.
    EXPECT_FALSE(UpdateCusumGeometryShift(s, 5.0, 0.0));
    EXPECT_FALSE(UpdateCusumGeometryShift(s, 5.0, 0.0));
    EXPECT_FALSE(UpdateCusumGeometryShift(s, 5.0, 0.0));
    EXPECT_TRUE(UpdateCusumGeometryShift(s, 5.0, 0.0));
}

TEST(CusumGeometryShiftTest, ResetClampPreventsNegativeAccumulation) {
    using spacecal::geometry_shift::CusumState;
    using spacecal::geometry_shift::UpdateCusumGeometryShift;
    CusumState s{};
    // Long quiet period must NOT accumulate negative S that would delay a
    // later real shift's fire. The clamp at S = max(0, ...) is what prevents
    // this; without it, 1000 quiet ticks would push S to -500 mm and a real
    // shift would need to overcome that before firing.
    for (int i = 0; i < 1000; i++) UpdateCusumGeometryShift(s, 0.0, 0.0);
    EXPECT_DOUBLE_EQ(s.S, 0.0);
    EXPECT_EQ(s.sustainedAboveThreshold, 0);
    // Now a real shift fires after 4 ticks (1 to reach threshold + 3 sustain).
    EXPECT_FALSE(UpdateCusumGeometryShift(s, 5.0, 0.0));
    EXPECT_FALSE(UpdateCusumGeometryShift(s, 5.0, 0.0));
    EXPECT_FALSE(UpdateCusumGeometryShift(s, 5.0, 0.0));
    EXPECT_TRUE(UpdateCusumGeometryShift(s, 5.0, 0.0));
}

// ---------------------------------------------------------------------------
// Post-fire cooldown gate. After a recovery fires, suppress further fires
// for kPostFireCooldownSeconds. The fire site at CalibrationTick checks
// ShouldSuppressForCooldown(now, cooldownUntil) before running the recovery
// path; the helper itself is pure.
// ---------------------------------------------------------------------------
TEST(GeometryShiftCooldownTest, UnsetDeadlineNeverSuppresses) {
    using spacecal::geometry_shift::ShouldSuppressForCooldown;
    EXPECT_FALSE(ShouldSuppressForCooldown(/*now=*/0.0, /*cooldownUntil=*/0.0));
    EXPECT_FALSE(ShouldSuppressForCooldown(/*now=*/1e9, 0.0));
}

TEST(GeometryShiftCooldownTest, HoldsInsideWindow) {
    using spacecal::geometry_shift::ShouldSuppressForCooldown;
    using spacecal::geometry_shift::kPostFireCooldownSeconds;
    const double firedAt = 1000.0;
    const double until = firedAt + kPostFireCooldownSeconds;
    EXPECT_TRUE(ShouldSuppressForCooldown(firedAt, until));
    EXPECT_TRUE(ShouldSuppressForCooldown(firedAt + 1.0, until));
    EXPECT_TRUE(ShouldSuppressForCooldown(until - 1e-6, until));
}

TEST(GeometryShiftCooldownTest, ReleasesAtAndAfterDeadline) {
    using spacecal::geometry_shift::ShouldSuppressForCooldown;
    const double until = 1000.0;
    EXPECT_FALSE(ShouldSuppressForCooldown(until, until));
    EXPECT_FALSE(ShouldSuppressForCooldown(until + 1e-6, until));
    EXPECT_FALSE(ShouldSuppressForCooldown(until + 30.0, until));
}

// constexpr pins for the cooldown contract.
static_assert(!spacecal::geometry_shift::ShouldSuppressForCooldown(0.0, 0.0),
    "unset cooldown deadline must never suppress");
static_assert(spacecal::geometry_shift::ShouldSuppressForCooldown(5.0, 10.0),
    "now < cooldownUntil must suppress");
static_assert(!spacecal::geometry_shift::ShouldSuppressForCooldown(10.0, 10.0),
    "now == cooldownUntil releases (strict less-than)");
