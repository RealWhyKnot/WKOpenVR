#pragma once

// Geometry-shift detector — fork-only fast watchdog at Calibration.cpp:2098-2142.
// Independent of the slower 50-rejection watchdog inside CalibrationCalc:
// catches catastrophic geometry shifts (lighthouse bumped, tracker through a
// portal) within ~3 ticks instead of ~25 s.
//
// Decision logic split into two trivially-testable parts:
//   - IsCurrentErrorSpike: per-tick boolean from the error time series
//   - ShouldFireGeometryShiftRecovery: gate on sustained spikes
//
// Both functions are pure + constexpr-friendly. Caller (CalibrationTick) owns
// the counter, increments on IsCurrentErrorSpike==true, resets on false.

namespace spacecal::geometry_shift {

// Threshold above which `currentError` is treated as anomalous relative to
// the rolling median. Empirically 5× the recent median catches genuine
// geometry shifts without firing on normal solver noise (per the original
// 9d0ba0b implementation comment).
constexpr double kSpikeRatio = 5.0;

// Floor on the rolling median: when the median itself is sub-nanometer-scale,
// the ratio test is meaningless (numerator might be tiny even at 5×). Skip
// the spike check entirely — there's no real signal to compare against.
constexpr double kMedianFloor = 1e-9;

// Required sustain count: don't fire on transient single-tick spikes that
// could be noise. 3 consecutive accepted samples gives ~100-300 ms of
// sustained anomaly before action.
constexpr int kMinSustainedSpikes = 3;

// Per-tick spike check. True when `currentError` exceeds the rolling median
// by at least kSpikeRatio×, with a floor on the median to avoid spurious
// firings on near-zero noise.
constexpr bool IsCurrentErrorSpike(double currentError, double rollingMedian) {
    return rollingMedian > kMedianFloor
        && currentError > kSpikeRatio * rollingMedian;
}

// Sustain gate. Caller passes the running count of consecutive ticks where
// IsCurrentErrorSpike returned true; this returns whether the count has
// reached the trigger.
constexpr bool ShouldFireGeometryShiftRecovery(int sustainedSpikes) {
    return sustainedSpikes >= kMinSustainedSpikes;
}

// -------------------------------------------------------------------------
// CUSUM (Page 1954) alternative path. Replaces the heuristic 5x-rolling-
// median rule with a cumulative-sum statistical test:
//     S_t = max(0, S_{t-1} + (r_t - baseline - k))
// Trigger when S_t > h. The drift parameter k inflates the per-sample
// expectation under H_0 so unbiased noise doesn't accumulate; threshold h
// gives a tunable false-alarm rate via standard ARL tables (Granjon 2013).
//
// Defaults are conservative -- ARL_0 ~ 10^4 ticks (~3 minutes at 60 Hz at
// noise-floor convergence) means roughly one false alarm per session of
// continuous calibration. Real geometry shifts produce a fast S_t climb
// regardless of these tuning values, so the detector still fires in well
// under a second on a real event.
//
// Pure decision helpers; the caller owns state via CusumState.
// -------------------------------------------------------------------------

// Drift parameter (millimetres). Subtracted from each residual sample before
// accumulating into S. Set just above the typical noise level so noise-only
// streams produce S near zero indefinitely.
constexpr double kCusumDriftMm = 0.5;

// Threshold (in the same units as the accumulated S, which after the
// drift subtraction is roughly mm-of-sustained-spike). h = 5.0 paired with
// k = 0.5 gives ARL_0 ~ 10^4 from Page's tables; tighter than the
// rolling-median rule but still well-tunable if a user reports false fires.
constexpr double kCusumThreshold = 5.0;

// Post-fire cooldown (seconds). After a geometry-shift recovery fires,
// suppress further fires for this long. Real geometry shifts (lighthouse
// bumped, tracker remounted) happen at minute-or-hour cadence; a window of
// rapid fires (the 2026-05-21 session had 52 in 2.2 h, one every ~2.4 min)
// is noise, not signal. Cooldown drops the false-fire rate without
// affecting real-shift response time -- a genuine shift after the cooldown
// expires still fires on the same fast cycle.
//
// CalibrationContext owns the deadline timestamp; this constant is the
// value to add on fire.
constexpr double kPostFireCooldownSeconds = 30.0;

struct CusumState {
    // Cumulative-sum statistic. Per-tick increment is
    //   (currentError - baseline) - drift
    // clamped at zero from below. When this exceeds kCusumThreshold the
    // detector is a candidate for firing; the sustain gate below decides
    // whether it actually fires this tick.
    double S = 0.0;

    // Consecutive ticks the S statistic has been above kCusumThreshold.
    // The CUSUM math alone fires on the first threshold breach, which on
    // cross-tracking-system rigs (Quest+Lighthouse) trips on routine
    // transient excursions. The sustain gate requires kMinSustainedSpikes
    // consecutive above-threshold ticks before firing -- same semantics as
    // the legacy 5x-median rule, which has not produced the same
    // firestorm pattern in observed logs.
    //
    // Reset to zero whenever S drops back to or below threshold (a
    // transient spike doesn't latch the counter) or when a fire commits.
    int sustainedAboveThreshold = 0;

    // Clear both fields together. Used at every reset site (grace window,
    // toggle flip, reanchor suppression, cooldown suppression, post-fire,
    // not-enough-samples) so the two counters never drift apart.
    constexpr void Reset() {
        S = 0.0;
        sustainedAboveThreshold = 0;
    }
};

// Update the CUSUM statistic with a new residual reading and return whether
// the trigger fires this tick. On fire, S resets to 0 so subsequent ticks
// start fresh. The baseline argument is the running mean residual under the
// no-shift hypothesis -- caller can supply a rolling EMA, the rolling
// median already maintained by the legacy detector, or 0.0 for a raw test.
//
// `outValueAtFire` (optional, write-only): when this function returns true,
// the caller-supplied pointer is set to S's value at the moment the fire
// decision was made, BEFORE the internal reset to 0. Without this, the
// diagnostic log at the fire site can only ever read S=0 (post-reset) and
// has no way to confirm which decision arm actually triggered the fire or
// how far past threshold the accumulator climbed.
//
// Sustain gate: S crossing threshold on a single tick increments the
// state's sustain counter but does not fire. Fire requires
// kMinSustainedSpikes consecutive above-threshold ticks. A single tick at
// or below threshold resets the sustain counter without disturbing S
// (the accumulated evidence stays; only the "is this sustained?" flag
// drops). When the fire commits, both S and the sustain counter reset.
constexpr bool UpdateCusumGeometryShift(CusumState& state,
                                         double currentErrorMm,
                                         double baselineMm,
                                         double driftMm = kCusumDriftMm,
                                         double threshold = kCusumThreshold,
                                         double* outValueAtFire = nullptr) {
    const double increment = (currentErrorMm - baselineMm) - driftMm;
    state.S = state.S + increment;
    if (state.S < 0.0) state.S = 0.0;
    if (state.S > threshold) {
        state.sustainedAboveThreshold++;
        if (state.sustainedAboveThreshold >= kMinSustainedSpikes) {
            if (outValueAtFire) *outValueAtFire = state.S;
            state.S = 0.0;
            state.sustainedAboveThreshold = 0;
            return true;
        }
    } else {
        state.sustainedAboveThreshold = 0;
    }
    return false;
}

// Post-fire cooldown gate. Returns true when `now` sits inside the
// cooldown window (now < cooldownUntil); caller should skip the recovery
// action for this tick. A zero/unset cooldownUntil never suppresses.
//
// Wired at the fire site in CalibrationTick: when this returns true, the
// recovery (Clear + demote to Standby + restart) is skipped, but the
// per-tick sustain counters still advance so the next fire decision after
// cooldown expires reflects the current state of the world.
constexpr bool ShouldSuppressForCooldown(double now, double cooldownUntil) {
    return now < cooldownUntil;
}

} // namespace spacecal::geometry_shift
