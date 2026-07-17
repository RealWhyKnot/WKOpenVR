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
constexpr bool IsCurrentErrorSpike(double currentError, double rollingMedian)
{
	return rollingMedian > kMedianFloor && currentError > kSpikeRatio * rollingMedian;
}

// Sustain gate. Caller passes the running count of consecutive ticks where
// IsCurrentErrorSpike returned true; this returns whether the count has
// reached the trigger.
constexpr bool ShouldFireGeometryShiftRecovery(int sustainedSpikes)
{
	return sustainedSpikes >= kMinSustainedSpikes;
}

// Restart-settling grace window (seconds). Armed at every StartCalibration
// so the first ~10 samples of a fresh cal cycle (where the solver converges
// from an empty buffer and error naturally fluctuates) cannot trip a
// back-to-back fire. 3 s at the observed ~3.5 Hz sample cadence.
constexpr double kGraceSeconds = 3.0;

// Grace gate. True while `now` sits before the armed deadline; the detector
// is skipped entirely for the tick. A zero/unset deadline never gates.
//
// Epoch contract: the deadline is armed as glfwGetTime() + kGraceSeconds and
// compared against CalibrationTick's `time` parameter, which carries the
// same glfwGetTime() clock. Arming from any other clock breaks expiry -- a
// deadline stamped from a clock that runs ahead of `now`'s epoch gates the
// detector forever.
constexpr bool InGraceWindow(double now, double graceUntil)
{
	return graceUntil > 0.0 && now < graceUntil;
}

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

// Post-fire cooldown gate. Returns true when `now` sits inside the
// cooldown window (now < cooldownUntil); caller should skip the recovery
// action for this tick. A zero/unset cooldownUntil never suppresses.
//
// Wired at the fire site in CalibrationTick: when this returns true, the
// recovery (Clear + demote to Standby + restart) is skipped, but the
// per-tick sustain counters still advance so the next fire decision after
// cooldown expires reflects the current state of the world.
constexpr bool ShouldSuppressForCooldown(double now, double cooldownUntil)
{
	return now < cooldownUntil;
}

} // namespace spacecal::geometry_shift
