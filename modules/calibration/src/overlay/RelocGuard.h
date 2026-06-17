#pragma once

// Pure helpers for the post-relocalization sample-quarantine toggle.
//
// The HMD relocalization detector (CalibrationRecoveryTick.cpp) fires a
// `hmd_relocalization_detected` event whenever the headset's tracked pose jumps
// more than ~5 cm between ticks while the base stations stay put -- the
// signature of a Quest inside-out SLAM relocalization. Field logs show these
// fire continuously (14x in one bad Continuous session) at 5-6 cm each, all
// below the 30 cm auto-recovery gate, so today the detector only logs and the
// continuous-cal solver keeps ingesting the post-jump pose pairs. Those samples
// are in the shifted Quest world frame and poison the ref<->target solve,
// driving the relative-pose MAD to hundreds of mm.
//
// When the quarantine toggle is on, the live sample-intake path consults this
// helper and drops samples for a short settle window after each detection so
// the shifted poses never enter the solver (or the AUTO-lock MAD window).
//
// All functions are pure: no CalCtx access, no vr::* calls. The caller passes
// the current time, the time of the last relocalization detection, and the
// window length.

namespace spacecal::reloc_guard {

// Default settle window after a detected relocalization, in seconds. One second
// at the ~3.5 Hz continuous-cal cadence drops the handful of ticks immediately
// following a SLAM snap while the tracking re-converges.
constexpr double kDefaultQuarantineSec = 1.0;

// A window of zero (or negative) disables quarantine, so a config that zeroes
// the window behaves as "toggle off" rather than quarantining every sample.
constexpr double kMinQuarantineSec = 0.0;

// Once the candidate relative-pose MAD is back near the settled floor, the
// quarantine can release before the full time window elapses.
constexpr double kDefaultClearMult = 1.5;

// True when `now` falls inside the quarantine window opened by the most recent
// relocalization at `lastRelocTime`.
//
// lastRelocTime < 0 is the sentinel "no relocalization seen yet" -> never
// quarantine. A non-positive window also disables quarantine. The window is
// half-open: an event at t arms [t, t + windowSec), so a sample exactly at the
// window edge is released (matches the "drop the next N ticks, then resume"
// intent and keeps the boundary deterministic).
inline bool ShouldQuarantineSample(double now, double lastRelocTime, double windowSec)
{
	if (lastRelocTime < 0.0) return false;
	if (windowSec <= kMinQuarantineSec) return false;
	const double age = now - lastRelocTime;
	return age >= 0.0 && age < windowSec;
}

// Time-window quarantine with an early-release escape hatch on restabilization.
//
// Behaves like ShouldQuarantineSample but additionally releases the quarantine
// early -- before the window elapses -- once the live relative-pose MAD has
// dropped back under settledFloorMm * clearMult, i.e. the solver has already
// reconverged and there is no reason to keep dropping good samples. When
// settledFloorMm or clearMult is non-positive the early-release check is
// skipped and this reduces to the plain time window.
inline bool QuarantineActive(double now, double lastRelocTime, double windowSec, double liveMadMm,
                             double settledFloorMm, double clearMult)
{
	if (!ShouldQuarantineSample(now, lastRelocTime, windowSec)) return false;
	if (settledFloorMm > 0.0 && clearMult > 0.0 && liveMadMm <= settledFloorMm * clearMult) {
		return false; // relative pose already restabilized -> stop dropping samples
	}
	return true;
}

} // namespace spacecal::reloc_guard
