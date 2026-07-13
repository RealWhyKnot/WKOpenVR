#pragma once

// Pure helpers for head-mount tracker snap-suppression logic. Extracted from
// Calibration.cpp and CalibrationRecoveryTick.cpp so unit tests can exercise
// the corroboration decisions without a live OpenVR runtime.
//
// Three sites in the calibration loop consult this logic:
//   1. ComputeEffectiveSpeedMps -- AUTO Lock stationary gate takes max of HMD
//      and head-tracker speeds when Corroborate is active.
//   2. JumpDetectorClassification -- 30 cm jump detector classifies the event
//      as SLAM snap when the head-tracker reports < 2 cm displacement.
//   3. GeometryShiftCoherenceSource -- who_moved block uses tracker actual
//      displacement instead of velocity-integrated HMD estimate.
//
// All functions are pure: no CalCtx access, no vr::* calls. Callers read
// device-pose arrays and pass the extracted values here.

#include "Calibration.h" // HeadMountMode, HeadMountConfig

#include <cmath>

namespace spacecal::snap_suppression {

// Promotes the head-mount mode to at least Corroborate whenever a witness puck
// is present (bound to a resolved device).
//
// The user-selected tracking style sets configMode = Off for Continuous and
// Manual (ApplyTrackingStylePreset), which disabled the entire corroboration
// subsystem even when a Lighthouse witness puck was bound and feeding pose
// pairs. Corroborate is a passive role -- it only reads the witness to classify
// who-moved and to suppress destructive snaps; it never synthesizes output
// (that is DriverSynth, which stays the user's explicit choice). So whenever a
// witness is present, the effective mode is at least Corroborate regardless of
// style. A higher configMode (DriverSynth) is preserved. When no witness is
// present nothing changes, so non-witness setups behave exactly as before.
//
// Pure: callers pass witnessPresent (device resolved) and never mutate the
// persisted config mode.
inline HeadMountMode EffectiveHeadMountMode(HeadMountMode configMode, bool witnessPresent)
{
	if (witnessPresent && configMode < HeadMountMode::Corroborate) return HeadMountMode::Corroborate;
	return configMode;
}

// Returns the effective speed in m/s for the AUTO Lock stationary gate.
//
// When mode >= Corroborate and trackerSpeedMps is non-negative (meaning the
// caller confirmed the tracker is valid and provided its speed), returns
// max(hmdSpeedMps, trackerSpeedMps). Either device reporting motion blocks
// the lock flip.
//
// When trackerSpeedMps < 0 (caller signals tracker invalid/unavailable) or
// mode < Corroborate, returns hmdSpeedMps unchanged.
inline double EffectiveSpeedMps(HeadMountMode mode, double hmdSpeedMps, double trackerSpeedMps)
{
	if (mode < HeadMountMode::Corroborate) return hmdSpeedMps;
	if (trackerSpeedMps < 0.0) return hmdSpeedMps; // tracker invalid
	return std::max(hmdSpeedMps, trackerSpeedMps);
}

// Jump detection classification for the 30 cm auto-recovery site.
//
// Returns true when the event is classified as a Quest SLAM snap:
//   - HMD reported a large jump (hmdDeltaM >= kSnapHmdJumpM)
//   - Head-tracker displacement was tiny (trackerDeltaM < kSnapTrackerMaxDispM)
//   - Mode is Corroborate or higher
//   - trackerDeltaM >= 0 (caller confirmed the tracker produced a valid
//     displacement reading this tick; negative means unknown/invalid)
//
// When true, the caller should substitute fast re-anchor for full recovery.
// When false, treat the event as a genuine physical jump and proceed with
// the normal recovery path unchanged.
//
// The thresholds are pinned by static_asserts in test_snap_suppression.cpp;
// retune the constants and the pins together.
constexpr double kSnapHmdJumpM = 0.30;        // 30 cm
constexpr double kSnapTrackerMaxDispM = 0.02; // 2 cm

inline bool IsJumpClassifiedAsSnap(HeadMountMode mode, double hmdDeltaM, double trackerDeltaM)
{
	if (mode < HeadMountMode::Corroborate) return false;
	if (trackerDeltaM < 0.0) return false; // tracker invalid; no corroboration
	return hmdDeltaM >= kSnapHmdJumpM && trackerDeltaM < kSnapTrackerMaxDispM;
}

// Geometry-shift coherence source selection.
//
// When mode >= Corroborate and trackerDeltaM >= 0 (valid tracker reading),
// the who_moved block should use the head-tracker's actual pose-to-pose
// displacement rather than the velocity-integrated HMD estimate. Returns
// true when the tracker displacement should be preferred.
inline bool ShouldUseTrackerDisplacement(HeadMountMode mode, double trackerDeltaM)
{
	if (mode < HeadMountMode::Corroborate) return false;
	return trackerDeltaM >= 0.0;
}

} // namespace spacecal::snap_suppression
