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

#include "Calibration.h"  // HeadMountMode, HeadMountConfig

#include <cmath>

namespace spacecal::snap_suppression {

// Returns the effective speed in m/s for the AUTO Lock stationary gate.
//
// When mode >= Corroborate and trackerSpeedMps is non-negative (meaning the
// caller confirmed the tracker is valid and provided its speed), returns
// max(hmdSpeedMps, trackerSpeedMps). Either device reporting motion blocks
// the lock flip.
//
// When trackerSpeedMps < 0 (caller signals tracker invalid/unavailable) or
// mode < Corroborate, returns hmdSpeedMps unchanged.
inline double EffectiveSpeedMps(HeadMountMode mode,
                                double hmdSpeedMps,
                                double trackerSpeedMps)
{
    if (mode < HeadMountMode::Corroborate) return hmdSpeedMps;
    if (trackerSpeedMps < 0.0) return hmdSpeedMps;   // tracker invalid
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
// Thresholds are per the plan spec and must not be tuned here without
// updating the plan.
constexpr double kSnapHmdJumpM        = 0.30;  // 30 cm
constexpr double kSnapTrackerMaxDispM = 0.02;  // 2 cm

inline bool IsJumpClassifiedAsSnap(HeadMountMode mode,
                                   double hmdDeltaM,
                                   double trackerDeltaM)
{
    if (mode < HeadMountMode::Corroborate) return false;
    if (trackerDeltaM < 0.0) return false;  // tracker invalid; no corroboration
    return hmdDeltaM >= kSnapHmdJumpM && trackerDeltaM < kSnapTrackerMaxDispM;
}

// Geometry-shift coherence source selection.
//
// When mode >= Corroborate and trackerDeltaM >= 0 (valid tracker reading),
// the who_moved block should use the head-tracker's actual pose-to-pose
// displacement rather than the velocity-integrated HMD estimate. Returns
// true when the tracker displacement should be preferred.
inline bool ShouldUseTrackerDisplacement(HeadMountMode mode,
                                         double trackerDeltaM)
{
    if (mode < HeadMountMode::Corroborate) return false;
    return trackerDeltaM >= 0.0;
}

}  // namespace spacecal::snap_suppression
