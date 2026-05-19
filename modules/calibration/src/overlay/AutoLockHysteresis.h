#pragma once

// AUTO Lock-relative-position hysteresis + stationary-gate decision logic.
//
// The relative-pose-stability detector compares a sliding window of
// (ref^-1 * target) samples. When the variance stays low enough, AUTO mode
// flips lockRelativePosition to true (rigid attachment); when it climbs,
// AUTO flips back to false.
//
// A single threshold on stddev / max-rotation flaps when a head-mounted
// tracker's actual stddev hovers near the boundary -- each flip changes the
// math regime inside CalibrationCalc::ComputeIncremental and produces a
// visible calibration jump. Two fixes compose:
//
// 1. Hysteresis -- separate thresholds for enter (tighter) vs leave
//    (looser). The deadband requires the user to do something physical to
//    re-resolve, so jitter in either direction is ignored.
//
// 2. Stationary gate -- pending flips are held until the next HMD-still
//    window before committing. Any residual visible jump lands while the
//    user is paused, hidden by the natural stillness rather than mid-motion.
//
// Pure helpers, header-only, testable in isolation. Same pattern as
// MotionGate.h, GeometryShiftDetector.h, WatchdogDecisions.h, TiltDiagnostic.h.

#include <Eigen/Geometry>

namespace spacecal::autolock {

// Window size of the relative-pose history. With samples arriving at the
// continuous-cal cadence (~2 Hz post-buffer-fill), 30 samples ~ 15 s.
constexpr size_t kHistoryMax = 30;
constexpr size_t kSamplesNeeded = 30;

// Hysteresis thresholds. Tighter on enter (need genuine rigid attachment)
// than on leave (don't release a locked relationship over a few mm of noise).
constexpr double kEnterTranslM = 0.003;                  // 3 mm
constexpr double kLeaveTranslM = 0.008;                  // 8 mm
constexpr double kEnterRotRad  = 0.7 * EIGEN_PI / 180.0; // 0.7 deg
constexpr double kLeaveRotRad  = 1.5 * EIGEN_PI / 180.0; // 1.5 deg

// HMD linear-speed threshold (m/s) below which a queued AUTO-lock flip is
// allowed to commit. Same order of magnitude as the existing TrackerLiveness
// HMD-motion gate. Picked so a casual gesture doesn't release the hold but
// a paused user (looking around, holding still to read) does.
constexpr double kStationaryHmdMps = 0.05;

// Returns the verdict the detector should produce given the current
// (translStdDev, rotMaxAngle) and the previously committed lock state.
// Single owner of the hysteresis decision; unit tests pin the contract.
inline bool VerdictWithHysteresis(double translStdDev,
                                  double rotMaxAngle,
                                  bool prevLocked)
{
    if (prevLocked) {
        const bool stillRigid =
            (translStdDev < kLeaveTranslM) && (rotMaxAngle < kLeaveRotRad);
        return stillRigid;
    }
    const bool genuinelyRigid =
        (translStdDev < kEnterTranslM) && (rotMaxAngle < kEnterRotRad);
    return genuinelyRigid;
}

// Returns true when an HMD linear speed is low enough to commit a queued
// flip without producing a visible mid-gesture jump.
inline bool HmdIsStationary(double hmdSpeedMps)
{
    return hmdSpeedMps < kStationaryHmdMps;
}

} // namespace spacecal::autolock
