#pragma once

// Pure helpers for the warm-restart snap path -- detects "user took the HMD
// off, came back, put it on again" via the proximity-sensor rising edge and
// decides whether to engage the saved-profile snap + acceptance-gate grace
// window. Extracted here so the threshold / eligibility logic can be unit-
// tested without standing up the full CalibrationTick + IVRSystem stubs.
// See CalibrationContext::warmRestartGraceSamples for the runtime state
// and Calibration.cpp's CalibrationTick for the call site.

namespace spacecal::warm_restart {

// Minimum proximity-false duration that counts as a real break. Below this,
// the rising edge is treated as sensor noise (some HMD runtimes report brief
// drops on radio glitches that don't reflect the user removing the headset).
// 5 s is well past any single-tick blip while still feeling immediate when
// the user actually does step away for a moment.
constexpr double kMinAwaySeconds = 5.0;

// Upper bound on awayForSeconds for the proximity-signal engage path. Beyond
// this duration, the saved profile is treated as stale enough that snapping
// without validation is unsafe -- e.g. base stations could have shifted, the
// room layout could have changed, or the rig could have been physically
// moved while powered down. Falls through to the normal cold-calibration
// path, which re-validates everything from scratch. 4 h covers a full work
// session of intermittent breaks; multi-day absences land in cold cal.
// Bypassed by the pose-jump fast-path (large physical movement is itself
// evidence that the saved profile must be re-evaluated regardless of how
// long the user was away).
constexpr double kMaxAwaySeconds = 4.0 * 3600.0;

// HMD position-jump threshold for the single-signal fast-path. When the
// activity-level signal fails (Quest variants over Link with flaky
// proximity, IMU stillness threshold not met on a wobbly desk, etc.), a
// large HMD position delta from before the user "went away" to when they
// came back is unambiguous evidence of repositioning. Single-signal fire
// at this magnitude bypasses kMinAwaySeconds entirely -- the physical jump
// IS the evidence of warm-restart, regardless of activity-level transitions.
// 30 cm chosen as well above noise-in-pose (typical IMU drift over seconds
// is sub-cm) and below the smallest "user wandered around the room and
// came back" delta that would routinely false-fire.
constexpr double kPositionJumpFastPathM = 0.30;

// Cold-start safety: suppress engages for the first N continuous-cal ticks
// of a session. Without this, a session that starts with the HMD off (user
// will put it on shortly) sees the rising edge as a warm restart, even
// though there is no prior calibration state to "snap back to". At ~3.5 Hz
// this is ~30 s of session warmup; long enough to cover normal "launch
// the overlay, then put on the HMD" sequencing.
constexpr int kColdStartGraceTicks = 100;

// Validation thresholds (metres). After the snap engages, the solver
// converges over kGraceSamples ticks. During that window the caller
// watches the rolling MAD floor:
//   - madFloor < kValidatedSettledMadM AND samples_since_snap >= 20:
//     declare Settled, end grace early. Profile snap was correct.
//   - At grace end with madFloor > kValidatedFailedMadM:
//     declare FailedAboveThreshold, trigger RecoverFromWedgedCalibration.
//     Snap landed on a profile that no longer matches reality.
//   - In between: Inconclusive, profile stays, no recovery (current
//     samples_exhausted behaviour, but logged with the actual MAD).
constexpr double kValidatedSettledMadM = 0.008;   // 8 mm
constexpr double kValidatedFailedMadM  = 0.020;   // 20 mm
constexpr int    kValidationMinSamples = 20;

enum class ValidationOutcome {
    Inconclusive = 0,
    Settled      = 1,
    Failed       = 2,
};

// Number of Continuous-mode ticks to keep the prior-vs-new error rejection
// gate bypassed once a warm restart engages. At the calibrator's ~3.5 Hz
// continuous-tick cadence this is ~30 s of grace -- long enough for the
// rolling sample buffer to refill and the solver to converge against the
// saved transform, short enough that a wrong snap (user moved a base station
// while away) is bounded in how long the bad calibration sits there before
// the normal continuous-cal recovery takes over.
constexpr int kGraceSamples = 100;

// Inputs to the engage decision. Built once per tick from the live OpenVR
// proximity reading + the cached CalibrationContext state. `awayForSeconds`
// is the wall-clock duration the user has been away; callers pass 0 when
// they don't have a previous "away" timestamp (no edge to evaluate).
//
// `awayPositionDeltaM` is the distance between the HMD's position captured
// at the falling edge (when the user took the HMD off) and the position
// observed at the rising edge (when the user put it back on). Callers pass
// 0 when no falling-edge position was captured (cold session start, or
// position not yet known). When this is >= kPositionJumpFastPathM, the
// engage gate bypasses the proximity-based kMinAwaySeconds and kMaxAwaySeconds
// thresholds entirely -- the physical jump is the warm-restart signal.
//
// `tickId` is the count of continuous-cal solver ticks since session start.
// Used for the cold-start safety check; values below kColdStartGraceTicks
// suppress engage to prevent session-startup false positives.
struct EngageInput {
    bool   wasPresent;        // ctx.lastUserPresent before this tick
    bool   nowPresent;        // freshly-read activity-level signal
    double awayForSeconds;    // time - ctx.userAwaySince (0 if not currently away)
    bool   validProfile;      // ctx.validProfile
    bool   stateEligible;     // ctx.state in {Continuous, ContinuousStandby}
    double awayPositionDeltaM = 0.0;  // HMD displacement while "away"
    int    tickId             = 1 << 30;  // tick counter; default safe-past-cold-start
};

// True iff the engage gate fires. Two paths compose:
//
//   1. Proximity-and-time path (the default since the original landing):
//      rising edge + awayForSeconds in [kMinAwaySeconds, kMaxAwaySeconds]
//      + validProfile + stateEligible + past cold-start grace.
//
//   2. Pose-jump fast-path (added 2026-05-25 evening): a >=
//      kPositionJumpFastPathM HMD displacement between "user went away" and
//      "user came back" is itself sufficient evidence, regardless of how
//      long the away gap was. Still requires a rising edge + validProfile +
//      stateEligible + past cold-start grace; fires only when the
//      proximity-and-time path would have been rejected for being too brief
//      or too long. Designed for HMDs whose activity-level signal is
//      unreliable (dead proximity sensor, IMU stillness not met).
constexpr bool ShouldEngage(const EngageInput& in) {
    if (in.wasPresent || !in.nowPresent) return false;
    if (!in.validProfile || !in.stateEligible) return false;
    if (in.tickId < kColdStartGraceTicks) return false;

    const bool poseJump = in.awayPositionDeltaM >= kPositionJumpFastPathM;
    const bool proximityAndTime =
        in.awayForSeconds >= kMinAwaySeconds
        && in.awayForSeconds <= kMaxAwaySeconds;

    return poseJump || proximityAndTime;
}

// Validation outcome from one tick's MAD reading. Pure; caller maintains
// the running tick count since snap.
constexpr ValidationOutcome EvaluateValidation(double madFloorM,
                                                int samplesSinceSnap,
                                                bool graceEndedThisTick)
{
    if (samplesSinceSnap >= kValidationMinSamples
        && madFloorM > 0.0
        && madFloorM < kValidatedSettledMadM) {
        return ValidationOutcome::Settled;
    }
    if (graceEndedThisTick && madFloorM > kValidatedFailedMadM) {
        return ValidationOutcome::Failed;
    }
    return ValidationOutcome::Inconclusive;
}

}  // namespace spacecal::warm_restart
