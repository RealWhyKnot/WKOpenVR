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
#include <algorithm>
#include <deque>
#include <vector>

namespace spacecal::autolock {

// Forward declarations for the leaf inline helpers. Lets composite helpers
// (EvaluateCommitGate and any future addition) call HmdIsStationary /
// ShouldSuppressForReanchor regardless of where the composite is placed in
// this file. Without these, defining a composite above its callees would
// fail to compile -- the d1a7e9e -> 988bbf8 work hit exactly that.
inline bool HmdIsStationary(double hmdSpeedMps);
inline bool ShouldSuppressForReanchor(double now, double suppressUntil);

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

// When a chi-square reanchor fires, the underlying re-anchor briefly spikes
// translation stddev past the leave threshold (kLeaveTranslM = 8 mm), which
// alone would trip an unlock for ~1 tick before stddev settles back. The
// detector's swing-back logic at UpdateAutoLockDetector drops the pending
// flip if the verdict reverts inside this window -- but only if the commit
// hasn't already fired. This gate holds the commit for kReanchorSuppressSeconds
// after each reanchor, giving the swing-back path time to absorb the spike.
//
// Sized to ~0.6 s -- just above the reanchor's own freezeUntil window
// (kFreezeWindowSec = 0.5 in ReanchorChiSquareDetector.h). Originally 2.0 s,
// but at the observed reanchor cadence of ~25 fires/min (one every ~2.4 s),
// a 2.0 s window unconditionally re-extended on every fire kept the
// suppression chain continuous for entire multi-hour sessions, locking AUTO
// Lock out completely. 0.6 s keeps the chain broken in steady state while
// still covering the reanchor's own dispersion window.
constexpr double kReanchorSuppressSeconds = 0.6;

// AUTO Lock commit-gate "pending flip held too long" threshold (seconds).
// When a target verdict has been queued but the commit gate has held it for
// this long, emit a diagnostic so we can see whether the gate is the dominant
// blocker. Throttled by the caller; this is just the threshold value.
constexpr double kAutoLockGateHeldWarnSeconds = 2.0;

// Asymmetric unlock-gate timeout (seconds). Lock commits require the user
// to be stationary so the resulting calibration jump is hidden under
// natural stillness. Unlock commits semantically don't need the same gate
// -- the user is in motion (which IS why the unlock was queued) and waiting
// for them to be stationary is exactly backwards. If a pending unlock has
// been held longer than this, commit it without the stationary check. Lock
// commits are unaffected.
//
// 5 s is long enough to ride through a quick gesture without releasing
// (the swing-back path catches those) but short enough that real sustained
// motion releases the lock promptly.
constexpr double kAutoLockUnlockMaxWaitSeconds = 5.0;

// Panic-unlock thresholds. When a currently-locked pair's robust deviation
// jumps this far past the normal leave threshold, the rigid relationship
// has clearly broken. The pending-flip queue exists to hide visible
// calibration jumps under natural stillness, but at panic-level deviation
// the jump is already happening -- holding the effective state locked for
// up to kAutoLockUnlockMaxWaitSeconds only extends the window of wrong
// output. Callers should bypass the queue and write the effective state
// directly when this predicate fires.
//
// 40 mm = 5 x kLeaveTranslM. Sized so a chi-square reanchor spike plus
// normal motion noise cannot land at panic level -- the reanchor freeze
// window (kFreezeWindowSec = 0.5 in ReanchorChiSquareDetector.h) bounds
// the reanchor displacement, and even worst-case translation jumps stay
// well under 40 mm in the 30-sample MAD. The 2026-05-22 session log
// (the calibration robustness pass) showed the worst-case real outlier
// at ~670 mm, comfortably past this threshold; sub-30 mm spikes are
// absorbed by the normal 5 s unlock-timeout path.
//
// 5 deg picked symmetrically: 3.3 x kLeaveRotRad. Any rigid attachment
// drifting 5 deg from its median quaternion is broken, not noisy.
constexpr double kPanicTranslM = 0.040;
constexpr double kPanicRotRad = 5.0 * EIGEN_PI / 180.0;

// True when the robust deviation metrics indicate a clearly-broken rigid
// relationship. Callers should consult both this and VerdictWithHysteresis:
// VerdictWithHysteresis answers "what does the detector decide" (queued
// through the stationary gate); IsPanicLevelDeviation answers "is the
// queue still appropriate" (no -- commit unlock now).
inline bool IsPanicLevelDeviation(double translStdDev, double rotMaxAngle)
{
    return translStdDev >= kPanicTranslM || rotMaxAngle >= kPanicRotRad;
}

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

// Returns true while a pending AUTO Lock flip should be held off because a
// chi-square reanchor recently fired. `now` is the current tick time in the
// same units as `suppressUntil` (the deadline timestamp written when the
// reanchor fired). A zero/unset suppressUntil never suppresses.
inline bool ShouldSuppressForReanchor(double now, double suppressUntil)
{
    return now < suppressUntil;
}

// Per-tick commit-gate decision for a pending AUTO Lock flip. Captures the
// three-way logic shared between the primary and extras detectors:
//   - HMD stationary lets any pending flip commit (lock or unlock).
//   - Reanchor-suppress window blocks commits regardless (primary only;
//     extras pass suppressUntil = 0.0 since they have no reanchor concept).
//   - Asymmetric unlock-timeout escape: pending unlocks commit after
//     kAutoLockUnlockMaxWaitSeconds regardless of motion. Lock-direction
//     flips have no escape -- locking under sustained motion is exactly
//     the case the stationary gate exists to prevent.
//
// The `mode` field carries a static string literal suitable for the
// "committed_via" diagnostic. Callers must not free it.
struct CommitGateDecision {
    bool commit;
    const char* mode;
};

inline CommitGateDecision EvaluateCommitGate(bool pendingFlipTo,
                                             double hmdSpeedMps,
                                             double now,
                                             double reanchorSuppressUntil,
                                             double pendingHeldSec)
{
    const bool stationary = HmdIsStationary(hmdSpeedMps);
    const bool suppressed = ShouldSuppressForReanchor(now, reanchorSuppressUntil);
    const bool isUnlock = (pendingFlipTo == false);
    const bool unlockTimeoutFired = isUnlock
        && pendingHeldSec >= kAutoLockUnlockMaxWaitSeconds;

    if ((!stationary || suppressed) && !unlockTimeoutFired) {
        return { false, "held" };
    }
    if (unlockTimeoutFired) return { true, "unlock_timeout" };
    if (stationary)         return { true, "stationary_gate" };
    return { true, "unknown" };
}

// MAD-based robust translation deviation over a window of relative-pose
// samples. Returns a stddev-equivalent value (MAD scaled by 1.4826, the
// Gaussian consistency factor) that ignores single-sample outliers.
//
// Cross-tracking-system pairs (e.g. Quest HMD + Lighthouse tracker) produce a
// bimodal noise pattern: most samples are tight (sub-2mm) with occasional
// large transient excursions (USB hiccups, pose extrapolation glitches,
// inter-system synchronisation jitter). Classic sqrt(variance) inflates
// badly on those spikes and prevents AUTO Lock from ever committing, even
// though 27 of 30 samples in the window agree on a rigid relationship. MAD
// stays at the steady-state noise level until a majority of samples shift,
// which is the correct signal for "rigid attachment has changed".
//
// Componentwise median on translation is computed via nth_element on each
// axis; deviation magnitudes use the L2 norm against the median vector.
// O(N) for N=30; trivial cost per tick.
inline double RobustTranslDeviation(const std::deque<Eigen::AffineCompact3d>& history)
{
    const size_t n = history.size();
    if (n == 0) return 0.0;

    std::vector<double> xs(n), ys(n), zs(n);
    for (size_t i = 0; i < n; ++i) {
        const auto& t = history[i].translation();
        xs[i] = t.x(); ys[i] = t.y(); zs[i] = t.z();
    }
    auto medianOf = [](std::vector<double>& v) {
        const size_t mid = v.size() / 2;
        std::nth_element(v.begin(), v.begin() + mid, v.end());
        return v[mid];
    };
    const Eigen::Vector3d median(medianOf(xs), medianOf(ys), medianOf(zs));

    std::vector<double> devs(n);
    for (size_t i = 0; i < n; ++i) {
        devs[i] = (history[i].translation() - median).norm();
    }
    const size_t mid = n / 2;
    std::nth_element(devs.begin(), devs.begin() + mid, devs.end());
    return devs[mid] * 1.4826;
}

// MAD-based robust rotation deviation. Median of geodesic distances from
// the median quaternion sample, scaled by 1.4826. Same outlier-rejection
// rationale as RobustTranslDeviation. The middle-sample approximation of
// the median quaternion matches the legacy code path's choice (the proper
// Frechet mean on SO(3) is not worth the complexity for a 30-sample
// outlier-detection metric).
inline double RobustRotDeviation(const std::deque<Eigen::AffineCompact3d>& history)
{
    const size_t n = history.size();
    if (n == 0) return 0.0;

    const Eigen::Quaterniond medianQ(history[n / 2].rotation());

    std::vector<double> devs(n);
    for (size_t i = 0; i < n; ++i) {
        devs[i] = medianQ.angularDistance(Eigen::Quaterniond(history[i].rotation()));
    }
    const size_t mid = n / 2;
    std::nth_element(devs.begin(), devs.begin() + mid, devs.end());
    return devs[mid] * 1.4826;
}

} // namespace spacecal::autolock
