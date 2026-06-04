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
// MotionGate.h, GeometryShiftDetector.h, WatchdogDecisions.h.

#include <Eigen/Geometry>
#include <algorithm>
#include <deque>
#include <vector>

namespace spacecal::autolock {

// Forward declarations for the leaf inline helpers. Lets composite helpers
// (EvaluateCommitGate and any future addition) call HmdIsStationary regardless
// of where the composite is placed in this file.
inline bool HmdIsStationary(double hmdSpeedMps);

// Window size of the relative-pose history. With samples arriving at the
// continuous-cal cadence (~2 Hz post-buffer-fill), 20 samples ~ 10 s. Was
// 30 (~15 s) before 2026-05-25; the longer window held a single transient
// outlier in the MAD computation for too long, so even a clean 30s
// settling period could be interrupted by one ~50mm spike from 12-14s
// earlier. The shorter window ages transients out faster while still
// covering enough samples for the MAD median to be statistically
// meaningful.
constexpr size_t kHistoryMax = 20;
constexpr size_t kSamplesNeeded = 20;

// Hysteresis thresholds. Tighter on enter (need genuine rigid attachment)
// than on leave (don't release a locked relationship over a few mm of noise).
//
// kLeaveTranslM widened from 8 mm to 15 mm on 2026-05-25: the prior 5 mm
// deadband (3 enter / 8 leave) was tight relative to the steady-state MAD
// scatter on cross-tracking-system pairs (Quest+Lighthouse routinely 3-6 mm
// MAD even when rigidly attached), so transient outliers in the 8-15 mm
// band were forcing unlock on otherwise-clean attachments. 15 mm leaves
// the panic-unlock at 40 mm comfortably above the new floor; locks that
// have genuinely broken still release via IsPanicLevelDeviation regardless
// of where the leave threshold sits.
constexpr double kEnterTranslM = 0.003;                 // 3 mm hard floor
constexpr double kLeaveTranslM = 0.015;                 // 15 mm
constexpr double kEnterRotRad = 0.7 * EIGEN_PI / 180.0; // 0.7 deg
constexpr double kLeaveRotRad = 1.5 * EIGEN_PI / 180.0; // 1.5 deg

// Adaptive enter threshold derived from the rolling MAD floor. The hard
// floor at kEnterTranslM keeps the gate tight on low-noise rigs (every
// attached pair must still demonstrate ~few-mm rigidity before locking);
// the ceiling at kLeaveTranslM - 1 mm preserves a 1 mm deadband even on
// the noisiest rigs. The 2x scale lets a pair whose observed steady-state
// MAD floor is e.g. 5 mm engage AUTO Lock at 10 mm rather than waiting
// forever for the MAD to drop below the hardcoded 3 mm constant -- the
// 2026-05-25 session log showed exactly this starvation, with autoLockEff
// alternating 0/1 indefinitely on a rig whose MAD floor sat near 4 mm.
//
// Caller maintains the rolling MAD floor (typically the minimum or low
// quantile of the last ~60 s of robust deviation readings) and passes
// it here per-tick. A floor of 0 (no observations yet) returns the hard
// floor unchanged so the gate doesn't relax before evidence exists.
constexpr double kEnterAdaptiveScale = 2.0;
inline double EnterThresholdFor(double madFloor)
{
	constexpr double kCeil = kLeaveTranslM - 0.001;
	double v = kEnterAdaptiveScale * madFloor;
	if (v < kEnterTranslM) v = kEnterTranslM;
	if (v > kCeil) v = kCeil;
	return v;
}

// HMD linear-speed threshold (m/s) below which a queued AUTO-lock flip is
// allowed to commit. Same order of magnitude as the existing TrackerLiveness
// HMD-motion gate. Picked so a casual gesture doesn't release the hold but
// a paused user (looking around, holding still to read) does.
constexpr double kStationaryHmdMps = 0.05;

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
// 40 mm = 5 x kLeaveTranslM. Sized so normal motion noise cannot land at
// panic level. The 2026-05-22 session log
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
//
// `enterTranslM` defaults to the kEnterTranslM hard floor so callers that
// don't track an adaptive MAD floor keep the historical contract. Callers
// that do (the primary detector in Calibration.cpp uses a rolling minimum
// of recent MAD readings as the floor input to EnterThresholdFor) pass the
// adaptive value here so cross-tracking-system pairs whose natural noise
// sits above 3 mm can still engage AUTO Lock.
inline bool VerdictWithHysteresis(double translStdDev, double rotMaxAngle, bool prevLocked,
                                  double enterTranslM = kEnterTranslM)
{
	if (prevLocked) {
		const bool stillRigid = (translStdDev < kLeaveTranslM) && (rotMaxAngle < kLeaveRotRad);
		return stillRigid;
	}
	const bool genuinelyRigid = (translStdDev < enterTranslM) && (rotMaxAngle < kEnterRotRad);
	return genuinelyRigid;
}

// Settled-state predicate. "Settled" means: currently locked, MAD inside
// the adaptive enter band (so we're well-inside the deadband, not riding
// the leave threshold), and the lock has held for at least
// kSettledMinHoldSec since the last flip (so a freshly-committed lock
// doesn't immediately count as settled). Pure helper; the caller maintains
// `secsSinceLastFlip` from the auto_lock_flip log path.
//
// Used by the [cal-heartbeat] settled= field; a >70% rate in real sessions
// is the success criterion for the 2026-05-25 settling fix.
constexpr double kSettledMinHoldSec = 3.0;
inline bool IsSettled(bool currentlyLocked, double translMad, double madFloor, double secsSinceLastFlip)
{
	if (!currentlyLocked) return false;
	if (secsSinceLastFlip < kSettledMinHoldSec) return false;
	return translMad < EnterThresholdFor(madFloor);
}

// Returns true when an HMD linear speed is low enough to commit a queued
// flip without producing a visible mid-gesture jump.
inline bool HmdIsStationary(double hmdSpeedMps)
{
	return hmdSpeedMps < kStationaryHmdMps;
}

// Per-tick commit-gate decision for a pending AUTO Lock flip. Captures the
// three-way logic shared between the primary and extras detectors:
//   - HMD stationary lets any pending flip commit (lock or unlock).
//   - Asymmetric unlock-timeout escape: pending unlocks commit after
//     kAutoLockUnlockMaxWaitSeconds regardless of motion. Lock-direction
//     flips have no escape -- locking under sustained motion is exactly
//     the case the stationary gate exists to prevent.
//
// The `mode` field carries a static string literal suitable for the
// "committed_via" diagnostic. Callers must not free it.
struct CommitGateDecision
{
	bool commit;
	const char* mode;
};

inline CommitGateDecision EvaluateCommitGate(bool pendingFlipTo, double hmdSpeedMps, double now, double pendingHeldSec)
{
	(void)now;
	const bool stationary = HmdIsStationary(hmdSpeedMps);
	const bool isUnlock = (pendingFlipTo == false);
	const bool unlockTimeoutFired = isUnlock && pendingHeldSec >= kAutoLockUnlockMaxWaitSeconds;

	if (!stationary && !unlockTimeoutFired) {
		return {false, "held"};
	}
	if (unlockTimeoutFired) return {true, "unlock_timeout"};
	if (stationary) return {true, "stationary_gate"};
	return {true, "unknown"};
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
		xs[i] = t.x();
		ys[i] = t.y();
		zs[i] = t.z();
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
