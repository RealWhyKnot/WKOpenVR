#include "Calibration.h"
#include "CalibrationInternal.h"
#include "CalibrationDevicePoseUtils.h"
#include "CalibrationPoseSampling.h"
#include "CalibrationProfileApply.h"
#include "CalibrationRecoveryTick.h"
#include "CalibrationMetrics.h"
#include "Configuration.h"
#include "IPCClient.h"
#include "CalibrationCalc.h"
#include "VRState.h"
#include "WedgeDetector.h"   // ShouldFireRuntimeWedgeRecovery, kMaxPlausibleCalibrationMagnitudeCm
#include "GeometryShiftDetector.h"  // IsCurrentErrorSpike, ShouldFireGeometryShiftRecovery
#include "CommonModeCoherence.h"   // spacecal::coherence::ComputeCoherenceScore
#include "MotionGate.h"      // ShouldBlendCycle -- auto-recovery snap decision
#include "LatencyEstimator.h"  // spacecal::latency::EstimateLagTimeDomain / EstimateLagGccPhat
#include "TiltDiagnostic.h"    // spacecal::gravity::EvaluateTilt -- sustained gravity-disagreement annotation
#include "Wizard.h"          // spacecal::wizard::IsActive -- gate the runtime wedge detector
                             // off while the user is mid-setup so it can't disrupt them.
#include "RestLockedYaw.h"   // spacecal::rest_yaw::* -- rest-locked yaw drift correction
                             // (continuous-cal-OFF mode). Default OFF; opt-in via Experimental tab.
#include "RecoveryDeltaBuffer.h" // spacecal::recovery_delta::* -- predictive pre-correction
                             // from the rolling buffer of 30 cm relocalization events.
#include "ReanchorChiSquareDetector.h" // spacecal::reanchor_chi::* -- sub-30 cm re-anchor
                             // sub-detector. Detection-only; freezes recs A/C on candidate.
#include "TrackerLiveness.h" // spacecal::liveness::* -- detect non-HMD calibration anchor
                             // going silent under SteamVR's "Running_OK + poseIsValid stays true
                             // while pose hash is frozen" disconnect path.
#include "RotationMatrix3.h" // AngleFromRotationMatrix3 / AxisFromRotationMatrix3 (clamped).
#include "AutoLockHysteresis.h" // spacecal::autolock::VerdictWithHysteresis -- AUTO Lock
                             // hysteresis + stationary-gate constants and pure helpers.
#include "WarmRestart.h"     // spacecal::warm_restart::ShouldEngage -- proximity-edge
                             // decision for the saved-profile snap path.

#include <string>
#include <vector>
#include <iostream>
#include <algorithm>
#include <map>

#include <Eigen/Dense>
#include <GLFW/glfw3.h>

CalibrationContext CalCtx;

// Runtime wedge detector state. Persists across CalibrationTick calls;
// updated by spacecal::wedge::ShouldFireRuntimeWedgeRecovery (header-only,
// pure). Sentinel <0 means "no active wedge candidate"; otherwise stores
// the glfwGetTime() timestamp of the first tick magnitude crossed the bound.
// File scope (not anonymous namespace) so CalibrationTick can read+write it
// directly from earlier in the translation unit.
static double g_runtimeWedgeSince = -1.0;

// CPU-pressure diagnostic. Samples GetProcessTimes() once per CalibrationTick
// (~20 Hz). Computes the % of total CPU the SC process used over the wall-clock
// delta since the last sample, divided by the logical-processor count so 100%
// means "fully saturating one core's worth of compute". Maintains a 5-second
// EMA so a one-tick computationTime spike doesn't false-trigger; emits a
// `cpu_pressure_warning_on` annotation when the EMA crosses 50% and a
// `_off` annotation when it falls back below 30% (hysteresis stops flapping).
//
// Pure diagnostic. No behavior change. The emit-once-on-transition pattern
// matches the existing gravity_disagreement_sustained_on/off annotations.
struct CpuPressureState {
    bool initialized = false;
    bool alarmed = false;
    uint64_t lastCpuNs = 0;
    uint64_t lastWallNs = 0;
    double emaPct = 0.0;       // 0..100 in single-core-equivalent percent
    int logicalProcessors = 1;
};
static CpuPressureState g_cpuPressureState;

constexpr double kCpuPressureOnThresholdPct  = 50.0;
constexpr double kCpuPressureOffThresholdPct = 30.0;
constexpr double kCpuPressureEmaTimeConstSec = 5.0;
constexpr double kCpuPressureSpikeMs         = 200.0;  // single-tick spike threshold

inline uint64_t FileTimeToNs100(const FILETIME& ft) {
    return ((uint64_t)ft.dwHighDateTime << 32) | ft.dwLowDateTime;  // units of 100 ns
}

// Samples this tick's CPU usage + folds into the EMA. Emits transition
// annotations on EMA crossing and a one-shot `cpu_pressure_spike` on any
// single ComputeIncremental that took >= kCpuPressureSpikeMs.
static void TickCpuPressureMonitor(double computationTimeMs, double now_s) {
    auto& s = g_cpuPressureState;
    FILETIME ftCreate, ftExit, ftKernel, ftUser;
    if (!GetProcessTimes(GetCurrentProcess(), &ftCreate, &ftExit, &ftKernel, &ftUser)) {
        return;  // bail silently; this is diagnostic only
    }
    const uint64_t cpuNow = FileTimeToNs100(ftKernel) + FileTimeToNs100(ftUser);
    LARGE_INTEGER pcNow;
    QueryPerformanceCounter(&pcNow);
    LARGE_INTEGER pcFreq;
    QueryPerformanceFrequency(&pcFreq);
    const uint64_t wallNow = (uint64_t)((pcNow.QuadPart * 10'000'000ull) / pcFreq.QuadPart);  // 100ns

    if (!s.initialized) {
        SYSTEM_INFO si{};
        GetSystemInfo(&si);
        s.logicalProcessors = si.dwNumberOfProcessors > 0 ? (int)si.dwNumberOfProcessors : 1;
        s.lastCpuNs = cpuNow;
        s.lastWallNs = wallNow;
        s.initialized = true;
        return;
    }

    const uint64_t cpuDelta = cpuNow >= s.lastCpuNs ? cpuNow - s.lastCpuNs : 0;
    const uint64_t wallDelta = wallNow >= s.lastWallNs ? wallNow - s.lastWallNs : 0;
    s.lastCpuNs = cpuNow;
    s.lastWallNs = wallNow;
    if (wallDelta == 0) return;

    const double instPct = 100.0 * (double)cpuDelta /
        ((double)wallDelta * (double)s.logicalProcessors);

    // EMA toward instPct with time constant kCpuPressureEmaTimeConstSec.
    const double dtSec = (double)wallDelta / 1.0e7;
    const double alpha = 1.0 - std::exp(-dtSec / kCpuPressureEmaTimeConstSec);
    s.emaPct = s.emaPct + alpha * (instPct - s.emaPct);

    // EMA-crossing transition annotations with hysteresis.
    if (!s.alarmed && s.emaPct > kCpuPressureOnThresholdPct) {
        s.alarmed = true;
        char buf[256];
        snprintf(buf, sizeof buf,
            "cpu_pressure_warning_on: ema_pct=%.1f inst_pct=%.1f cores=%d"
            " upstream=%d gcc_phat=%d cusum=%d velocity_aware=%d tukey=%d kalman=%d"
            " auto_detect_latency=%d state=%d",
            s.emaPct, instPct, s.logicalProcessors,
            (int)CalCtx.useUpstreamMath,
            (int)CalCtx.useGccPhatLatency, (int)CalCtx.useCusumGeometryShift,
            (int)CalCtx.useVelocityAwareWeighting, (int)CalCtx.useTukeyBiweight,
            (int)CalCtx.useBlendFilter, (int)CalCtx.latencyAutoDetect,
            (int)CalCtx.state);
        Metrics::WriteLogAnnotation(buf);
    } else if (s.alarmed && s.emaPct < kCpuPressureOffThresholdPct) {
        s.alarmed = false;
        char buf[128];
        snprintf(buf, sizeof buf,
            "cpu_pressure_warning_off: ema_pct=%.1f", s.emaPct);
        Metrics::WriteLogAnnotation(buf);
    }

    // Per-tick spike annotation. Independent of the EMA; a single
    // ComputeIncremental that took >= 200 ms is worth surfacing on its own
    // because it indicates a stall-class event regardless of session-average
    // CPU. Throttled to one annotation per 5 seconds so a sustained-high
    // window doesn't flood.
    static double s_lastSpikeAnnotation = -1e9;
    if (computationTimeMs >= kCpuPressureSpikeMs && (now_s - s_lastSpikeAnnotation) >= 5.0) {
        s_lastSpikeAnnotation = now_s;
        char buf[256];
        snprintf(buf, sizeof buf,
            "cpu_pressure_spike: computationTime_ms=%.1f ema_pct=%.1f"
            " upstream=%d gcc_phat=%d cusum=%d velocity_aware=%d tukey=%d kalman=%d state=%d",
            computationTimeMs, s.emaPct,
            (int)CalCtx.useUpstreamMath,
            (int)CalCtx.useGccPhatLatency, (int)CalCtx.useCusumGeometryShift,
            (int)CalCtx.useVelocityAwareWeighting, (int)CalCtx.useTukeyBiweight,
            (int)CalCtx.useBlendFilter, (int)CalCtx.state);
        Metrics::WriteLogAnnotation(buf);
    }
}

// Auto-recovery snap flag (option-3 bundle, 2026-05-04). Set true by
// RecoverFromWedgedCalibration so the next ScanAndApplyProfile cycle sends
// every device transform with payload.lerp=false, which routes through the
// driver's existing snap path in SetDeviceTransform (transform := target,
// no blending). Without this, the post-recovery cal would be smoothly
// interpolated through whatever wrong steady-state the driver had cached
// -- defeating the point of recovery.
//
// Rationale (per feedback_calibration_blending_request.md): blending is
// for smooth steady-state convergence. Recovery events are catastrophic
// state changes -- smoothing them is wrong. Snap on the next tick, then
// resume normal blending.
//
// Cleared by ScanAndApplyProfile after consuming. One-shot.
bool g_snapNextProfileApply = false;

// AdditionalCalibration's special members live inline in the header now --
// CalibrationCalc is complete at the include point, so the implicitly-defined
// destructor handles the unique_ptr just fine.

// AUTO Lock hysteresis + stationary-gate constants and pure helpers live in
// AutoLockHysteresis.h so they can be unit-tested without instantiating
// CalibrationContext. See that header for the why behind the threshold pair.

// Diagnostic snapshot of the most recent AUTO Lock detector readings and
// the geometry-shift detector accumulator state. The AUTO Lock pair is
// written by UpdateAutoLockDetector each time the rigidity verdict is
// recomputed; the geometry-shift pair is owned by the detector block in
// CalibrationTick (promoted from function-block static so the [cal-
// heartbeat] log emitter, which runs earlier in the same tick, can see
// the latest values from the previous tick).
//
// These are diagnostics only -- no consumer reads them for control flow.
// Their job is to make the next session log self-describing without
// requiring an instrumentation pass.
static double g_lastAutoLockTranslMad = 0.0;
static double g_lastAutoLockRotMad = 0.0;
static int g_geomShiftConsecutiveBadTicks = 0;
static spacecal::geometry_shift::CusumState g_cusumState;
static spacecal::geometry_shift::DeferralState g_geomShiftDeferral;

void CalibrationContext::UpdateAutoLockDetector(
	const Eigen::AffineCompact3d& refWorld,
	const Eigen::AffineCompact3d& targetWorld)
{
	using namespace spacecal::autolock;

	// Relative pose: target expressed in the reference's local frame. For a
	// rigidly attached pair (tracker glued to HMD), this is constant; for an
	// independent pair (tracker on hip vs HMD on head), it varies as the
	// user moves.
	const Eigen::AffineCompact3d rel = refWorld.inverse() * targetWorld;
	autoLockHistory.push_back(rel);
	while (autoLockHistory.size() > kHistoryMax) autoLockHistory.pop_front();

	if (autoLockHistory.size() < kSamplesNeeded) {
		// Not enough data yet -- stay in "not detected" state. AUTO mode
		// users see the calibration unlocked and re-solving until the
		// detector earns confidence.
		autoLockEffectivelyLocked = false;
		autoLockHasPendingFlip = false;
		return;
	}

	// MAD-based robust deviation metrics. The previous sqrt(variance) +
	// max-from-median pair inflated badly on single-sample outliers, which
	// on cross-tracking-system rigs (Quest HMD + Lighthouse tracker) are
	// frequent enough that pending LOCKs queued during steady-state windows
	// got cancelled within a tick. See AutoLockHysteresis.h for the why.
	const double translStdDev = RobustTranslDeviation(autoLockHistory);
	const double rotMaxAngle  = RobustRotDeviation(autoLockHistory);
	g_lastAutoLockTranslMad = translStdDev;
	g_lastAutoLockRotMad = rotMaxAngle;

	// Maintain the rolling MAD floor used by EnterThresholdFor to scale the
	// enter threshold to the rig's natural noise level. Window covers the
	// last 60 s of MAD readings; floor is the min across the in-window
	// samples. Pairs (time, mad) are pushed each tick and trimmed when they
	// age out. See CalibrationContext::autoLockMadFloor for the field
	// rationale.
	{
		constexpr double kFloorWindowSec = 60.0;
		const double nowSec = Metrics::CurrentTime;
		autoLockMadHistory.emplace_back(nowSec, translStdDev);
		while (!autoLockMadHistory.empty()
			&& (nowSec - autoLockMadHistory.front().first) > kFloorWindowSec) {
			autoLockMadHistory.pop_front();
		}
		// Track the timestamp of the sample that produced the floor too;
		// the warm-restart validator uses it to label `mad_floor_source`
		// as preSnap vs postSnap. A floor inherited from pre-snap quiet
		// can mask a wrong snap by satisfying the dispersion gate.
		auto floorEntry = autoLockMadHistory.front();
		for (const auto& p : autoLockMadHistory) {
			if (p.second < floorEntry.second) floorEntry = p;
		}
		autoLockMadFloor = floorEntry.second;
		autoLockMadFloorTs = floorEntry.first;
	}

	// Panic-unlock: at clearly-broken deviation, skip the pending-flip queue
	// and drop the effective lock immediately so downstream cal output stops
	// using the stale rigid attachment. ResolveLockMode runs inline because
	// the normal commit path's ResolveLockMode (CalibrationTick after
	// CommitPendingAutoLockFlipIfStationary) won't fire -- we bypass that
	// helper entirely. The stationary-HMD gate guards against UX-visible cal
	// jumps mid-motion, but at this magnitude the jump has already happened.
	if (autoLockEffectivelyLocked &&
		spacecal::autolock::IsPanicLevelDeviation(translStdDev, rotMaxAngle)) {
		autoLockEffectivelyLocked = false;
		autoLockHasPendingFlip = false;
		autoLockPendingFlipFirstSeen = 0.0;
		autoLockGateHeldWarned = false;
		autoLockLastFlipTime = Metrics::CurrentTime;
		ResolveLockMode();
		char buf[224];
		snprintf(buf, sizeof buf,
			"auto_lock_panic_unlock: translMad=%.4fm rotMad=%.4frad",
			translStdDev, rotMaxAngle);
		Metrics::WriteLogAnnotation(buf);
		return;
	}

	const bool verdict = spacecal::autolock::VerdictWithHysteresis(
		translStdDev, rotMaxAngle, autoLockEffectivelyLocked,
		spacecal::autolock::EnterThresholdFor(autoLockMadFloor));

	// Queue rather than commit when the verdict differs from the currently
	// effective state. CommitPendingAutoLockFlipIfStationary in
	// CalibrationTick promotes the queued value once the HMD speed drops
	// below kStationaryHmdMps -- visible jumps on flip then land during a
	// natural pause in motion rather than mid-gesture.
	if (verdict != autoLockEffectivelyLocked) {
		const bool prevPending = autoLockHasPendingFlip;
		const bool prevTarget  = autoLockPendingFlipTo;
		autoLockHasPendingFlip = true;
		autoLockPendingFlipTo = verdict;
		// Log only on transitions in the queue state to avoid per-tick
		// spam while the user is moving and the flip is held. Captures
		// both new-queue and changed-target events.
		if (!prevPending || prevTarget != verdict) {
			char buf[224];
			snprintf(buf, sizeof buf,
				"auto_lock_flip_pending: target=%d current=%d translMad=%.4fm rotMad=%.4frad samples=%zu",
				(int)verdict, (int)autoLockEffectivelyLocked,
				translStdDev, rotMaxAngle, autoLockHistory.size());
			Metrics::WriteLogAnnotation(buf);
		}
	} else if (autoLockHasPendingFlip) {
		// The detector swung back to agreement with the effective state
		// before the stationary gate fired -- drop the pending flip so we
		// don't commit a change the user wouldn't see as warranted.
		autoLockHasPendingFlip = false;
	}
}

// HMD linear speed in m/s from the latest device-pose snapshot. Single
// source of truth for both the primary AUTO Lock commit path and the
// extras detector loop in CalibrationTick. Both call sites used to inline
// the same sqrt-of-dot-product, which created a scope trap: the two
// `if (ctx.state == Continuous)` blocks in CalibrationTick are textually
// separate, so the primary block's local was invisible to the extras
// block. Centralising prevents future blocks from tripping the same trap.
static double ComputeHmdSpeedMps(const CalibrationContext& ctx)
{
	const auto& hmd = ctx.devicePoses[vr::k_unTrackedDeviceIndex_Hmd];
	return std::sqrt(
		hmd.vecVelocity[0] * hmd.vecVelocity[0] +
		hmd.vecVelocity[1] * hmd.vecVelocity[1] +
		hmd.vecVelocity[2] * hmd.vecVelocity[2]);
}

// Commit a queued AUTO Lock flip when the user is still enough that the
// resulting calibration jump won't be jarring. Returns true if a commit
// happened this call (caller may want to log / kick the calibration math).
//
// `now` is the CalibrationTick time stamp. Used to gate commits during the
// post-reanchor suppression window -- see kReanchorSuppressSeconds for why
// reanchor noise needs to be held off the lock-state decision.
bool CommitPendingAutoLockFlipIfStationary(CalibrationContext& ctx, double hmdSpeedMps, double now)
{
	if (!ctx.autoLockHasPendingFlip) {
		// Pending dropped or never queued -- reset hold tracking so the
		// next pending starts a fresh held-duration measurement.
		ctx.autoLockPendingFlipFirstSeen = 0.0;
		ctx.autoLockGateHeldWarned = false;
		return false;
	}

	// Track when the current pending flip first appeared so we can attribute
	// long holds to the right gate. Don't overwrite if already tracking.
	if (ctx.autoLockPendingFlipFirstSeen <= 0.0) {
		ctx.autoLockPendingFlipFirstSeen = now;
		ctx.autoLockGateHeldWarned = false;
	}

	const double heldSec = now - ctx.autoLockPendingFlipFirstSeen;
	const auto gate = spacecal::autolock::EvaluateCommitGate(
		ctx.autoLockPendingFlipTo, hmdSpeedMps, now,
		ctx.autoLockReanchorSuppressUntil, heldSec);

	if (!gate.commit) {
		// Held by a gate. Emit a one-shot diagnostic per pending flip once
		// the hold exceeds the warn threshold, so a chronic block becomes
		// visible without per-tick log noise. Re-armed by the !pending
		// path above. The reason string needs the raw stationary/suppressed
		// pair so a reader can distinguish motion from reanchor-suppress;
		// the gate helper collapses both into mode="held".
		if (!ctx.autoLockGateHeldWarned
			&& heldSec >= spacecal::autolock::kAutoLockGateHeldWarnSeconds) {
			ctx.autoLockGateHeldWarned = true;
			const bool stationary = spacecal::autolock::HmdIsStationary(hmdSpeedMps);
			const bool suppressed = spacecal::autolock::ShouldSuppressForReanchor(
				now, ctx.autoLockReanchorSuppressUntil);
			char gbuf[280];
			snprintf(gbuf, sizeof gbuf,
				"[autolock][gate-held] pending_target=%d current=%d held_sec=%.2f"
				" reason=%s hmdSpeed=%.3fmps suppress_until=%.3f now=%.3f",
				(int)ctx.autoLockPendingFlipTo, (int)ctx.autoLockEffectivelyLocked,
				heldSec,
				(!stationary && suppressed) ? "motion+suppress"
					: !stationary           ? "motion"
					:                          "suppress",
				hmdSpeedMps, ctx.autoLockReanchorSuppressUntil, now);
			Metrics::WriteLogAnnotation(gbuf);
		}
		return false;
	}

	const bool prev = ctx.autoLockEffectivelyLocked;
	ctx.autoLockEffectivelyLocked = ctx.autoLockPendingFlipTo;
	ctx.autoLockHasPendingFlip = false;
	ctx.autoLockPendingFlipFirstSeen = 0.0;
	ctx.autoLockGateHeldWarned = false;
	ctx.autoLockLastFlipTime = now;

	char buf[280];
	snprintf(buf, sizeof buf,
		"auto_lock_flip: previous=%d now=%d hmdSpeed=%.3fmps held_sec=%.2f committed_via=%s",
		(int)prev, (int)ctx.autoLockEffectivelyLocked, hmdSpeedMps, heldSec, gate.mode);
	Metrics::WriteLogAnnotation(buf);
	return true;
}

void CalibrationContext::ResolveLockMode()
{
	const bool prev = lockRelativePosition;
	switch (lockRelativePositionMode) {
	case LockMode::OFF:  lockRelativePosition = false; break;
	case LockMode::ON:   lockRelativePosition = true;  break;
	case LockMode::AUTO: lockRelativePosition = autoLockEffectivelyLocked; break;
	}
	// Diagnostic: annotate every resolved-value change. The UI-side toggle of
	// "Lock relative position" is invisible in post-session logs unless we
	// trace the resolve step; without this a user-reported "I toggled Lock
	// and nothing happened" cannot be distinguished from "the toggle did
	// take effect but didn't help."
	if (prev != lockRelativePosition) {
		char buf[200];
		snprintf(buf, sizeof buf,
			"lockRelativePosition_change: prev=%d now=%d mode=%d autoLockEffectivelyLocked=%d",
			(int)prev, (int)lockRelativePosition,
			(int)lockRelativePositionMode, (int)autoLockEffectivelyLocked);
		Metrics::WriteLogAnnotation(buf);
	}
}

// Auto-speed resolver. Reads the recent jitter samples from Metrics:: and picks
// the calibration speed that should give a stable result without bogging down
// convergence. Sticky: requires the new bucket to be right for ~5 consecutive
// evaluations before switching, so transient noise doesn't oscillate the
// sample-buffer size. Hysteresis is baked in by separate up-shift / down-shift
// thresholds (1mm / 5mm) so a marginal value doesn't flap between buckets.
CalibrationContext::Speed CalibrationContext::ResolvedCalibrationSpeed() const {
	if (calibrationSpeed != AUTO) {
		return calibrationSpeed;
	}

	// AUTO only re-evaluates meaningfully during continuous calibration --
	// it watches drift evidence and slows the buffer down when conditions
	// degrade. Outside continuous mode there's no second chance to switch,
	// so AUTO would just gamble on the first jitter sample. Default to FAST:
	// the user can pick SLOW or VERY_SLOW explicitly if they want a larger
	// buffer.
	if (state != CalibrationState::Continuous &&
	    state != CalibrationState::ContinuousStandby) {
		return FAST;
	}

	// Sticky state. Mutable-via-cast is fine here -- these are pure caches.
	static Speed s_lastResolved = SLOW;          // safe default before first sample
	static Speed s_pendingResolved = SLOW;
	static int s_pendingTicks = 0;
	constexpr int kRequiredStableTicks = 5;      // ~5 samples of consistency
	// Biased toward FAST: typical lighthouse and tracker-on-HMD setups show
	// 2-5mm of measured jitter even when stationary (lighthouse interpolation
	// + microvibration), but their data is still clean enough for the direct
	// O(N) translation solve. Previously these landed in SLOW (250 samples,
	// ~14s buffer fill) which was punitively slow. The diversity gate already
	// rejects genuinely bad data; the speed selector is a buffer-size hint,
	// not a quality gate.
	constexpr double kFastThreshold = 0.005;     // 5 mm
	constexpr double kSlowThreshold = 0.010;     // 10 mm

	const double jRef = Metrics::jitterRef.last();
	const double jTgt = Metrics::jitterTarget.last();
	// Worst-of-the-two dominates: the noisiest tracker sets the cadence.
	const double j = std::max(jRef, jTgt);

	Speed candidate;
	if (j <= 0.0 || j < kFastThreshold) {
		candidate = FAST;
	} else if (j < kSlowThreshold) {
		candidate = SLOW;
	} else {
		candidate = VERY_SLOW;
	}

	if (candidate == s_lastResolved) {
		s_pendingResolved = candidate;
		s_pendingTicks = 0;
	} else if (candidate == s_pendingResolved) {
		++s_pendingTicks;
		if (s_pendingTicks >= kRequiredStableTicks) {
			s_lastResolved = candidate;
			s_pendingTicks = 0;
		}
	} else {
		s_pendingResolved = candidate;
		s_pendingTicks = 1;
	}

	return s_lastResolved;
}
SCIPCClient Driver;
protocol::DriverPoseShmem shmem;


void InitCalibrator()
{
	Driver.Connect();
	shmem.Open(OPENVR_PAIRDRIVER_SHMEM_NAME);
	// Finger smoothing config is now owned by the Smoothing overlay
	// (Protocol v12, 2026-05-11). Its plugin pushes the persisted config
	// on its own driver connect; SC no longer participates in that path.
}

// Called by SCIPCClient::SendBlocking after a successful reconnect. vrserver crashing
// and respawning destroys the named file mapping that backs the shmem segment; the
// overlay's mapped view silently detaches and ReadNewPoses begins reading zeros.
// Tearing down and reopening the segment restores the link to the new driver process.
// On Open() failure we leave shmem in a closed state -- the next reconnect will retry,
// and ReadNewPoses already guards against pData == nullptr by throwing.
void ReopenShmem()
{
	try {
		shmem.Close();
		shmem.Open(OPENVR_PAIRDRIVER_SHMEM_NAME);
	}
	catch (const std::exception& e) {
		// Close already happened; leave the segment closed and let the next reconnect retry.
		fprintf(stderr, "[ReopenShmem] failed to reopen pose shmem after reconnect: %s\n", e.what());
	}
}

void StartCalibration(const char* reason)
{
	CalCtx.wantedUpdateInterval = 0.0;
	CalCtx.messages.clear();
	calibration.Clear();
	// Reset paired-motion tracking so the first sample of the new run seeds
	// from current positions instead of comparing against stale data left
	// over from the previous calibration.
	CalCtx.pairedMotionPosSeeded = false;
	CalCtx.pairedMotionMismatchCount = 0;
	// Sample-collection boundary state. Reset here so the first post-restart
	// frame cannot compute a delta against pre-restart pose / time -- that
	// path produces inf speeds and meter-scale translation deltas which then
	// drive the translation solver to a NaN result.
	CalCtx.pairedMotionPrevRefPos = Eigen::Vector3d::Zero();
	CalCtx.pairedMotionPrevTgtPos = Eigen::Vector3d::Zero();
	// Error / offset time-series. The TimeSeries deques would otherwise carry
	// last-cycle samples into the new cycle's rolling window -- a 30s window
	// at 3.5 Hz holds ~100 samples, so a fresh-start cal that ran for only a
	// few seconds before another spike would have the geometry-shift detector
	// compare its first few new samples against a median computed mostly from
	// pre-restart history. The errTail wrap-around shape ("first 3 entries
	// are identical to the previous fire's last 3 entries") observed in
	// session logs is exactly that artifact.
	Metrics::error_currentCal.Clear();
	Metrics::error_byRelPose.Clear();
	Metrics::error_rawComputed.Clear();
	Metrics::posOffset_currentCal.Clear();
	Metrics::posOffset_byRelPose.Clear();
	Metrics::posOffset_rawComputed.Clear();
	// Reset per-extra AUTO Lock detector state so the post-restart history
	// can't be contaminated by samples from the previous cal cycle. Same
	// boundary-state hazard class as the primary's pairedMotionPrev* reset
	// above. Without this, the first post-restart MAD computation can
	// include up to 30 pre-restart samples and produce a stale verdict.
	for (auto& extra : CalCtx.additionalCalibrations) {
		extra.autoLockHistory.clear();
		extra.autoLockEffectivelyLocked = false;
		extra.autoLockHasPendingFlip = false;
		extra.autoLockPendingFlipTo = false;
		extra.autoLockPendingFlipFirstSeen = 0.0;
		extra.autoLockGateHeldWarned = false;
	}

	// Arm the geometry-shift grace window. While `time < this deadline` the
	// shift detector is skipped entirely so the first dozen samples of the
	// fresh cal cycle (during which the solver is settling and error
	// naturally fluctuates more) can't trip a back-to-back fire. 3 s at the
	// observed ~3.5 Hz sample cadence covers ~10 samples -- enough for the
	// rolling median to populate against post-restart data. Uses
	// glfwGetTime() to match CalibrationTick's `time` parameter epoch.
	constexpr double kGeometryShiftGraceSeconds = 3.0;
	CalCtx.geometryShiftGraceUntil = glfwGetTime() + kGeometryShiftGraceSeconds;
	char resetBuf[240];
	snprintf(resetBuf, sizeof resetBuf,
		"StartCalibration_state_reset: reason=%s pairedMotionPrevRefPos pairedMotionPrevTgtPos errSeries_cleared=1"
		" geometry_shift_grace_until=%.3f",
		(reason && reason[0]) ? reason : "unknown",
		CalCtx.geometryShiftGraceUntil);
	Metrics::WriteLogAnnotation(resetBuf);
}

void StartContinuousCalibration(const char* reason) {
	CalCtx.hasAppliedCalibrationResult = false;
	AssignTargets();
	StartCalibration(reason);
	CalCtx.state = CalibrationState::Continuous;
	calibration.setRelativeTransformation(CalCtx.refToTargetPose, CalCtx.relativePosCalibrated);
	calibration.lockRelativePosition = CalCtx.lockRelativePosition;
	if (CalCtx.lockRelativePosition) {
		CalCtx.Log("Relative position locked");
	}
	else {
		CalCtx.Log("Collecting initial samples...");
	}
	// Liveness detector state is cleared whenever continuous-cal restarts so
	// the post-restart freeze window starts from this moment, not from
	// whatever stale stamp survived the previous run. Edge-tracking flags
	// reset too -- the next tick is a fresh online observation.
	spacecal::liveness::Reset(g_refLiveness);
	spacecal::liveness::Reset(g_tgtLiveness);
	g_refWasOffline = false;
	g_tgtWasOffline = false;
	char startBuf[200];
	snprintf(startBuf, sizeof startBuf,
		"StartContinuousCalibration: reason=%s",
		(reason && reason[0]) ? reason : "unknown");
	Metrics::WriteLogAnnotation(startBuf);
}

void EndContinuousCalibration() {
	CalCtx.state = CalibrationState::None;
	CalCtx.relativePosCalibrated = false;
	SaveProfile(CalCtx);
	Metrics::WriteLogAnnotation("EndContinuousCalibration");
}

void CalibrationTick(double time)
{
	if (!vr::VRSystem())
		return;

	auto &ctx = CalCtx;
	if ((time - ctx.timeLastTick) < 0.05)
		return;

	// Resolve LockMode -> lockRelativePosition every tick before any code
	// downstream reads the bool. The detector itself is updated in
	// CollectSample further down; this just transcribes mode + detector
	// state into the resolved field.
	ctx.ResolveLockMode();

	// Propagate the resolved lock bool to the CalibrationCalc instance the
	// solver actually reads. Without this, calibration.lockRelativePosition
	// only updates at StartContinuousCalibration time (once per cal cycle),
	// so an AUTO Lock engagement that happens mid-cycle never reaches the
	// ComputeIncremental relPose-constraint branch until the next restart.
	// The geometry-shift fire annotation also reads this bool, which is why
	// post-fix log readers will see lockRelativePosition=1 in fires after
	// AUTO Lock engages (previously stuck at 0).
	calibration.lockRelativePosition = ctx.lockRelativePosition;
	calibration.warmRestartGraceActive = (ctx.warmRestartGraceSamples > 0);

	// Warm-restart detection. The user takes the HMD off (activity level
	// falls to Standby), comes back later, puts it on (activity level
	// snaps back to UserInteraction). If the away duration cleared the
	// threshold and the saved profile is valid, snap the driver to the
	// saved transform and grant a grace window in which CalibrationCalc
	// skips the prior-vs-new error rejection gate. Convergence then picks
	// the saved offset back up within ~30 s instead of fighting it back
	// from an empty sample buffer over the next 4-7 min.
	//
	// GetTrackedDeviceActivityLevel is preferred over Prop_UserPresent_Bool
	// here because the activity-level path is driven by both the proximity
	// sensor AND motion -- so an HMD with no working proximity sensor
	// (some Quest variants over Link) still produces a usable signal as
	// long as the IMU sees the HMD sitting still long enough for the
	// runtime to transition to Standby (>= 5 s of stillness, configurable
	// in SteamVR Power Management).
	//
	// k_EDeviceActivityLevel_Unknown returns for devices that aren't
	// reporting yet (e.g. a fresh HMD that hasn't woken up); we treat
	// Unknown as "not present" so no spurious edges fire during startup.
	{
		const vr::EDeviceActivityLevel activity =
			vr::VRSystem()->GetTrackedDeviceActivityLevel(vr::k_unTrackedDeviceIndex_Hmd);
		const bool nowPresent =
			(activity == vr::k_EDeviceActivityLevel_UserInteraction)
			|| (activity == vr::k_EDeviceActivityLevel_UserInteraction_Timeout);
		// activity != Unknown is the "we have a signal at all" gate;
		// without this, a freshly-launched session before the HMD has
		// reported anything would look like a "false" reading and
		// immediately start the away timer at session start.
		if (activity != vr::k_EDeviceActivityLevel_Unknown) {
			// Session-level tick counter for the cold-start safety gate
			// in ShouldEngage. Incremented every poll-cycle (the outer
			// {} guards on Continuous/ContinuousStandby state, which is
			// where warm-restart can meaningfully fire). See
			// WarmRestart.h::kColdStartGraceTicks for the threshold.
			++ctx.warmRestartTickId;

			// Current HMD position from the latest device pose, used for
			// the pose-jump fallback signal. DriverPose_t carries position
			// in vecPosition[3]; same field other code paths in this file
			// read for HMD position (e.g. line 1583).
			const auto& hmdPose = ctx.devicePoses[vr::k_unTrackedDeviceIndex_Hmd];
			const Eigen::Vector3d hmdPosNow(
				hmdPose.vecPosition[0],
				hmdPose.vecPosition[1],
				hmdPose.vecPosition[2]);
			const bool hmdPoseValid = hmdPose.poseIsValid
				&& hmdPose.result == vr::ETrackingResult::TrackingResult_Running_OK;

			const bool wasPresent = ctx.lastUserPresent;
			if (wasPresent && !nowPresent) {
				ctx.userAwaySince = time;
				// Capture the HMD's position at the moment of falling
				// edge so the rising edge can compute displacement and
				// fast-path the engage decision on large jumps (HMD
				// physically moved while away, regardless of how briefly).
				if (hmdPoseValid) {
					ctx.hmdLastKnownPosWhenAway = hmdPosNow;
					ctx.hmdLastKnownPosValid = true;
				}
			} else if (!wasPresent && nowPresent) {
				const double awayFor = (ctx.userAwaySince > 0.0)
					? (time - ctx.userAwaySince) : 0.0;
				const double awayPosDelta =
					(ctx.hmdLastKnownPosValid && hmdPoseValid)
						? (hmdPosNow - ctx.hmdLastKnownPosWhenAway).norm()
						: 0.0;
				const spacecal::warm_restart::EngageInput engageIn = {
					wasPresent,
					nowPresent,
					awayFor,
					ctx.validProfile,
					ctx.state == CalibrationState::Continuous
						|| ctx.state == CalibrationState::ContinuousStandby,
					awayPosDelta,
					ctx.warmRestartTickId,
				};
				const bool engaged = spacecal::warm_restart::ShouldEngage(engageIn);

				// Diagnostic: max-away ceiling. When the proximity path
				// would have engaged but awayFor crossed the ceiling, log
				// it explicitly so a session that "went to sleep then woke
				// to cold cal" leaves a paper trail rather than being
				// invisible. Only logs when this is the suppress reason
				// (not when pose-jump fast-path took over).
				if (!engaged
					&& ctx.validProfile
					&& (ctx.state == CalibrationState::Continuous
						|| ctx.state == CalibrationState::ContinuousStandby)
					&& ctx.warmRestartTickId >= spacecal::warm_restart::kColdStartGraceTicks
					&& awayFor > spacecal::warm_restart::kMaxAwaySeconds
					&& awayPosDelta < spacecal::warm_restart::kPositionJumpFastPathM) {
					char cbuf[200];
					snprintf(cbuf, sizeof cbuf,
						"[warm-restart][ceiling-suppressed] away_for_s=%.1f"
						" max_away_s=%.0f pos_delta_m=%.3f",
						awayFor, spacecal::warm_restart::kMaxAwaySeconds,
						awayPosDelta);
					Metrics::WriteLogAnnotation(cbuf);
				}

				if (engaged) {
					ctx.warmRestartGraceSamples =
						spacecal::warm_restart::kGraceSamples;
					ctx.warmRestartMadAtSnap = ctx.autoLockMadFloor;
					ctx.warmRestartValidationState =
						spacecal::warm_restart::ValidationOutcome::Inconclusive;
					// Reset the post-snap bias accumulator and pin the
					// last-consumed err timestamp to the latest pre-snap
					// sample, so pre-snap retargeting errors do not feed
					// into the post-snap mean. warmRestartSnapTime is
					// surfaced in the heartbeat to label mad_floor_source.
					ctx.postSnapErrorSumMm = 0.0;
					ctx.postSnapErrorSampleCount = 0;
					ctx.warmRestartLastConsumedErrTs =
						Metrics::error_currentCal.lastTs();
					ctx.warmRestartSnapTime = Metrics::CurrentTime;
					g_snapNextProfileApply = true;
					const double mag = std::sqrt(
						  ctx.calibratedTranslation.x() * ctx.calibratedTranslation.x()
						+ ctx.calibratedTranslation.y() * ctx.calibratedTranslation.y()
						+ ctx.calibratedTranslation.z() * ctx.calibratedTranslation.z());
					const bool fastPath =
						awayPosDelta >= spacecal::warm_restart::kPositionJumpFastPathM
						&& awayFor < spacecal::warm_restart::kMinAwaySeconds;
					char wbuf[260];
					snprintf(wbuf, sizeof wbuf,
						"[warm-restart][snap] away_for_s=%.1f state=%d"
						" grace_samples=%d profile_magnitude_cm=%.2f"
						" pos_delta_m=%.3f mad_at_snap_mm=%.3f path=%s",
						awayFor, (int)ctx.state,
						ctx.warmRestartGraceSamples, mag,
						awayPosDelta, ctx.warmRestartMadAtSnap * 1000.0,
						fastPath ? "pose_jump_fast_path" : "proximity_and_time");
					Metrics::WriteLogAnnotation(wbuf);
				}
				ctx.userAwaySince = 0.0;
				ctx.hmdLastKnownPosValid = false;
			}
			ctx.lastUserPresent = nowPresent;
		}
	}

	// Diagnostic: trace relPose-cal validity flips. The flag is set/cleared
	// from several call sites and is currently only externally visible inside
	// the rate-limited usingRelPose_fired event. Catching every change is
	// cheap (one bool compare per tick) and reveals the cycle: cal converges
	// -> relPosCal=1 -> geometry-shift fire historically cleared it -> 0.
	// After the T1.5 fix this trace tells us whether the constraint actually
	// survives geometry-shift events.
	{
		static bool s_lastRelPosCal = false;
		const bool nowRelPosCal = ctx.relativePosCalibrated;
		if (nowRelPosCal != s_lastRelPosCal) {
			char rpcBuf[160];
			snprintf(rpcBuf, sizeof rpcBuf,
				"[relposcal-change] prev=%d now=%d state=%d lockMode=%d",
				(int)s_lastRelPosCal, (int)nowRelPosCal,
				(int)ctx.state, (int)ctx.lockRelativePositionMode);
			Metrics::WriteLogAnnotation(rpcBuf);
			s_lastRelPosCal = nowRelPosCal;
		}
	}

	// Tracker pose-freshness check. The driver writes a QPC timestamp into
	// devicePoseSampleTimes[] each time a pose is published. If the ref or
	// target sample timestamp hasn't advanced in the last 5 s, that device
	// has gone silent (the pose value may still appear valid because the
	// last-known position is still in the array, but no new data has
	// arrived). Log throttled to once per 30 s per device so a chronic
	// silence doesn't flood. ID < 0 (unassigned) is skipped.
	if (ctx.state == CalibrationState::Continuous
		|| ctx.state == CalibrationState::ContinuousStandby) {
		static double s_lastFreshnessLogTime = -1e9;
		const double freshnessWarnSec = 5.0;
		LARGE_INTEGER freq;
		QueryPerformanceFrequency(&freq);
		LARGE_INTEGER nowCounter;
		QueryPerformanceCounter(&nowCounter);

		auto checkFresh = [&](int id, const char* whichLabel) {
			if (id < 0 || id >= (int)vr::k_unMaxTrackedDeviceCount) return;
			const auto& sampleTime = ctx.devicePoseSampleTimes[id];
			if (sampleTime.QuadPart == 0) return;  // never sampled
			const double ageSec =
				double(nowCounter.QuadPart - sampleTime.QuadPart) / double(freq.QuadPart);
			if (ageSec >= freshnessWarnSec
				&& (time - s_lastFreshnessLogTime) >= 30.0) {
				s_lastFreshnessLogTime = time;
				char freshBuf[200];
				snprintf(freshBuf, sizeof freshBuf,
					"[tracker-pose-stale] which=%s id=%d age_sec=%.2f"
					" result=%d poseIsValid=%d",
					whichLabel, id, ageSec,
					(int)ctx.devicePoses[id].result,
					(int)ctx.devicePoses[id].poseIsValid);
				Metrics::WriteLogAnnotation(freshBuf);
			}
		};
		checkFresh(ctx.referenceID, "reference");
		checkFresh(ctx.targetID, "target");
	}

	// Stuck-cal watchdog. If we've been in Continuous state for >60 s but
	// error_currentCal has not received a new sample in the last 30 s, the
	// cal solver is not actually running -- ComputeIncremental isn't being
	// called, or is rejecting every input, or the time series has otherwise
	// stopped advancing. Edge-triggered, one log per detection, re-armed
	// when error_currentCal advances again.
	if (ctx.state == CalibrationState::Continuous) {
		static double s_lastCalActiveTs = 0.0;
		static double s_lastStuckLogTime = -1e9;
		const double errLastTs = Metrics::error_currentCal.lastTs();
		if (errLastTs > s_lastCalActiveTs) {
			s_lastCalActiveTs = errLastTs;
		}
		const bool stuck = (Metrics::error_currentCal.size() > 0
			&& time - Metrics::CurrentTime > -1e-6  // safety: clocks aligned
			&& Metrics::CurrentTime - s_lastCalActiveTs >= 30.0);
		if (stuck && (time - s_lastStuckLogTime) >= 30.0) {
			s_lastStuckLogTime = time;
			char stuckBuf[280];
			snprintf(stuckBuf, sizeof stuckBuf,
				"[cal-stuck] no_compute_for_sec=%.2f state=%d lockRel=%d"
				" err_samples=%d refID=%d targetID=%d",
				Metrics::CurrentTime - s_lastCalActiveTs,
				(int)ctx.state, (int)ctx.lockRelativePosition,
				Metrics::error_currentCal.size(),
				ctx.referenceID, ctx.targetID);
			Metrics::WriteLogAnnotation(stuckBuf);
		}
	}

	// Periodic cal heartbeat. Throttled to once per 10 s while in Continuous
	// or ContinuousStandby. Emits a one-line "you are here" snapshot so a
	// post-session reader can scrub the log without grepping multiple event
	// types just to learn the cal's current state. Fields chosen to maximize
	// signal-per-character: state, lock resolution (mode + resolved + detector
	// internal), recent error level, sample-buffer size, time since last
	// reset / reanchor.
	if (ctx.state == CalibrationState::Continuous
		|| ctx.state == CalibrationState::ContinuousStandby)
	{
		static double s_lastHeartbeatTime = -1e9;
		if ((time - s_lastHeartbeatTime) >= 10.0) {
			s_lastHeartbeatTime = time;
			const auto& errSeries = Metrics::error_currentCal;
			const double errLast = errSeries.size() > 0 ? errSeries.last() : 0.0;
			const double secSinceReanchor = SecondsSinceLastReanchorChiLog(time);
			// Geometry-shift cooldown remaining (0 when no active cooldown).
			// The deadline ctx.geometryShiftCooldownUntil is wall-clock-style
			// time matching CalibrationTick's `time` argument; the heartbeat
			// log shows seconds until expiry so a reader doesn't have to
			// subtract the session time themselves.
			const double cooldownRemaining =
				(ctx.geometryShiftCooldownUntil > time)
					? (ctx.geometryShiftCooldownUntil - time)
					: 0.0;
			const double translMadMm = g_lastAutoLockTranslMad * 1000.0;
			const double rotMadDeg = g_lastAutoLockRotMad * 180.0 / EIGEN_PI;
			// Pending-flip held duration. Zero when no flip is queued so a
			// reader can distinguish a stable autoLockEff from one that is
			// about to commit a transition. autoLockPendingFlipFirstSeen
			// is set the first tick a pending flip appears and reset on
			// commit / abandon, so a non-zero value during pending means
			// the held-duration is meaningful.
			const double autoLockHeldSec =
				(ctx.autoLockHasPendingFlip && ctx.autoLockPendingFlipFirstSeen > 0.0)
					? (time - ctx.autoLockPendingFlipFirstSeen)
					: 0.0;
			// Settled signal: see AutoLockHysteresis.h::IsSettled. The settled
			// rate over a session is the headline success metric for the
			// 2026-05-25 settling fix -- a healthy run should sit at
			// settled=yes for the majority of heartbeats once initial motion
			// has finished. settledSinceSec is the elapsed time since the
			// last AUTO Lock flip when settled, zero otherwise; lets a
			// reader scrub the timeline of stable lock windows.
			const double secsSinceLastFlip = (ctx.autoLockLastFlipTime > 0.0)
				? (time - ctx.autoLockLastFlipTime)
				: 0.0;
			const bool settled = spacecal::autolock::IsSettled(
				ctx.autoLockEffectivelyLocked,
				g_lastAutoLockTranslMad,
				ctx.autoLockMadFloor,
				secsSinceLastFlip);
			const double madFloorMm = ctx.autoLockMadFloor * 1000.0;
			const double enterMm =
				spacecal::autolock::EnterThresholdFor(ctx.autoLockMadFloor) * 1000.0;
			// Warm-restart heartbeat fields: the post-snap bias mean and
			// the mad-floor source distinguish "Settled by post-snap
			// convergence" from "Settled by inherited pre-snap quiet
			// floor". post_snap_bias_mm is the validator's correctness
			// signal; mad_floor_source is its provenance label. Both are
			// emitted regardless of whether the grace window is active --
			// outside the window they read as zero / "n/a" so a triage
			// reader can grep one line per heartbeat for the relevant
			// state.
			const bool warmRestartActive = (ctx.warmRestartGraceSamples > 0);
			const double postSnapBiasMm =
				(ctx.postSnapErrorSampleCount > 0)
				? (ctx.postSnapErrorSumMm
					/ static_cast<double>(ctx.postSnapErrorSampleCount))
				: 0.0;
			const char* madFloorSourceHb;
			if (!warmRestartActive) {
				madFloorSourceHb = "n/a";
			} else if (ctx.warmRestartSnapTime > 0.0
				&& ctx.autoLockMadFloorTs > 0.0
				&& ctx.autoLockMadFloorTs >= ctx.warmRestartSnapTime) {
				madFloorSourceHb = "postSnap";
			} else {
				madFloorSourceHb = "preSnap";
			}
			const char* validationStateHb;
			switch (ctx.warmRestartValidationState) {
				case spacecal::warm_restart::ValidationOutcome::Settled:
					validationStateHb = "settled"; break;
				case spacecal::warm_restart::ValidationOutcome::Failed:
					validationStateHb = "failed"; break;
				default:
					validationStateHb = "inconclusive"; break;
			}
			char hbBuf[1024];
			snprintf(hbBuf, sizeof hbBuf,
				"[cal-heartbeat] state=%d lockMode=%d lockRel=%d autoLockEff=%d"
				" autoLockPending=%d autoLockPendingTo=%d autoLockHeldSec=%.2f"
				" autoLockHistory=%zu/%zu translMad_mm=%.3f rotMad_deg=%.3f"
				" mad_floor_mm=%.3f enter_threshold_mm=%.3f"
				" settled=%s settled_since_sec=%.1f"
				" err_last_mm=%.2f err_samples=%d"
				" sec_since_reanchor=%.2f autolock_suppress_until=%.3f"
				" reloc_cooldown_until=%.3f grace_until=%.3f"
				" geom_cusumS=%.3f geom_sustain=%d/%d"
				" geom_cooldown_remaining_sec=%.1f"
				" relPosCal=%d hmdStalls=%d"
				" wr_active=%d wr_grace_remaining=%d"
				" post_snap_bias_mm=%.3f post_snap_samples=%d"
				" mad_floor_source=%s wr_validation=%s",
				(int)ctx.state, (int)ctx.lockRelativePositionMode,
				(int)ctx.lockRelativePosition, (int)ctx.autoLockEffectivelyLocked,
				(int)ctx.autoLockHasPendingFlip, (int)ctx.autoLockPendingFlipTo,
				autoLockHeldSec,
				ctx.autoLockHistory.size(), spacecal::autolock::kSamplesNeeded,
				translMadMm, rotMadDeg,
				madFloorMm, enterMm,
				settled ? "yes" : "no",
				settled ? secsSinceLastFlip : 0.0,
				errLast, errSeries.size(),
				secSinceReanchor, ctx.autoLockReanchorSuppressUntil,
				ctx.relocalizationCooldownUntil, ctx.geometryShiftGraceUntil,
				g_cusumState.S, g_cusumState.sustainedAboveThreshold,
				spacecal::geometry_shift::kMinSustainedSpikes,
				cooldownRemaining,
				(int)ctx.relativePosCalibrated, ctx.consecutiveHmdStalls,
				(int)warmRestartActive, ctx.warmRestartGraceSamples,
				postSnapBiasMm, ctx.postSnapErrorSampleCount,
				madFloorSourceHb, validationStateHb);
			Metrics::WriteLogAnnotation(hbBuf);
		}
	}

	// One-shot session-start config dump. Fires on the first non-skipped
	// CalibrationTick after the profile has been loaded, so the annotation
	// reflects the user's actual saved settings. Captures every experimental
	// toggle + the load-bearing tunables. Lets a session reader skip the
	// "what version of the math is running" reverse-derivation from code.
	{
		static bool s_loggedConfigDump = false;
		if (!s_loggedConfigDump) {
			s_loggedConfigDump = true;
			char dumpBuf[512];
			snprintf(dumpBuf, sizeof dumpBuf,
				"session_config_dump: upstream=%d gcc_phat=%d cusum=%d velocity_aware=%d tukey=%d kalman=%d"
				" auto_detect_latency=%d ignore_outliers=%d static_recal=%d"
				" recalibrate_on_movement=%d cal_speed=%.2f jitter_threshold=%.2f",
				(int)ctx.useUpstreamMath,
				(int)ctx.useGccPhatLatency, (int)ctx.useCusumGeometryShift,
				(int)ctx.useVelocityAwareWeighting, (int)ctx.useTukeyBiweight,
				(int)ctx.useBlendFilter,
				(int)ctx.latencyAutoDetect, (int)ctx.ignoreOutliers,
				(int)ctx.enableStaticRecalibration, (int)ctx.recalibrateOnMovement,
				(double)ctx.calibrationSpeed, (double)ctx.jitterThreshold);
			Metrics::WriteLogAnnotation(dumpBuf);
		}
	}

	// Bounds-check the device IDs once at the top of the tick. Many code paths
	// downstream index devicePoses[ctx.referenceID] / devicePoses[ctx.targetID]
	// directly (CollectSample, the sample-history pose recording near the end of
	// this function, etc.), and a stale negative or out-of-range value reaches
	// for memory outside the array. We tolerate -1 (the not-yet-assigned sentinel)
	// because state machines below explicitly handle that, but anything else that
	// isn't in [0, k_unMaxTrackedDeviceCount) means we cannot run any per-device
	// logic this tick -- bail out and try again next tick.
	const int32_t maxId = (int32_t)vr::k_unMaxTrackedDeviceCount;
	auto idInRangeOrUnset = [maxId](int32_t id) {
		return id == -1 || (id >= 0 && id < maxId);
	};
	if (!idInRangeOrUnset(ctx.referenceID) || !idInRangeOrUnset(ctx.targetID)) {
		// Defensive reset: a corrupted ID is unrecoverable for this tick. Don't
		// touch state -- we just skip the tick so the next AssignTargets() call can
		// reseat the IDs cleanly.
		return;
	}

	// Hybrid HMD-relocalization detector. Runs in every state where a tick
	// proceeds; the function itself skips active calibration sub-states
	// where the HMD is being deliberately moved. Logging-only -- emits a
	// `# [time] hmd_relocalization_detected: ...` annotation when the
	// triple-AND trigger fires, but doesn't modify R or the chaperone yet.
	// We run this BEFORE TickBaseStationDrift so the base-station cache it
	// shares is populated with the previous tick's poses (the relocalization
	// detector compares against THIS tick's poses; the universe-shift
	// detector cares about pose changes between consecutive ticks).
	TickHmdRelocalizationDetector(time);

	// Chi-square re-anchor sub-detector (rec F). When fired, returns true so
	// recs A and C skip their tick for the freeze window. Detection-only:
	// the 30 cm detector above is still the only path to actual recovery.
	const bool restAndRecCFrozen = TickReanchorChiSquare(time);

	// Rest-locked yaw drift correction (rec A). Opt-in via Experimental tab,
	// default OFF. Skips Continuous mode (continuous-cal already handles drift)
	// and active one-shot sub-states. When at-rest signal is available, applies
	// a bounded-rate yaw nudge to ctx.calibratedRotation(1); the existing
	// publish path picks up the change via SendFallbackIfChanged.
	if (!restAndRecCFrozen) {
		TickRestLockedYaw(time);
	}

	// Predictive recovery pre-correction (rec C). Opt-in via Experimental tab,
	// default OFF. Reads the rolling buffer of 30 cm relocalization events and
	// applies a bounded-rate translation nudge if the gate passes.
	if (!restAndRecCFrozen) {
		TickPredictiveRecovery(time);
	}

	if (ctx.state == CalibrationState::Continuous || ctx.state == CalibrationState::ContinuousStandby) {
		ctx.ClearLogOnMessage();

		if (CalCtx.requireTriggerPressToApply && (time - ctx.timeLastAssign) > 10) {
			// rescan devices every 10 seconds or so if we are using controller data
			ctx.timeLastAssign = time;
			AssignTargets();
		}
	}

	// Sudden-tracking-shift watchdog. The 50-rejection watchdog inside CalibrationCalc
	// only fires after ~25s of consistent rejection; that's appropriate for genuinely
	// degraded calibration but too slow for catastrophic geometry shifts (a lighthouse
	// gets bumped, a tracker goes through a portal, etc.). Here we look at the recent
	// error_currentCal time series: when the last error sample is more than 5x the
	// 30-tick rolling median for 3 consecutive ticks, the calibration is almost
	// certainly invalid even if CalibrationCalc still considers it valid. We force a
	// Clear() and demote to ContinuousStandby so the next AssignTargets cycle starts a
	// fresh calibration. Done from the overlay side (not CalibrationCalc) so we don't
	// touch shared math code.
	{
		// Detector accumulator state lives at file scope (g_geomShiftConsecutiveBadTicks,
		// g_cusumState) so the [cal-heartbeat] emitter further up in this tick
		// can include them in its periodic dump. Throttle timestamps stay
		// block-local -- they're only consulted from inside this block.
		static double s_lastErrorTs = 0.0;
		static double s_lastSpikeLogTime = -1e9;  // throttle per-tick spike-candidate logging to ~1/s
		static double s_lastGraceLogTime = -1e9;  // throttle the grace-active log to ~1/s

		// Grace window: while a recent cal restart is still settling, skip
		// the detector entirely. The solver's first ~10 samples of a fresh
		// cycle naturally fluctuate as it converges from zero buffer; treating
		// that fluctuation as a geometry shift triggers the back-to-back-
		// fire pattern that contaminates the error history for several cycles
		// thereafter. The deadline is set at StartCalibration and naturally
		// expires.
		const bool inGeometryShiftGrace =
			(ctx.geometryShiftGraceUntil > 0.0 && time < ctx.geometryShiftGraceUntil);
		if (inGeometryShiftGrace) {
			g_geomShiftConsecutiveBadTicks = 0;
			g_cusumState.Reset();
			g_geomShiftDeferral.Reset();
			if ((time - s_lastGraceLogTime) >= 1.0) {
				s_lastGraceLogTime = time;
				char gBuf[200];
				snprintf(gBuf, sizeof gBuf,
					"[geometry-shift][grace-active] grace_until=%.3f now=%.3f remaining=%.3fs",
					ctx.geometryShiftGraceUntil, time,
					ctx.geometryShiftGraceUntil - time);
				Metrics::WriteLogAnnotation(gBuf);
			}
		} else {
		const auto& errSeries = Metrics::error_currentCal;
		const int N = errSeries.size();
		if (N >= 5 && calibration.isValid()) {
			// Only count an excursion once per new error sample (the time series is
			// shared and may not advance every tick).
			double thisTs = errSeries[N - 1].first;
			if (thisTs > s_lastErrorTs) {
				s_lastErrorTs = thisTs;
				const int window = std::min(30, N);
				std::vector<double> tail;
				tail.reserve(window);
				for (int i = N - window; i < N; i++) tail.push_back(errSeries[i].second);
				std::sort(tail.begin(), tail.end());
				double median = tail[tail.size() / 2];
				double current = errSeries[N - 1].second;
				const double ratio = (median > spacecal::geometry_shift::kMedianFloor)
					? current / median : 0.0;

				bool fire = false;
				bool isSpike = false;
				double cusumValueAtFire = 0.0;  // captured before the in-function reset
				int cusumSustainAtFire = 0;     // ditto -- post-reset reads as 0
				if (ctx.useCusumGeometryShift) {
					// Page CUSUM: accumulates (current - baseline - drift) per
					// tick, fires when the running sum crosses threshold. Median
					// stays as the baseline so the test is centered on the recent
					// no-shift behavior. Fire path resets the CUSUM state to zero
					// inside UpdateCusumGeometryShift -- we capture the pre-reset
					// values via the out-params so the fire log can show what S
					// climbed to AND that the sustain gate was satisfied (>=3),
					// rather than the post-reset 0.
					fire = spacecal::geometry_shift::UpdateCusumGeometryShift(
						g_cusumState, current, median,
						spacecal::geometry_shift::kCusumDriftMm,
						spacecal::geometry_shift::kCusumThreshold,
						&cusumValueAtFire,
						&cusumSustainAtFire);
					// Mirror the consecutive-bad-tick counter to zero in this path
					// so a toggle flip mid-session doesn't leave stale state.
					g_geomShiftConsecutiveBadTicks = 0;
				} else {
					isSpike = spacecal::geometry_shift::IsCurrentErrorSpike(current, median);
					if (isSpike) {
						g_geomShiftConsecutiveBadTicks++;
					} else {
						g_geomShiftConsecutiveBadTicks = 0;
					}
					fire = spacecal::geometry_shift::ShouldFireGeometryShiftRecovery(g_geomShiftConsecutiveBadTicks);
					// Reset CUSUM accumulator when the legacy path is active so a
					// later toggle flip starts from a clean state rather than a
					// stale running-sum from before the toggle change.
					g_cusumState.Reset();
				}

				// Common-mode coherence check. The primary detector above
				// votes from the primary HMD<->target pair's residual
				// stream alone. If a vote-to-fire actually reflects a
				// shared-frame event (worldFromDriver reanchor coming
				// from outside the chi-square detector, runtime
				// relocalization, base-station perturbation), then the
				// active multi-system extras should be showing the same
				// spike with the same shape. Score that explicitly and
				// suppress the fire when the spike is coherent across
				// pairs -- pair-local geometry shifts only spike one
				// pair's residual, not all of them.
				if (fire) {
					std::vector<double> extraRatios;
					extraRatios.reserve(ctx.additionalCalibrations.size());
					for (const auto& extra : ctx.additionalCalibrations) {
						if (!extra.enabled || !extra.valid) continue;
						if (extra.recentErrorsMm.size() < 5) continue;
						std::vector<double> tail(
							extra.recentErrorsMm.begin(),
							extra.recentErrorsMm.end());
						std::sort(tail.begin(), tail.end());
						const double xMedian = tail[tail.size() / 2];
						const double xCurrent = extra.recentErrorsMm.back();
						if (xMedian <= spacecal::geometry_shift::kMedianFloor) continue;
						extraRatios.push_back(xCurrent / xMedian);
					}
					const int extrasCount = static_cast<int>(extraRatios.size());
					const double coherence =
						spacecal::coherence::ComputeCoherenceScore(ratio, extraRatios);
					if (spacecal::coherence::ShouldSuppressFire(coherence, extrasCount)) {
						char csBuf[320];
						snprintf(csBuf, sizeof csBuf,
							"[geometry-shift][coherence-suppressed] coherence=%.3f"
							" primary_ratio=%.2fx extras_count=%d threshold=%.2f"
							" current_mm=%.3f median_mm=%.3f mode=%s",
							coherence, ratio, extrasCount,
							spacecal::coherence::kSuppressThreshold,
							current, median,
							ctx.useCusumGeometryShift ? "cusum" : "legacy");
						Metrics::WriteLogAnnotation(csBuf);
						// Wipe the accumulators that drove the fire so a
						// subsequent pair-local spike can re-accumulate
						// cleanly without inheriting this suppressed
						// evidence.
						fire = false;
						g_cusumState.Reset();
						g_geomShiftConsecutiveBadTicks = 0;
					} else if (extrasCount >= spacecal::coherence::kMinExtrasForCoherence) {
						// Multi-extra session with a fire that the coherence
						// check let through: log the score so the next session
						// log shows the discriminator did its work rather
						// than being dead code. Throttled with the existing
						// spike-candidate-log throttle so it cannot run away.
						if ((time - s_lastSpikeLogTime) >= 1.0) {
							s_lastSpikeLogTime = time;
							char clBuf[280];
							snprintf(clBuf, sizeof clBuf,
								"[geometry-shift][coherence-check] coherence=%.3f"
								" primary_ratio=%.2fx extras_count=%d threshold=%.2f"
								" verdict=pair_local",
								coherence, ratio, extrasCount,
								spacecal::coherence::kSuppressThreshold);
							Metrics::WriteLogAnnotation(clBuf);
						}
					}
				}

				// Reanchor-gate with deferral. When the chi-square reanchor is
				// frozen the underlying worldFromDriver was just reset and the
				// cal solver naturally produces larger residuals as it re-
				// anchors against the new frame. The previous suppress-and-
				// reset path dropped those fires entirely, which on rigs with
				// continuous reanchor activity (Quest+Lighthouse, reanchor
				// every 0.8-1.5 s with 0.5 s freeze each) starved real shifts
				// of any remediation -- the 2026-05-25 session log showed 1
				// committed fire across 5.3 h despite continuous candidate
				// activity. Deferral preserves the fire intent: commit on the
				// next non-frozen tick, or fire through after
				// kFireThroughSeconds of continuous freeze (storm itself is
				// shift signature at that duration). See
				// GeometryShiftDetector.h::DeferralState for the contract.
				const bool reanchorFrozen = IsReanchorChiFrozen(time);

				// Process an existing deferral first -- it can promote into a
				// real fire even if no new fire was decided this tick.
				if (g_geomShiftDeferral.HasPending()) {
					const double deferredFor = time - g_geomShiftDeferral.pendingFireSince;
					const double secSinceReanchor = SecondsSinceLastReanchorChiLog(time);
					if (!reanchorFrozen) {
						// Reanchor cleared. Commit the deferred fire using the
						// latched diagnostics so the upcoming [fire] log
						// describes the originally-triggering values, not the
						// post-defer current sample.
						char dcBuf[320];
						snprintf(dcBuf, sizeof dcBuf,
							"[geometry-shift][deferred-fire-committed] deferred_for_sec=%.3f"
							" current_mm=%.3f median_mm=%.3f ratio=%.2fx"
							" cusum_S_at_pending=%.3f sustain_at_pending=%d mode=%s"
							" sec_since_reanchor=%.3f",
							deferredFor,
							g_geomShiftDeferral.currentMmAtPending,
							g_geomShiftDeferral.medianMmAtPending,
							g_geomShiftDeferral.ratioAtPending,
							g_geomShiftDeferral.cusumSAtPending,
							g_geomShiftDeferral.sustainAtPending,
							g_geomShiftDeferral.modeWasCusum ? "cusum" : "legacy",
							secSinceReanchor);
						Metrics::WriteLogAnnotation(dcBuf);
						current = g_geomShiftDeferral.currentMmAtPending;
						median  = g_geomShiftDeferral.medianMmAtPending;
						cusumValueAtFire   = g_geomShiftDeferral.cusumSAtPending;
						cusumSustainAtFire = g_geomShiftDeferral.sustainAtPending;
						fire = true;
						g_geomShiftDeferral.Reset();
					} else if (spacecal::geometry_shift::ShouldFireThroughDeferral(
							time, g_geomShiftDeferral)) {
						// Reanchor still frozen past kFireThroughSeconds --
						// fire anyway. Continued suppression at this point
						// would leave the bad calibration sitting indefinitely.
						char ftBuf[320];
						snprintf(ftBuf, sizeof ftBuf,
							"[geometry-shift][fired-through-reanchor] deferred_for_sec=%.3f"
							" current_mm=%.3f median_mm=%.3f ratio=%.2fx"
							" cusum_S_at_pending=%.3f sustain_at_pending=%d mode=%s"
							" sec_since_reanchor=%.3f",
							deferredFor,
							g_geomShiftDeferral.currentMmAtPending,
							g_geomShiftDeferral.medianMmAtPending,
							g_geomShiftDeferral.ratioAtPending,
							g_geomShiftDeferral.cusumSAtPending,
							g_geomShiftDeferral.sustainAtPending,
							g_geomShiftDeferral.modeWasCusum ? "cusum" : "legacy",
							secSinceReanchor);
						Metrics::WriteLogAnnotation(ftBuf);
						current = g_geomShiftDeferral.currentMmAtPending;
						median  = g_geomShiftDeferral.medianMmAtPending;
						cusumValueAtFire   = g_geomShiftDeferral.cusumSAtPending;
						cusumSustainAtFire = g_geomShiftDeferral.sustainAtPending;
						fire = true;
						g_geomShiftDeferral.Reset();
					} else if (!fire) {
						// Reanchor still frozen, no new fire AND signal has
						// subsided below the relevant accumulator's noise
						// floor. The original spike turned out to be
						// transient -- clear the deferral without firing.
						// CUSUM mode: S well below threshold (S>=1 indicates
						// the increment is still pushing past the drift).
						// Legacy mode: consecutive bad ticks dropped to 0
						// (any tick without a 5x median spike resets it).
						const bool subsided = ctx.useCusumGeometryShift
							? (g_cusumState.S < 1.0)
							: (g_geomShiftConsecutiveBadTicks == 0);
						if (subsided) {
							char clrBuf[240];
							snprintf(clrBuf, sizeof clrBuf,
								"[geometry-shift][deferral-cleared] deferred_for_sec=%.3f"
								" mode=%s reason=subsided cusum_S=%.3f legacy_sustained=%d",
								deferredFor,
								g_geomShiftDeferral.modeWasCusum ? "cusum" : "legacy",
								g_cusumState.S, g_geomShiftConsecutiveBadTicks);
							Metrics::WriteLogAnnotation(clrBuf);
							g_geomShiftDeferral.Reset();
						}
					}
					// else: deferred, reanchor frozen, elapsed < kFireThroughSeconds,
					// new fire still active or signal still hot -- keep waiting.
				}

				// New fire while reanchor is frozen and no deferral already
				// pending: latch the deferral so the fire intent survives the
				// freeze instead of being dropped. If a deferral is already
				// pending, the original timestamp wins (don't re-latch and
				// extend the fire-through window).
				if (fire && reanchorFrozen && !g_geomShiftDeferral.HasPending()) {
					char defBuf[320];
					const double secSinceReanchor = SecondsSinceLastReanchorChiLog(time);
					snprintf(defBuf, sizeof defBuf,
						"[geometry-shift][deferred-by-reanchor] current_mm=%.3f median_mm=%.3f"
						" ratio=%.2fx cusum_S_at_fire=%.3f sustain=%d mode=%s"
						" sec_since_reanchor=%.3f lockRelativePosition=%d",
						current, median, ratio,
						cusumValueAtFire, cusumSustainAtFire,
						ctx.useCusumGeometryShift ? "cusum" : "legacy",
						secSinceReanchor, (int)ctx.lockRelativePosition);
					Metrics::WriteLogAnnotation(defBuf);
					g_geomShiftDeferral.Latch(time,
						cusumValueAtFire, cusumSustainAtFire,
						current, median, ratio,
						ctx.useCusumGeometryShift);
					fire = false;
					// Reset accumulators so the next tick's spike detection
					// starts fresh; the latched values carry the original
					// fire decision. If the underlying shift is real, the
					// fire-through timeout will commit it; if transient, the
					// subsided check will clear it.
					g_geomShiftConsecutiveBadTicks = 0;
					g_cusumState.Reset();
				}

				// Diagnostic: per-tick spike candidate trace. Throttled to ~1/s
				// so a sustained spike storm produces a readable trail rather
				// than a per-tick flood. Logs early-warning data (current vs
				// median + cusum state + reanchor proximity) so the next
				// session log can show what was building toward a fire that
				// did or did not happen.
				const bool spikeWorthLogging = isSpike || g_cusumState.S > 0.5;
				if (spikeWorthLogging && (time - s_lastSpikeLogTime) >= 1.0) {
					s_lastSpikeLogTime = time;
					const bool reanchorFrozen =
						IsReanchorChiFrozen(time);
					const double secSinceReanchor = SecondsSinceLastReanchorChiLog(time);
					char spikeBuf[320];
					snprintf(spikeBuf, sizeof spikeBuf,
						"[geometry-shift][spike-candidate] current_mm=%.3f median_mm=%.3f"
						" ratio=%.2fx sustained=%d/%d cusum_S=%.3f cusum_h=%.3f mode=%s"
						" reanchor_frozen=%d sec_since_reanchor=%.3f",
						current, median, ratio,
						g_geomShiftConsecutiveBadTicks, spacecal::geometry_shift::kMinSustainedSpikes,
						g_cusumState.S, spacecal::geometry_shift::kCusumThreshold,
						ctx.useCusumGeometryShift ? "cusum" : "legacy",
						(int)reanchorFrozen, secSinceReanchor);
					Metrics::WriteLogAnnotation(spikeBuf);
				}

				// Cooldown gate: after a previous fire, suppress further fires
				// for kPostFireCooldownSeconds. Real geometry shifts happen
				// rarely; a burst of fires inside the cooldown window is noise
				// (the 2026-05-21 session showed 52 fires in 2.2 h, one every
				// ~2.4 min on Quest+Lighthouse cross-system pose noise). When
				// suppressed, log the inputs that would have fired and reset
				// the accumulators so the post-cooldown decision starts fresh.
				if (fire && spacecal::geometry_shift::ShouldSuppressForCooldown(
						time, ctx.geometryShiftCooldownUntil))
				{
					char cdBuf[280];
					snprintf(cdBuf, sizeof cdBuf,
						"[geometry-shift][suppressed-by-cooldown] current_mm=%.3f"
						" median_mm=%.3f ratio=%.2fx cusum_S_at_fire=%.3f mode=%s"
						" cooldown_remaining_sec=%.1f",
						current, median, ratio, cusumValueAtFire,
						ctx.useCusumGeometryShift ? "cusum" : "legacy",
						ctx.geometryShiftCooldownUntil - time);
					Metrics::WriteLogAnnotation(cdBuf);
					fire = false;
					g_geomShiftConsecutiveBadTicks = 0;
					g_cusumState.Reset();
				}

				if (fire) {
					// Diagnostic: fire annotation. The user-facing CalCtx.Log
					// below goes to the UI message buffer, not the spacecal log.
					// Without this annotation, every geometry-shift demote
					// (which is the proximate cause of every
					// continuous_standby_transition restart) is invisible to
					// post-session triage. Dump the inputs that triggered the
					// fire + a short tail of the error history so a reader can
					// see the buildup, not just the conclusion.
					std::string tailStr;
					tailStr.reserve(160);
					const int tailLen = std::min<int>(10, N);
					for (int i = N - tailLen; i < N; ++i) {
						char tbuf[24];
						snprintf(tbuf, sizeof tbuf, "%.2f%s",
							errSeries[i].second, (i + 1 < N) ? "," : "");
						tailStr += tbuf;
					}
					// errTail slope (mm per sample, linear-fit over the tail).
					// Discriminates a sudden single-tick spike (slope near 0
					// with one outlier) from a sustained drift (positive
					// slope). Computed against the tail-window index because
					// the time series's per-sample dt is not exposed here.
					double slopeMmPerSample = 0.0;
					if (tailLen >= 2) {
						const double n = (double)tailLen;
						double sumX = 0.0, sumY = 0.0, sumXY = 0.0, sumXX = 0.0;
						for (int i = 0; i < tailLen; ++i) {
							const double x = (double)i;
							const double y = errSeries[N - tailLen + i].second;
							sumX += x; sumY += y;
							sumXY += x * y; sumXX += x * x;
						}
						const double denom = n * sumXX - sumX * sumX;
						if (std::abs(denom) > 1e-12) {
							slopeMmPerSample = (n * sumXY - sumX * sumY) / denom;
						}
					}
					const bool reanchorFrozen =
						IsReanchorChiFrozen(time);
					const double secSinceReanchor = SecondsSinceLastReanchorChiLog(time);
					const std::string chiSqTail = RenderChiSqTail();
					const double cooldownStarts =
						time + spacecal::geometry_shift::kPostFireCooldownSeconds;
					// Pre-reset sustain count: in CUSUM mode the in-function
					// reset clears g_cusumState.sustainedAboveThreshold to 0
					// before this line runs, so read the captured out-param;
					// in legacy mode g_geomShiftConsecutiveBadTicks is still
					// at its pre-reset value (reset happens further below).
					const int sustainedAtFire = ctx.useCusumGeometryShift
						? cusumSustainAtFire
						: g_geomShiftConsecutiveBadTicks;
					char fireBuf[800];
					snprintf(fireBuf, sizeof fireBuf,
						"[geometry-shift][fire] current_mm=%.3f median_mm=%.3f"
						" ratio=%.2fx sustained=%d cusum_S_at_fire=%.3f mode=%s"
						" reanchor_frozen=%d sec_since_reanchor=%.3f"
						" lockRelativePosition=%d lockMode=%d"
						" errTail_slope_mm_per_sample=%.3f errTail=[%s]"
						" chi_sq_tail=%s cooldown_until=%.3f",
						current, median, ratio,
						sustainedAtFire, cusumValueAtFire,
						ctx.useCusumGeometryShift ? "cusum" : "legacy",
						(int)reanchorFrozen, secSinceReanchor,
						(int)ctx.lockRelativePosition, (int)ctx.lockRelativePositionMode,
						slopeMmPerSample, tailStr.c_str(),
						chiSqTail.c_str(), cooldownStarts);
					Metrics::WriteLogAnnotation(fireBuf);

					CalCtx.Log("Tracking geometry shifted -- restarting calibration\n");
					// A geometry-shift fire while a warm-restart grace was
					// active means the snap landed on a profile that no
					// longer matches reality (typically a base station got
					// nudged while the user was away). Drop the grace so
					// the normal continuous-cal recovery path takes over
					// -- the fast path traded safety for speed, and the
					// detector just learned the trade was wrong this time.
					if (ctx.warmRestartGraceSamples > 0) {
						char gbuf[160];
						snprintf(gbuf, sizeof gbuf,
							"[warm-restart][grace-ended] reason=geometry_shift"
							" remaining=%d",
							ctx.warmRestartGraceSamples);
						Metrics::WriteLogAnnotation(gbuf);
						ctx.warmRestartGraceSamples = 0;
					}
					calibration.Clear();
					ctx.state = CalibrationState::ContinuousStandby;
					// Intentionally NOT clearing ctx.relativePosCalibrated here.
					// Previously this path set it false, which wiped the
					// relative-pose constraint that AUTO Lock would have used
					// against the next cycle's noise. Log analysis showed
					// every restart killed the constraint permanently: only
					// the session-opening cal ever reached relPosCal=1. With
					// the constraint preserved, AUTO Lock can re-engage
					// quickly post-restart instead of waiting another full
					// convergence (and another ~290 s blackout). If a true
					// geometry shift occurred, the next cal cycle's solver
					// will overwrite refToTargetPose against fresh samples
					// anyway -- the value flowing back from CalibrationCalc
					// becomes the new "trusted" relative pose.
					ctx.geometryShiftCooldownUntil =
						time + spacecal::geometry_shift::kPostFireCooldownSeconds;
					g_geomShiftConsecutiveBadTicks = 0;
					g_cusumState.Reset();
				}
			}
		} else {
			g_geomShiftConsecutiveBadTicks = 0;
			g_cusumState.Reset();
		}
		}  // close `if (inGeometryShiftGrace) ... else { ...`
	}

	// Latency cross-correlation. Once per second, if the rolling speed buffers
	// are full and the user has actually been moving (RMS speed > 0.1 m/s on both
	// signals), compute a discrete cross-correlation and update the EMA estimate.
	// The active value is then used by CollectSample on subsequent ticks when
	// latencyAutoDetect is on.
	if (!ctx.useUpstreamMath
		&& (time - ctx.timeLastLatencyEstimate) > 1.0
		&& ctx.refSpeedHistory.size() >= CalibrationContext::kLatencyHistoryCapacity
		&& ctx.targetSpeedHistory.size() >= CalibrationContext::kLatencyHistoryCapacity
		&& ctx.speedSampleTimes.size() >= CalibrationContext::kLatencyHistoryCapacity)
	{
		ctx.timeLastLatencyEstimate = time;
		double lagSamples = 0.0;
		const int kMaxTau = 10;
		if (EstimateLatencyLagSamples(ctx.refSpeedHistory, ctx.targetSpeedHistory, kMaxTau, ctx.useGccPhatLatency, &lagSamples)) {
			// Convert sample lag to ms using the *empirical* sample rate from the
			// timestamp ring. This is more honest than assuming a fixed 20 Hz: the
			// rate is whatever CollectSample is being called at right now.
			double dur =
				ctx.speedSampleTimes.back() - ctx.speedSampleTimes.front();
			size_t intervals = ctx.speedSampleTimes.size() - 1;
			if (dur > 1e-3 && intervals > 0) {
				double sampleRateHz = (double)intervals / dur;
				double lagMs = lagSamples * 1000.0 / sampleRateHz;
				// Bound the per-update step to keep one bad correlation from
				// teleporting the offset estimate.
				if (std::isfinite(lagMs) && std::fabs(lagMs) <= 200.0) {
					const double prevEma = ctx.estimatedLatencyOffsetMs;
					ctx.estimatedLatencyOffsetMs = 0.7 * ctx.estimatedLatencyOffsetMs + 0.3 * lagMs;
					// Forensic diagnostic for audit row #12 (project_upstream_regression_audit_2026-05-04).
					// The cross-correlation buffer doesn't reset across HMD
					// stalls -- `dur` here can span the stall duration while
					// `intervals` is bounded by buffer size, producing a
					// deflated sample rate and inflating lagMs. The 200 ms
					// clamp above bounds this, but the EMA can still drift
					// for a few cycles.
					//
					// Log policy: emit every time when the EMA moves more than
					// 0.5 ms vs the last logged value, OR when it's been more
					// than 30 s since the last log (heartbeat). Otherwise the
					// loop produces ~290 lines/session of essentially-identical
					// rows. Anomalies (negative EMA, rate dropout) are logged
					// separately below regardless of throttle.
					static double s_lastLoggedEma = -1e9;
					static double s_lastLatencyLogTime = -1e9;
					const bool changed = std::fabs(ctx.estimatedLatencyOffsetMs - s_lastLoggedEma) > 0.5;
					const bool heartbeat = (time - s_lastLatencyLogTime) >= 30.0;
					if (changed || heartbeat || s_lastLoggedEma == -1e9) {
						s_lastLoggedEma = ctx.estimatedLatencyOffsetMs;
						s_lastLatencyLogTime = time;
						char latbuf[280];
						snprintf(latbuf, sizeof latbuf,
							"latency_ema_update: lagMs=%.2f sampleRateHz=%.2f intervals=%zu dur=%.2fs prev=%.2f new=%.2f reason=%s",
							lagMs, sampleRateHz, intervals, dur,
							prevEma, ctx.estimatedLatencyOffsetMs,
							changed ? "ema_changed" : "heartbeat");
						Metrics::WriteLogAnnotation(latbuf);
					}

					// Latency anomaly: physically lagMs cannot be negative
					// (samples can't arrive before their reference timestamp).
					// When it crosses the zero negative threshold, log a
					// dedicated diagnostic so the cause can be hunted (clock
					// skew, post-stall sample-rate deflation, timestamp
					// domain mismatch). Edge-triggered: emit on the
					// transition from prev_ema >= -1.0 to new_ema < -1.0,
					// not every tick while negative.
					if (prevEma >= -1.0 && ctx.estimatedLatencyOffsetMs < -1.0) {
						char anomBuf[280];
						snprintf(anomBuf, sizeof anomBuf,
							"[latency][anomaly] lagMs=%.2f new_ema=%.2f prev_ema=%.2f"
							" sampleRateHz=%.2f intervals=%zu dur=%.2fs"
							" sampleRateOk=%d durOk=%d note=negative_ema_crossed",
							lagMs, ctx.estimatedLatencyOffsetMs, prevEma,
							sampleRateHz, intervals, dur,
							(int)(sampleRateHz >= 9.0),
							(int)(dur < 20.0));
						Metrics::WriteLogAnnotation(anomBuf);
					}

					// Sample-rate dropout: log when sampleRateHz drops below
					// 9 Hz (well under the ~11 Hz typical). This catches the
					// post-stall buffer-not-reset case where `dur` inflates
					// without `intervals` keeping up.
					if (sampleRateHz < 9.0) {
						char rateBuf[240];
						snprintf(rateBuf, sizeof rateBuf,
							"[latency][rate-dropout] sampleRateHz=%.2f intervals=%zu dur=%.2fs"
							" lagMs=%.2f note=below_9hz_threshold",
							sampleRateHz, intervals, dur, lagMs);
						Metrics::WriteLogAnnotation(rateBuf);
					}
				}
			}
		}
	}

	// External smoothing-tool detection moved to the Smoothing overlay's
	// Tick (Protocol v12, 2026-05-11); its plugin scans on its own 5-second
	// cadence and surfaces the banner inside its Prediction sub-tab.

	ctx.timeLastTick = time;
	shmem.ReadNewPoses([&](const protocol::DriverPoseShmem::AugmentedPose& augmented_pose) {
		if (augmented_pose.deviceId >= 0 && augmented_pose.deviceId < (int)vr::k_unMaxTrackedDeviceCount) {
			ctx.devicePoses[augmented_pose.deviceId] = augmented_pose.pose;
			// Track per-device shmem QPC timestamps so CollectSample can compute the
			// inter-system time delta when applying targetLatencyOffsetMs.
			ctx.devicePoseSampleTimes[augmented_pose.deviceId] = augmented_pose.sample_time;
		}
	});

	// Sample driver-side telemetry counters and push the per-tick deltas (in Hz)
	// into the metrics time series. Initialize the prior snapshot lazily on the
	// first valid sample so the first delta is zero rather than a huge spike
	// representing the entire driver-uptime accumulation.
	{
		static bool s_telemetryPrimed = false;
		static uint64_t s_lastFallback = 0, s_lastPerId = 0, s_lastQuash = 0;
		static double s_lastTelemetryTime = 0;

		uint64_t fallback = 0, perId = 0, quash = 0;
		if (shmem.GetTelemetry(fallback, perId, quash)) {
			Metrics::RecordTimestamp();
			double now = Metrics::CurrentTime;
			if (!s_telemetryPrimed) {
				s_telemetryPrimed = true;
				s_lastFallback = fallback;
				s_lastPerId = perId;
				s_lastQuash = quash;
				s_lastTelemetryTime = now;
			} else {
				double dt = now - s_lastTelemetryTime;
				if (dt > 1e-6) {
					Metrics::fallbackApplyRate.Push((fallback - s_lastFallback) / dt);
					Metrics::perIdApplyRate.Push((perId - s_lastPerId) / dt);
					Metrics::quashApplyRate.Push((quash - s_lastQuash) / dt);
				}
				s_lastFallback = fallback;
				s_lastPerId = perId;
				s_lastQuash = quash;
				s_lastTelemetryTime = now;
			}
		}
	}

	// check for non-updating headset tracking space (caused by quest out of bounds or taken off head for example) and abort everything for this tick
	auto p = ctx.devicePoses[vr::k_unTrackedDeviceIndex_Hmd].vecPosition;
	if ((p[0] == 0.0 && p[1] == 0.0 && p[2] == 0.0) || (ctx.xprev == p[0] && ctx.yprev == p[1] && ctx.zprev == p[2])) {
		// std::cerr << "HMD tracking didn't update, skipping update" << std::endl;
		// Counter is preserved for the existing diagnostic UI in
		// UserInterface.cpp ("Stall purge: N events") and the
		// "hmd_stall_recovered after N ticks" log annotation below.
		if (ctx.consecutiveHmdStalls == 0) {
			// Stall-entered edge: companion to the existing recovered log so
			// a reader can compute the wall-clock stall duration without
			// having to guess the start by subtracting tick count from the
			// recovered timestamp. One line per stall, no per-tick noise.
			char enterBuf[200];
			snprintf(enterBuf, sizeof enterBuf,
				"[hmd-stall][entered] t=%.3f hmd_pos=(%.4f,%.4f,%.4f) prev=(%.4f,%.4f,%.4f)",
				time, p[0], p[1], p[2],
				(double)ctx.xprev, (double)ctx.yprev, (double)ctx.zprev);
			Metrics::WriteLogAnnotation(enterBuf);
		}
		ctx.consecutiveHmdStalls++;
		// REVERTED 2026-05-04: previously, after MaxHmdStalls=30 ticks of stalled
		// HMD tracking, the sample buffer was purged via calibration.Clear() and
		// state was demoted to ContinuousStandby. The intent was "stale samples no
		// longer represent reality" -- but the actual effect was much worse than
		// the problem it solved: on stall recovery, StartContinuousCalibration()
		// re-applies the saved refToTargetPose warm-start (relativePosCalibrated
		// is NOT reset, asymmetric vs the geometry-shift detector at line 2120
		// which DOES reset it), and continuous-cal converges from new post-stall
		// samples against that stale constraint. Each HMD-off/on cycle landed at
		// a slightly different local minimum; SaveProfile persisted it; cumulative
		// drift across many cycles wedged the saved profile.
		//
		// Empirical evidence (spacecal_log.2026-05-04T17-14-50.txt): two HMD off/on
		// events at t=1918 (56 ticks) and t=2096 (95 ticks) each produced a 7-9 cm
		// Z-axis shift in posOffset_currentCal IMMEDIATELY post-recovery, with the
		// cal magnitude climbing toward the wedge bound across the session.
		// Upstream (hyblocker) just `return`s on stall -- no clear, no demote -- and
		// the user reports this drift didn't happen on the old fork.
		//
		// Now matching upstream behavior: just return. Stale samples in the rolling
		// buffer naturally age out as fresh ones come in post-stall; the existing
		// rolling-window solver handles the transition without a discrete reset.
		return;
	}
	if (ctx.consecutiveHmdStalls > 0) {
		// Annotate recovery with both the legacy tick count and the
		// approximate duration in seconds (computed from tick rate).
		// Most stalls in normal sessions are 1-2 ticks; the long ones
		// (200+) are the interesting cases worth investigating, and
		// having a seconds value next to the tick count saves a
		// conversion step during triage.
		char buf[200];
		const double approxDurSec = (double)ctx.consecutiveHmdStalls / 90.0; // ~90 Hz typical
		snprintf(buf, sizeof buf,
		         "hmd_stall_recovered after %d ticks (approx %.2fs at 90Hz, t=%.3f)",
		         ctx.consecutiveHmdStalls, approxDurSec, time);
		Metrics::WriteLogAnnotation(buf);
	}
	ctx.consecutiveHmdStalls = 0;
	ctx.xprev = (float) p[0];
	ctx.yprev = (float) p[1];
	ctx.zprev = (float) p[2];

	// Run the scan in every state where a profile can be active. Previously the scan
	// was skipped once continuous calibration had a valid result, which meant a tracker
	// powered on mid-session never received its offset until calibration was restarted.
	// Per-ID dedupe inside ScanAndApplyProfile keeps IPC churn near zero when nothing
	// has changed.
	if (ctx.state == CalibrationState::None
		|| ctx.state == CalibrationState::ContinuousStandby
		|| ctx.state == CalibrationState::Continuous)
	{
		if ((time - ctx.timeLastScan) >= 1.0)
		{
			ScanAndApplyProfile(ctx);
			ctx.timeLastScan = time;
		}
	}

	if (ctx.state == CalibrationState::ContinuousStandby) {
		if (AssignTargets()) {
			StartContinuousCalibration("continuous_standby_transition");
		}
		else {
			ctx.wantedUpdateInterval = 0.5;
			ctx.Log("Waiting for devices...\n");
			return;
		}
	}

	if (ctx.state == CalibrationState::None) {
		// Base station drift correction (one-shot mode only): catch SteamVR
		// universe shifts so body trackers stay aligned with the user's
		// physical position. No-op when no base stations are detected.
		// Continuous mode is intentionally skipped -- the live solver would
		// converge through the shift on its own within a few seconds.
		TickBaseStationDrift(time);

		ctx.wantedUpdateInterval = 1.0;
		return;
	}

	if (ctx.state == CalibrationState::Editing)
	{
		ctx.wantedUpdateInterval = 0.1;

		if ((time - ctx.timeLastScan) >= 0.1)
		{
			ScanAndApplyProfile(ctx);
			ctx.timeLastScan = time;
		}
		return;
	}

	bool ok = true;

	if (ctx.referenceID == -1 || ctx.referenceID >= vr::k_unMaxTrackedDeviceCount) {
		CalCtx.Log("Missing reference device\n");
		ok = false;
	}
	if (ctx.targetID == -1 || ctx.targetID >= vr::k_unMaxTrackedDeviceCount)
	{
		CalCtx.Log("Missing target device\n");
		ok = false;
	}

	if (ctx.state == CalibrationState::Begin)
	{

		char referenceSerial[256], targetSerial[256];
		referenceSerial[0] = targetSerial[0] = 0;
		vr::VRSystem()->GetStringTrackedDeviceProperty(ctx.referenceID, vr::Prop_SerialNumber_String, referenceSerial, 256);
		vr::VRSystem()->GetStringTrackedDeviceProperty(ctx.targetID, vr::Prop_SerialNumber_String, targetSerial, 256);

		char buf[256];
		snprintf(buf, sizeof buf, "Reference device ID: %d, serial: %s\n", ctx.referenceID, referenceSerial);
		CalCtx.Log(buf);
		snprintf(buf, sizeof buf, "Target device ID: %d, serial %s\n", ctx.targetID, targetSerial);
		CalCtx.Log(buf);

		ScanAndApplyProfile(ctx);

		// (Removed: the original code pushed jitter here, in the Begin state,
		// before CollectSample had ever populated calibration.m_samples. The
		// pushed value was always 0.0 from an empty buffer, which then
		// poisoned ResolvedCalibrationSpeed's read of Metrics::jitterRef.last()
		// and locked AUTO onto FAST regardless of real tracker quality. The
		// authoritative push now lives at the end of CollectSample, so jitter
		// reflects the live buffer every active-calibration tick.)

		if (!CalCtx.ReferencePoseIsValidSimple())
		{
			CalCtx.Log("Reference device is not tracking\n"); ok = false;
		}

		if (!CalCtx.TargetPoseIsValidSimple())
		{
			CalCtx.Log("Target device is not tracking\n"); ok = false;
		}
		
		// @TOOD: Determine if the tracking is jittery
		if (calibration.ReferenceJitter() > ctx.jitterThreshold) {
			CalCtx.Log("Reference device is not tracking\n"); ok = false;
		}
		if (calibration.TargetJitter() > ctx.jitterThreshold) {
			CalCtx.Log("Target device is not tracking\n"); ok = false;
		}

		if (ok) {
			//ResetAndDisableOffsets(ctx.targetID);
			ctx.state = CalibrationState::Rotation;
			ctx.wantedUpdateInterval = 0.0;

			CalCtx.Log("Starting calibration...\n");
			return;
		}
	}

	if (!ok)
	{
		if (ctx.state != CalibrationState::Continuous) {
			ctx.state = CalibrationState::None;

			CalCtx.Log("Aborting calibration!\n");
		}
		return;
	}

	// Tracker liveness gate (TrackerLiveness.h). Active only in continuous-
	// calibration mode; one-shot states (Begin/Rotation/Translation) run the
	// user through a deliberate motion sequence and have their own validity
	// checks. The gate exists to catch the case where SteamVR reports
	// `Running_OK + poseIsValid=true` for a silently-disconnected Vive
	// tracker (or any non-HMD anchor) and CollectSample's existing pose-
	// flag gate cannot tell the difference between a live tracker and a
	// frozen-pose ghost. Returning early here implicitly suppresses
	// CollectSample, ComputeIncremental, and SaveProfile for the same tick
	// (control flow falls through to all three immediately below), which
	// stops the on-disk profile from drifting during the offline window.
	if (ctx.state == CalibrationState::Continuous) {
		const double hmdSpeedMps = ComputeHmdSpeedMps(ctx);

		// Commit any queued AUTO-Lock flip if the user is paused enough to
		// hide the resulting calibration jump. Re-resolve lockRelativePosition
		// afterward so the same tick's downstream `calibration.lockRelativePosition`
		// write below reflects the new state. `time` participates in the
		// post-reanchor suppression gate inside the commit helper.
		if (CommitPendingAutoLockFlipIfStationary(ctx, hmdSpeedMps, time)) {
			ctx.ResolveLockMode();
		}

		auto tickOne = [&](int32_t id,
		                   const char* whichLabel,
		                   spacecal::liveness::TrackerLivenessState& state,
		                   bool& wasOffline)
		{
			if (id < 0 || IsHmdDevice(id)) {
				wasOffline = spacecal::liveness::IsOffline(state);
				return;  // HMD handled by separate stall + relocalization detectors
			}
			const auto& dp = ctx.devicePoses[id];
			spacecal::liveness::TrackerLivenessInputs in{};
			in.posHash           = HashPositionLow64(dp.vecPosition);
			in.deviceIsConnected = dp.deviceIsConnected;
			in.hmdSpeedMps       = hmdSpeedMps;
			in.lastEmaUpdateSec  = ctx.timeLastLatencyEstimate;
			in.now               = time;

			const bool wentOffline = spacecal::liveness::TickTrackerLiveness(state, in);
			if (wentOffline) {
				char buf[256];
				const double frozenForSec = state.poseHashSinceSec >= 0.0
					? (time - state.poseHashSinceSec) : 0.0;
				const double emaGapSec = ctx.timeLastLatencyEstimate > 0.0
					? (time - ctx.timeLastLatencyEstimate) : 0.0;
				snprintf(buf, sizeof buf,
					"tracker_offline_detected: which=%s id=%d frozenForSec=%.1f "
					"deviceIsConnected=%d emaGapSec=%.1f",
					whichLabel, (int)id, frozenForSec,
					(int)dp.deviceIsConnected, emaGapSec);
				Metrics::WriteLogAnnotation(buf);
				CalCtx.Log("Calibration anchor offline -- waiting to reconnect\n");
			}
			wasOffline = spacecal::liveness::IsOffline(state);
		};

		bool refOfflineNow = false;
		bool tgtOfflineNow = false;
		tickOne(ctx.referenceID, "reference", g_refLiveness, refOfflineNow);
		// When "Hide tracker" is on, the driver freezes the target's pose at
		// the last good calibrated value with TrackingResult_Calibrating_OutOfRange.
		// The frozen pose would otherwise trip the liveness detector's
		// frozen-pose path after a few seconds of HMD movement; reset the
		// state and skip so a hide-driven freeze doesn't masquerade as a
		// disconnect.
		if (ctx.quashTargetInContinuous) {
			spacecal::liveness::Reset(g_tgtLiveness);
			tgtOfflineNow = false;
		} else {
			tickOne(ctx.targetID, "target", g_tgtLiveness, tgtOfflineNow);
		}

		// Online -> offline edge fires above. Detect the reverse edge here
		// (was offline last tick, online this tick) so we can fire a clean
		// StartContinuousCalibration to discard any contaminated samples
		// and re-anchor from live poses. Per-anchor edge tracking so a
		// reference recovery while target is still offline correctly waits
		// for the target before firing.
		const bool refReturned = g_refWasOffline && !refOfflineNow;
		const bool tgtReturned = g_tgtWasOffline && !tgtOfflineNow;
		g_refWasOffline = refOfflineNow;
		g_tgtWasOffline = tgtOfflineNow;

		if (refOfflineNow || tgtOfflineNow) {
			ctx.wantedUpdateInterval = 0.5;
			return;  // skip CollectSample + ComputeIncremental + SaveProfile
		}

		if (refReturned || tgtReturned) {
			char buf[200];
			const char* whichLabel = refReturned
				? (tgtReturned ? "reference+target" : "reference")
				: "target";
			// For rigidly-locked setups (head-mounted anchor, AUTO-locked or
			// manual Lock=ON with a valid relative pose), the existing cal is
			// still correct through a brief silence -- restarting only causes
			// a visible jump and pollutes the log with "Collecting initial
			// samples..." every time the user's wireless tracker briefly drops.
			// Reset the liveness state in place and continue.
			if (CalCtx.lockRelativePosition && CalCtx.relativePosCalibrated) {
				snprintf(buf, sizeof buf,
					"tracker_reconnected: which=%s skip_restart=locked_rel_pose",
					whichLabel);
				Metrics::WriteLogAnnotation(buf);
				spacecal::liveness::Reset(g_refLiveness);
				spacecal::liveness::Reset(g_tgtLiveness);
				g_refWasOffline = false;
				g_tgtWasOffline = false;
				// Don't return -- let the normal CollectSample path resume
				// this tick now that the anchors are confirmed alive again.
			} else {
				snprintf(buf, sizeof buf,
					"tracker_reconnected: which=%s StartContinuousCalibration",
					whichLabel);
				Metrics::WriteLogAnnotation(buf);
				// StartContinuousCalibration internally Resets both liveness
				// states + the edge-tracking flags, so the next tick starts
				// from a clean baseline.
				StartContinuousCalibration("tracker_liveness_reconnect");
				return;
			}
		}
	}

	if (!CollectSample(ctx))
	{
		return;
	}

	CalCtx.Progress((int) calibration.SampleCount(), (int)CalCtx.SampleCount());

	if (calibration.SampleCount() < CalCtx.SampleCount()) return;
	while (calibration.SampleCount() > CalCtx.SampleCount()) calibration.ShiftSample();

	// Two-phase one-shot motion-variety gate. Continuous mode bypasses this --
	// it has its own incremental accept/reject loop that doesn't need a "stop
	// here" signal.
	//
	// Earlier this was a single combined gate ("both diversities >= 70 % or
	// keep rolling"). That trapped users in an unwinnable game with the
	// rolling 250-sample buffer: rotate first -> rotation samples age out
	// before translation samples accumulate; translate first -> vice versa.
	// The user-visible symptom was the "Translation %" bar that "never
	// reaches 100" because the buffer recycled rotation-rich content out
	// before the user could fill the translation half.
	//
	// Two-phase flow:
	//   Rotation phase: gate on rotationDiversity only. Buffer rolls until
	//   the user has rotated through >= 90 deg between some pair of samples.
	//   When the gate passes, freeze the buffer (FreezeRotationPhaseSamples)
	//   and transition to Translation. The freeze preserves the rotation
	//   samples for the final solve regardless of how slowly the user fills
	//   the translation half.
	//
	//   Translation phase: gate on translationDiversity only, computed on a
	//   fresh live buffer. Buffer rolls until translationDiversity >= 0.55
	//   (with kDesiredAxisRange=0.20m that means ~11cm on the weakest axis).
	//   When the gate passes, fall through to ComputeOneshot, which splices
	//   the frozen rotation samples back in for the math.
	if (CalCtx.state == CalibrationState::Rotation) {
		constexpr double kPhaseDiversity = 0.70;
		if (calibration.RotationDiversity() < kPhaseDiversity) {
			calibration.ShiftSample();
			return;
		}
		// Rotation phase complete. Freeze the buffer, transition state, and
		// return so the next CollectSample tick starts populating a fresh
		// translation-phase buffer. The popup's Rotation% bar will visually
		// drop to 0 (the new buffer is empty) but the UI latches it at 100%
		// while CalCtx.state == Translation so the user sees the achievement
		// preserved.
		calibration.FreezeRotationPhaseSamples();
		CalCtx.state = CalibrationState::Translation;
		CalCtx.Log("Rotation phase complete. Now wave the tracker through ~15 cm on every axis.\n");
		Metrics::WriteLogAnnotation("RotationPhaseFrozen");
		return;
	}
	if (CalCtx.state == CalibrationState::Translation) {
		// Lowered from 0.70 (2026-05-13): combined with kDesiredAxisRange=0.20m
		// the 70% gate demanded 21cm per axis, which a tracker rigidly mounted
		// to an HMD struggles to hit on the weakest axis via normal head movement.
		// 0.55 * 0.20m = 11cm per axis -- achievable with a deliberate nod and
		// lateral lean. The math gates in ComputeOneshot still reject genuinely
		// under-constrained solutions; this only speeds up the collection trigger.
		constexpr double kPhaseDiversity = 0.55;
		if (calibration.TranslationDiversity() < kPhaseDiversity) {
			calibration.ShiftSample();
			return;
		}
		// Translation diversity satisfied -- fall through to ComputeOneshot
		// below. ComputeOneshot's RotationFreezeSplice will prepend the
		// frozen rotation samples for the duration of the solve.
	}

	if (CalCtx.state == CalibrationState::Continuous && CalCtx.requireTriggerPressToApply && CalCtx.hasAppliedCalibrationResult) {
		bool triggerPressed = true;
		vr::VRControllerState_t state;
		for (int i = 0; i < CalCtx.MAX_CONTROLLERS; i++) {
			if (CalCtx.controllerIDs[i] >= 0) {
				vr::VRSystem()->GetControllerState(CalCtx.controllerIDs[i], &state, sizeof(state));
				triggerPressed &= state.rAxis[vr::k_eControllerAxis_TrackPad /* matches trigger on Index controllers?? */].x > 0.75f
					|| state.rAxis[vr::k_eControllerAxis_Trigger].x > 0.75f;
				//printf("Controller %d tracpad: %f\n", i, state.rAxis[vr::k_eControllerAxis_TrackPad].x);
				//printf("Controller %d trigger: %f\n", i, state.rAxis[vr::k_eControllerAxis_Trigger].x);
				if (!triggerPressed) {
					break;
				}
			}
		}

		if (!triggerPressed) {
			CalCtx.Log("Waiting for trigger press...\n");
			CalCtx.wasWaitingForTriggers = true;
			return;
		}

		if (CalCtx.wasWaitingForTriggers) {
			CalCtx.Log("Triggers pressed, continuing calibration...\n");
			CalCtx.wasWaitingForTriggers = false;
		}
	}

	LARGE_INTEGER start_time;
	QueryPerformanceCounter(&start_time);
		
	bool lerp = false;
	calibration.useVelocityAwareWeighting = CalCtx.useVelocityAwareWeighting && !CalCtx.useUpstreamMath;
	calibration.useTukeyBiweight = CalCtx.useTukeyBiweight && !CalCtx.useUpstreamMath;
	calibration.useBlendFilter = CalCtx.useBlendFilter && !CalCtx.useUpstreamMath;

	if (CalCtx.state == CalibrationState::Continuous) {
		CalCtx.messages.clear();
		calibration.enableStaticRecalibration = CalCtx.enableStaticRecalibration;
		calibration.lockRelativePosition = CalCtx.lockRelativePosition;

		const double hmdSpeedMps = ComputeHmdSpeedMps(CalCtx);

		// User-toggled "Pause updates" from the continuous-cal UI: keep the
		// already-applied driver offset live, skip any new solve cycle so the
		// math doesn't fight the user trying to inspect the current result.
		if (!CalCtx.calibrationPaused) {
			calibration.ComputeIncremental(lerp, CalCtx.continuousCalibrationThreshold, CalCtx.maxRelativeErrorThreshold, CalCtx.ignoreOutliers);

			// Warm-restart grace counts down per Continuous-mode solve. When
			// it hits zero, the prior-vs-new error gate snaps back on -- the
			// solver has had ~30 s of bypassed acceptance and should be sitting
			// on the saved offset now. Validation phase replaces the prior
			// silent samples_exhausted success path: each tick checks the
			// rolling MAD floor, ends grace early on convergence (Settled),
			// triggers RecoverFromWedgedCalibration at grace end with elevated
			// MAD (Failed -- snap landed on a profile that no longer matches
			// reality), or rides out the window as Inconclusive (MAD between
			// thresholds, current behaviour but logged with the actual reading
			// so a reader can see whether the snap was load-bearing).
			if (CalCtx.warmRestartGraceSamples > 0) {
				--CalCtx.warmRestartGraceSamples;
				const int samplesSinceSnap =
					spacecal::warm_restart::kGraceSamples - CalCtx.warmRestartGraceSamples;
				const bool graceEndedThisTick = (CalCtx.warmRestartGraceSamples == 0);
				const double madFloor = CalCtx.autoLockMadFloor;

				// Accumulate any fresh error_currentCal sample produced by
				// the ComputeIncremental call above into the post-snap
				// bias mean. error_currentCal.Push only fires when the
				// solver successfully ValidateCalibration's the applied
				// transform, so some ticks contribute no sample; the
				// timestamp gate keeps stale samples (carried over from
				// before the snap, or from a missed-push earlier tick)
				// out of the mean. This is the correctness signal the
				// previous validator was missing: it tracks how well the
				// applied calibration fits the live samples, not just how
				// quiet the relative-pose dispersion is.
				const double latestErrTs = Metrics::error_currentCal.lastTs();
				if (latestErrTs > CalCtx.warmRestartLastConsumedErrTs) {
					CalCtx.postSnapErrorSumMm += Metrics::error_currentCal.last();
					CalCtx.postSnapErrorSampleCount += 1;
					CalCtx.warmRestartLastConsumedErrTs = latestErrTs;
				}
				const double meanBiasTransM =
					(CalCtx.postSnapErrorSampleCount > 0)
					? (CalCtx.postSnapErrorSumMm
						/ static_cast<double>(CalCtx.postSnapErrorSampleCount))
						/ 1000.0
					: 0.0;

				const spacecal::warm_restart::ValidationInputs vin{
					madFloor, samplesSinceSnap, graceEndedThisTick,
					meanBiasTransM};
				const auto outcome = spacecal::warm_restart::EvaluateValidation(vin);

				// mad_floor_source labels whether the rolling-min floor
				// was produced by a pre-snap sample (inherited quiet
				// floor; Settled-by-floor is suspect) or a post-snap
				// sample (genuine convergence).
				const char* madFloorSource =
					(CalCtx.warmRestartSnapTime > 0.0
						&& CalCtx.autoLockMadFloorTs > 0.0
						&& CalCtx.autoLockMadFloorTs >= CalCtx.warmRestartSnapTime)
					? "postSnap" : "preSnap";

				if (outcome == spacecal::warm_restart::ValidationOutcome::Settled
					&& CalCtx.warmRestartValidationState
						!= spacecal::warm_restart::ValidationOutcome::Settled) {
					CalCtx.warmRestartValidationState =
						spacecal::warm_restart::ValidationOutcome::Settled;
					CalCtx.warmRestartGraceSamples = 0;  // end grace early
					char vbuf[320];
					snprintf(vbuf, sizeof vbuf,
						"[warm-restart][validated] mad_mm=%.3f samples_since_snap=%d"
						" mad_at_snap_mm=%.3f post_snap_bias_mm=%.3f"
						" post_snap_samples=%d mad_floor_source=%s reason=settled",
						madFloor * 1000.0, samplesSinceSnap,
						CalCtx.warmRestartMadAtSnap * 1000.0,
						meanBiasTransM * 1000.0,
						CalCtx.postSnapErrorSampleCount, madFloorSource);
					Metrics::WriteLogAnnotation(vbuf);
					Metrics::WriteLogAnnotation(
						"[warm-restart][grace-ended] reason=validated_settled");
				} else if (outcome == spacecal::warm_restart::ValidationOutcome::Failed
					&& CalCtx.warmRestartValidationState
						!= spacecal::warm_restart::ValidationOutcome::Failed) {
					// Bias-Failed can fire mid-grace (post-snap retargeting
					// error too large); end grace immediately and trigger
					// recovery. MAD-Failed only fires at grace end and is
					// handled in the graceEndedThisTick branch below.
					CalCtx.warmRestartValidationState =
						spacecal::warm_restart::ValidationOutcome::Failed;
					CalCtx.warmRestartGraceSamples = 0;
					const char* failReason =
						(meanBiasTransM > spacecal::warm_restart::kFailBiasTransM)
						? "bias_above_threshold" : "above_failed_threshold";
					char fbuf[320];
					snprintf(fbuf, sizeof fbuf,
						"[warm-restart][failed] mad_mm=%.3f samples_since_snap=%d"
						" mad_at_snap_mm=%.3f post_snap_bias_mm=%.3f"
						" post_snap_samples=%d mad_floor_source=%s reason=%s",
						madFloor * 1000.0, samplesSinceSnap,
						CalCtx.warmRestartMadAtSnap * 1000.0,
						meanBiasTransM * 1000.0,
						CalCtx.postSnapErrorSampleCount, madFloorSource,
						failReason);
					Metrics::WriteLogAnnotation(fbuf);
					Metrics::WriteLogAnnotation(
						"[warm-restart][grace-ended] reason=validation_failed");
					RecoverFromWedgedCalibration(
						"Warm-restart validation failed -- recalibrating from scratch\n",
						"warm_restart_validation_failed");
				} else if (graceEndedThisTick) {
					// Inconclusive at grace end: MAD between thresholds and
					// bias under the fail threshold. Profile stays; log
					// loudly so this case is visible in triage.
					char ibuf[320];
					snprintf(ibuf, sizeof ibuf,
						"[warm-restart][inconclusive] mad_mm=%.3f samples_since_snap=%d"
						" mad_at_snap_mm=%.3f post_snap_bias_mm=%.3f"
						" post_snap_samples=%d mad_floor_source=%s"
						" reason=between_thresholds",
						madFloor * 1000.0, samplesSinceSnap,
						CalCtx.warmRestartMadAtSnap * 1000.0,
						meanBiasTransM * 1000.0,
						CalCtx.postSnapErrorSampleCount, madFloorSource);
					Metrics::WriteLogAnnotation(ibuf);
					Metrics::WriteLogAnnotation(
						"[warm-restart][grace-ended] reason=samples_exhausted");
				}
			}

			// Sustained gravity-axis disagreement diagnostic. Push the per-tick
			// residual pitch+roll reading into a rolling 60s window and ask
			// the pure-helper decision (TiltDiagnostic.h) whether the median
			// has crossed the 1.0 deg sustained threshold. Logging-only:
			// surfaces "your two systems disagree about which way is down"
			// as a sustained signal so the user can re-run room setup. No
			// calibration behavior change. See project_future_improvements_
			// 2026-05-05.md for the eventual Pacher-2021 RFU-style
			// correction that this diagnostic is the prerequisite for.
			{
				const double tiltDeg = calibration.m_residualPitchRollDeg;
				if (std::isfinite(tiltDeg) && tiltDeg >= 0.0) {
					CalCtx.tiltDiagnosticWindow.push_back({time, tiltDeg});
					const double cutoff = time - spacecal::gravity::kSustainedWindowSeconds;
					while (!CalCtx.tiltDiagnosticWindow.empty()
						&& CalCtx.tiltDiagnosticWindow.front().timestamp_s < cutoff) {
						CalCtx.tiltDiagnosticWindow.pop_front();
					}
					const auto decision = spacecal::gravity::EvaluateTilt(
						CalCtx.tiltDiagnosticWindow, time, CalCtx.tiltSustainedAlarmed);
					if (decision.sustainedDisagreement != CalCtx.tiltSustainedAlarmed) {
						CalCtx.tiltSustainedAlarmed = decision.sustainedDisagreement;
						CalCtx.tiltLastAnnotatedMedian = decision.medianDeg;
						char tbuf[224];
						snprintf(tbuf, sizeof tbuf,
							"gravity_disagreement_%s: median_tilt_deg=%.3f window_samples=%zu threshold=%.2f",
							decision.sustainedDisagreement ? "sustained_on" : "sustained_off",
							decision.medianDeg, CalCtx.tiltDiagnosticWindow.size(),
							spacecal::gravity::kSustainedTiltThresholdDeg);
						Metrics::WriteLogAnnotation(tbuf);
					}
				}
			}
		}

		// Multi-ecosystem extras: each runs its own continuous calibration
		// loop in parallel with the primary, against the SAME reference
		// device (the HMD) and its own target. Each extra has its own
		// sample buffer (extra.calc) so noisy samples on one don't taint
		// another. Cheap -- the math is bounded by sample-buffer size and
		// runs at the same low cadence as the primary.
		for (auto& extra : CalCtx.additionalCalibrations) {
			if (!extra.enabled) continue;
			if (extra.referenceID < 0 || extra.targetID < 0) continue;
			if (extra.referenceID >= maxId || extra.targetID >= maxId) continue;

			const auto& refPose = CalCtx.devicePoses[extra.referenceID];
			const auto& tgtPose = CalCtx.devicePoses[extra.targetID];
			if (!refPose.poseIsValid || !tgtPose.poseIsValid) continue;
			if (refPose.result != vr::ETrackingResult::TrackingResult_Running_OK) continue;
			if (tgtPose.result != vr::ETrackingResult::TrackingResult_Running_OK) continue;

			auto vmag = [](const double v[3]) -> double {
				const double s = std::sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
				return std::isfinite(s) ? s : 0.0;
			};
			Sample s(ConvertPose(refPose), ConvertPose(tgtPose), glfwGetTime(),
			         vmag(refPose.vecVelocity), vmag(tgtPose.vecVelocity));
			extra.calc->PushSample(s);
			while (extra.calc->SampleCount() > CalCtx.SampleCount()) extra.calc->ShiftSample();

			// Per-extra auto-lock detector update.
			Eigen::AffineCompact3d refW = Eigen::AffineCompact3d::Identity();
			refW.linear() = ConvertPose(refPose).rot;
			refW.translation() = ConvertPose(refPose).trans;
			Eigen::AffineCompact3d tgtW = Eigen::AffineCompact3d::Identity();
			tgtW.linear() = ConvertPose(tgtPose).rot;
			tgtW.translation() = ConvertPose(tgtPose).trans;
			const Eigen::AffineCompact3d rel = refW.inverse() * tgtW;
			extra.autoLockHistory.push_back(rel);
			while (extra.autoLockHistory.size() > spacecal::autolock::kHistoryMax) {
				extra.autoLockHistory.pop_front();
			}

			// Mirrors CalibrationContext::UpdateAutoLockDetector. MAD-based
			// robust deviation + enter/leave hysteresis + panic-unlock +
			// pending-flip queue + stationary-HMD commit gate. The previous
			// raw-stddev + single-threshold path flapped on cross-system
			// extras for the same reasons the primary did pre-d1a7e9e.
			if (extra.autoLockHistory.size() < spacecal::autolock::kSamplesNeeded) {
				extra.autoLockEffectivelyLocked = false;
				extra.autoLockHasPendingFlip = false;
			} else {
				const double translMad =
					spacecal::autolock::RobustTranslDeviation(extra.autoLockHistory);
				const double rotMad =
					spacecal::autolock::RobustRotDeviation(extra.autoLockHistory);

				if (extra.autoLockEffectivelyLocked &&
					spacecal::autolock::IsPanicLevelDeviation(translMad, rotMad)) {
					extra.autoLockEffectivelyLocked = false;
					extra.autoLockHasPendingFlip = false;
					extra.autoLockPendingFlipFirstSeen = 0.0;
					extra.autoLockGateHeldWarned = false;
					char buf[224];
					snprintf(buf, sizeof buf,
						"auto_lock_panic_unlock extra=%s: translMad=%.4fm rotMad=%.4frad",
						extra.targetTrackingSystem.c_str(), translMad, rotMad);
					Metrics::WriteLogAnnotation(buf);
				} else {
					const bool verdict = spacecal::autolock::VerdictWithHysteresis(
						translMad, rotMad, extra.autoLockEffectivelyLocked);

					if (verdict != extra.autoLockEffectivelyLocked) {
						const bool prevPending = extra.autoLockHasPendingFlip;
						const bool prevTarget  = extra.autoLockPendingFlipTo;
						extra.autoLockHasPendingFlip = true;
						extra.autoLockPendingFlipTo  = verdict;
						if (!prevPending || prevTarget != verdict) {
							char buf[240];
							snprintf(buf, sizeof buf,
								"auto_lock_flip_pending extra=%s: target=%d current=%d"
								" translMad=%.4fm rotMad=%.4frad",
								extra.targetTrackingSystem.c_str(),
								(int)verdict, (int)extra.autoLockEffectivelyLocked,
								translMad, rotMad);
							Metrics::WriteLogAnnotation(buf);
						}
					} else if (extra.autoLockHasPendingFlip) {
						extra.autoLockHasPendingFlip = false;
					}

					if (extra.autoLockHasPendingFlip) {
						if (extra.autoLockPendingFlipFirstSeen <= 0.0) {
							extra.autoLockPendingFlipFirstSeen = time;
							extra.autoLockGateHeldWarned = false;
						}
						const double heldSec = time - extra.autoLockPendingFlipFirstSeen;
						// Extras have no reanchor concept -- suppressUntil = 0.0.
						const auto gate = spacecal::autolock::EvaluateCommitGate(
							extra.autoLockPendingFlipTo, hmdSpeedMps, time,
							/*reanchorSuppressUntil=*/0.0, heldSec);
						if (gate.commit) {
							const bool prev = extra.autoLockEffectivelyLocked;
							extra.autoLockEffectivelyLocked = extra.autoLockPendingFlipTo;
							extra.autoLockHasPendingFlip = false;
							extra.autoLockPendingFlipFirstSeen = 0.0;
							extra.autoLockGateHeldWarned = false;
							char buf[256];
							snprintf(buf, sizeof buf,
								"auto_lock_flip extra=%s: previous=%d now=%d hmdSpeed=%.3fmps"
								" held_sec=%.2f committed_via=%s",
								extra.targetTrackingSystem.c_str(),
								(int)prev, (int)extra.autoLockEffectivelyLocked,
								hmdSpeedMps, heldSec, gate.mode);
							Metrics::WriteLogAnnotation(buf);
						}
					} else {
						extra.autoLockPendingFlipFirstSeen = 0.0;
						extra.autoLockGateHeldWarned = false;
					}
				}
			}

			// Resolve effective lock for this extra.
			switch (extra.lockMode) {
			case 0:  extra.lockRelativePosition = false; break;
			case 1:  extra.lockRelativePosition = true; break;
			default: extra.lockRelativePosition = extra.autoLockEffectivelyLocked; break;
			}

			extra.calc->lockRelativePosition = extra.lockRelativePosition;
			extra.calc->enableStaticRecalibration = CalCtx.enableStaticRecalibration;

			if (!CalCtx.calibrationPaused && extra.calc->SampleCount() >= CalCtx.SampleCount()) {
				bool extraLerp = false;
				if (extra.calc->ComputeIncremental(extraLerp,
						CalCtx.continuousCalibrationThreshold,
						CalCtx.maxRelativeErrorThreshold,
						CalCtx.ignoreOutliers)) {
					if (extra.calc->isValid()) {
						extra.calibratedRotation = extra.calc->EulerRotation();
						extra.calibratedTranslation =
							extra.calc->Transformation().translation() * 100.0;
						extra.refToTargetPose = extra.calc->RelativeTransformation();
						extra.relativePosCalibrated = extra.calc->isRelativeTransformationCalibrated();
						extra.valid = true;
					}
				}
				// Track this extra's recent priorCalibrationError values
				// for the geometry-shift common-mode coherence check.
				// LastPriorErrorM is INFINITY until a validated compute
				// has happened; skip pushes until then so the rolling
				// median is built from real readings, not sentinels.
				const double extraErrM = extra.calc->LastPriorErrorM();
				if (std::isfinite(extraErrM)) {
					extra.recentErrorsMm.push_back(extraErrM * 1000.0);
					while (extra.recentErrorsMm.size() > 30) {
						extra.recentErrorsMm.pop_front();
					}
				}
			}
		}
	}
	else {
		calibration.enableStaticRecalibration = false;
		calibration.ComputeOneshot(CalCtx.ignoreOutliers);
	}

	if (calibration.isValid()) {
		const Eigen::Matrix3d R = calibration.Transformation().rotation();
		const Eigen::Vector3d T = calibration.Transformation().translation();
		const double rotAngle =
			std::acos(std::clamp((R.trace() - 1.0) * 0.5, -1.0, 1.0));
		const bool finiteT = T.allFinite();
		const bool nonTrivialRot = rotAngle > 1e-3;  // > ~0.06 deg from identity
		if (!finiteT || !nonTrivialRot) {
			char rejBuf[160];
			std::snprintf(rejBuf, sizeof rejBuf,
				"calibration_rejected_degenerate: finiteT=%d rotAngle=%.6f",
				finiteT ? 1 : 0, rotAngle);
			Metrics::WriteLogAnnotation(rejBuf);
			CalCtx.Log("Degenerate solve rejected; keeping previous profile.\n");
		} else {
			ctx.calibratedRotation = calibration.EulerRotation();
			ctx.calibratedTranslation = calibration.Transformation().translation() * 100.0; // convert to cm units for profile storage
			ctx.refToTargetPose = calibration.RelativeTransformation();
			ctx.relativePosCalibrated = calibration.isRelativeTransformationCalibrated();

			auto vrTrans = VRTranslationVec(ctx.calibratedTranslation);
			auto vrRot = VRRotationQuat(Eigen::Quaterniond(calibration.Transformation().rotation()));

			ctx.validProfile = true;
			SaveProfile(ctx);

			ScanAndApplyProfile(ctx);

			CalCtx.hasAppliedCalibrationResult = true;

			CalCtx.Log("Finished calibration, profile saved\n");

			// Runtime wedge detector REMOVED 2026-05-05 -- fired in a 3-fire
			// reset loop on the user's Quest+Lighthouse setup whose legitimate
			// continuous-cal convergence values (~265-295 cm) sit above any
			// fixed magnitude threshold we picked. See
			// project_wedge_guard_removed_2026-05-05.md (memory). Quest
			// re-localization auto-recovery in TickHmdRelocalizationDetector
			// (HMD-jump signal, not magnitude) is unchanged and still active.
		}
	} else {
		CalCtx.Log("Calibration failed.\n");
	}

	LARGE_INTEGER end_time;
	QueryPerformanceCounter(&end_time);
	LARGE_INTEGER freq;
	QueryPerformanceFrequency(&freq);
	double duration = (end_time.QuadPart - start_time.QuadPart) / (double)freq.QuadPart;
	const double computationTimeMs = duration * 1000.0;
	Metrics::computationTime.Push(computationTimeMs);

	// CPU-pressure diagnostic. Sampled at the end of each CalibrationTick so the
	// computationTime above is in scope for the per-tick spike check. Pure
	// logging: emits cpu_pressure_warning_on/_off transitions when the
	// 5-second EMA of process CPU% crosses 50%/30% (with hysteresis), and a
	// throttled cpu_pressure_spike on any single ComputeIncremental >= 200 ms.
	TickCpuPressureMonitor(computationTimeMs, time);

	// Hand the raw reference + target poses to the metrics writer so the v2 CSV
	// columns get filled. Reconstructing these in the replay harness (tools/replay/)
	// gives us the same Sample values that fed CalibrationCalc::PushSample, which
	// is the whole point of the harness -- the metric-level columns alone aren't
	// enough to re-run the math offline.
	{
		const vr::DriverPose_t& refPose = ctx.devicePoses[ctx.referenceID];
		const vr::DriverPose_t& tgtPose = ctx.devicePoses[ctx.targetID];

		auto driverPoseToWorld = [](const vr::DriverPose_t& dp,
			Eigen::Vector3d& outTrans, Eigen::Quaterniond& outRot) {
			Eigen::Quaterniond worldFromDriver(
				dp.qWorldFromDriverRotation.w,
				dp.qWorldFromDriverRotation.x,
				dp.qWorldFromDriverRotation.y,
				dp.qWorldFromDriverRotation.z);
			Eigen::Vector3d worldFromDriverTrans(
				dp.vecWorldFromDriverTranslation[0],
				dp.vecWorldFromDriverTranslation[1],
				dp.vecWorldFromDriverTranslation[2]);
			Eigen::Quaterniond rot(dp.qRotation.w, dp.qRotation.x, dp.qRotation.y, dp.qRotation.z);
			Eigen::Vector3d pos(dp.vecPosition[0], dp.vecPosition[1], dp.vecPosition[2]);
			outRot = (worldFromDriver * rot).normalized();
			outTrans = worldFromDriverTrans + worldFromDriver * pos;
		};

		Eigen::Vector3d refT, tgtT;
		Eigen::Quaterniond refQ, tgtQ;
		driverPoseToWorld(refPose, refT, refQ);
		driverPoseToWorld(tgtPose, tgtT, tgtQ);

		// Map CalibrationState (Calibration.h) to TickPhase (CalibrationMetrics.h).
		// The two enums intentionally mirror each other; we don't share the type
		// so the metrics module doesn't need to include Calibration.h.
		Metrics::TickPhase phase = Metrics::TickPhase::None;
		switch (CalCtx.state) {
		case CalibrationState::None:              phase = Metrics::TickPhase::None; break;
		case CalibrationState::Begin:             phase = Metrics::TickPhase::Begin; break;
		case CalibrationState::Rotation:          phase = Metrics::TickPhase::Rotation; break;
		case CalibrationState::Translation:       phase = Metrics::TickPhase::Translation; break;
		case CalibrationState::Editing:           phase = Metrics::TickPhase::Editing; break;
		case CalibrationState::Continuous:        phase = Metrics::TickPhase::Continuous; break;
		case CalibrationState::ContinuousStandby: phase = Metrics::TickPhase::ContinuousStandby; break;
		}

		Metrics::SetTickRawPoses(refT, refQ, tgtT, tgtQ, phase);
	}

	Metrics::WriteLogEntry();
		
	if (CalCtx.state != CalibrationState::Continuous) {
		ctx.state = CalibrationState::None;
		calibration.Clear();
	}
	else {
		size_t drop_samples = CalCtx.SampleCount() / 10;
		for (int i = 0; i < drop_samples; i++) {
			calibration.ShiftSample();
		}
	}
}

void LoadChaperoneBounds()
{
	vr::VRChaperoneSetup()->RevertWorkingCopy();

	uint32_t quadCount = 0;
	vr::VRChaperoneSetup()->GetLiveCollisionBoundsInfo(nullptr, &quadCount);

	CalCtx.chaperone.geometry.resize(quadCount);
	vr::VRChaperoneSetup()->GetLiveCollisionBoundsInfo(&CalCtx.chaperone.geometry[0], &quadCount);
	vr::VRChaperoneSetup()->GetWorkingStandingZeroPoseToRawTrackingPose(&CalCtx.chaperone.standingCenter);
	vr::VRChaperoneSetup()->GetWorkingPlayAreaSize(&CalCtx.chaperone.playSpaceSize.v[0], &CalCtx.chaperone.playSpaceSize.v[1]);
	CalCtx.chaperone.valid = true;
}

void ApplyChaperoneBounds()
{
	vr::VRChaperoneSetup()->RevertWorkingCopy();
	vr::VRChaperoneSetup()->SetWorkingCollisionBoundsInfo(&CalCtx.chaperone.geometry[0], (uint32_t)CalCtx.chaperone.geometry.size());
	vr::VRChaperoneSetup()->SetWorkingStandingZeroPoseToRawTrackingPose(&CalCtx.chaperone.standingCenter);
	vr::VRChaperoneSetup()->SetWorkingPlayAreaSize(CalCtx.chaperone.playSpaceSize.v[0], CalCtx.chaperone.playSpaceSize.v[1]);
	vr::VRChaperoneSetup()->CommitWorkingCopy(vr::EChaperoneConfigFile_Live);
}

void DebugApplyRandomOffset() {
	protocol::Request req(protocol::RequestDebugOffset);
	Driver.SendBlocking(req);
}

int GetWatchdogResetCount() {
	return calibration.m_watchdogResets;
}

// Math derivation (so the next person editing this can double-check):
//
//   SZP is the standing-universe origin's pose expressed in the raw
//   tracking universe. SZP = [R_szp | t_szp; 0 | 1]. A device's pose in
//   the standing universe is then:
//
//     standing_pose = SZP^-1 * raw_pose
//                   -> translation = R_szp^-1 * (raw_pos - t_szp)
//
//   We want the user's HMD standing-pose translation to become (0, y, 0)
//   in X/Z while preserving Y. Setting:
//
//     t_szp_new = t_szp_old + R_szp * (hmd_standing_x, 0, hmd_standing_z)
//
//   makes hmd_standing_new = hmd_standing_old - (hmd_standing_x, 0, hmd_standing_z)
//                          = (0, hmd_standing_y, 0).
//
//   For the common case where SZP's rotation is yaw-only (room
//   calibration's typical state), R_szp * (x, 0, z) is just (x, 0, z)
//   rotated about the Y axis -- so we have to apply the rotation, not
//   just add the X/Z components. (An earlier draft of this function got
//   that wrong; the addition only happened to be correct when yaw was
//   exactly 0.)
//
// Returns true on success. Failures: VRChaperoneSetup unavailable, HMD
// pose not valid.
bool RecenterPlayspaceToCurrentHmd() {
	if (!vr::VRSystem() || !vr::VRChaperoneSetup()) return false;

	// Read HMD's current pose in the standing universe. With predictionSecs=0
	// we get the "now" pose -- no extrapolation, which is what we want for
	// a one-shot snapshot anchor.
	vr::TrackedDevicePose_t poses[1] = {};
	vr::VRSystem()->GetDeviceToAbsoluteTrackingPose(
		vr::TrackingUniverseStanding, 0.0f, poses, 1);
	if (!poses[0].bPoseIsValid
		|| poses[0].eTrackingResult != vr::TrackingResult_Running_OK) {
		CalCtx.Log("Recenter playspace: HMD pose not valid; aborting.\n");
		return false;
	}
	const auto& hmdMat = poses[0].mDeviceToAbsoluteTracking;
	const float hmdStandingX = hmdMat.m[0][3];
	const float hmdStandingZ = hmdMat.m[2][3];

	// Defensive: throw away any pending working copy so we read the live
	// state, not whatever some other code path is in the middle of editing.
	// Mirrors what ApplyChaperoneBounds does.
	vr::VRChaperoneSetup()->RevertWorkingCopy();

	vr::HmdMatrix34_t szp{};
	vr::VRChaperoneSetup()->GetWorkingStandingZeroPoseToRawTrackingPose(&szp);

	// Delta = R_szp * (hmd_standing_x, 0, hmd_standing_z). Use the X/Z columns
	// of R_szp's row-major 3x3 (the [_][0] and [_][2] columns).
	const float dx = szp.m[0][0] * hmdStandingX + szp.m[0][2] * hmdStandingZ;
	const float dz = szp.m[2][0] * hmdStandingX + szp.m[2][2] * hmdStandingZ;
	// Y delta would be szp.m[1][0] * hmdStandingX + szp.m[1][2] * hmdStandingZ.
	// For yaw-only SZP that's 0 (Y-axis row is (0, 1, 0)). For tilted SZPs
	// it's small and cancels naturally if the user wants Y preserved.
	// Either way we do NOT modify Y to keep floor calibration intact.
	const float oldTx = szp.m[0][3];
	const float oldTz = szp.m[2][3];

	szp.m[0][3] = oldTx + dx;
	szp.m[2][3] = oldTz + dz;

	vr::VRChaperoneSetup()->SetWorkingStandingZeroPoseToRawTrackingPose(&szp);
	vr::VRChaperoneSetup()->CommitWorkingCopy(vr::EChaperoneConfigFile_Live);

	// Log enough that a debug-log reader can verify the action did what was
	// intended: HMD pose pre-shift, delta applied, expected post-shift HMD
	// position (~0, y, ~0 in X/Z if the math is right).
	char logbuf[256];
	snprintf(logbuf, sizeof logbuf,
		"playspace_recenter: hmd_standing=(%.3f, %.3f, %.3f) -> "
		"szp_translation_delta=(%.3f, 0, %.3f); szp_translation %.3f,%.3f -> %.3f,%.3f",
		hmdStandingX, hmdMat.m[1][3], hmdStandingZ,
		dx, dz,
		oldTx, oldTz, szp.m[0][3], szp.m[2][3]);
	CalCtx.Log(logbuf);
	Metrics::WriteLogAnnotation(logbuf);

	return true;
}
