#include "Calibration.h"
#include "CalibrationProgress.h"
#include "CalibrationOneShotDiagnostics.h"
#include "CalibrationInternal.h"
#include "CalibrationDevicePoseUtils.h"
#include "CalibrationPoseSampling.h"
#include "CalibrationProfileApply.h"
#include "CalibrationRejectReason.h"
#include "CalibrationWatchdogs.h"
#include "CalibrationHeadMountShadow.h"
#include "CalibrationMetrics.h"
#include "Configuration.h"
#include "ContinuousPersistDecision.h"
#include "IPCClient.h"
#include "CalibrationCalc.h"
#include "VRState.h"
#include "GravityAlignment.h"      // spacecal::gravity::TiltAngleDeg -- seed/heartbeat tilt diagnostics
#include "MotionGate.h"            // ShouldBlendCycle -- profile-apply snap decision
#include "ControllerInput.h"
#include "HeadFromTrackerSolve.h"
#include "HeadMountOffsetModal.h" // wkopenvr::headmount::FeedSolverTick -- offset modal solver feed.
#include "HeadMountPoseSampling.h"
#include "HeadMountShadowOffset.h"
#include "HeadMountSourceGuard.h"
#include "HeadMountTargetBinding.h"
#include "TrackingStyle.h"
#include "UserInterfaceHeadMount.h"
#include "RotationMatrix3.h" // AngleFromRotationMatrix3 / AxisFromRotationMatrix3 (clamped).

#include <string>
#include <vector>
#include <iostream>
#include <algorithm>
#include <map>
#include <cmath>
#include <cstring>

#include <Eigen/Dense>
#include <GLFW/glfw3.h>

CalibrationContext CalCtx;

static const char* CalibrationStateName(CalibrationState state)
{
	switch (state) {
		case CalibrationState::None:
			return "None";
		case CalibrationState::Begin:
			return "Begin";
		case CalibrationState::Rotation:
			return "Rotation";
		case CalibrationState::Translation:
			return "Translation";
		case CalibrationState::Editing:
			return "Editing";
		case CalibrationState::Continuous:
			return "Continuous";
		case CalibrationState::ContinuousStandby:
			return "ContinuousStandby";
		default:
			return "Unknown";
	}
}

static bool RestoreCalibrationSolverFromProfile(CalibrationContext& ctx)
{
	if (!ctx.validProfile) {
		return false;
	}

	calibration.SeedEstimatedTransformation(ProfileTransform(ctx.calibratedRotation, ctx.calibratedTranslation),
	                                        /*annotate=*/false);
	calibration.setRelativeTransformation(ctx.refToTargetPose, ctx.relativePosCalibrated);
	calibration.lockRelativePosition = ctx.lockRelativePosition;
	return true;
}

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
struct CpuPressureState
{
	bool initialized = false;
	bool alarmed = false;
	uint64_t lastCpuNs = 0;
	uint64_t lastWallNs = 0;
	double emaPct = 0.0; // 0..100 in single-core-equivalent percent
	int logicalProcessors = 1;
};
static CpuPressureState g_cpuPressureState;

constexpr double kCpuPressureOnThresholdPct = 50.0;
constexpr double kCpuPressureOffThresholdPct = 30.0;
constexpr double kCpuPressureEmaTimeConstSec = 5.0;
constexpr double kCpuPressureSpikeMs = 200.0; // single-tick spike threshold

inline uint64_t FileTimeToNs100(const FILETIME& ft)
{
	return ((uint64_t)ft.dwHighDateTime << 32) | ft.dwLowDateTime; // units of 100 ns
}

// Samples this tick's CPU usage + folds into the EMA. Emits transition
// annotations on EMA crossing and a one-shot `cpu_pressure_spike` on any
// single ComputeIncremental that took >= kCpuPressureSpikeMs.
static void TickCpuPressureMonitor(double computationTimeMs, double now_s)
{
	auto& s = g_cpuPressureState;
	FILETIME ftCreate, ftExit, ftKernel, ftUser;
	if (!GetProcessTimes(GetCurrentProcess(), &ftCreate, &ftExit, &ftKernel, &ftUser)) {
		return; // bail silently; this is diagnostic only
	}
	const uint64_t cpuNow = FileTimeToNs100(ftKernel) + FileTimeToNs100(ftUser);
	LARGE_INTEGER pcNow;
	QueryPerformanceCounter(&pcNow);
	LARGE_INTEGER pcFreq;
	QueryPerformanceFrequency(&pcFreq);
	const uint64_t wallNow = (uint64_t)((pcNow.QuadPart * 10'000'000ull) / pcFreq.QuadPart); // 100ns

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

	const double instPct = 100.0 * (double)cpuDelta / ((double)wallDelta * (double)s.logicalProcessors);

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
		         " state=%d",
		         s.emaPct, instPct, s.logicalProcessors, (int)CalCtx.state);
		Metrics::WriteLogAnnotation(buf);
	}
	else if (s.alarmed && s.emaPct < kCpuPressureOffThresholdPct) {
		s.alarmed = false;
		char buf[128];
		snprintf(buf, sizeof buf, "cpu_pressure_warning_off: ema_pct=%.1f", s.emaPct);
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
		         " state=%d",
		         computationTimeMs, s.emaPct, (int)CalCtx.state);
		Metrics::WriteLogAnnotation(buf);
	}
}

// One-shot snap flag: the next ScanAndApplyProfile cycle sends every device
// transform with payload.lerp=false so the driver applies it without blending.
// Set by the head-mount shadow/offset paths for step changes that must not be
// smoothed. Cleared by ScanAndApplyProfile after consuming.
bool g_snapNextProfileApply = false;

// AdditionalCalibration's special members live inline in the header now --
// CalibrationCalc is complete at the include point, so the implicitly-defined
// destructor handles the unique_ptr just fine.

// Wall-time stage marks for the [cal-tick-slow] breakdown. Reset at the top
// of each full tick pass; the diagnostic only fires at the end of a full
// pass, so aborted ticks never report stale stages.
static double g_tickGatesMs = 0.0;
static double g_tickDetectMs = 0.0;
static double g_tickSampleMs = 0.0;

static double QpcMsBetween(const LARGE_INTEGER& from, const LARGE_INTEGER& to)
{
	LARGE_INTEGER freq;
	QueryPerformanceFrequency(&freq);
	if (freq.QuadPart <= 0) return 0.0;
	return (to.QuadPart - from.QuadPart) * 1000.0 / (double)freq.QuadPart;
}

double ComputeHmdSpeedMps(const CalibrationContext& ctx);

Eigen::Affine3d CalibrationTransformFromContext(const CalibrationContext& ctx)
{
	const Eigen::Vector3d euler = ctx.calibratedRotation * EIGEN_PI / 180.0;
	const Eigen::Quaterniond rot = Eigen::AngleAxisd(euler(0), Eigen::Vector3d::UnitZ()) *
	                               Eigen::AngleAxisd(euler(1), Eigen::Vector3d::UnitY()) *
	                               Eigen::AngleAxisd(euler(2), Eigen::Vector3d::UnitX());
	return Eigen::Translation3d(ctx.calibratedTranslation * 0.01) * rot;
}

double ComputeHmdSpeedMps(const CalibrationContext& ctx)
{
	const auto& hmd = ctx.devicePoses[vr::k_unTrackedDeviceIndex_Hmd];
	return std::sqrt(hmd.vecVelocity[0] * hmd.vecVelocity[0] + hmd.vecVelocity[1] * hmd.vecVelocity[1] +
	                 hmd.vecVelocity[2] * hmd.vecVelocity[2]);
}

void CalibrationContext::ResolveLockMode()
{
	const bool prev = lockRelativePosition;
	lockRelativePosition = ResolveLockRelativePositionValue(lockRelativePositionMode);
	// Diagnostic: annotate every resolved-value change. The UI-side toggle of
	// "Lock relative position" is invisible in post-session logs unless we
	// trace the resolve step; without this a user-reported "I toggled Lock
	// and nothing happened" cannot be distinguished from "the toggle did
	// take effect but didn't help."
	if (prev != lockRelativePosition) {
		char buf[200];
		snprintf(buf, sizeof buf, "lockRelativePosition_change: prev=%d now=%d mode=%d", (int)prev,
		         (int)lockRelativePosition, (int)lockRelativePositionMode);
		Metrics::WriteLogAnnotation(buf);
	}
}

SCIPCClient Driver;
protocol::DriverPoseShmem shmem;

void SendFreezeAllTracking()
{
	protocol::Request req(protocol::RequestSetFreezeAllTracking);
	req.freeze.frozen = CalCtx.freezeAllTracking;
	req.freeze.includeHmd = CalCtx.freezeIncludeHmd;
	try {
		Driver.SendBlocking(req);
	}
	catch (const std::exception& e) {
		char buf[200];
		std::snprintf(buf, sizeof buf, "[freeze] state push failed: %s", e.what());
		Metrics::WriteLogAnnotation(buf);
	}
}

// Global freeze hotkey + heartbeat. Runs every CalibrationTick (before the tick's
// pose-reactive work) so the hotkey works regardless of which tab is shown and
// even when another app has focus (GetAsyncKeyState reads global key state). While
// frozen, resend the state at ~1 Hz so the driver fails open to live tracking if
// this overlay dies. Default chord: Ctrl+Alt+F.
static void TickFreezeAllTracking(CalibrationContext& ctx, double time)
{
	const bool chordDown = (GetAsyncKeyState(VK_CONTROL) & 0x8000) && (GetAsyncKeyState(VK_MENU) & 0x8000) &&
	                       (GetAsyncKeyState('F') & 0x8000);
	static bool s_chordDownLast = false;
	if (chordDown && !s_chordDownLast) {
		ctx.freezeAllTracking = !ctx.freezeAllTracking;
		SendFreezeAllTracking();
		Metrics::LogAnnotationf("freeze_all_tracking: source=hotkey frozen=%d include_hmd=%d",
		                        (int)ctx.freezeAllTracking, (int)ctx.freezeIncludeHmd);
	}
	s_chordDownLast = chordDown;

	if (ctx.freezeAllTracking) {
		static double s_lastHeartbeat = -1e9;
		if (time - s_lastHeartbeat >= 1.0) {
			s_lastHeartbeat = time;
			SendFreezeAllTracking();
		}
	}
}

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
	// Restoring the trio of resets dropped during the modularization refactor.
	// Without these, clicking "Start Calibration" cleared messages and the
	// sample buffer but left state == None, so CalibrationTick returned at the
	// "if (state == None)" early-exit before CollectSample could push any
	// Progress message -- the one-shot popup just displayed nothing.
	CalCtx.hasAppliedCalibrationResult = false;
	AssignTargets();
	CalCtx.state = CalibrationState::Begin;
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
	// last-cycle samples into the new cycle's rolling window.
	Metrics::error_currentCal.Clear();
	Metrics::error_byRelPose.Clear();
	Metrics::error_rawComputed.Clear();
	Metrics::jitterRef.Clear();
	Metrics::jitterTarget.Clear();
	Metrics::posOffset_currentCal.Clear();
	Metrics::posOffset_byRelPose.Clear();
	Metrics::posOffset_rawComputed.Clear();

	char resetBuf[240];
	snprintf(resetBuf, sizeof resetBuf,
	         "StartCalibration_state_reset: reason=%s pairedMotionPrevRefPos pairedMotionPrevTgtPos errSeries_cleared=1",
	         (reason && reason[0]) ? reason : "unknown");
	Metrics::WriteLogAnnotation(resetBuf);
}

void StartContinuousCalibration(const char* reason)
{
	CalCtx.hasAppliedCalibrationResult = false;
	CalCtx.continuousStartSnapshot = CalCtx.CaptureProfileSnapshot();
	CalCtx.lastAcceptedContinuousSnapshot = {};
	AssignTargets();
	if (CalCtx.headMount.mode != HeadMountMode::Off || !CalCtx.headMount.trackerSerial.empty()) {
		if (wkopenvr::headmount::BindHeadMountToContinuousTarget(CalCtx)) {
			Metrics::WriteLogAnnotation("[head-mount] bound config to continuous target");
		}
	}
	StartCalibration(reason);
	CalCtx.state = CalibrationState::Continuous;
	if (CalCtx.validProfile) {
		calibration.SeedEstimatedTransformation(
		    ProfileTransform(CalCtx.calibratedRotation, CalCtx.calibratedTranslation));
		const double magCm = CalCtx.calibratedTranslation.norm();
		const double seedTiltDeg = spacecal::gravity::TiltAngleDeg(
		    Eigen::Quaterniond(ProfileTransform(CalCtx.calibratedRotation, CalCtx.calibratedTranslation).rotation()));
		char seedBuf[340];
		snprintf(
		    seedBuf, sizeof seedBuf,
		    "StartContinuousCalibration_seed_profile: trans_cm=(%.2f,%.2f,%.2f) mag_cm=%.2f rot_deg=(%.3f,%.3f,%.3f)"
		    " tilt_deg=%.2f",
		    CalCtx.calibratedTranslation.x(), CalCtx.calibratedTranslation.y(), CalCtx.calibratedTranslation.z(), magCm,
		    CalCtx.calibratedRotation.x(), CalCtx.calibratedRotation.y(), CalCtx.calibratedRotation.z(), seedTiltDeg);
		Metrics::WriteLogAnnotation(seedBuf);
	}
	else {
		Metrics::WriteLogAnnotation("StartContinuousCalibration_seed_profile: skipped validProfile=0");
	}
	calibration.setRelativeTransformation(CalCtx.refToTargetPose, CalCtx.relativePosCalibrated);
	calibration.lockRelativePosition = CalCtx.lockRelativePosition;
	if (CalCtx.lockRelativePosition) {
		CalCtx.Log("Relative position locked");
	}
	else {
		CalCtx.Log("Collecting initial samples...");
	}
	char startBuf[280];
	snprintf(startBuf, sizeof startBuf,
	         "StartContinuousCalibration: reason=%s snapshot_valid=%d validProfile=%d relPosCal=%d lockRelPos=%d",
	         (reason && reason[0]) ? reason : "unknown", (int)CalCtx.continuousStartSnapshot.validProfile,
	         (int)CalCtx.validProfile, (int)CalCtx.continuousStartSnapshot.relativePosCalibrated,
	         (int)CalCtx.lockRelativePosition);
	Metrics::WriteLogAnnotation(startBuf);
}

void CancelCalibration(const char* reason)
{
	if (CalCtx.state != CalibrationState::Begin && CalCtx.state != CalibrationState::Rotation &&
	    CalCtx.state != CalibrationState::Translation) {
		return;
	}

	CalCtx.state = CalibrationState::None;
	CalCtx.wantedUpdateInterval = 1.0;
	CalCtx.messages.clear();
	CalCtx.pairedMotionPosSeeded = false;
	CalCtx.pairedMotionMismatchCount = 0;
	CalCtx.pairedMotionPrevRefPos = Eigen::Vector3d::Zero();
	CalCtx.pairedMotionPrevTgtPos = Eigen::Vector3d::Zero();
	calibration.Clear();

	char buf[160];
	snprintf(buf, sizeof buf, "CancelCalibration: reason=%s", (reason && reason[0]) ? reason : "unknown");
	Metrics::WriteLogAnnotation(buf);
}

void EndContinuousCalibration()
{
	const bool hadAccepted = CalCtx.lastAcceptedContinuousSnapshot.captured;
	const bool hadStart = CalCtx.continuousStartSnapshot.captured;
	CalibrationProfileSnapshot selected =
	    hadAccepted ? CalCtx.lastAcceptedContinuousSnapshot : CalCtx.continuousStartSnapshot;
	CalCtx.state = CalibrationState::None;
	CalCtx.wantedUpdateInterval = 1.0;
	if (selected.captured) {
		CalCtx.RestoreProfileSnapshot(selected);
		if (CalCtx.validProfile) {
			SaveProfile(CalCtx);
			ScanAndApplyProfile(CalCtx);
		}
	}
	CalCtx.continuousStartSnapshot = {};
	CalCtx.lastAcceptedContinuousSnapshot = {};
	// The selected snapshot above already persisted the final offset, so no
	// throttled save is still pending.
	CalCtx.continuousSaveDirty = false;
	char endBuf[240];
	snprintf(endBuf, sizeof endBuf, "EndContinuousCalibration: selected=%s start_snapshot=%d relPosCal=%d valid=%d",
	         hadAccepted ? "last_accepted" : (hadStart ? "entry" : "none"), (int)hadStart,
	         (int)CalCtx.relativePosCalibrated, (int)CalCtx.validProfile);
	Metrics::WriteLogAnnotation(endBuf);
}

void FlushPendingContinuousSave()
{
	if (!CalCtx.continuousSaveDirty) {
		return;
	}
	if (CalCtx.validProfile) {
		// Same oversized-delta guard as the in-session persist path: a value
		// that never proved itself must not reach the registry on the way
		// out, or it poisons every later launch. The previously persisted
		// profile stays on disk instead.
		const double deltaCm = (CalCtx.calibratedTranslation - CalCtx.lastPersistedContinuousTranslation).norm();
		const double dwellSec =
		    (CalCtx.anomalousPersistFirstSeen > 0.0) ? (glfwGetTime() - CalCtx.anomalousPersistFirstSeen) : 0.0;
		if (spacecal::persist::ShouldDeferAnomalousPersist(deltaCm, dwellSec, CalCtx.anomalousPersistAgreeCount)) {
			CalCtx.continuousSaveDirty = false;
			Metrics::LogAnnotationf("[profile-save][flush-skipped] reason=oversized_unproven delta_cm=%.2f"
			                        " dwell_sec=%.2f agreeing=%d",
			                        deltaCm, dwellSec, CalCtx.anomalousPersistAgreeCount);
			return;
		}
		SaveProfile(CalCtx);
		CalCtx.lastContinuousSaveTime = Metrics::CurrentTime;
		CalCtx.lastPersistedContinuousTranslation = CalCtx.calibratedTranslation;
	}
	CalCtx.continuousSaveDirty = false;
	Metrics::WriteLogAnnotation("[profile-save][flush] reason=continuous_pending_on_shutdown");
}

static void TickDeviceRescan(CalibrationContext& ctx, double time)
{
	// Device rescan: runs in every state at 1 Hz. AssignTargets is the only
	// site that resolves targetID and headMount.deviceID by serial; without
	// an idle-state rescan, a continuous target picked on the Basic tab
	// stayed at id=-1 (and therefore the Headset-tab status line stayed red)
	// until the user clicked Start. AssignTargets is idempotent: the
	// reference/target re-resolve branches gate on id < 0, the head-mount
	// branch only touches head-mount fields, and controllerIDs reseats
	// from state.devices each call. The 1 Hz cadence matches the existing
	// ScanAndApplyProfile timer below so we are not introducing new VRState
	// load overhead beyond what already runs at this rate.
	if ((time - ctx.timeLastAssign) >= 1.0) {
		ctx.timeLastAssign = time;
		AssignTargets();
	}
}

static void TickDriverTelemetryDeltas(CalibrationContext& ctx)
{
	// Sample driver-side telemetry counters and push the per-tick deltas (in Hz)
	// into the metrics time series. Initialize the prior snapshot lazily on the
	// first valid sample so the first delta is zero rather than a huge spike
	// representing the entire driver-uptime accumulation.
	{
		static bool s_telemetryPrimed = false;
		static uint64_t s_lastFallback = 0, s_lastPerId = 0, s_lastQuash = 0;
		static uint64_t s_lastSynthFallback = 0;
		static double s_lastTelemetryTime = 0;

		uint64_t fallback = 0, perId = 0, quash = 0, synthFallback = 0;
		if (shmem.GetTelemetry(fallback, perId, quash, synthFallback)) {
			ctx.driverSynthFallbackTotal = synthFallback;
			Metrics::RecordTimestamp();
			double now = Metrics::CurrentTime;
			if (!s_telemetryPrimed) {
				s_telemetryPrimed = true;
				s_lastFallback = fallback;
				s_lastPerId = perId;
				s_lastQuash = quash;
				s_lastSynthFallback = synthFallback;
				s_lastTelemetryTime = now;
			}
			else {
				double dt = now - s_lastTelemetryTime;
				if (dt > 1e-6) {
					Metrics::fallbackApplyRate.Push((fallback - s_lastFallback) / dt);
					Metrics::perIdApplyRate.Push((perId - s_lastPerId) / dt);
					Metrics::quashApplyRate.Push((quash - s_lastQuash) / dt);
				}
				// driverSynthFallbackCount is a monotonic session total, not a rate.
				if (synthFallback != s_lastSynthFallback) {
					Metrics::driverSynthFallbackCount.Push(static_cast<uint32_t>(synthFallback));
				}
				s_lastFallback = fallback;
				s_lastPerId = perId;
				s_lastQuash = quash;
				s_lastSynthFallback = synthFallback;
				s_lastTelemetryTime = now;
			}
		}
	}
}

static bool TickHmdStallGuard(CalibrationContext& ctx, double time)
{
	// check for non-updating headset tracking space (caused by quest out of bounds or taken off head for example) and
	// abort everything for this tick
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
			         "[hmd-stall][entered] t=%.3f hmd_pos=(%.4f,%.4f,%.4f) prev=(%.4f,%.4f,%.4f)", time, p[0], p[1],
			         p[2], (double)ctx.xprev, (double)ctx.yprev, (double)ctx.zprev);
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
		return true;
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
		snprintf(buf, sizeof buf, "hmd_stall_recovered after %d ticks (approx %.2fs at 90Hz, t=%.3f)",
		         ctx.consecutiveHmdStalls, approxDurSec, time);
		Metrics::WriteLogAnnotation(buf);
	}
	ctx.consecutiveHmdStalls = 0;
	ctx.xprev = (float)p[0];
	ctx.yprev = (float)p[1];
	ctx.zprev = (float)p[2];
	return false;
}

static void TickHeadMountSolverFeed(CalibrationContext& ctx)
{
	// Feed the head-mount offset solver with fresh pose pairs. The modal's
	// Solver is a no-op when not in Collecting state, so this call is cheap on
	// every tick that doesn't have an active calibration modal.
	// Log edge: modal is open but the tracker hasn't resolved yet (common root
	// cause: user opened modal before picking a mode or before the tracker
	// appeared in VRState).
	{
		static bool s_noDeviceWhileModalOpen = false;
		const bool modalOpen = wkopenvr::headmount::OffsetModalIsOpen();
		const bool noDevice = ctx.headMount.deviceID < 0;
		if (modalOpen && noDevice && !s_noDeviceWhileModalOpen) {
			s_noDeviceWhileModalOpen = true;
			Metrics::WriteLogAnnotation("[head-mount-solver] modal open but deviceID=-1 (tracker not resolved)");
		}
		else if (!modalOpen || !noDevice) {
			s_noDeviceWhileModalOpen = false;
		}
	}
	if (ctx.headMount.deviceID >= 0 && (uint32_t)ctx.headMount.deviceID < vr::k_unMaxTrackedDeviceCount) {
		const vr::DriverPose_t& hmRaw = ctx.devicePoses[ctx.headMount.deviceID];
		const vr::DriverPose_t& hmdRaw = ctx.devicePoses[vr::k_unTrackedDeviceIndex_Hmd];
		if (hmRaw.poseIsValid && hmRaw.result == vr::ETrackingResult::TrackingResult_Running_OK && hmdRaw.poseIsValid) {
			{
				static bool s_feedLogged = false;
				if (!s_feedLogged) {
					s_feedLogged = true;
					char fbuf[128];
					snprintf(fbuf, sizeof fbuf, "[head-mount-solver] feeding pose pairs: deviceID=%d",
					         (int)ctx.headMount.deviceID);
					Metrics::WriteLogAnnotation(fbuf);
				}
			}
			// Build world-space poses from DriverPose_t fields.
			auto poseFromDriver = [](const vr::DriverPose_t& raw) -> Eigen::Affine3d {
				const Eigen::Quaterniond wfd(raw.qWorldFromDriverRotation.w, raw.qWorldFromDriverRotation.x,
				                             raw.qWorldFromDriverRotation.y, raw.qWorldFromDriverRotation.z);
				const Eigen::Quaterniond localRot(raw.qRotation.w, raw.qRotation.x, raw.qRotation.y, raw.qRotation.z);
				const Eigen::Vector3d wfdTrans(raw.vecWorldFromDriverTranslation[0],
				                               raw.vecWorldFromDriverTranslation[1],
				                               raw.vecWorldFromDriverTranslation[2]);
				const Eigen::Vector3d localPos(raw.vecPosition[0], raw.vecPosition[1], raw.vecPosition[2]);
				return Eigen::Translation3d(wfdTrans + wfd * localPos) * (wfd * localRot).normalized();
			};
			const Eigen::Affine3d trackerWorld = poseFromDriver(hmRaw);
			const Eigen::Affine3d hmdWorld = poseFromDriver(hmdRaw);
			Eigen::Affine3d targetFromReference = Eigen::Affine3d::Identity();
			if (ctx.validProfile) {
				targetFromReference = CalibrationTransformFromContext(ctx).inverse();
			}
			const double hmdSpeed = ComputeHmdSpeedMps(ctx);
			wkopenvr::headmount::FeedSolverTick(hmdWorld, trackerWorld, targetFromReference, ctx.validProfile,
			                                    hmdSpeed);
		}
	}
}

static void TickDriverSynthContinuousStatus(CalibrationContext& ctx, double time)
{
	if (ctx.state == CalibrationState::Continuous && ctx.headMount.mode == HeadMountMode::DriverSynth) {
		const bool targetMatches = wkopenvr::headmount::HeadMountMatchesContinuousTarget(ctx);
		const auto synthStatus = spacecal::headmount::EvaluateDriverSynthContinuousStatus(
		    ctx.headMount,
		    /*inContinuous=*/true, targetMatches, ctx.devicePoses, vr::k_unMaxTrackedDeviceCount);

		static bool s_lastDriverSynthReady = false;
		static double s_lastDriverSynthStatusLog = -1e9;
		static std::string s_lastDriverSynthReason;
		const bool reasonChanged = s_lastDriverSynthReason != synthStatus.reason;
		const bool shouldLog =
		    reasonChanged || synthStatus.ready != s_lastDriverSynthReady || (time - s_lastDriverSynthStatusLog) >= 5.0;

		if (shouldLog) {
			double calibratedProxyDeltaM = -1.0;
			if (synthStatus.ready && ctx.validProfile) {
				const vr::DriverPose_t& trackerPose = ctx.devicePoses[ctx.headMount.deviceID];
				const vr::DriverPose_t& hmdPose = ctx.devicePoses[vr::k_unTrackedDeviceIndex_Hmd];
				const Eigen::AffineCompact3d proxyHeadRaw =
				    spacecal::headmount::ComputeHeadWorldPose(trackerPose, ctx.headMount.headFromTracker);
				const Eigen::AffineCompact3d calibratedProxyHead =
				    ProfileTransform(ctx.calibratedRotation, ctx.calibratedTranslation) * proxyHeadRaw;
				const Pose hmdRaw = ConvertPose(hmdPose);
				calibratedProxyDeltaM = (hmdRaw.trans - calibratedProxyHead.translation()).norm();
				if (!std::isfinite(calibratedProxyDeltaM)) {
					calibratedProxyDeltaM = -1.0;
				}
			}

			char sbuf[480];
			snprintf(sbuf, sizeof sbuf,
			         "continuous_solve_running: mode=driver_synth synth_ready=%d reason=%s"
			         " target_matches=%d deviceID=%d profile_valid=%d"
			         " hmd_proxy_raw_delta_cm=%.2f hmd_proxy_calibrated_delta_cm=%.2f",
			         (int)synthStatus.ready, synthStatus.reason, (int)targetMatches, (int)ctx.headMount.deviceID,
			         (int)ctx.validProfile,
			         synthStatus.hmdProxyDeltaM >= 0.0 ? synthStatus.hmdProxyDeltaM * 100.0 : -1.0,
			         calibratedProxyDeltaM >= 0.0 ? calibratedProxyDeltaM * 100.0 : -1.0);
			Metrics::WriteLogAnnotation(sbuf);
			s_lastDriverSynthStatusLog = time;
			s_lastDriverSynthReason = synthStatus.reason;
		}
		s_lastDriverSynthReady = synthStatus.ready;
	}
}

static bool TickOneShotMotionVarietyGate()
{
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
		constexpr double kPhaseDiversity = spacecal::calibration_progress::kOneShotRotationReadyDiversity;
		if (calibration.RotationDiversity() < kPhaseDiversity) {
			calibration.ShiftSample();
			return true;
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
		return true;
	}
	if (CalCtx.state == CalibrationState::Translation) {
		// Lowered from 0.70 (2026-05-13): combined with kDesiredAxisRange=0.20m
		// the 70% gate demanded 21cm per axis, which a tracker rigidly mounted
		// to an HMD struggles to hit on the weakest axis via normal head movement.
		// 0.55 * 0.20m = 11cm per axis -- achievable with a deliberate nod and
		// lateral lean. The math gates in ComputeOneshot still reject genuinely
		// under-constrained solutions; this only speeds up the collection trigger.
		constexpr double kPhaseDiversity = spacecal::calibration_progress::kOneShotTranslationReadyDiversity;
		if (calibration.TranslationDiversity() < kPhaseDiversity) {
			calibration.ShiftSample();
			return true;
		}
		// Translation diversity satisfied -- fall through to ComputeOneshot
		// below. ComputeOneshot's RotationFreezeSplice will prepend the
		// frozen rotation samples for the duration of the solve.
	}
	return false;
}

static bool TickTriggerHoldGate()
{
	if (CalCtx.state == CalibrationState::Continuous && CalCtx.requireTriggerPressToApply &&
	    CalCtx.hasAppliedCalibrationResult) {
		bool triggerPressed = true;
		bool sawController = false;
		auto* vrs = vr::VRSystem();
		for (int i = 0; i < CalCtx.MAX_CONTROLLERS; i++) {
			if (CalCtx.controllerIDs[i] >= 0) {
				sawController = true;
				vr::VRControllerState_t state = {};
				const int controllerId = CalCtx.controllerIDs[i];
				if (!vrs || !vrs->GetControllerState(controllerId, &state, sizeof(state)) ||
				    !wkopenvr::controller_input::IsTriggerHeld(vrs, controllerId, state)) {
					triggerPressed = false;
				}
				if (!triggerPressed) {
					break;
				}
			}
		}

		if (sawController && !triggerPressed) {
			CalCtx.Log("Waiting for trigger press...\n");
			CalCtx.wasWaitingForTriggers = true;
			return true;
		}

		if (CalCtx.wasWaitingForTriggers) {
			CalCtx.Log("Triggers pressed, continuing calibration...\n");
			CalCtx.wasWaitingForTriggers = false;
		}
	}
	return false;
}

static void TickAdditionalCalibrations(double time)
{
	const int32_t maxId = (int32_t)vr::k_unMaxTrackedDeviceCount;

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

		Sample s(ConvertPose(refPose), ConvertPose(tgtPose), glfwGetTime());
		extra.calc->PushSample(s);
		while (extra.calc->SampleCount() > CalCtx.SampleCount())
			extra.calc->ShiftSample();

		// Resolve effective lock for this extra.
		switch (extra.lockMode) {
			case 0:
				extra.lockRelativePosition = false;
				break;
			case 1:
				extra.lockRelativePosition = true;
				break;
			default:
				extra.lockRelativePosition = false;
				break;
		}

		extra.calc->lockRelativePosition = extra.lockRelativePosition;
		extra.calc->enableStaticRecalibration = CalCtx.enableStaticRecalibration;

		if (!CalCtx.calibrationPaused && extra.calc->SampleCount() >= CalCtx.SampleCount()) {
			bool extraLerp = false;
			if (extra.calc->ComputeIncremental(extraLerp, CalCtx.continuousCalibrationThreshold,
			                                   CalCtx.maxRelativeErrorThreshold, CalCtx.ignoreOutliers)) {
				if (extra.calc->isValid()) {
					extra.calibratedRotation = extra.calc->EulerRotation();
					extra.calibratedTranslation = extra.calc->Transformation().translation() * 100.0;
					extra.refToTargetPose = extra.calc->RelativeTransformation();
					extra.relativePosCalibrated = extra.calc->isRelativeTransformationCalibrated();
					extra.valid = true;
				}
			}
		}
	}
}

static void TickCandidateAcceptAndPersist(CalibrationContext& ctx, double time, bool solveAttempted,
                                          bool solveProducedCandidate)
{
	const bool inContinuousState = ctx.state == CalibrationState::Continuous;
	const bool hasPublishableCandidate = solveProducedCandidate && calibration.isValid();

	if (hasPublishableCandidate) {
		const Eigen::Vector3d candidateTranslationCm = calibration.Transformation().translation() * 100.0;
		const bool firstContinuousCandidate = inContinuousState && !ctx.lastAcceptedContinuousSnapshot.captured;
		const bool hasContinuousStartBaseline =
		    ctx.continuousStartSnapshot.captured && ctx.continuousStartSnapshot.validProfile;
		const double firstContinuousJumpCm =
		    hasContinuousStartBaseline
		        ? (candidateTranslationCm - ctx.continuousStartSnapshot.calibratedTranslation).norm()
		        : 0.0;
		const double candidateErrorM = calibration.LastCandidateErrorM();
		const double solveUncertaintyCm = std::isfinite(candidateErrorM) ? candidateErrorM * 100.0 : 0.0;
		const bool snapFirstContinuousCandidate = spacecal::motiongate::ShouldSnapFirstContinuousCandidate(
		    /*inContinuousState=*/inContinuousState,
		    /*hasAcceptedSnapshot=*/ctx.lastAcceptedContinuousSnapshot.captured,
		    /*hasGuardBaseline=*/hasContinuousStartBaseline,
		    /*jumpCm=*/firstContinuousJumpCm,
		    /*solveUncertaintyCm=*/solveUncertaintyCm);

		if (firstContinuousCandidate) {
			char firstBuf[360];
			snprintf(firstBuf, sizeof firstBuf,
			         "first_continuous_candidate_apply: snap=%d hasBaseline=%d"
			         " jump_cm=%.2f solve_uncertainty_cm=%.2f snap_cap_cm=%.0f start_valid=%d",
			         (int)snapFirstContinuousCandidate, (int)hasContinuousStartBaseline, firstContinuousJumpCm,
			         solveUncertaintyCm, spacecal::motiongate::kFirstContinuousSnapMaxCm,
			         (int)ctx.continuousStartSnapshot.validProfile);
			Metrics::WriteLogAnnotation(firstBuf);
		}

		ctx.calibratedRotation = calibration.EulerRotation();
		ctx.calibratedTranslation = candidateTranslationCm;
		ctx.refToTargetPose = calibration.RelativeTransformation();
		ctx.relativePosCalibrated = calibration.isRelativeTransformationCalibrated();
		if (inContinuousState && ctx.relativePosCalibrated && ctx.headMountNeedsFreshRelativePose) {
			ctx.headMountNeedsFreshRelativePose = false;
		}

		ctx.validProfile = true;

		// Persist throttle (continuous mode only): the in-memory offset above is
		// already current and is republished to the driver via ScanAndApplyProfile
		// below; the registry copy is read only at startup, so persisting a
		// sub-millimetre micro-update every tick is wasted serialization + I/O.
		// One-shot and every other state still save immediately. Continuous saves
		// on a cadence; the lastAcceptedContinuousSnapshot capture below stays
		// per-tick, so EndContinuousCalibration still persists the latest value,
		// and FlushPendingContinuousSave() covers shutdown mid-continuous.
		if (inContinuousState) {
			const double offsetDeltaCm = (ctx.calibratedTranslation - ctx.lastPersistedContinuousTranslation).norm();

			// Oversized-delta guard: a translation implausibly far from the
			// last persisted value must prove itself (dwell or consecutive
			// agreeing attempts) before it may reach the registry. The applied
			// in-memory offset is untouched -- only persistence waits.
			bool deferAnomalousPersist = false;
			if (offsetDeltaCm > spacecal::persist::kDeferDeltaCm) {
				const bool agreesWithPrevious = ctx.anomalousPersistFirstSeen > 0.0 &&
				                                (ctx.calibratedTranslation - ctx.anomalousPersistLastAttempt).norm() <=
				                                    spacecal::persist::kDeferAgreeToleranceCm;
				if (agreesWithPrevious) {
					ctx.anomalousPersistAgreeCount += 1;
				}
				else {
					ctx.anomalousPersistFirstSeen = time;
					ctx.anomalousPersistAgreeCount = 1;
				}
				ctx.anomalousPersistLastAttempt = ctx.calibratedTranslation;
				const double dwellSec = time - ctx.anomalousPersistFirstSeen;
				deferAnomalousPersist = spacecal::persist::ShouldDeferAnomalousPersist(offsetDeltaCm, dwellSec,
				                                                                       ctx.anomalousPersistAgreeCount);
				static double s_lastDeferLog = -1e9;
				if (deferAnomalousPersist && (time - s_lastDeferLog) >= 5.0) {
					s_lastDeferLog = time;
					Metrics::LogAnnotationf("[profile-save][deferred] reason=oversized_delta delta_cm=%.2f"
					                        " dwell_sec=%.2f agreeing=%d need_dwell_sec=%.0f need_agreeing=%d",
					                        offsetDeltaCm, dwellSec, ctx.anomalousPersistAgreeCount,
					                        spacecal::persist::kDeferDwellSec,
					                        spacecal::persist::kDeferAgreeingAttempts);
				}
				else if (!deferAnomalousPersist) {
					Metrics::LogAnnotationf("[profile-save][oversized-allowed] delta_cm=%.2f dwell_sec=%.2f"
					                        " agreeing=%d",
					                        offsetDeltaCm, dwellSec, ctx.anomalousPersistAgreeCount);
				}
			}
			else if (ctx.anomalousPersistFirstSeen > 0.0) {
				ctx.anomalousPersistFirstSeen = 0.0;
				ctx.anomalousPersistAgreeCount = 0;
			}

			if (!deferAnomalousPersist &&
			    spacecal::persist::ShouldPersistContinuous(Metrics::CurrentTime, ctx.lastContinuousSaveTime,
			                                               offsetDeltaCm, firstContinuousCandidate)) {
				SaveProfile(ctx);
				ctx.lastContinuousSaveTime = Metrics::CurrentTime;
				ctx.lastPersistedContinuousTranslation = ctx.calibratedTranslation;
				ctx.continuousSaveDirty = false;
				ctx.anomalousPersistFirstSeen = 0.0;
				ctx.anomalousPersistAgreeCount = 0;
			}
			else {
				ctx.continuousSaveDirty = true;
			}
		}
		else {
			SaveProfile(ctx);
			ctx.continuousSaveDirty = false;
		}

		ScanAndApplyProfile(ctx, snapFirstContinuousCandidate,
		                    snapFirstContinuousCandidate ? "first_continuous_candidate" : nullptr);

		CalCtx.hasAppliedCalibrationResult = true;
		if (ctx.state == CalibrationState::Continuous) {
			ctx.lastAcceptedContinuousSnapshot = ctx.CaptureProfileSnapshot();
		}

		// Accepts land several times a second in steady state and mostly
		// repeat the applied value within solver noise. Log the accepts that
		// moved the calibration a centimetre from the last logged value, plus
		// a 5 s heartbeat so the stream never goes fully quiet.
		{
			static Eigen::Vector3d s_lastLoggedAcceptCm = Eigen::Vector3d::Zero();
			static double s_lastAcceptLogTime = -1e9;
			const double movedCm = (ctx.calibratedTranslation - s_lastLoggedAcceptCm).norm();
			if (movedCm >= 1.0 || (time - s_lastAcceptLogTime) >= 5.0) {
				s_lastLoggedAcceptCm = ctx.calibratedTranslation;
				s_lastAcceptLogTime = time;
				Metrics::LogAnnotationf(
				    "calibration_candidate_accepted: state=%d(%s) source=%s trans_cm=(%.2f,%.2f,%.2f) mag_cm=%.2f "
				    "relPosCal=%d validProfile=%d",
				    (int)ctx.state, CalibrationStateName(ctx.state),
				    calibration.LastComputeUsedRelPose() ? "relpose" : "full", ctx.calibratedTranslation.x(),
				    ctx.calibratedTranslation.y(), ctx.calibratedTranslation.z(), ctx.calibratedTranslation.norm(),
				    (int)ctx.relativePosCalibrated, (int)ctx.validProfile);
			}
		}

		CalCtx.Log("Finished calibration, profile saved\n");
	}
	else {
		if (!inContinuousState || (solveAttempted && !calibration.isValid())) {
			CalCtx.Log("Calibration failed.\n");
		}
		else if (solveAttempted && calibration.isValid() && !solveProducedCandidate) {
			static double s_lastNoCandidateAnnotation = -1e9;
			if (time - s_lastNoCandidateAnnotation >= 5.0) {
				s_lastNoCandidateAnnotation = time;
				const char* rejectReason =
				    Metrics::lastRejectReason.empty() ? "none" : Metrics::lastRejectReason.c_str();
				char skipBuf[360];
				std::snprintf(skipBuf, sizeof skipBuf,
				              "calibration_candidate_skipped: state=%d(%s) reason=no_new_solver_candidate"
				              " source=%s calc_reject_reason=%s sample_count=%zu required=%zu prior_valid=1",
				              (int)ctx.state, CalibrationStateName(ctx.state),
				              calibration.LastComputeUsedRelPose() ? "relpose" : "full", rejectReason,
				              calibration.SampleCount(), CalCtx.SampleCount());
				Metrics::WriteLogAnnotation(skipBuf);
			}
		}
	}
}

void CalibrationTick(double time)
{
	if (!vr::VRSystem()) {
		static double s_lastNoVrSystemLog = -1e9;
		if (time - s_lastNoVrSystemLog >= 5.0) {
			s_lastNoVrSystemLog = time;
			Metrics::WriteLogAnnotation("[tick-skip] reason=no_vrsystem");
		}
		return;
	}

	auto& ctx = CalCtx;

	// Global freeze hotkey + heartbeat runs every frame (before the throttle) so
	// the toggle stays responsive and the driver keeps getting heartbeats. While
	// frozen, hold everything: skip the rest of the pose-reactive tick so the
	// frozen poses aren't misread as tracker dropout (which would otherwise fire a
	// destructive auto-recovery, especially when the HMD is left live).
	TickFreezeAllTracking(ctx, time);
	if (ctx.freezeAllTracking) return;

	if ((time - ctx.timeLastTick) < 0.05) return;

	// Stage clock for the [cal-tick-slow] breakdown at the end of the tick.
	LARGE_INTEGER tickWallStart;
	QueryPerformanceCounter(&tickWallStart);
	LARGE_INTEGER stageMark = tickWallStart;
	g_tickGatesMs = g_tickDetectMs = g_tickSampleMs = 0.0;

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

	TraceRelPoseCalFlips(ctx);

	TickPoseFreshnessWatchdog(ctx, time);

	TickStuckCalWatchdog(ctx, time);

	EmitCalHeartbeat(ctx, time);

	EmitSessionConfigDumpOnce(ctx);

	{
		LARGE_INTEGER now;
		QueryPerformanceCounter(&now);
		g_tickGatesMs = QpcMsBetween(stageMark, now);
		stageMark = now;
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

	if (ctx.state == CalibrationState::Continuous || ctx.state == CalibrationState::ContinuousStandby) {
		ctx.ClearLogOnMessage();
	}

	TickDeviceRescan(ctx, time);

	// External smoothing-tool detection moved to the Smoothing overlay's
	// Tick (Protocol v12, 2026-05-11); its plugin scans on its own 5-second
	// cadence and surfaces the banner inside its Prediction sub-tab.

	{
		LARGE_INTEGER now;
		QueryPerformanceCounter(&now);
		g_tickDetectMs = QpcMsBetween(stageMark, now);
		stageMark = now;
	}

	ctx.timeLastTick = time;
	shmem.ReadNewPoses([&](const protocol::DriverPoseShmem::AugmentedPose& augmented_pose) {
		if (augmented_pose.deviceId >= 0 && augmented_pose.deviceId < (int)vr::k_unMaxTrackedDeviceCount) {
			ctx.devicePoses[augmented_pose.deviceId] = augmented_pose.pose;
			ctx.devicePoseSampleTimes[augmented_pose.deviceId] = augmented_pose.sample_time;
		}
	});

	TickDriverTelemetryDeltas(ctx);

	TickHeadMountSolverFeed(ctx);

	if (TickHmdStallGuard(ctx, time)) {
		return;
	}

	// Run the scan in every state where a profile can be active. Previously the scan
	// was skipped once continuous calibration had a valid result, which meant a tracker
	// powered on mid-session never received its offset until calibration was restarted.
	// Per-ID dedupe inside ScanAndApplyProfile keeps IPC churn near zero when nothing
	// has changed.
	if (ctx.state == CalibrationState::None || ctx.state == CalibrationState::ContinuousStandby ||
	    ctx.state == CalibrationState::Continuous) {
		if ((time - ctx.timeLastScan) >= 1.0) {
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
		static double s_lastNoneStateAnnotation = -1e9;
		if (time - s_lastNoneStateAnnotation >= 5.0) {
			s_lastNoneStateAnnotation = time;
			char noneBuf[512];
			snprintf(noneBuf, sizeof noneBuf,
			         "cal_state_none: validProfile=%d enabled=%d profile_trans_cm=(%.2f,%.2f,%.2f)"
			         " profile_mag_cm=%.2f ref='%s' target='%s' refID=%d targetID=%d"
			         " continuous_snapshot=%d last_accepted=%d",
			         (int)ctx.validProfile, (int)ctx.enabled, ctx.calibratedTranslation.x(),
			         ctx.calibratedTranslation.y(), ctx.calibratedTranslation.z(), ctx.calibratedTranslation.norm(),
			         ctx.referenceTrackingSystem.c_str(), ctx.targetTrackingSystem.c_str(), ctx.referenceID,
			         ctx.targetID, (int)ctx.continuousStartSnapshot.captured,
			         (int)ctx.lastAcceptedContinuousSnapshot.captured);
			Metrics::WriteLogAnnotation(noneBuf);
		}

		ctx.wantedUpdateInterval = 1.0;
		return;
	}

	if (ctx.state == CalibrationState::Editing) {
		ctx.wantedUpdateInterval = 0.1;

		if ((time - ctx.timeLastScan) >= 0.1) {
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
	if (ctx.targetID == -1 || ctx.targetID >= vr::k_unMaxTrackedDeviceCount) {
		CalCtx.Log("Missing target device\n");
		ok = false;
	}

	if (ctx.state == CalibrationState::Begin) {
		char referenceSerial[256], targetSerial[256];
		referenceSerial[0] = targetSerial[0] = 0;
		if (auto* vrSystem = vr::VRSystem()) {
			vrSystem->GetStringTrackedDeviceProperty(ctx.referenceID, vr::Prop_SerialNumber_String, referenceSerial,
			                                         256);
			vrSystem->GetStringTrackedDeviceProperty(ctx.targetID, vr::Prop_SerialNumber_String, targetSerial, 256);
		}

		char buf[256];
		snprintf(buf, sizeof buf, "Reference device ID: %d, serial: %s\n", ctx.referenceID, referenceSerial);
		CalCtx.Log(buf);
		snprintf(buf, sizeof buf, "Target device ID: %d, serial %s\n", ctx.targetID, targetSerial);
		CalCtx.Log(buf);

		ScanAndApplyProfile(ctx);

		// (Removed: the original code pushed position-spread metrics here,
		// in the Begin state, before CollectSample had ever populated
		// calibration.m_samples. The pushed value was always 0.0 from an
		// empty buffer. Those metrics now live at the end of CollectSample.)

		if (!CalCtx.ReferencePoseIsValidSimple()) {
			CalCtx.Log("Reference device is not tracking\n");
			ok = false;
		}

		if (!CalCtx.TargetPoseIsValidSimple()) {
			CalCtx.Log("Target device is not tracking\n");
			ok = false;
		}

		// @TODO: Determine if the tracking is jittery
		if (calibration.ReferenceJitter() > ctx.jitterThreshold) {
			CalCtx.Log("Reference device is not tracking\n");
			ok = false;
		}
		if (calibration.TargetJitter() > ctx.jitterThreshold) {
			CalCtx.Log("Target device is not tracking\n");
			ok = false;
		}

		if (ok) {
			// ResetAndDisableOffsets(ctx.targetID);
			ctx.state = CalibrationState::Rotation;
			ctx.wantedUpdateInterval = 0.0;

			CalCtx.Log("Starting calibration...\n");
			return;
		}
	}

	if (!ok) {
		if (ctx.state != CalibrationState::Continuous) {
			ctx.state = CalibrationState::None;

			CalCtx.Log("Aborting calibration!\n");
		}
		return;
	}

	TickDriverSynthContinuousStatus(ctx, time);

	TickHeadMountSourceTransitionGuard(ctx, time);
	TickHeadMountShadowOffsetEstimator(ctx, time);

	if (!CollectSample(ctx)) {
		return;
	}

	const int sampleProgress = (int)calibration.SampleCount();
	const int sampleTarget = (int)ctx.SampleCount();
	CalCtx.Progress(sampleProgress, sampleTarget);
	spacecal::oneshot::MaybeLogReadiness(ctx, sampleProgress, sampleTarget, calibration.RotationDiversity(),
	                                     calibration.TranslationDiversity(), time);

	if (calibration.SampleCount() < CalCtx.SampleCount()) return;
	while (calibration.SampleCount() > CalCtx.SampleCount())
		calibration.ShiftSample();

	if (TickOneShotMotionVarietyGate()) {
		return;
	}

	if (TickTriggerHoldGate()) {
		return;
	}

	LARGE_INTEGER start_time;
	QueryPerformanceCounter(&start_time);
	g_tickSampleMs = QpcMsBetween(stageMark, start_time);

	bool lerp = false;
	bool solveAttempted = false;
	bool solveProducedCandidate = false;

	if (CalCtx.state == CalibrationState::Continuous) {
		CalCtx.messages.clear();
		calibration.enableStaticRecalibration = CalCtx.enableStaticRecalibration;
		const bool blockStaleRelPose =
		    CalCtx.headMountNeedsFreshRelativePose && CalCtx.lockRelativePosition && !CalCtx.relativePosCalibrated;
		calibration.lockRelativePosition = CalCtx.lockRelativePosition && !blockStaleRelPose;
		if (blockStaleRelPose) {
			static double s_lastHeadMountRelPoseGuardLog = -1e9;
			if ((time - s_lastHeadMountRelPoseGuardLog) >= 1.0) {
				s_lastHeadMountRelPoseGuardLog = time;
				char gbuf[480];
				std::snprintf(gbuf, sizeof gbuf,
				              "head_mount_relpose_guard: bug_condition=1"
				              " reason=fresh_relative_pose_required source=%s"
				              " offset_version=%u lockRel=%d relPosCal=%d"
				              " needsFreshRelPose=%d profile_mag_cm=%.2f",
				              HeadMountSampleSourceName(CurrentHeadMountSampleSource(CalCtx)),
				              (unsigned)CalCtx.headMountOffsetVersion, (int)CalCtx.lockRelativePosition,
				              (int)CalCtx.relativePosCalibrated, (int)CalCtx.headMountNeedsFreshRelativePose,
				              CalCtx.calibratedTranslation.norm());
				Metrics::WriteLogAnnotation(gbuf);
			}
		}

		// User-toggled "Pause updates" from the continuous-cal UI: keep the
		// already-applied driver offset live, skip any new solve cycle so the
		// math doesn't fight the user trying to inspect the current result.
		if (!CalCtx.calibrationPaused) {
			solveAttempted = true;
			solveProducedCandidate = calibration.ComputeIncremental(
			    lerp, CalCtx.continuousCalibrationThreshold, CalCtx.maxRelativeErrorThreshold, CalCtx.ignoreOutliers);
			{
				// Steady-state continuous solves repeat the same outcome
				// several times a second for hours (26k lines in one recorded
				// session). Log when the outcome or reject reason changes,
				// otherwise at most once per 5 s -- transitions carry the
				// signal.
				static double s_lastContinuousSolveAnnotation = -1e9;
				static bool s_lastSolveProduced = false;
				static std::string s_lastSolveRejectReason;
				const bool producedValidCandidate = solveProducedCandidate && calibration.isValid();
				const char* rejectReason =
				    producedValidCandidate
				        ? "none"
				        : (Metrics::lastRejectReason.empty() ? "none" : Metrics::lastRejectReason.c_str());
				const bool outcomeChanged =
				    solveProducedCandidate != s_lastSolveProduced || s_lastSolveRejectReason != rejectReason;
				if (outcomeChanged || time - s_lastContinuousSolveAnnotation >= 5.0) {
					s_lastContinuousSolveAnnotation = time;
					s_lastSolveProduced = solveProducedCandidate;
					s_lastSolveRejectReason = rejectReason;
					const Eigen::Vector3d candidateCm = calibration.Transformation().translation() * 100.0;
					Metrics::LogAnnotationf(
					    "continuous_solve_tick: state=%d(%s) paused=%d attempted=1 produced=%d"
					    " calc_valid=%d source=%s sample_count=%zu required=%zu"
					    " relPosCal=%d lockRel=%d validProfile=%d hasAccepted=%d"
					    " candidate_cm=(%.2f,%.2f,%.2f) candidate_mag_cm=%.2f"
					    " reject_reason=%s",
					    (int)ctx.state, CalibrationStateName(ctx.state), (int)CalCtx.calibrationPaused,
					    (int)solveProducedCandidate, (int)calibration.isValid(),
					    calibration.LastComputeUsedRelPose() ? "relpose" : "full", calibration.SampleCount(),
					    CalCtx.SampleCount(), (int)CalCtx.relativePosCalibrated, (int)CalCtx.lockRelativePosition,
					    (int)CalCtx.validProfile, (int)CalCtx.lastAcceptedContinuousSnapshot.captured, candidateCm.x(),
					    candidateCm.y(), candidateCm.z(), candidateCm.norm(), rejectReason);
				}
			}
		}
		else {
			static double s_lastPausedSolveAnnotation = -1e9;
			if (time - s_lastPausedSolveAnnotation >= 2.0) {
				s_lastPausedSolveAnnotation = time;
				char pausedBuf[320];
				snprintf(pausedBuf, sizeof pausedBuf,
				         "continuous_solve_skipped: state=%d(%s) reason=paused sample_count=%zu required=%zu"
				         " validProfile=%d hasAccepted=%d",
				         (int)ctx.state, CalibrationStateName(ctx.state), calibration.SampleCount(),
				         CalCtx.SampleCount(), (int)CalCtx.validProfile,
				         (int)CalCtx.lastAcceptedContinuousSnapshot.captured);
				Metrics::WriteLogAnnotation(pausedBuf);
			}
		}

		TickAdditionalCalibrations(time);
	}
	else {
		calibration.enableStaticRecalibration = false;
		solveAttempted = true;
		solveProducedCandidate = calibration.ComputeOneshot(CalCtx.ignoreOutliers);
	}

	TickCandidateAcceptAndPersist(ctx, time, solveAttempted, solveProducedCandidate);

	LARGE_INTEGER end_time;
	QueryPerformanceCounter(&end_time);
	LARGE_INTEGER freq;
	QueryPerformanceFrequency(&freq);
	double duration = (end_time.QuadPart - start_time.QuadPart) / (double)freq.QuadPart;
	const double computationTimeMs = duration * 1000.0;
	Metrics::computationTime.Push(computationTimeMs);

	// Whole-tick stage breakdown, so a slow-tick report is attributable to a
	// phase instead of the tick as a whole (frame-hitch diagnostics measure
	// the full plugin tick; computationTime above covers only solve+accept).
	{
		const double totalMs = QpcMsBetween(tickWallStart, end_time);
		static double s_lastTickSlowLog = -1e9;
		if (totalMs >= 50.0 && (time - s_lastTickSlowLog) >= 5.0) {
			s_lastTickSlowLog = time;
			Metrics::LogAnnotationf("[cal-tick-slow] total_ms=%.1f gates_ms=%.1f detect_ms=%.1f sample_ms=%.1f"
			                        " solve_accept_ms=%.1f state=%d samples=%zu",
			                        totalMs, g_tickGatesMs, g_tickDetectMs, g_tickSampleMs, computationTimeMs,
			                        (int)ctx.state, calibration.SampleCount());
		}
	}

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

		auto driverPoseToWorld = [](const vr::DriverPose_t& dp, Eigen::Vector3d& outTrans, Eigen::Quaterniond& outRot) {
			Eigen::Quaterniond worldFromDriver(dp.qWorldFromDriverRotation.w, dp.qWorldFromDriverRotation.x,
			                                   dp.qWorldFromDriverRotation.y, dp.qWorldFromDriverRotation.z);
			Eigen::Vector3d worldFromDriverTrans(dp.vecWorldFromDriverTranslation[0],
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
			case CalibrationState::None:
				phase = Metrics::TickPhase::None;
				break;
			case CalibrationState::Begin:
				phase = Metrics::TickPhase::Begin;
				break;
			case CalibrationState::Rotation:
				phase = Metrics::TickPhase::Rotation;
				break;
			case CalibrationState::Translation:
				phase = Metrics::TickPhase::Translation;
				break;
			case CalibrationState::Editing:
				phase = Metrics::TickPhase::Editing;
				break;
			case CalibrationState::Continuous:
				phase = Metrics::TickPhase::Continuous;
				break;
			case CalibrationState::ContinuousStandby:
				phase = Metrics::TickPhase::ContinuousStandby;
				break;
		}

		Metrics::SetTickRawPoses(refT, refQ, tgtT, tgtQ, phase);

		// v4 locked-snap corroboration inputs: raw world-space HMD + head-tracker
		// poses recorded per row so the replay harness can reconstruct per-row
		// jump/displacement deltas (driverPoseToWorld matches
		// CalibrationPoseSampling::ConvertPose).
		Metrics::ReplayLockedSnapInputs lockedSnap;
		driverPoseToWorld(ctx.devicePoses[vr::k_unTrackedDeviceIndex_Hmd], lockedSnap.hmdTrans, lockedSnap.hmdRot);

		// Head-mount tracker validity: poseIsValid + deviceIsConnected +
		// Running_OK. Recorded mode-agnostically (not gated on HeadMountMode) so
		// a capture can be A/B-replayed as if a locked style had been active
		// even when it wasn't.
		const int32_t headMountId = ctx.headMount.deviceID;
		if (headMountId >= 0 && headMountId < maxId) {
			const vr::DriverPose_t& headTrackerPose = ctx.devicePoses[headMountId];
			if (headTrackerPose.poseIsValid && headTrackerPose.deviceIsConnected &&
			    headTrackerPose.result == vr::ETrackingResult::TrackingResult_Running_OK) {
				driverPoseToWorld(headTrackerPose, lockedSnap.headTrackerTrans, lockedSnap.headTrackerRot);
				lockedSnap.headTrackerValid = true;
			}
		}

		lockedSnap.relocDetected = false;

		Metrics::SetTickLockedSnapInputs(lockedSnap);

		Metrics::SetTickExperimentalFlags(0);
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

void DebugApplyRandomOffset()
{
	protocol::Request req(protocol::RequestDebugOffset);
	Driver.SendBlocking(req);
}

// (RecenterPlayspaceToCurrentHmd removed: superseded by lighthouse-anchored
// boundary push -- shifting the SZP to the HMD pose would push the boundary
// off the physical room geometry.)
