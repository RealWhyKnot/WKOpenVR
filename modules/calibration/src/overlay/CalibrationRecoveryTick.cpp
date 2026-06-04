#include "CalibrationRecoveryTick.h"

#include "AutoLockHysteresis.h"
#include "CalibrationDevicePoseUtils.h"
#include "CalibrationInternal.h"
#include "CalibrationMetrics.h"
#include "CalibrationPoseSampling.h"
#include "CalibrationProfileApply.h"
#include "CommonModeCoherence.h"
#include "HeadMountPoseSampling.h"
#include "SnapSuppression.h"

#include <GLFW/glfw3.h>

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <deque>
#include <map>
#include <string>
#include <vector>

// Tracker liveness state for the two non-HMD calibration anchors. CalibrationTick
// owns the actual liveness gate, but the recovery module owns the durable state
// because relocalization and recovery paths reset the same baseline.
spacecal::liveness::TrackerLivenessState g_refLiveness;
spacecal::liveness::TrackerLivenessState g_tgtLiveness;
bool g_refWasOffline = false;
bool g_tgtWasOffline = false;

// Single monotonic snap-suppression counter shared by all detection sites so
// Metrics::snapSuppressedCount is a non-decreasing series across the session.
// This file is single-threaded (called only from CalibrationTick); no mutex needed.
static uint32_t g_snapSuppressedCount = 0;

namespace {

// === Base station drift correction (one-shot mode) =======================
//
// Watches TrackingReference (Lighthouse base station) poses for runtime
// universe shifts. Base stations are physically static -- their poses in
// the runtime's tracking universe only change when the runtime re-origins
// (chaperone reset, SetSeatedZeroPose, etc.). When ALL base stations in a
// tracking system shift by the same rigid delta D between two consecutive
// ticks, that's a re-origin -- we apply D (or D-^-1, depending on which
// system shifted) to the stored calibration so body trackers stay aligned
// with the user's physical position.
//
// Why this works where the deleted "HMD pose jumped" heuristic (D in the
// old plan) didn't: HMD pose changes during natural user motion. Base
// stations don't. The signal-to-noise ratio is dramatically higher.
//
// Failure mode is benign: if OpenVR doesn't update base station poses on
// some recenter event, the detector simply doesn't fire. Worst case it
// catches nothing; never makes tracking worse.
struct BaseStationCacheEntry
{
	std::string trackingSystem;
	Eigen::Affine3d pose;
};
std::map<std::string, BaseStationCacheEntry> baseStationCache; // serial -> entry
double lastBaseStationShiftAcceptedTime = -1e9;

// Tolerance for "this delta is significant (above pose noise)": 1 mm,
// 0.05 deg. Static base station poses are byte-stable in the OpenVR
// runtime under normal operation, so any motion above this is signal.
constexpr double kBsDeltaSignificanceTransM = 0.001;
constexpr double kBsDeltaSignificanceRotRad = 0.05 * EIGEN_PI / 180.0;

// Tolerance for "all base stations moved by the same delta" (consensus):
// 5 mm, 0.5 deg. Slightly looser than the significance threshold to
// allow for sub-tick interpolation differences across base stations.
constexpr double kBsConsensusTransM = 0.005;
constexpr double kBsConsensusRotRad = 0.5 * EIGEN_PI / 180.0;

double RigidDeltaAngleRad(const Eigen::Affine3d& delta)
{
	const Eigen::Quaterniond q(delta.linear());
	return 2.0 * std::acos(std::min(1.0, std::abs(q.w())));
}

// Apply a universe shift D to every calibration whose ref or target
// tracking system matches `system`. The math:
//   reference world shifted by D => R_new = D * R_old
//   target    world shifted by D => R_new = R_old * D-^-1
// Primary calibration uses CalCtx.referenceTrackingSystem and
// CalCtx.targetTrackingSystem. Each AdditionalCalibration only stores
// targetTrackingSystem; its reference is implicitly the same as the
// primary's (always HMD-side).
void ApplyUniverseShiftToCalibrations(const Eigen::Affine3d& D, const std::string& system)
{
	auto applyDelta = [&D](Eigen::Vector3d& transCm, Eigen::Vector3d& rotDeg, bool refSide) {
		const Eigen::Vector3d eulerRad = rotDeg * EIGEN_PI / 180.0;
		const Eigen::Quaterniond rotQ = Eigen::AngleAxisd(eulerRad(0), Eigen::Vector3d::UnitZ()) *
		                                Eigen::AngleAxisd(eulerRad(1), Eigen::Vector3d::UnitY()) *
		                                Eigen::AngleAxisd(eulerRad(2), Eigen::Vector3d::UnitX());
		const Eigen::Affine3d cal = Eigen::Translation3d(transCm * 0.01) * rotQ;
		const Eigen::Affine3d newCal = refSide ? (D * cal) : (cal * D.inverse());
		transCm = newCal.translation() * 100.0;
		rotDeg = newCal.linear().eulerAngles(2, 1, 0) * 180.0 / EIGEN_PI;
	};

	const bool primaryRefShift = (system == CalCtx.referenceTrackingSystem);
	const bool primaryTargetShift = (system == CalCtx.targetTrackingSystem);
	if (primaryRefShift) {
		applyDelta(CalCtx.calibratedTranslation, CalCtx.calibratedRotation, /*refSide=*/true);
	}
	else if (primaryTargetShift) {
		applyDelta(CalCtx.calibratedTranslation, CalCtx.calibratedRotation, /*refSide=*/false);
	}

	// Additional calibrations: their reference IS the primary's reference,
	// so a reference-side shift hits all of them. Their target is per-entry.
	for (auto& extra : CalCtx.additionalCalibrations) {
		if (!extra.valid) continue;
		if (primaryRefShift) {
			applyDelta(extra.calibratedTranslation, extra.calibratedRotation, /*refSide=*/true);
		}
		else if (extra.targetTrackingSystem == system) {
			applyDelta(extra.calibratedTranslation, extra.calibratedRotation, /*refSide=*/false);
		}
	}

	char logbuf[256];
	const double angDeg = RigidDeltaAngleRad(D) * 180.0 / EIGEN_PI;
	snprintf(logbuf, sizeof logbuf,
	         "Universe shift detected in %s system (%.1f cm, %.1f deg) - "
	         "calibration delta-corrected from base stations\n",
	         system.c_str(), D.translation().norm() * 100.0, angDeg);
	CalCtx.Log(logbuf);
	Metrics::WriteLogAnnotation("base_station_shift: calibration delta-corrected");

	InvalidateAllTransformCaches();
}

// Per-tick detector. Cheap: a few property reads + matrix arithmetic per
// base station. Skipped entirely when the toggle is off or no base stations
// are present.
void TickBaseStationDriftImpl(double now)
{
	if (!CalCtx.baseStationDriftCorrectionEnabled) return;
	if (!vr::VRSystem()) return;
	if (!CalCtx.validProfile || !CalCtx.enabled) return;

	// Throttle: at most one accepted shift per 5 seconds. A second shift
	// arriving rapidly after the first usually means the first applied
	// delta was wrong (we've already cached the post-shift poses, so a
	// follow-up shift would be a brand-new event). Throttling avoids
	// thrashing the calibration on a runtime that's still settling.
	const bool throttled = (now - lastBaseStationShiftAcceptedTime) < 5.0;

	struct Observation
	{
		std::string serial;
		std::string system;
		Eigen::Affine3d pose;
	};
	std::vector<Observation> current;
	current.reserve(8);

	char buf[vr::k_unMaxPropertyStringSize] = {};
	for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id) {
		if (vr::VRSystem()->GetTrackedDeviceClass(id) != vr::TrackedDeviceClass_TrackingReference) continue;
		const auto& dp = CalCtx.devicePoses[id];
		if (!dp.poseIsValid || !dp.deviceIsConnected) continue;
		if (dp.result != vr::ETrackingResult::TrackingResult_Running_OK) continue;

		vr::ETrackedPropertyError err = vr::TrackedProp_Success;
		vr::VRSystem()->GetStringTrackedDeviceProperty(id, vr::Prop_SerialNumber_String, buf, sizeof buf, &err);
		if (err != vr::TrackedProp_Success) continue;
		std::string serial = buf;

		vr::VRSystem()->GetStringTrackedDeviceProperty(id, vr::Prop_TrackingSystemName_String, buf, sizeof buf, &err);
		if (err != vr::TrackedProp_Success) continue;
		std::string system = buf;

		Pose p = ConvertPose(dp);
		Eigen::Affine3d pose = Eigen::Affine3d::Identity();
		pose.linear() = p.rot;
		pose.translation() = p.trans;

		current.push_back({serial, system, pose});
	}

	// No base stations -> AUTO mode self-disables, leaving the cache empty.
	if (current.empty()) {
		baseStationCache.clear();
		return;
	}

	// Group observations by tracking system. A user with multiple
	// Lighthouse setups (rare) might have base stations from different
	// systems; each system's universe is independent so they're checked
	// separately.
	std::map<std::string, std::vector<const Observation*>> bySystem;
	for (const auto& obs : current)
		bySystem[obs.system].push_back(&obs);

	for (const auto& [system, obsPtrs] : bySystem) {
		// Only base stations belonging to a system that's actually part
		// of the active calibration are useful for drift correction.
		// Stations from an unrelated third system can't tell us anything
		// about our calibration's coordinate frames.
		const bool refSystem = (system == CalCtx.referenceTrackingSystem);
		bool targetSystem = (system == CalCtx.targetTrackingSystem);
		if (!targetSystem) {
			for (const auto& extra : CalCtx.additionalCalibrations) {
				if (extra.valid && extra.targetTrackingSystem == system) {
					targetSystem = true;
					break;
				}
			}
		}
		if (!refSystem && !targetSystem) continue;

		// Compute deltas vs. previous tick. Only base stations we've
		// previously seen can produce a delta; first-sighting entries
		// just populate the cache.
		std::vector<Eigen::Affine3d> deltas;
		deltas.reserve(obsPtrs.size());
		for (const auto* obs : obsPtrs) {
			auto it = baseStationCache.find(obs->serial);
			if (it == baseStationCache.end()) continue;
			if (it->second.trackingSystem != system) continue;
			deltas.push_back(obs->pose * it->second.pose.inverse());
		}

		if (!throttled && !deltas.empty()) {
			// Significance: at least one delta meaningfully bigger than
			// pose noise. Otherwise everything's stationary -- nothing
			// to do.
			bool anySignificant = false;
			for (const auto& d : deltas) {
				if (d.translation().norm() > kBsDeltaSignificanceTransM ||
				    RigidDeltaAngleRad(d) > kBsDeltaSignificanceRotRad) {
					anySignificant = true;
					break;
				}
			}

			// Consensus: all deltas approximately equal (= same rigid
			// universe shift). For multi-base-station setups this is
			// the principled disambiguator between "universe shifted"
			// and "one base station's pose was internally refined".
			//
			// Single-base-station setups can't cross-validate, so we
			// require a substantial shift (>= 10 cm or >= 5 deg) to
			// reduce false positives from runtime base-station
			// refinement.
			bool consensus = true;
			if (anySignificant) {
				if (deltas.size() >= 2) {
					const Eigen::Affine3d& ref = deltas[0];
					for (size_t i = 1; i < deltas.size(); ++i) {
						const Eigen::Affine3d diff = deltas[i] * ref.inverse();
						if (diff.translation().norm() > kBsConsensusTransM ||
						    RigidDeltaAngleRad(diff) > kBsConsensusRotRad) {
							consensus = false;
							break;
						}
					}
				}
				else {
					const Eigen::Affine3d& only = deltas[0];
					const double angRad = RigidDeltaAngleRad(only);
					if (only.translation().norm() < 0.10 && angRad < 5.0 * EIGEN_PI / 180.0) {
						consensus = false;
					}
				}
			}

			if (anySignificant && consensus) {
				ApplyUniverseShiftToCalibrations(deltas[0], system);
				lastBaseStationShiftAcceptedTime = now;
				// Cache update happens unconditionally below, picking
				// up the post-shift poses as the new baseline.
			}
		}
	}

	// Refresh cache for the next tick. We always update -- even when
	// throttled, so that when the throttle releases we're comparing
	// against the most recent poses, not pre-shift ones.
	baseStationCache.clear();
	for (const auto& obs : current) {
		baseStationCache[obs.serial] = {obs.system, obs.pose};
	}
}

// === Hybrid HMD-relocalization detector (logging-only) ===================
//
// Triple-AND signal that the HMD's tracking system (e.g. Quest SLAM)
// re-localized -- its reported pose in the OpenVR universe jumps even
// though physically the user didn't move:
//
//   1. HMD pose changed by >5 cm (translation) since the previous tick.
//   2. Base stations are stable (max delta <1 mm). Rules out a SteamVR
//      universe re-origin -- that's the other detector's domain.
//   3. Body trackers in a DIFFERENT tracking system from the HMD didn't
//      follow the HMD jump (their per-tick deltas are all <5 cm). Rules
//      out natural fast user motion -- when the user moves their head,
//      the body trackers (worn on the body) move along with it.
//
// The conjunction can't false-fire on natural motion: real motion has
// the HMD and body trackers moving together, so condition 3 fails. Only
// a re-localization (HMD jumps independently of body) trips all three.
//
// This is the LOGGING-ONLY first cut. When the trigger fires we emit a
// `# [time] hmd_relocalization_detected: dx=<m> dy=<m> dz=<m> dt=<rad>`
// annotation to the debug log and update the cache. We do NOT modify R
// or touch the chaperone -- the user wants to gather real-world data
// confirming the trigger fires only on actual events before we take
// corrective action.
//
// Requires >=2 base stations to even consider firing -- on Quest-only
// setups (no Lighthouse) the second condition can't be cross-checked
// and the detector is silent.
//
// Runs in continuous AND one-shot None mode. Skipped during active
// calibration sub-states (Begin / Rotation / Translation) where the
// HMD is being deliberately moved.

struct RelocalizationDetectorState
{
	bool havePrevHmd = false;
	Eigen::Affine3d prevHmd = Eigen::Affine3d::Identity();
	std::string hmdTrackingSystem;
	// Per-device previous poses, keyed by OpenVR ID. ID is the right
	// key here (not serial) because we re-read every tick and the OpenVR
	// IDs are stable within a session for the duration of our use.
	std::map<int32_t, Eigen::Vector3d> prevBodyTrans;
	double lastFireTime = -1e9;

	// Head-mount tracker position from the previous tick, used by the
	// snap-suppression corroboration check (site 504). Only valid when
	// havePrevHmTracker is true; reset whenever the tracker loses validity.
	bool havePrevHeadTracker = false;
	Eigen::Vector3d prevHeadTrackerTrans = Eigen::Vector3d::Zero();

	// Snapshot of the most recent fire's measured deltas, exposed via
	// LastDetectedRelocalization() so the UI can surface "your last
	// detected drift event was X cm at T seconds ago" alongside the
	// recenter button.
	Eigen::Vector3d lastFireDelta = Eigen::Vector3d::Zero();
	double lastFireRotRad = 0.0;

	// Last time auto-recovery actually clobbered the calibration. Throttled
	// separately from the 5-second logging fire -- the cost of a too-eager
	// auto-recover is much higher than a too-eager log line. Continuous-cal
	// needs uninterrupted time to converge after each recover, so we keep
	// at least 30s between consecutive auto-recovers.
	double lastAutoRecoverTime = -1e9;

	// Last time HMD tracking transitioned from valid -> invalid (a stall
	// began). Used to enforce a post-stall grace period: a "relocalization"
	// detected within seconds of a stall recovery is more likely the Quest
	// settling its post-stall pose than a true SLAM teleport. The existing
	// stall-recovery flow ALREADY calls StartContinuousCalibration to
	// bootstrap, so a same-instant auto-recover would just double-tap that
	// flow AND save an empty profile.
	//
	// Set to "now" whenever we see hmdValid transition false (stall begin).
	// Auto-recovery checks (now - lastHmdInvalidTime) and refuses to fire
	// if the post-stall window hasn't elapsed.
	//
	// Triggered the first false-positive on 2026-05-02 (build 2026.5.2.8):
	// 3.5s stall ended at t=1527.69, the existing flow auto-restarted
	// continuous-cal at t=1527.69, my detector saw a 29cm pose delta at
	// t=1527.74, auto-recovery fired and clobbered the working cal.
	// Adding this gate so the same scenario can't repeat.
	//
	// NOTE: claimed-but-empirically-false: "stamped on every non-OK tick
	// below". 2026-05-04 logs show this stays at the session-start
	// sentinel even across a 94-second stall, meaning either the
	// !hmdValid branch isn't being entered during real stalls, or
	// devicePoses[Hmd] reports valid-but-stale during stalls. Tracking
	// as a follow-up; new debug logging (reloc_tick, reloc_hmd_invalid_
	// stamped) added in the same change set as this comment will surface
	// the data needed to fix it.
	double lastHmdInvalidTime = -1e9;

	// --- Auto-recovery snapshot: pre-clear state so Undo can restore.
	// Captured INSIDE the auto-recover action block, just before
	// calibration.Clear() runs. Snapshot is "valid" until the user
	// hits Undo (which restores + invalidates) or Dismiss (which only
	// hides the UI banner; the snapshot stays so a later Undo button
	// rendered through some other path would still work, though there
	// currently isn't one).
	struct AutoRecoverySnapshot
	{
		bool valid = false;
		Eigen::AffineCompact3d refToTargetPose = Eigen::AffineCompact3d::Identity();
		bool relativePosCalibrated = false;
		bool hasAppliedCalibrationResult = false;
	};
	AutoRecoverySnapshot lastAutoRecoverSnapshot;

	// Set by DismissAutoRecoveryBanner. Reset to false on every new
	// auto-recover firing so subsequent recoveries get their own banner
	// even if the user dismissed an earlier one.
	bool autoRecoverBannerDismissed = false;

	// --- DIAGNOSTIC ONLY (no gating). Per-base-station HMD-relative
	// distance from the previous tick, used to dump per-base distance
	// jumps into the log when hmd_relocalization_detected fires. The
	// fd81e83 commit tried to USE this as a corroboration gate, but
	// the math is broken for cross-system setups (Quest+Lighthouse)
	// because hmdPose and base poses live in different tracking-system
	// frames in CalCtx.devicePoses. Reverted as a gate; kept here only
	// as diagnostic data so a future log diff can see what cross-frame
	// geometry the detector observed.
	std::map<std::string, double> prevHmdToBaseDist;

	// Throttle for the per-tick reloc_tick diagnostic log. 1 Hz cap so
	// we always have a recent baseline state in the log without flooding.
	double lastTickLogTime = -1e9;

	// Throttle for the per-tick reloc_hmd_invalid_stamped diagnostic log
	// inside the !hmdValid branch. Same 1 Hz cap so a long stall produces
	// 1 line per second instead of 60+.
	double lastInvalidLogTime = -1e9;
};
RelocalizationDetectorState g_relocDetector;

constexpr double kRelocHmdJumpM = 0.05;      // 5 cm
constexpr double kRelocBodyMaxDeltaM = 0.05; // 5 cm (any body tracker moving more = rule out)
constexpr double kRelocBaseStableM = 0.001;  // 1 mm
constexpr double kRelocThrottleSec = 5.0;
constexpr int kRelocMinBaseStations = 2;

// Auto-recovery thresholds. Stricter than the logging trigger because the
// cost of a false-positive auto-recovery (clobbering a working cal) is
// much higher than a false-positive log line. The user's reported wedged
// event was 86 cm; the false-positive that clobbered a working cal was
// 29 cm right after a stall. So we need BOTH a higher magnitude floor AND
// a post-stall lockout.
//
// Threshold: 30 cm (raised from 15 cm after the 2026-05-02 false-positive).
// SLAM noise is typically <2 cm/tick, real teleport events tend to be
// 30-100+ cm. 30 cm filters out the post-stall-settling case (which can
// look like 20-30 cm) without losing the catastrophic-wedge case (60+ cm).
//
// Startup grace: don't fire in the first 30 s of the session. Pose data
// settling + initial driver bootstrap can produce spurious large deltas
// in the first few ticks; better to let the system stabilise before we
// start invalidating.
//
// Throttle: at least 30 s between auto-recovers. Continuous-cal needs
// uninterrupted time to converge after each recovery; if we re-fire too
// quickly we'd interrupt the convergence and prevent the calibration
// from ever stabilising.
//
// Post-stall grace: 10 s after an HMD-tracking-lost event, refuse to
// auto-recover. The stall flow has its own restart logic; let it converge
// before considering further intervention. Long enough that even a chunky
// post-stall pose-settling jitter window doesn't trigger us. (Was briefly
// raised to 30 s in fd81e83 alongside the per-base corroboration gate;
// reverted with that gate in 2026-05-04 -- the 30 s value never actually
// applied in practice because lastHmdInvalidTime doesn't advance during
// real stalls, see TODO above. Restoring 10 s as the original behavior
// while we figure out the timestamp-not-updating problem separately.)
constexpr double kRelocAutoRecoverThresholdM = 0.30;   // 30 cm (was 15, too low)
constexpr double kRelocAutoRecoverStartupSec = 30.0;   // 30 s startup grace
constexpr double kRelocAutoRecoverThrottleSec = 30.0;  // 30 s between recovers
constexpr double kRelocAutoRecoverPostStallSec = 10.0; // 10 s after stall recovery

// Snap-suppression corroboration thresholds are in SnapSuppression.h as
// spacecal::snap_suppression::kSnapHmdJumpM / kSnapTrackerMaxDispM.

void TickHmdRelocalizationDetectorImpl(double now)
{
	if (!vr::VRSystem()) return;

	// Skip during active one-shot calibration sub-states (the HMD is
	// being deliberately swung around). Continuous, ContinuousStandby,
	// None, Editing all OK.
	if (CalCtx.state == CalibrationState::Begin || CalCtx.state == CalibrationState::Rotation ||
	    CalCtx.state == CalibrationState::Translation) {
		// Reset cache so the post-calibration baseline is fresh.
		g_relocDetector.havePrevHmd = false;
		g_relocDetector.prevBodyTrans.clear();
		g_relocDetector.havePrevHeadTracker = false;
		return;
	}

	auto& s = g_relocDetector;

	// HMD pose. Index 0 by the static_assert above.
	const auto& hmdRaw = CalCtx.devicePoses[vr::k_unTrackedDeviceIndex_Hmd];
	const bool hmdValid = hmdRaw.poseIsValid && hmdRaw.deviceIsConnected &&
	                      hmdRaw.result == vr::ETrackingResult::TrackingResult_Running_OK;

	// Diagnostic: per-tick state baseline, throttled to 1 Hz. Future
	// triage of "tracking went weird" reports can grep `reloc_tick` to
	// see what the detector observed across a problematic window. Also
	// the only way to confirm the function is being called every tick
	// (vs. some upstream skipping it during stalls -- which is what we
	// suspect is preventing lastHmdInvalidTime from advancing during
	// real stalls, see comment on the field).
	if ((now - s.lastTickLogTime) >= 1.0) {
		s.lastTickLogTime = now;
		char tickbuf[256];
		snprintf(tickbuf, sizeof tickbuf,
		         "reloc_tick: hmdValid=%d havePrevHmd=%d state=%d hmdRaw.result=%d hmdRaw.poseIsValid=%d "
		         "hmdRaw.deviceIsConnected=%d lastHmdInvalidTime=%.3f secSinceStall=%.2f",
		         (int)hmdValid, (int)s.havePrevHmd, (int)CalCtx.state, (int)hmdRaw.result, (int)hmdRaw.poseIsValid,
		         (int)hmdRaw.deviceIsConnected, s.lastHmdInvalidTime, now - s.lastHmdInvalidTime);
		Metrics::WriteLogAnnotation(tickbuf);
	}

	if (!hmdValid) {
		// Tracking dropout. Drop the cache so we don't compare across
		// the dropout (which would produce a spurious jump). Also stamp
		// the dropout time -- the auto-recovery gate uses it to enforce
		// a post-stall grace window so we don't double-tap the existing
		// stall-recovery flow's StartContinuousCalibration.

		// Diagnostic: log every entry into the !hmdValid branch (1 Hz
		// throttled). This is the data we need to debug why
		// lastHmdInvalidTime doesn't advance during real stalls --
		// either this branch isn't being entered (devicePoses[Hmd]
		// reports stale-but-valid during stalls), or it IS being
		// entered but the stamp isn't reaching the auto-recovery gate
		// for some other reason. Either way, having a log line per
		// stall second will surface the truth on the next reproduction.
		if ((now - s.lastInvalidLogTime) >= 1.0) {
			s.lastInvalidLogTime = now;
			char invbuf[224];
			snprintf(invbuf, sizeof invbuf,
			         "reloc_hmd_invalid_stamped: now=%.3f prev_lastHmdInvalidTime=%.3f hmdRaw.result=%d "
			         "hmdRaw.poseIsValid=%d hmdRaw.deviceIsConnected=%d",
			         now, s.lastHmdInvalidTime, (int)hmdRaw.result, (int)hmdRaw.poseIsValid,
			         (int)hmdRaw.deviceIsConnected);
			Metrics::WriteLogAnnotation(invbuf);
		}

		s.havePrevHmd = false;
		s.prevBodyTrans.clear();
		s.havePrevHeadTracker = false;
		s.lastHmdInvalidTime = now;
		return;
	}

	// HMD's tracking system, cached once. Used to identify "body trackers
	// in a DIFFERENT system" below.
	if (s.hmdTrackingSystem.empty()) {
		char buf[vr::k_unMaxPropertyStringSize] = {};
		vr::ETrackedPropertyError err = vr::TrackedProp_Success;
		vr::VRSystem()->GetStringTrackedDeviceProperty(vr::k_unTrackedDeviceIndex_Hmd,
		                                               vr::Prop_TrackingSystemName_String, buf, sizeof buf, &err);
		if (err != vr::TrackedProp_Success) return;
		s.hmdTrackingSystem = buf;
	}

	Pose hmdPoseWorld = ConvertPose(hmdRaw);
	Eigen::Affine3d hmdPose = Eigen::Affine3d::Identity();
	hmdPose.linear() = hmdPoseWorld.rot;
	hmdPose.translation() = hmdPoseWorld.trans;

	// Base stations: count + max delta vs last tick. Reuse the same
	// cache the existing universe-shift detector populates -- but we
	// can't read its state directly without coupling to it, so do a
	// fresh scan here (cheap; ~4 base stations max).
	//
	// Also track per-base HMD-relative distance (cur + prev) for
	// DIAGNOSTIC dumping in the reloc_base_dists log when a fire
	// event triggers below. NOT used as a gate (the fd81e83 attempt
	// at gating on this was reverted -- HMD and base poses live in
	// different tracking-system frames in CalCtx.devicePoses, so
	// the cross-frame distance has no consistent physical meaning).
	// Kept here purely so the log shows what the detector saw.
	double bsMaxDelta = 0.0;
	int bsCount = 0;
	std::map<std::string, double> currentHmdToBaseDist;
	char propBuf[vr::k_unMaxPropertyStringSize] = {};
	for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id) {
		if (vr::VRSystem()->GetTrackedDeviceClass(id) != vr::TrackedDeviceClass_TrackingReference) continue;
		const auto& dp = CalCtx.devicePoses[id];
		if (!dp.poseIsValid || !dp.deviceIsConnected) continue;
		if (dp.result != vr::ETrackingResult::TrackingResult_Running_OK) continue;
		++bsCount;
		vr::ETrackedPropertyError err = vr::TrackedProp_Success;
		vr::VRSystem()->GetStringTrackedDeviceProperty(id, vr::Prop_SerialNumber_String, propBuf, sizeof propBuf, &err);
		if (err != vr::TrackedProp_Success) continue;
		std::string serial = propBuf;
		Pose p = ConvertPose(dp);
		currentHmdToBaseDist[serial] = (hmdPose.translation() - p.trans).norm();
		auto it = baseStationCache.find(serial); // populated by TickBaseStationDrift
		if (it == baseStationCache.end()) continue;
		double d = (p.trans - it->second.pose.translation()).norm();
		if (d > bsMaxDelta) bsMaxDelta = d;
	}

	// Body trackers in OTHER tracking system(s). For Quest HMD + Lighthouse
	// trackers, this picks up the Lighthouse trackers; for Lighthouse HMD +
	// Quest trackers, it picks up the Quest trackers. Either way, the trackers
	// we care about are the ones that DIDN'T re-localize when the HMD did.
	double bodyMaxDelta = 0.0;
	std::map<int32_t, Eigen::Vector3d> currentBodyTrans;
	for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id) {
		const auto cls = vr::VRSystem()->GetTrackedDeviceClass(id);
		if (cls != vr::TrackedDeviceClass_GenericTracker && cls != vr::TrackedDeviceClass_Controller) continue;
		const auto& dp = CalCtx.devicePoses[id];
		if (!dp.poseIsValid || !dp.deviceIsConnected) continue;
		if (dp.result != vr::ETrackingResult::TrackingResult_Running_OK) continue;
		vr::ETrackedPropertyError err = vr::TrackedProp_Success;
		vr::VRSystem()->GetStringTrackedDeviceProperty(id, vr::Prop_TrackingSystemName_String, propBuf, sizeof propBuf,
		                                               &err);
		if (err != vr::TrackedProp_Success) continue;
		if (std::string(propBuf) == s.hmdTrackingSystem) continue; // skip same-system devices
		Pose p = ConvertPose(dp);
		currentBodyTrans[(int32_t)id] = p.trans;
		auto it = s.prevBodyTrans.find((int32_t)id);
		if (it == s.prevBodyTrans.end()) continue;
		double d = (p.trans - it->second).norm();
		if (d > bodyMaxDelta) bodyMaxDelta = d;
	}

	// Head-mount tracker displacement over this tick. Used by the snap-
	// suppression corroboration check: a Quest SLAM snap moves the HMD pose
	// without moving the physical head, so the lighthouse-tracked head-
	// mount tracker should report near-zero displacement even when the HMD
	// reports a large jump.
	//
	// Only computed when mode >= Corroborate and the tracker is valid and
	// fresh. On validity loss the cache is cleared so the next valid tick
	// starts a fresh window.
	double headTrackerDelta = -1.0; // negative = no valid reading this tick
	{
		const auto& hm = CalCtx.headMount;
		const bool corroborateActive = hm.mode >= HeadMountMode::Corroborate && hm.deviceID >= 0 &&
		                               (uint32_t)hm.deviceID < vr::k_unMaxTrackedDeviceCount;
		if (corroborateActive) {
			const auto& tp = CalCtx.devicePoses[hm.deviceID];
			const bool trackerValid =
			    tp.poseIsValid && tp.deviceIsConnected && tp.result == vr::ETrackingResult::TrackingResult_Running_OK;
			if (trackerValid) {
				const Eigen::Vector3d trackerTrans(tp.vecPosition[0], tp.vecPosition[1], tp.vecPosition[2]);
				if (s.havePrevHeadTracker) {
					headTrackerDelta = (trackerTrans - s.prevHeadTrackerTrans).norm();
				}
				s.prevHeadTrackerTrans = trackerTrans;
				s.havePrevHeadTracker = true;
			}
			else {
				s.havePrevHeadTracker = false;
			}
		}
		else {
			s.havePrevHeadTracker = false;
		}
	}

	// Trigger evaluation. Need a previous HMD pose and >=2 base stations
	// (so condition 2 isn't trivially passing). Note we still update the
	// cache even when no trigger fires -- always tracking the latest
	// values so the next tick's delta is over a one-tick interval.
	bool fired = false;
	if (s.havePrevHmd && bsCount >= kRelocMinBaseStations) {
		const double hmdDelta = (hmdPose.translation() - s.prevHmd.translation()).norm();
		const Eigen::Quaterniond qNew(hmdPose.linear());
		const Eigen::Quaterniond qOld(s.prevHmd.linear());
		Eigen::Quaterniond rotDelta = qNew * qOld.conjugate();
		rotDelta.normalize();
		const double angRad = 2.0 * std::acos(std::min(1.0, std::abs(rotDelta.w())));

		const bool throttled = (now - s.lastFireTime) < kRelocThrottleSec;
		if (!throttled && hmdDelta > kRelocHmdJumpM && bodyMaxDelta < kRelocBodyMaxDeltaM &&
		    bsMaxDelta < kRelocBaseStableM) {
			const Eigen::Vector3d dpos = hmdPose.translation() - s.prevHmd.translation();
			char logbuf[256];
			snprintf(logbuf, sizeof logbuf,
			         "hmd_relocalization_detected: dx=%.4f dy=%.4f dz=%.4f dt=%.4f"
			         " (hmdDelta=%.3f bodyMax=%.3f bsMax=%.4f bsCount=%d)",
			         dpos.x(), dpos.y(), dpos.z(), angRad, hmdDelta, bodyMaxDelta, bsMaxDelta, bsCount);
			Metrics::WriteLogAnnotation(logbuf);

			// Diagnostic: per-base distance dump alongside every fire,
			// so a future log can show whether the cross-frame HMD-to-
			// base distance jumped on this event (it usually doesn't on
			// cross-system Quest+Lighthouse setups -- that's why the
			// fd81e83 corroboration gate had to be reverted). Format
			// chunks one base station per snprintf so the line stays
			// bounded.
			{
				char baseBuf[1024];
				int written = snprintf(baseBuf, sizeof baseBuf, "reloc_base_dists:");
				for (const auto& kv : currentHmdToBaseDist) {
					auto prevIt = s.prevHmdToBaseDist.find(kv.first);
					double prev = (prevIt != s.prevHmdToBaseDist.end()) ? prevIt->second : -1.0;
					double jump = (prev >= 0.0) ? std::abs(kv.second - prev) : 0.0;
					int n =
					    snprintf(baseBuf + written, sizeof(baseBuf) - written,
					             " {serial=%s prev=%.3f cur=%.3f jump=%.3f}", kv.first.c_str(), prev, kv.second, jump);
					if (n <= 0 || static_cast<size_t>(written) + static_cast<size_t>(n) >= sizeof baseBuf) break;
					written += n;
				}
				Metrics::WriteLogAnnotation(baseBuf);
			}

			// "Who moved" diagnostic: compare the HMD's observed pose delta
			// against an independent displacement estimate. Quest re-anchor
			// signature: the HMD's world frame jumped while physical motion
			// was near zero (Quest didn't physically move).
			//
			// Displacement estimate source (kHeadMountCorroboration path):
			//   When head-mount mode >= Corroborate and the tracker is valid
			//   this tick, use the tracker's ACTUAL pose-to-pose displacement
			//   (headTrackerDelta, already computed above). This is more
			//   accurate than velocity-integrated IMU for re-anchor
			//   detection: the lighthouse tracker's world frame is stable,
			//   so its reported displacement directly reflects real motion.
			//   Falls back to the velocity-integrated HMD estimate when the
			//   tracker is unavailable.
			{
				const double dt = std::max(1e-3, now - s.lastTickLogTime); // ~ tick interval
				auto vmag = [](const double v[3]) -> double {
					return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
				};
				auto vmagFinite = [&](const double v[3]) -> double {
					const double m = vmag(v);
					return std::isfinite(m) ? m : 0.0;
				};

				// Choose displacement estimate. Prefer the head-mount tracker
				// actual displacement (kHeadMountCorroboration) when valid,
				// fall back to velocity-integrated HMD estimate otherwise.
				const bool useHeadMount =
				    headTrackerDelta >= 0.0 && CalCtx.headMount.mode >= HeadMountMode::Corroborate;
				const auto& hmdRawNow = CalCtx.devicePoses[vr::k_unTrackedDeviceIndex_Hmd];
				const double hmdSpeed = vmagFinite(hmdRawNow.vecVelocity);
				const double hmdImuDisp = hmdSpeed * dt;
				const double hmdDispEst = useHeadMount ? headTrackerDelta : hmdImuDisp;
				const auto dispSource = useHeadMount ? spacecal::coherence::CorroborationSource::kHeadMountCorroboration
				                                     : spacecal::coherence::CorroborationSource::kExtraPairs;
				(void)dispSource; // used in log label only

				// Body-tracker (other-system) integrated displacement: max
				// across all body trackers. If this is near zero, no body
				// device physically moved either, so the geometry shift
				// is purely a world-frame change of one of the systems.
				double bodyMaxImuDisp = 0.0;
				int bodyCount = 0;
				for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id) {
					const auto cls = vr::VRSystem()->GetTrackedDeviceClass(id);
					if (cls != vr::TrackedDeviceClass_GenericTracker && cls != vr::TrackedDeviceClass_Controller)
						continue;
					const auto& dp = CalCtx.devicePoses[id];
					if (!dp.poseIsValid || !dp.deviceIsConnected) continue;
					if (dp.result != vr::ETrackingResult::TrackingResult_Running_OK) continue;
					const double d = vmagFinite(dp.vecVelocity) * dt;
					if (d > bodyMaxImuDisp) bodyMaxImuDisp = d;
					++bodyCount;
				}

				// Ratio interpretation (for the human grepping this later):
				//   hmdRatio = observedHmdDelta / hmdDispEst
				//     >> 1: HMD's world frame jumped without device moving
				//           (most likely Quest re-anchor)
				//     ~= 1: HMD device physically moved (gate should have
				//           rejected on bodyMaxDelta but did not -- worth
				//           investigating; possible false positive)
				//   dispSource field shows which estimate was used.
				const double hmdRatio = (hmdDispEst > 1e-6) ? (hmdDelta / hmdDispEst) : -1.0;
				char wmoBuf[384];
				snprintf(wmoBuf, sizeof wmoBuf,
				         "who_moved: dt=%.4f hmd_observed=%.3f hmd_disp_est=%.4f hmd_ratio=%.2f"
				         " body_observed_max=%.3f body_imu_disp_max=%.4f body_count=%d"
				         " disp_source=%s",
				         dt, hmdDelta, hmdDispEst, hmdRatio, bodyMaxDelta, bodyMaxImuDisp, bodyCount,
				         useHeadMount ? "head_mount" : "hmd_imu");
				Metrics::WriteLogAnnotation(wmoBuf);
			}

			s.lastFireTime = now;
			s.lastFireDelta = dpos;
			s.lastFireRotRad = angRad;
			fired = true;
		}
	}

	// =================================================================
	// AUTO-RECOVERY: clobber the wedged calibration when a real
	// re-localization is detected.
	//
	// Background: continuous-cal can converge to a "self-consistent fit
	// at the wrong offset" after a Quest re-localization. Once wedged,
	// continuous-cal cannot recover on its own -- the saved relative
	// pose constraint pulls every refinement back to the bad neighborhood.
	// Until 2026-05-02 the only fix was to restart the overlay and
	// re-do calibration manually. This block makes recovery automatic.
	//
	// Entry conditions are intentionally stricter than the logging fire:
	//   - hmdDelta >= 15 cm (logging fires at 5 cm)
	//   - State == Continuous or ContinuousStandby (don't surprise the
	//     user mid-wizard or in None mode where there's nothing to fix)
	//   - Session has been running >= 30 s (avoid bootstrap noise)
	//   - >= 30 s since last auto-recover (let cal converge between resets)
	//
	// The triple-AND of the underlying detector means we already have
	// strong evidence this is a real re-localization, not natural motion.
	// The extra gates here are about being conservative on the *action*,
	// since clobbering a working cal is bad and we want zero false-fires.
	//
	// Recovery procedure:
	//   1. calibration.Clear() -- wipes m_estimatedTransformation,
	//      m_isValid, m_samples, m_refToTargetPose, m_relativePosCalibrated.
	//   2. CalCtx.refToTargetPose / relativePosCalibrated reset, so the
	//      restart in step 4 doesn't immediately re-apply the saved bad
	//      relative-pose constraint via setRelativeTransformation.
	//   3. SaveProfile() -- persist the cleared state. Without this, a
	//      subsequent program restart would re-load the bad cal from
	//      disk and the recovery would only have helped the live session.
	//   4. StartContinuousCalibration() -- restart cold. Continuous-cal
	//      will bootstrap from new pose pairs and converge to the
	//      correct (post-relocalization) calibration within seconds.
	//
	// User feedback: the fresh-start period IS visible -- body trackers
	// will appear at their lighthouse-system positions for a few seconds
	// before the new fit locks in. That's a much better outcome than
	// "calibration is permanently 86 cm off until you restart the
	// overlay manually."
	// Auto-recover gate. Each clause excludes a specific false-positive
	// scenario; if you change one, document why.
	const double currentHmdDelta = s.havePrevHmd ? (hmdPose.translation() - s.prevHmd.translation()).norm() : 0.0;
	const double secSinceStall = now - s.lastHmdInvalidTime;
	const bool postStallGrace = secSinceStall < kRelocAutoRecoverPostStallSec;
	const bool stateOK =
	    (CalCtx.state == CalibrationState::Continuous || CalCtx.state == CalibrationState::ContinuousStandby);
	const bool magnitudeOK = currentHmdDelta >= kRelocAutoRecoverThresholdM;
	const bool startupOK = now >= kRelocAutoRecoverStartupSec;
	const bool throttleOK = (now - s.lastAutoRecoverTime) >= kRelocAutoRecoverThrottleSec;

	// If `fired` is true (a relocalization log line was emitted) but a
	// gate blocked the recovery, log WHY -- gives us debug evidence for
	// every borderline event so we can tune thresholds against real data
	// instead of guessing. Throttled to once per fire (the fire itself
	// is throttled to 5s by the existing code), so this won't flood.
	if (fired && (!magnitudeOK || !stateOK || !startupOK || !throttleOK || postStallGrace)) {
		char skipbuf[384];
		snprintf(skipbuf, sizeof skipbuf,
		         "auto_recover_skipped: hmdDelta=%.3f magnitudeOK=%d stateOK=%d startupOK=%d throttleOK=%d "
		         "postStallGrace=%d (secSinceStall=%.2f) state=%d",
		         currentHmdDelta, (int)magnitudeOK, (int)stateOK, (int)startupOK, (int)throttleOK, (int)postStallGrace,
		         secSinceStall, (int)CalCtx.state);
		Metrics::WriteLogAnnotation(skipbuf);
	}

	if (fired && magnitudeOK && stateOK && startupOK && throttleOK && !postStallGrace) {
		const double hmdDelta = currentHmdDelta;
		const Eigen::Vector3d dpos = hmdPose.translation() - s.prevHmd.translation();

		// TODO(diag): remove after snap-suppress corroboration confirmed.
		// Field logs (2026-05-29) show headTrackerDelta=-1 at every relocalization,
		// so the snap-suppress bypass never engages and all events take destructive
		// full recovery. Capture the corroboration inputs at the decision point to
		// pin which precondition fails (mode / tracker pose validity / prior sample).
		{
			const auto& hmDbg = CalCtx.headMount;
			int poseValid = -1, connected = -1, result = -1;
			if (hmDbg.deviceID >= 0 && (uint32_t)hmDbg.deviceID < vr::k_unMaxTrackedDeviceCount) {
				const auto& tpDbg = CalCtx.devicePoses[hmDbg.deviceID];
				poseValid = (int)tpDbg.poseIsValid;
				connected = (int)tpDbg.deviceIsConnected;
				result = (int)tpDbg.result;
			}
			char hmbuf[320];
			snprintf(hmbuf, sizeof hmbuf,
			         "snap_corroboration_inputs: hmMode=%d deviceID=%d poseIsValid=%d connected=%d"
			         " result=%d havePrevTracker=%d headTrackerDelta=%.4f hmdDelta=%.3f",
			         (int)hmDbg.mode, hmDbg.deviceID, poseValid, connected, result, (int)s.havePrevHeadTracker,
			         headTrackerDelta, hmdDelta);
			Metrics::WriteLogAnnotation(hmbuf);
		}

		// Head-mount corroboration: when mode >= Corroborate and the tracker
		// produced a valid displacement reading this tick, check whether the
		// tracker actually moved. A Quest SLAM snap relocates the HMD's world
		// frame without any physical head motion, so the lighthouse-tracked
		// head-mount should report near-zero displacement even for a 30+ cm
		// HMD jump. When the HMD exceeds kSnapHmdJumpM but the tracker stays
		// below kSnapTrackerMaxDispM, classify as snap and substitute a fast
		// re-anchor (profile snap to saved state + grace window) for the
		// destructive full recovery.
		//
		// Falls back to full recovery unchanged when: corroborate mode is off,
		// the tracker was invalid this tick (headTrackerDelta < 0), or both
		// the HMD AND tracker exceeded their thresholds (genuine physical jump).
		const bool snapCorroborated =
		    spacecal::snap_suppression::IsJumpClassifiedAsSnap(CalCtx.headMount.mode, hmdDelta, headTrackerDelta);

		if (snapCorroborated) {
			// SLAM snap confirmed: HMD jumped but tracker didn't.
			// Increment the shared session counter and push so the TimeSeries
			// stays monotonically increasing across all suppression sites.
			++g_snapSuppressedCount;
			Metrics::snapSuppressedCount.Push(g_snapSuppressedCount);

			char snapbuf[384];
			snprintf(snapbuf, sizeof snapbuf,
			         "[snap-suppress] classified Quest SLAM snap:"
			         " hmd_delta_mm=%.1f tracker_delta_mm=%.1f"
			         " -> full_recovery suppressed, fast_reanchor requested",
			         hmdDelta * 1000.0, headTrackerDelta * 1000.0);
			Metrics::WriteLogAnnotation(snapbuf);

			// Fast re-anchor: snap to the saved profile and arm the
			// warm-restart validation grace window. The saved profile was
			// correct before the Quest SLAM snap; re-applying it without
			// clearing the solver lets the continuous-cal loop converge
			// back to the pre-snap state rather than starting from zero.
			// This is the same path as the warm-restart engage (see
			// Calibration.cpp::ShouldEngage) but fired from the snap
			// classification rather than the proximity sensor.
			CalCtx.warmRestartGraceSamples = spacecal::warm_restart::kGraceSamples;
			CalCtx.warmRestartMadAtSnap = CalCtx.autoLockMadFloor;
			CalCtx.warmRestartValidationState = spacecal::warm_restart::ValidationOutcome::Inconclusive;
			CalCtx.postSnapErrorSumMm = 0.0;
			CalCtx.postSnapErrorSampleCount = 0;
			CalCtx.warmRestartLastConsumedErrTs = Metrics::error_currentCal.lastTs();
			CalCtx.warmRestartSnapTime = Metrics::CurrentTime;
			g_snapNextProfileApply = true;
		}
		else {
			char logbuf[384];
			snprintf(logbuf, sizeof logbuf,
			         "auto_recover_from_relocalization: hmdDelta=%.3f dpos=(%.3f,%.3f,%.3f) rotRad=%.3f"
			         " priorState=%s priorValid=%d secSinceStall=%.2f trackerDelta=%.4f"
			         " -> calibration cleared, continuous-cal restarting",
			         hmdDelta, dpos.x(), dpos.y(), dpos.z(), s.lastFireRotRad,
			         (CalCtx.state == CalibrationState::Continuous) ? "Continuous" : "ContinuousStandby",
			         (int)calibration.isValid(), secSinceStall, headTrackerDelta);
			Metrics::WriteLogAnnotation(logbuf);

			// Step 0 (audit UX #3): snapshot the pre-recovery calibration
			// state so the UI's "Undo" button can restore it. Snapshot the
			// CalCtx fields the recovery is about to clear -- restoring
			// these is sufficient to put the user back in the wedged
			// calibration (which is, after all, what they were running
			// happily in until the false-positive auto-recovery clobbered
			// it). Reset the dismissed flag so this new event gets a
			// fresh banner even if a previous one was dismissed.
			s.lastAutoRecoverSnapshot.valid = true;
			s.lastAutoRecoverSnapshot.refToTargetPose = CalCtx.refToTargetPose;
			s.lastAutoRecoverSnapshot.relativePosCalibrated = CalCtx.relativePosCalibrated;
			s.lastAutoRecoverSnapshot.hasAppliedCalibrationResult = CalCtx.hasAppliedCalibrationResult;
			s.autoRecoverBannerDismissed = false;

			// Steps 1-4: wipe + restart cold. The helper does
			// calibration.Clear() + zero CalCtx fields (incl. calibratedTranslation
			// / calibratedRotation, which Clear() doesn't touch -- this was the
			// 2026-05-03 SaveProfile-persisted-wedge bug; see the helper's
			// comment) + StartContinuousCalibration() + posts the user-facing
			// message AFTER the restart (StartContinuousCalibration clears
			// CalCtx.messages internally). Step 5 (the user banner) is folded
			// in via the helper's userFacingMessage argument.
			//
			// SaveProfile is intentionally NOT called here. The next valid
			// ComputeIncremental will write the post-recovery values via the
			// existing path at the end of CalibrationTick, which is exactly
			// what we want.

			char uimsg[128];
			snprintf(uimsg, sizeof uimsg, "Quest re-localized (%.0f cm jump). Recalibrating...\n", hmdDelta * 100.0);
			// Arm the recovery-convergence watch so the next post-recovery
			// usingRelPose_fired event can emit a `[recovery][converged]`
			// line tying physical jump severity to convergence time. Uses
			// Metrics::CurrentTime so the CalibrationCalc reader sees the
			// same clock epoch (it doesn't include GLFW).
			CalCtx.recoveryWaitingSince = Metrics::CurrentTime;
			CalCtx.recoveryHmdDeltaAtStart = hmdDelta;
			RecoverFromWedgedCalibration(uimsg, "quest_relocalization_recovery");
		}

		s.lastAutoRecoverTime = now;
	}

	// Update cache for next tick.
	s.prevHmd = hmdPose;
	s.havePrevHmd = true;
	s.prevBodyTrans = std::move(currentBodyTrans);
	// Diagnostic-only: per-base distance cache for next-tick comparison
	// in the reloc_base_dists log.
	s.prevHmdToBaseDist = std::move(currentHmdToBaseDist);
}
} // namespace

void TickBaseStationDrift(double now)
{
	TickBaseStationDriftImpl(now);
}

void TickHmdRelocalizationDetector(double now)
{
	TickHmdRelocalizationDetectorImpl(now);
}

void DumpDriftSubsystemState()
{
	const double now = glfwGetTime();
	char b[240];
	snprintf(b, sizeof b, "[drift][state-dump] header now=%.3f state=%d last_reloc_time=%.3f", now, (int)CalCtx.state,
	         g_relocDetector.lastFireTime);
	Metrics::WriteLogAnnotation(b);
	Metrics::WriteLogAnnotation("[drift][state-dump] footer");
}

bool LastDetectedRelocalization(double& outAgeSeconds, double& outDeltaMeters, double& outDeltaDegrees)
{
	if (g_relocDetector.lastFireTime <= -1e8) return false;
	const double now = glfwGetTime();
	outAgeSeconds = now - g_relocDetector.lastFireTime;
	outDeltaMeters = g_relocDetector.lastFireDelta.norm();
	outDeltaDegrees = g_relocDetector.lastFireRotRad * 180.0 / EIGEN_PI;
	return true;
}

bool LastAutoRecoveryActive(double& outAge, double& outDeltaMeters)
{
	auto& s = g_relocDetector;
	if (!s.lastAutoRecoverSnapshot.valid) return false;
	if (s.autoRecoverBannerDismissed) return false;
	if (s.lastAutoRecoverTime <= -1e8) return false;
	const double now = glfwGetTime();
	const double age = now - s.lastAutoRecoverTime;
	// 60 s sticky window matches the audit suggestion. After that the
	// banner self-dismisses; the snapshot itself stays valid in case some
	// other path wants to expose Undo (currently nothing does).
	if (age > 60.0) return false;
	outAge = age;
	outDeltaMeters = s.lastFireDelta.norm();
	return true;
}

bool UndoLastAutoRecovery()
{
	auto& s = g_relocDetector;
	if (!s.lastAutoRecoverSnapshot.valid) return false;

	// Diagnostic: dump the snapshot we're about to restore. Useful when the
	// user reports "I hit Undo and tracking is still wrong" -- we want to
	// see what state we put them back into. Logged BEFORE the restore so
	// even if the restore is a no-op for some reason, we have the data.
	{
		const auto& snap = s.lastAutoRecoverSnapshot;
		const Eigen::Vector3d t = snap.refToTargetPose.translation();
		char dumpbuf[256];
		snprintf(dumpbuf, sizeof dumpbuf,
		         "undo_snapshot_dump (Undo): refToTarget_t=(%.3f,%.3f,%.3f) magnitude=%.3f relativePosCalibrated=%d "
		         "hasAppliedCalibrationResult=%d",
		         t.x(), t.y(), t.z(), t.norm(), (int)snap.relativePosCalibrated, (int)snap.hasAppliedCalibrationResult);
		Metrics::WriteLogAnnotation(dumpbuf);
	}

	// Restore the three CalCtx fields the recovery had cleared. The
	// continuous-cal tick will then re-apply the saved relative-pose
	// constraint via setRelativeTransformation on the next frame, putting
	// the body trackers back where they were before the (false-positive)
	// auto-recover clobbered the calibration.
	CalCtx.refToTargetPose = s.lastAutoRecoverSnapshot.refToTargetPose;
	CalCtx.relativePosCalibrated = s.lastAutoRecoverSnapshot.relativePosCalibrated;
	CalCtx.hasAppliedCalibrationResult = s.lastAutoRecoverSnapshot.hasAppliedCalibrationResult;

	// Invalidate the snapshot so a second Undo click does nothing.
	s.lastAutoRecoverSnapshot.valid = false;
	s.autoRecoverBannerDismissed = true;

	Metrics::WriteLogAnnotation("auto_recover_undone: pre-recovery calibration restored from snapshot");
	CalCtx.Log("Auto-recovery undone. Restored pre-recovery calibration.\n");
	return true;
}

void DismissAutoRecoveryBanner()
{
	g_relocDetector.autoRecoverBannerDismissed = true;
}

// Wedge recovery -- the canonical wipe routine. Used by both the Quest
// re-localization auto-recovery (TickHmdRelocalizationDetector) and the
// runtime wedge detector (CalibrationTick post-cal-update block).
//
// Wipes:
//   - calibration.m_estimatedTransformation, m_isValid, m_samples, etc.
//   - CalCtx.refToTargetPose (warm-start for the relative-pose constraint)
//   - CalCtx.relativePosCalibrated (so StartContinuousCalibration's
//     setRelativeTransformation call passes `false` and doesn't re-anchor
//     to the wedged value)
//   - CalCtx.hasAppliedCalibrationResult (lets the trigger-press gate
//     re-arm if the user has it enabled)
//   - CalCtx.calibratedTranslation / calibratedRotation (the values that
//     are actually persisted to the saved profile and applied to the
//     driver via ScanAndApplyProfile). project_auto_recovery_2026-05-03.md
//     called out that calibration.Clear() doesn't touch these -- leaving
//     them wedged here is what made the earlier SaveProfile-after-Clear
//     persist bad state. Zero them explicitly.
//
// Does NOT call SaveProfile. The next continuous-cal tick that produces
// a valid result will overwrite the on-disk profile via the existing
// path at the end of CalibrationTick (~line 2620), at which point we'll
// be persisting the post-recovery values, not the wedged ones.
//
// `userFacingMessage` is appended to CalCtx.messages AFTER the restart
// (StartContinuousCalibration internally clears the buffer, so it must be
// last). Pass nullptr to suppress the user-facing log if the trigger is
// ambient/silent (e.g. a runtime wedge clear that shouldn't surface UI text
// per the 2026-05-04 "user notices nothing" directive). The metrics
// annotation is the caller's responsibility -- this helper deliberately
// doesn't write one so each caller's grep key can differ.
void RecoverFromWedgedCalibration(const char* userFacingMessage, const char* recoverReason)
{
	// Capture the prior cal state BEFORE we discard it, so the log line
	// records what we just threw away. Anyone reading the session log
	// later can reconstruct "the cal we cleared was X cm with Y mm RMS"
	// without having to grep for the latest values from earlier in the
	// file.
	{
		const double priorTransMagCm = calibration.Transformation().translation().norm() * 100.0;
		const Eigen::Vector3d priorEulerDeg = calibration.EulerRotation();
		const bool priorWasValid = calibration.isValid();
		const double priorBufferSamples = static_cast<double>(calibration.SampleCount());
		char priorBuf[320];
		snprintf(priorBuf, sizeof priorBuf,
		         "recovery_prior_state: was_valid=%d trans_mag_cm=%.2f euler_deg=(%.2f,%.2f,%.2f)"
		         " sample_count=%.0f relativePosCalibrated=%d",
		         (int)priorWasValid, priorTransMagCm, priorEulerDeg.x(), priorEulerDeg.y(), priorEulerDeg.z(),
		         priorBufferSamples, (int)CalCtx.relativePosCalibrated);
		Metrics::WriteLogAnnotation(priorBuf);
	}

	calibration.Clear();
	CalCtx.ClearRuntimeCalibrationForRecovery();

	// Snap the next ScanAndApplyProfile send (one-shot). The driver's
	// SetDeviceTransform handler will see payload.lerp=false and assign
	// transform := targetTransform directly, bypassing BlendTransform.
	// Without this, the recovery's brand-new cal would be smoothly
	// interpolated through the driver's stale cached transform -- which
	// defeats the point of recovery (we WANT a discontinuity here).
	g_snapNextProfileApply = true;

	StartContinuousCalibration(recoverReason);

	if (userFacingMessage != nullptr) {
		CalCtx.Log(userFacingMessage);
	}
}
