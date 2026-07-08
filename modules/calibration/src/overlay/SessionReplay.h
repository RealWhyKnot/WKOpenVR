#pragma once

// Offline session-layer replay over a loaded spacecal recording.
//
// RunReplay (MotionRecording.h) re-runs the SOLVER closed-loop; this header
// re-runs the decision layers around it, starting with the AUTO Lock
// detector + stationary commit gate, so hysteresis tunings can be swept
// against recorded sessions instead of live VR time.
//
// AutoLockSim mirrors the live split in Calibration.cpp:
//   - UpdateDetector      <- CalibrationContext::UpdateAutoLockDetector
//   - TickCommitGate      <- CommitPendingAutoLockFlipIfStationary
// and the live cadence: the commit gate runs every tick BEFORE that tick's
// sample collection; the detector updates only on rows that produced an
// accepted sample (CollectSample -> UpdateAutoLockDetector). If the live
// ordering in CalibrationTick changes, this simulator must follow; the
// flip-parity metric against recorded auto_lock_flip annotations is the
// regression check for that coupling.
//
// Live differences, by construction:
//   - Device speeds are finite-differenced from the recorded v4 HMD /
//     head-tracker world poses (vecVelocity is not recorded).
//   - The forced head-mount lock branch (offsetCalibrated AutoPaired
//     targets) is not modeled; recordings from such sessions would pin the
//     lock on anyway.

#include "AutoLockHysteresis.h"
#include "ContinuousPrecisionFusion.h"
#include "MotionRecording.h"
#include "RecoveryPolicy.h"
#include "SnapSuppression.h"
#include "WarmRestart.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <deque>
#include <string>
#include <utility>
#include <vector>

namespace spacecal::replay {

// Live-mirror of the AUTO Lock detector state machine, parameterized for
// sweeps. See the header comment for the exact live anchors.
struct AutoLockSim
{
	explicit AutoLockSim(const autolock::HysteresisParams& params) : p(params) {}

	autolock::HysteresisParams p;
	std::deque<Eigen::AffineCompact3d> history;
	std::deque<std::pair<double, double>> madHistory; // (time, translMad)
	bool locked = false;
	bool hasPending = false;
	bool pendingTo = false;
	double pendingFirstSeen = 0.0;
	double lastFlipTime = 0.0;
	double madFloor = 0.0;
	double lastTranslMad = 0.0;
	double lastRotMad = 0.0;
	bool haveMad = false;

	int flips = 0;         // gate-committed flips (live: auto_lock_flip)
	int panics = 0;        // immediate unlocks (live: auto_lock_panic_unlock)
	int pendingQueued = 0; // queue transitions (live: auto_lock_flip_pending)

	// Every tick, before the detector update -- mirrors
	// CommitPendingAutoLockFlipIfStationary. Returns true when a flip commits.
	bool TickCommitGate(double effSpeedMps, double now)
	{
		if (!hasPending) {
			pendingFirstSeen = 0.0;
			return false;
		}
		if (pendingFirstSeen <= 0.0) pendingFirstSeen = now;
		const double heldSec = now - pendingFirstSeen;
		const auto gate = autolock::EvaluateCommitGate(pendingTo, effSpeedMps, now, heldSec, p);
		if (!gate.commit) return false;
		locked = pendingTo;
		hasPending = false;
		pendingFirstSeen = 0.0;
		lastFlipTime = now;
		++flips;
		return true;
	}

	// Only on rows that produced an accepted sample -- mirrors
	// UpdateAutoLockDetector (minus the forced head-mount branch).
	void UpdateDetector(const Eigen::AffineCompact3d& refWorld, const Eigen::AffineCompact3d& tgtWorld, double now)
	{
		const Eigen::AffineCompact3d rel = refWorld.inverse() * tgtWorld;
		history.push_back(rel);
		while (history.size() > p.historyMax)
			history.pop_front();
		if (history.size() < p.samplesNeeded) {
			locked = false;
			hasPending = false;
			return;
		}

		const double translMad = autolock::RobustTranslDeviation(history);
		const double rotMad = autolock::RobustRotDeviation(history);
		lastTranslMad = translMad;
		lastRotMad = rotMad;
		haveMad = true;

		madHistory.emplace_back(now, translMad);
		while (!madHistory.empty() && (now - madHistory.front().first) > p.madFloorWindowSec)
			madHistory.pop_front();
		double floor = madHistory.front().second;
		for (const auto& entry : madHistory)
			floor = std::min(floor, entry.second);
		madFloor = floor;

		if (locked && autolock::IsPanicLevelDeviation(translMad, rotMad, p)) {
			locked = false;
			hasPending = false;
			pendingFirstSeen = 0.0;
			lastFlipTime = now;
			++panics;
			return;
		}

		const bool verdict =
		    autolock::VerdictWithHysteresis(translMad, rotMad, locked, autolock::EnterThresholdFor(madFloor, p), p);
		if (verdict != locked) {
			if (!hasPending || pendingTo != verdict) ++pendingQueued;
			hasPending = true;
			pendingTo = verdict;
		}
		else if (hasPending) {
			hasPending = false;
		}
	}

	bool SettledNow(double now) const
	{
		return autolock::IsSettled(locked, lastTranslMad, madFloor, now - lastFlipTime, p);
	}
};

struct AutoLockSimResult
{
	bool succeeded = false;
	std::string error;
	int rowsProcessed = 0;
	int detectorUpdates = 0;
	int lockFlips = 0;
	int panicUnlocks = 0;
	int pendingQueued = 0;
	double flipsPerMin = 0.0;
	int measuredTicks = 0; // rows with enough MAD history for a verdict
	int settledTicks = 0;
	int lockedTicks = 0;
	double settledPct = 0.0;
	double lockedPct = 0.0;
	double medianTranslMadMm = 0.0;
	double medianEnterThresholdMm = 0.0;
	// The live session's own decision stream, for parity checks (0 when the
	// recording carries no such annotations).
	int recordedFlips = 0;
	int recordedPanics = 0;
	int recordedPendings = 0;
};

// Replay the AUTO Lock decision layer over a recording. Requires the v4 raw
// HMD (+ head tracker) columns for the stationary gate's speed source.
inline AutoLockSimResult RunAutoLockSim(const LoadedRecording& rec, const autolock::HysteresisParams& params)
{
	AutoLockSimResult res;
	if (!rec.error.empty()) {
		res.error = rec.error;
		return res;
	}
	if (rec.rows.empty()) {
		res.error = "Recording has no replayable rows.";
		return res;
	}
	if (!rec.hasLockedSnapColumns) {
		res.error = "autolock_sim_requires_v4";
		return res;
	}

	for (const auto& event : rec.annotations) {
		if (event.key == "auto_lock_flip")
			++res.recordedFlips;
		else if (event.key == "auto_lock_panic_unlock")
			++res.recordedPanics;
		else if (event.key == "auto_lock_flip_pending")
			++res.recordedPendings;
	}

	AutoLockSim sim(params);
	std::vector<double> madSamplesMm;
	std::vector<double> enterSamplesMm;
	bool havePrev = false;
	double prevTime = 0.0;
	Eigen::Vector3d prevHmd = Eigen::Vector3d::Zero();
	bool havePrevTracker = false;
	Eigen::Vector3d prevTracker = Eigen::Vector3d::Zero();
	double lastHmdSpeed = 0.0;
	double lastTrackerSpeed = -1.0;

	for (const auto& row : rec.rows) {
		++res.rowsProcessed;

		// Finite-difference speeds from the recorded world poses. dt of zero
		// (duplicate timestamps) keeps the previous speed instead of dividing
		// by zero; a tracker-invalid row resets its baseline so a long
		// invalid gap can't read as a huge jump.
		if (row.hasHmdPose) {
			if (havePrev) {
				const double dt = row.timestamp - prevTime;
				if (dt > 1e-6) {
					lastHmdSpeed = (row.hmd.trans - prevHmd).norm() / dt;
					if (row.headTrackerValid && havePrevTracker) {
						lastTrackerSpeed = (row.headTracker.trans - prevTracker).norm() / dt;
					}
					else {
						lastTrackerSpeed = -1.0;
					}
				}
			}
			prevHmd = row.hmd.trans;
			prevTime = row.timestamp;
			havePrev = true;
			if (row.headTrackerValid) {
				prevTracker = row.headTracker.trans;
				havePrevTracker = true;
			}
			else {
				havePrevTracker = false;
			}
		}
		// The witness rig runs Corroborate whenever the tracker is valid;
		// EffectiveSpeedMps falls back to HMD-only on the -1 sentinel.
		const double effSpeed =
		    snap_suppression::EffectiveSpeedMps(HeadMountMode::Corroborate, lastHmdSpeed, lastTrackerSpeed);

		sim.TickCommitGate(effSpeed, row.timestamp);

		const bool sampleAccepted =
		    !row.hasSampleDiagnostics || (row.sampleObserved && row.sampleAccepted && row.sample.valid);
		if (sampleAccepted) {
			Eigen::AffineCompact3d refW;
			refW.linear() = row.ref.rot;
			refW.translation() = row.ref.trans;
			Eigen::AffineCompact3d tgtW;
			tgtW.linear() = row.target.rot;
			tgtW.translation() = row.target.trans;
			sim.UpdateDetector(refW, tgtW, row.timestamp);
			++res.detectorUpdates;
		}

		if (sim.haveMad) {
			++res.measuredTicks;
			if (sim.SettledNow(row.timestamp)) ++res.settledTicks;
			if (sim.locked) ++res.lockedTicks;
			madSamplesMm.push_back(sim.lastTranslMad * 1000.0);
			enterSamplesMm.push_back(autolock::EnterThresholdFor(sim.madFloor, params) * 1000.0);
		}
	}

	res.lockFlips = sim.flips;
	res.panicUnlocks = sim.panics;
	res.pendingQueued = sim.pendingQueued;
	const double durationSec = rec.rows.back().timestamp - rec.rows.front().timestamp;
	if (durationSec > 1.0) {
		res.flipsPerMin = (res.lockFlips + res.panicUnlocks) * 60.0 / durationSec;
	}
	if (res.measuredTicks > 0) {
		res.settledPct = 100.0 * res.settledTicks / res.measuredTicks;
		res.lockedPct = 100.0 * res.lockedTicks / res.measuredTicks;
	}
	auto median = [](std::vector<double>& v) {
		if (v.empty()) return 0.0;
		const std::size_t mid = v.size() / 2;
		std::nth_element(v.begin(), v.begin() + mid, v.end());
		return v[mid];
	};
	res.medianTranslMadMm = median(madSamplesMm);
	res.medianEnterThresholdMm = median(enterSamplesMm);
	res.succeeded = true;
	return res;
}

// ---------------------------------------------------------------------------
// Full session replay: solver + auto-lock + relocalization recovery,
// sequenced per row in the live CalibrationTick order (commit gate ->
// sample/solve -> reloc handling -> detector update).

struct SessionReplayOptions
{
	autolock::HysteresisParams autoLock;
	// Solver shape (mirrors ReplayOptions).
	std::size_t maxContinuousSamples = 200;
	double threshold = 1.5;
	double maxRelError = 0.005;
	bool ignoreOutliers = true;
	bool lockRelativePosition = true;
	// Weighted relpose average (live default on) vs the experimental Kalman
	// fusion accept (live default off) -- decoupled exactly like the runtime.
	bool precisionWeightedRelPose = true;
	bool fusionAccept = false;
	ReplaySeedMode seedMode = ReplaySeedMode::Recorded;
	Eigen::Vector3d seedTransCm = Eigen::Vector3d::Zero();
	Eigen::Vector3d seedRotDeg = Eigen::Vector3d::Zero();
	// Relocalization recovery. Jumps >= threshold take the recovery path;
	// smaller ones are the live-uncorrected band.
	double relocRecoverThresholdM = 0.30; // kRelocAutoRecoverThresholdM
	// Mirror the runtime's dead-frame sample eviction: a classified snap or
	// reanchor drops samples collected before the jump (they describe the
	// pre-reanchor frame). Off = pre-eviction runtime behavior, for A/B.
	bool evictSamplesOnFrameJump = true;
	// Mirror the locked-relpose accept gates (quality band + step bounds,
	// RelPoseLockGate.h). Off = pre-gate runtime behavior, for A/B.
	bool lockedStepGate = true;
};

struct SessionReplayResult
{
	bool succeeded = false;
	std::string error;
	int rowsProcessed = 0;
	int accepts = 0;
	// Auto-lock layer.
	int lockFlips = 0;
	int panicUnlocks = 0;
	double settledPct = 0.0;
	// Reloc layer.
	int relocsSeen = 0;
	int snapSuppressed = 0;
	int recoveryHolds = 0;
	int recoveryReanchors = 0;
	int destructiveClears = 0;
	int samplesEvicted = 0;
	int warmRestartSnaps = 0;
	int subThresholdRelocs = 0;
	double subThresholdResidualCm = 0.0; // uncorrected world offset left behind
	// Applied-transform trajectory (net movement of the world).
	double totalAppliedPathCm = 0.0;
	double peakAppliedStepCm = 0.0;
	// The subset of the applied path that did NOT arrive through a classified
	// event (session-first candidate, snap, reanchor, warm-restart re-acquire):
	// solver wander the user experiences as the world sliding.
	double unclassifiedPathCm = 0.0;
	double maxUnclassifiedStepCm = 0.0;
	double wanderPer10MinCm = 0.0;
	Eigen::Vector3d netAppliedDriftCm = Eigen::Vector3d::Zero();
	bool seedApplied = false;
};

inline SessionReplayResult RunSessionReplay(const LoadedRecording& rec, const SessionReplayOptions& opts)
{
	SessionReplayResult res;
	if (!rec.error.empty()) {
		res.error = rec.error;
		return res;
	}
	if (rec.rows.empty()) {
		res.error = "Recording has no replayable rows.";
		return res;
	}
	if (!rec.hasLockedSnapColumns) {
		res.error = "session_replay_requires_v4";
		return res;
	}

	CalibrationCalc calc;
	calc.enableStaticRecalibration = false;
	calc.lockRelativePosition = opts.lockRelativePosition;
	calc.SetPrecisionWeightedRelPose(opts.precisionWeightedRelPose);

	// Applied transform: what the driver would render. Solver accepts write it
	// absolutely (or fuse into it) -- the same single variable the live tick
	// shares.
	Eigen::AffineCompact3d applied = Eigen::AffineCompact3d::Identity();
	bool hasApplied = false;
	double accumPrecision = 0.0;
	int disagreeStreak = 0;
	Eigen::AffineCompact3d seedTransform = Eigen::AffineCompact3d::Identity();
	{
		bool seed = false;
		if (opts.seedMode == ReplaySeedMode::Recorded && rec.seedProfile.valid) {
			seedTransform = ReplayProfileTransform(rec.seedProfile.rotDeg, rec.seedProfile.transCm);
			seed = true;
		}
		else if (opts.seedMode == ReplaySeedMode::Explicit) {
			seedTransform = ReplayProfileTransform(opts.seedRotDeg, opts.seedTransCm);
			seed = true;
		}
		if (seed) {
			calc.SeedEstimatedTransformation(seedTransform, /*annotate=*/false);
			applied = seedTransform;
			hasApplied = true;
			accumPrecision = spacecal::precision::kSeedPriorPrecision;
			res.seedApplied = true;
		}
	}
	Eigen::Vector3d firstAppliedCm = applied.translation() * 100.0;
	Eigen::Vector3d prevAppliedCm = firstAppliedCm;

	AutoLockSim sim(opts.autoLock);

	// Reloc events primarily from the detector annotations, which carry the
	// live-measured jump vector; the per-row reloc_detected column
	// under-records in field logs (its float-equality stamp misses most
	// fires). Each event is consumed by the first row at-or-after its time.
	struct RelocEvent
	{
		double time = 0.0;
		bool hasDelta = false;
		Eigen::Vector3d deltaM = Eigen::Vector3d::Zero();
		// Live witness displacement from the snap_corroboration_inputs
		// annotation, written at the runtime decision point. Row-cadence
		// finite differences overstate the witness delta (rows are much
		// coarser than detector ticks), which flips live-classified snaps
		// into "witness moved" holds in replay -- so classification prefers
		// the recorded value whenever the session logged one.
		bool hasLiveTrackerDelta = false;
		double liveTrackerDeltaM = -1.0;
	};
	// Warm-restart engages (put-headset-back-on). Replayed so the away-gap
	// sample eviction behaves as it did live.
	struct AwayEvent
	{
		double time = 0.0;
		double awayForS = 0.0;
	};
	std::vector<RelocEvent> relocEvents;
	std::vector<AwayEvent> awayEvents;
	for (const auto& event : rec.annotations) {
		if (event.key == "hmd_relocalization_detected") {
			RelocEvent e;
			e.time = event.time;
			double dx = 0.0, dy = 0.0, dz = 0.0;
			const std::size_t pos = event.raw.find("dx=");
			if (pos != std::string::npos &&
			    std::sscanf(event.raw.c_str() + pos, "dx=%lf dy=%lf dz=%lf", &dx, &dy, &dz) == 3) {
				e.deltaM = Eigen::Vector3d(dx, dy, dz);
				e.hasDelta = true;
			}
			relocEvents.push_back(e);
		}
		else if (event.key == "snap_corroboration_inputs") {
			// Logged within the same detector tick as its reloc event; attach
			// to the most recent one.
			double trackerDelta = -1.0;
			const std::size_t pos = event.raw.find("headTrackerDelta=");
			if (pos != std::string::npos &&
			    std::sscanf(event.raw.c_str() + pos, "headTrackerDelta=%lf", &trackerDelta) == 1 &&
			    !relocEvents.empty() && event.time - relocEvents.back().time < 0.25) {
				relocEvents.back().hasLiveTrackerDelta = true;
				relocEvents.back().liveTrackerDeltaM = trackerDelta;
			}
		}
		else if (event.key == "[warm-restart][snap]") {
			AwayEvent e;
			e.time = event.time;
			const std::size_t pos = event.raw.find("away_for_s=");
			if (pos != std::string::npos && std::sscanf(event.raw.c_str() + pos, "away_for_s=%lf", &e.awayForS) == 1) {
				awayEvents.push_back(e);
			}
		}
	}
	std::size_t nextReloc = 0;
	std::size_t nextAway = 0;

	const std::size_t window = opts.maxContinuousSamples > 0 ? opts.maxContinuousSamples : 200;
	const std::size_t drop = std::max<std::size_t>(1, window / 10);

	bool havePrev = false;
	double prevTime = 0.0;
	Eigen::Vector3d prevHmd = Eigen::Vector3d::Zero();
	bool havePrevTracker = false;
	Eigen::Vector3d prevTracker = Eigen::Vector3d::Zero();
	double lastHmdSpeed = 0.0;
	double lastTrackerSpeed = -1.0;
	int measuredTicks = 0, settledTicks = 0;

	// The first accepted candidate after a frame-moving event (session start,
	// snap, reanchor, warm-restart wake) lands through the motion gate /
	// reanchor absorption live -- its step is classified, not wander.
	int classifiedAcceptBudget = 1;
	auto stepApplied = [&](const Eigen::Vector3d& newCm, bool classified) {
		if (hasApplied) {
			const double stepCm = (newCm - prevAppliedCm).norm();
			res.totalAppliedPathCm += stepCm;
			res.peakAppliedStepCm = std::max(res.peakAppliedStepCm, stepCm);
			if (!classified) {
				res.unclassifiedPathCm += stepCm;
				res.maxUnclassifiedStepCm = std::max(res.maxUnclassifiedStepCm, stepCm);
			}
		}
		prevAppliedCm = newCm;
		if (!hasApplied) {
			firstAppliedCm = newCm;
			hasApplied = true;
		}
	};

	for (const auto& row : rec.rows) {
		++res.rowsProcessed;
		const double now = row.timestamp;

		// Per-row deltas + finite-diff speeds from the recorded world poses.
		double hmdDeltaM = -1.0, trackerDeltaM = -1.0;
		Eigen::Vector3d hmdDeltaVec = Eigen::Vector3d::Zero();
		if (row.hasHmdPose) {
			if (havePrev) {
				hmdDeltaVec = row.hmd.trans - prevHmd;
				hmdDeltaM = hmdDeltaVec.norm();
				const double dt = now - prevTime;
				if (dt > 1e-6) {
					lastHmdSpeed = hmdDeltaM / dt;
					if (row.headTrackerValid && havePrevTracker) {
						trackerDeltaM = (row.headTracker.trans - prevTracker).norm();
						lastTrackerSpeed = trackerDeltaM / dt;
					}
					else {
						lastTrackerSpeed = -1.0;
					}
				}
			}
			prevHmd = row.hmd.trans;
			prevTime = now;
			havePrev = true;
			if (row.headTrackerValid) {
				prevTracker = row.headTracker.trans;
				havePrevTracker = true;
			}
			else {
				havePrevTracker = false;
			}
		}
		const double effSpeed =
		    snap_suppression::EffectiveSpeedMps(HeadMountMode::Corroborate, lastHmdSpeed, lastTrackerSpeed);

		// 1. Commit-gate tick (live: before sample collection).
		sim.TickCommitGate(effSpeed, now);

		// 2. Sample intake + solve (accepted-sample rows only).
		const bool sampleAccepted =
		    !row.hasSampleDiagnostics || (row.sampleObserved && row.sampleAccepted && row.sample.valid);
		if (sampleAccepted) {
			Sample s = row.hasSampleDiagnostics ? row.sample : Sample(row.ref, row.target, row.timestamp);
			calc.PushSample(s);
			while (calc.SampleCount() > window)
				calc.ShiftSample();
			if (calc.SampleCount() >= window) {
				bool lerp = false;
				calc.SetStepGateBypass(!opts.lockedStepGate || classifiedAcceptBudget > 0 || opts.fusionAccept);
				if (calc.ComputeIncremental(lerp, opts.threshold, opts.maxRelError, opts.ignoreOutliers)) {
					++res.accepts;
					if (opts.fusionAccept) {
						const double measPrec = spacecal::precision::MeasurementPrecision(calc.MeanSquaredLeverArmM2());
						const double disagreeM = (calc.Transformation().translation() - applied.translation()).norm();
						if (spacecal::precision::NoteSeedDisagreement(disagreeStreak, disagreeM)) {
							applied = calc.Transformation();
							accumPrecision = std::min(measPrec, spacecal::precision::kMaxConfidence);
						}
						else {
							const double gain = spacecal::precision::FusionGain(accumPrecision, measPrec);
							applied = spacecal::precision::Fuse(applied, calc.Transformation(), gain);
							accumPrecision = std::min(accumPrecision + measPrec, spacecal::precision::kMaxConfidence);
						}
					}
					else {
						applied = calc.Transformation();
					}
					const bool classifiedStep = classifiedAcceptBudget > 0 || calc.LastAcceptWasConsensusStep();
					if (classifiedAcceptBudget > 0) --classifiedAcceptBudget;
					stepApplied(applied.translation() * 100.0, classifiedStep);
				}
				for (std::size_t i = 0; i < drop; ++i)
					calc.ShiftSample();
			}
		}

		// 3.5 Warm-restart engages: mirror the runtime's away-gap eviction so
		// pre-gap samples leave the window in replay exactly as they did live.
		while (nextAway < awayEvents.size() && awayEvents[nextAway].time <= now) {
			++res.warmRestartSnaps;
			if (opts.evictSamplesOnFrameJump &&
			    awayEvents[nextAway].awayForS >= spacecal::warm_restart::kSampleEvictionAwayGapSeconds) {
				res.samplesEvicted += (int)calc.EvictSamplesBefore(now);
			}
			classifiedAcceptBudget = 1;
			++nextAway;
		}

		// 4. Relocalization handling: annotation events (preferred, exact
		// live-measured delta) plus any column-flagged row not covered by one.
		bool relocThisRow = row.relocDetected;
		Eigen::Vector3d relocDeltaVec = hmdDeltaVec;
		double relocDeltaM = hmdDeltaM;
		bool haveLiveTrackerDelta = false;
		double liveTrackerDeltaM = -1.0;
		while (nextReloc < relocEvents.size() && relocEvents[nextReloc].time <= now) {
			relocThisRow = true;
			if (relocEvents[nextReloc].hasDelta) {
				relocDeltaVec = relocEvents[nextReloc].deltaM;
				relocDeltaM = relocDeltaVec.norm();
			}
			if (relocEvents[nextReloc].hasLiveTrackerDelta) {
				haveLiveTrackerDelta = true;
				liveTrackerDeltaM = relocEvents[nextReloc].liveTrackerDeltaM;
			}
			++nextReloc;
		}
		if (relocThisRow && relocDeltaM >= 0.0) {
			++res.relocsSeen;
			const double gateTrackerDeltaM = haveLiveTrackerDelta ? liveTrackerDeltaM : trackerDeltaM;
			const bool snap =
			    snap_suppression::IsJumpClassifiedAsSnap(HeadMountMode::Corroborate, relocDeltaM, gateTrackerDeltaM);
			if (snap) {
				// Live: fast re-anchor, world holds; no residual introduced.
				++res.snapSuppressed;
				if (opts.evictSamplesOnFrameJump) {
					res.samplesEvicted += (int)calc.EvictSamplesBefore(now);
				}
				classifiedAcceptBudget = 1;
			}
			else if (relocDeltaM >= opts.relocRecoverThresholdM) {
				const bool witnessValid = gateTrackerDeltaM >= 0.0;
				const bool witnessSaysHeadMoved =
				    witnessValid && gateTrackerDeltaM >= snap_suppression::kSnapTrackerMaxDispM;
				switch (spacecal::recovery::ChooseRelocRecoveryAction(witnessValid, witnessSaysHeadMoved,
				                                                      res.seedApplied,
				                                                      /*warmRestartFailed=*/false)) {
					case spacecal::recovery::RecoveryAction::Hold:
						++res.recoveryHolds;
						break;
					case spacecal::recovery::RecoveryAction::ReanchorToProfile:
						++res.recoveryReanchors;
						if (opts.evictSamplesOnFrameJump) {
							res.samplesEvicted += (int)calc.EvictSamplesBefore(now);
						}
						applied = seedTransform;
						stepApplied(applied.translation() * 100.0, /*classified=*/true);
						classifiedAcceptBudget = 1;
						break;
					case spacecal::recovery::RecoveryAction::DestructiveClear:
						++res.destructiveClears;
						calc.Clear();
						break;
				}
			}
			else {
				// The live-uncorrected band: a frame jump under the recovery
				// threshold while the physical head (witness) stayed put.
				const bool headStill = trackerDeltaM >= 0.0 && trackerDeltaM < snap_suppression::kSnapTrackerMaxDispM;
				if (headStill) {
					++res.subThresholdRelocs;
					res.subThresholdResidualCm += relocDeltaM * 100.0;
				}
			}
		}

		// 5. Detector update (accepted-sample rows; live cadence).
		if (sampleAccepted) {
			Eigen::AffineCompact3d refW;
			refW.linear() = row.ref.rot;
			refW.translation() = row.ref.trans;
			Eigen::AffineCompact3d tgtW;
			tgtW.linear() = row.target.rot;
			tgtW.translation() = row.target.trans;
			sim.UpdateDetector(refW, tgtW, now);
		}
		if (sim.haveMad) {
			++measuredTicks;
			if (sim.SettledNow(now)) ++settledTicks;
		}
	}

	res.lockFlips = sim.flips;
	res.panicUnlocks = sim.panics;
	if (measuredTicks > 0) res.settledPct = 100.0 * settledTicks / measuredTicks;
	if (hasApplied) res.netAppliedDriftCm = prevAppliedCm - firstAppliedCm;
	const double sessionSec = rec.rows.back().timestamp - rec.rows.front().timestamp;
	if (sessionSec > 1.0) {
		res.wanderPer10MinCm = res.unclassifiedPathCm * 600.0 / sessionSec;
	}
	res.succeeded = true;
	return res;
}

} // namespace spacecal::replay
