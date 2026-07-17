#include "CalibrationWatchdogs.h"

#include "CalibrationInternal.h" // calibration solver + shared auto-lock/geometry-shift counters
#include "CalibrationMetrics.h"
#include "CalibrationRecoveryTick.h" // ArmReanchorToProfile / EvictDeadFrameSamples / LastWitnessHealth
#include "AutoLockHysteresis.h"      // spacecal::autolock::IsSettled / EnterThresholdFor
#include "GeometryShiftDetector.h"   // spacecal::geometry_shift::kMinSustainedSpikes
#include "HeadMountTargetBinding.h"  // wkopenvr::headmount::EffectiveHeadMountMode
#include "TrackingStyle.h"           // HmdPoseEventRecoveryEligible
#include "WarmRestart.h"             // spacecal::warm_restart::ShouldEngage + tunables
#include "WitnessHealth.h"           // spacecal::witness_health::ValidPct / LastValidSec

#include <cmath>
#include <cstdio>

#include <Eigen/Dense>

// Warm-restart detection. In plain Continuous mode, the user takes the
// HMD off (activity level falls to Standby), comes back later, puts it
// on (activity level snaps back to UserInteraction). If the away duration
// cleared the threshold and the saved profile is valid, snap the driver
// to the saved transform and grant a validation grace window.
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
void TickWarmRestartDetection(CalibrationContext& ctx, double time)
{
	{
		auto* vrSystem = vr::VRSystem();
		const vr::EDeviceActivityLevel activity =
		    vrSystem ? vrSystem->GetTrackedDeviceActivityLevel(vr::k_unTrackedDeviceIndex_Hmd)
		             : vr::k_EDeviceActivityLevel_UserInteraction;
		const bool nowPresent = (activity == vr::k_EDeviceActivityLevel_UserInteraction) ||
		                        (activity == vr::k_EDeviceActivityLevel_UserInteraction_Timeout);
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
			const Eigen::Vector3d hmdPosNow(hmdPose.vecPosition[0], hmdPose.vecPosition[1], hmdPose.vecPosition[2]);
			const bool hmdPoseValid =
			    hmdPose.poseIsValid && hmdPose.result == vr::ETrackingResult::TrackingResult_Running_OK;

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
			}
			else if (!wasPresent && nowPresent) {
				const double awayFor = (ctx.userAwaySince > 0.0) ? (time - ctx.userAwaySince) : 0.0;
				const double awayPosDelta =
				    (ctx.hmdLastKnownPosValid && hmdPoseValid) ? (hmdPosNow - ctx.hmdLastKnownPosWhenAway).norm() : 0.0;
				const bool hmdEventRecoveryEligible = HmdPoseEventRecoveryEligible(ctx.state, ctx.trackingStyle);
				const spacecal::warm_restart::EngageInput engageIn = {
				    wasPresent,
				    nowPresent,
				    awayFor,
				    ctx.validProfile,
				    hmdEventRecoveryEligible,
				    awayPosDelta,
				    ctx.warmRestartTickId,
				};
				const bool engaged = ctx.CustomChecksActive() && spacecal::warm_restart::ShouldEngage(engageIn);

				// Diagnostic: max-away ceiling. When the proximity path
				// would have engaged but awayFor crossed the ceiling, log
				// it explicitly so a session that "went to sleep then woke
				// to cold cal" leaves a paper trail rather than being
				// invisible. Only logs when this is the suppress reason
				// (not when pose-jump fast-path took over).
				if (!engaged && ctx.CustomChecksActive() && ctx.validProfile && hmdEventRecoveryEligible &&
				    ctx.warmRestartTickId >= spacecal::warm_restart::kColdStartGraceTicks &&
				    awayFor > spacecal::warm_restart::kMaxAwaySeconds &&
				    awayPosDelta < spacecal::warm_restart::kPositionJumpFastPathM) {
					char cbuf[200];
					snprintf(cbuf, sizeof cbuf,
					         "[warm-restart][ceiling-suppressed] away_for_s=%.1f"
					         " max_away_s=%.0f pos_delta_m=%.3f",
					         awayFor, spacecal::warm_restart::kMaxAwaySeconds, awayPosDelta);
					Metrics::WriteLogAnnotation(cbuf);
				}

				if (engaged) {
					// Put-headset-back-on re-anchor: arm grace and ramp to the
					// saved profile at constant velocity (ArmReanchorToProfile)
					// rather than snapping. Resets the post-snap bias accumulator
					// and pins the last-consumed err timestamp so pre-snap
					// retargeting errors don't feed the post-snap mean.
					ctx.warmRestartReanchorCount = 0; // fresh warm-restart episode
					// A long off-head gap is long enough for an inside-out
					// headset to re-anchor its world frame; samples from
					// before the gap would then poison the solve window until
					// they age out. Short breaks keep their samples.
					if (awayFor >= spacecal::warm_restart::kSampleEvictionAwayGapSeconds) {
						EvictDeadFrameSamples(ctx, "away_gap");
					}
					// An eviction-length gap is long enough for an inside-out
					// headset to have re-anchored its frame while off-head;
					// that provenance decides the validation-failure action.
					ArmReanchorToProfile(ctx,
					                     /*frameMoved=*/awayFor >=
					                         spacecal::warm_restart::kSampleEvictionAwayGapSeconds);
					const double mag = std::sqrt(ctx.calibratedTranslation.x() * ctx.calibratedTranslation.x() +
					                             ctx.calibratedTranslation.y() * ctx.calibratedTranslation.y() +
					                             ctx.calibratedTranslation.z() * ctx.calibratedTranslation.z());
					const bool fastPath = awayPosDelta >= spacecal::warm_restart::kPositionJumpFastPathM &&
					                      awayFor < spacecal::warm_restart::kMinAwaySeconds;
					char wbuf[260];
					snprintf(wbuf, sizeof wbuf,
					         "[warm-restart][snap] away_for_s=%.1f state=%d"
					         " grace_samples=%d profile_magnitude_cm=%.2f"
					         " pos_delta_m=%.3f mad_at_snap_mm=%.3f path=%s",
					         awayFor, (int)ctx.state, ctx.warmRestartGraceSamples, mag, awayPosDelta,
					         ctx.warmRestartMadAtSnap * 1000.0,
					         fastPath ? "pose_jump_fast_path" : "proximity_and_time");
					Metrics::WriteLogAnnotation(wbuf);
				}
				ctx.userAwaySince = 0.0;
				ctx.hmdLastKnownPosValid = false;
			}
			ctx.lastUserPresent = nowPresent;
		}
	}
}

// Diagnostic: trace relPose-cal validity flips. The flag is set/cleared
// from several call sites and is currently only externally visible inside
// the rate-limited usingRelPose_fired event. Catching every change is
// cheap (one bool compare per tick) and reveals the cycle: cal converges
// -> relPosCal=1 -> geometry-shift fire historically cleared it -> 0.
// After the T1.5 fix this trace tells us whether the constraint actually
// survives geometry-shift events.
void TraceRelPoseCalFlips(CalibrationContext& ctx)
{
	{
		static bool s_lastRelPosCal = false;
		const bool nowRelPosCal = ctx.relativePosCalibrated;
		if (nowRelPosCal != s_lastRelPosCal) {
			char rpcBuf[160];
			snprintf(rpcBuf, sizeof rpcBuf, "[relposcal-change] prev=%d now=%d state=%d lockMode=%d",
			         (int)s_lastRelPosCal, (int)nowRelPosCal, (int)ctx.state, (int)ctx.lockRelativePositionMode);
			Metrics::WriteLogAnnotation(rpcBuf);
			s_lastRelPosCal = nowRelPosCal;
		}
	}
}

// Tracker pose-freshness check. The driver writes a QPC timestamp into
// devicePoseSampleTimes[] each time a pose is published. If the ref or
// target sample timestamp hasn't advanced in the last 5 s, that device
// has gone silent (the pose value may still appear valid because the
// last-known position is still in the array, but no new data has
// arrived). Log throttled to once per 30 s per device so a chronic
// silence doesn't flood. ID < 0 (unassigned) is skipped.
void TickPoseFreshnessWatchdog(CalibrationContext& ctx, double time)
{
	if (ctx.state == CalibrationState::Continuous || ctx.state == CalibrationState::ContinuousStandby) {
		static double s_lastFreshnessLogTime = -1e9;
		const double freshnessWarnSec = 5.0;
		LARGE_INTEGER freq;
		QueryPerformanceFrequency(&freq);
		LARGE_INTEGER nowCounter;
		QueryPerformanceCounter(&nowCounter);

		auto checkFresh = [&](int id, const char* whichLabel) {
			if (id < 0 || id >= (int)vr::k_unMaxTrackedDeviceCount) return;
			const auto& sampleTime = ctx.devicePoseSampleTimes[id];
			if (sampleTime.QuadPart == 0) return; // never sampled
			const double ageSec = double(nowCounter.QuadPart - sampleTime.QuadPart) / double(freq.QuadPart);
			if (ageSec >= freshnessWarnSec && (time - s_lastFreshnessLogTime) >= 30.0) {
				s_lastFreshnessLogTime = time;
				char freshBuf[200];
				snprintf(freshBuf, sizeof freshBuf,
				         "[tracker-pose-stale] which=%s id=%d age_sec=%.2f"
				         " result=%d poseIsValid=%d",
				         whichLabel, id, ageSec, (int)ctx.devicePoses[id].result, (int)ctx.devicePoses[id].poseIsValid);
				Metrics::WriteLogAnnotation(freshBuf);
			}
		};
		checkFresh(ctx.referenceID, "reference");
		checkFresh(ctx.targetID, "target");
	}
}

// Stuck-cal watchdog. If we've been in Continuous state for >60 s but
// error_currentCal has not received a new sample in the last 30 s, the
// cal solver is not actually running -- ComputeIncremental isn't being
// called, or is rejecting every input, or the time series has otherwise
// stopped advancing. Edge-triggered, one log per detection, re-armed
// when error_currentCal advances again.
void TickStuckCalWatchdog(CalibrationContext& ctx, double time)
{
	if (ctx.state == CalibrationState::Continuous) {
		static double s_lastCalActiveTs = 0.0;
		static double s_lastStuckLogTime = -1e9;
		const double errLastTs = Metrics::error_currentCal.lastTs();
		if (errLastTs > s_lastCalActiveTs) {
			s_lastCalActiveTs = errLastTs;
		}
		// Staleness is decided entirely in the Metrics epoch: the series
		// timestamps and Metrics::CurrentTime come from the same
		// QueryPerformanceCounter base, so no cross-clock guard is needed.
		// Only the log throttle below uses the tick's `time`.
		const bool stuck = (Metrics::error_currentCal.size() > 0 && Metrics::CurrentTime - s_lastCalActiveTs >= 30.0);
		if (stuck && (time - s_lastStuckLogTime) >= 30.0) {
			s_lastStuckLogTime = time;
			char stuckBuf[280];
			snprintf(stuckBuf, sizeof stuckBuf,
			         "[cal-stuck] no_compute_for_sec=%.2f state=%d lockRel=%d"
			         " err_samples=%d refID=%d targetID=%d",
			         Metrics::CurrentTime - s_lastCalActiveTs, (int)ctx.state, (int)ctx.lockRelativePosition,
			         Metrics::error_currentCal.size(), ctx.referenceID, ctx.targetID);
			Metrics::WriteLogAnnotation(stuckBuf);
		}
	}
}

// Periodic cal heartbeat. Throttled to once per 10 s while in Continuous
// or ContinuousStandby. Emits a one-line "you are here" snapshot so a
// post-session reader can scrub the log without grepping multiple event
// types just to learn the cal's current state. Fields chosen to maximize
// signal-per-character: state, lock resolution (mode + resolved + detector
// internal), recent error level, sample-buffer size, time since last
// reset.
void EmitCalHeartbeat(CalibrationContext& ctx, double time)
{
	if (ctx.state == CalibrationState::Continuous || ctx.state == CalibrationState::ContinuousStandby) {
		static double s_lastHeartbeatTime = -1e9;
		if ((time - s_lastHeartbeatTime) >= 10.0) {
			s_lastHeartbeatTime = time;
			const auto& errSeries = Metrics::error_currentCal;
			const double errLast = errSeries.size() > 0 ? errSeries.last() : 0.0;
			// Geometry-shift cooldown remaining (0 when no active cooldown).
			// The deadline ctx.geometryShiftCooldownUntil is wall-clock-style
			// time matching CalibrationTick's `time` argument; the heartbeat
			// log shows seconds until expiry so a reader doesn't have to
			// subtract the session time themselves.
			const double cooldownRemaining =
			    (ctx.geometryShiftCooldownUntil > time) ? (ctx.geometryShiftCooldownUntil - time) : 0.0;
			const double translMadMm = g_lastAutoLockTranslMad * 1000.0;
			const double rotMadDeg = g_lastAutoLockRotMad * 180.0 / EIGEN_PI;
			// Pending-flip held duration. Zero when no flip is queued so a
			// reader can distinguish a stable autoLockEff from one that is
			// about to commit a transition. autoLockPendingFlipFirstSeen
			// is set the first tick a pending flip appears and reset on
			// commit / abandon, so a non-zero value during pending means
			// the held-duration is meaningful.
			const double autoLockHeldSec = (ctx.autoLockHasPendingFlip && ctx.autoLockPendingFlipFirstSeen > 0.0)
			                                   ? (time - ctx.autoLockPendingFlipFirstSeen)
			                                   : 0.0;
			// Settled signal: see AutoLockHysteresis.h::IsSettled. The settled
			// rate over a session is the headline success metric for the
			// 2026-05-25 settling fix -- a healthy run should sit at
			// settled=yes for the majority of heartbeats once initial motion
			// has finished. settledSinceSec is the elapsed time since the
			// last AUTO Lock flip when settled, zero otherwise; lets a
			// reader scrub the timeline of stable lock windows.
			const double secsSinceLastFlip = (ctx.autoLockLastFlipTime > 0.0) ? (time - ctx.autoLockLastFlipTime) : 0.0;
			// With an explicit lock mode the detector's effective-lock output is
			// idle; the pinned lockRelativePosition is the truthful lock state
			// for the settled metric.
			const bool effectiveLock = ctx.lockRelativePositionMode == CalibrationContext::LockMode::AUTO
			                               ? ctx.autoLockEffectivelyLocked
			                               : ctx.lockRelativePosition;
			const bool settled = spacecal::autolock::IsSettled(effectiveLock, g_lastAutoLockTranslMad,
			                                                   ctx.autoLockMadFloor, secsSinceLastFlip);
			const double madFloorMm = ctx.autoLockMadFloor * 1000.0;
			const double enterMm = spacecal::autolock::EnterThresholdFor(ctx.autoLockMadFloor) * 1000.0;
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
			        ? (ctx.postSnapErrorSumMm / static_cast<double>(ctx.postSnapErrorSampleCount))
			        : 0.0;
			const char* madFloorSourceHb;
			if (!warmRestartActive) {
				madFloorSourceHb = "n/a";
			}
			else if (ctx.warmRestartSnapTime > 0.0 && ctx.autoLockMadFloorTs > 0.0 &&
			         ctx.autoLockMadFloorTs >= ctx.warmRestartSnapTime) {
				madFloorSourceHb = "postSnap";
			}
			else {
				madFloorSourceHb = "preSnap";
			}
			const char* validationStateHb;
			switch (ctx.warmRestartValidationState) {
				case spacecal::warm_restart::ValidationOutcome::Settled:
					validationStateHb = "settled";
					break;
				case spacecal::warm_restart::ValidationOutcome::Failed:
					validationStateHb = "failed";
					break;
				default:
					validationStateHb = "inconclusive";
					break;
			}
			const auto& wh = LastWitnessHealth();
			char hbBuf[1152];
			snprintf(hbBuf, sizeof hbBuf,
			         "[cal-heartbeat] state=%d trackingStyle=%d headMountMode=%d lockMode=%d lockRel=%d autoLockEff=%d"
			         " autoLockPending=%d autoLockPendingTo=%d autoLockHeldSec=%.2f"
			         " autoLockHistory=%zu/%zu translMad_mm=%.3f rotMad_deg=%.3f"
			         " mad_floor_mm=%.3f enter_threshold_mm=%.3f"
			         " settled=%s settled_since_sec=%.1f"
			         " err_last_mm=%.2f err_samples=%d"
			         " grace_until=%.3f"
			         " geom_sustain=%d/%d"
			         " geom_cooldown_remaining_sec=%.1f"
			         " relPosCal=%d hmdStalls=%d"
			         " wr_active=%d wr_grace_remaining=%d"
			         " post_snap_bias_mm=%.3f post_snap_samples=%d"
			         " mad_floor_source=%s wr_validation=%s"
			         " witness_eff_mode=%d reanchor_pending=%d wr_reanchors=%d synth_fallbacks=%llu"
			         " witness_valid_pct=%.1f witness_last_valid_sec=%.1f subthreshold_relocs=%llu"
			         " enhanced_checks=%d obs_lambda_min=%.2f",
			         (int)ctx.state, (int)ctx.trackingStyle, (int)ctx.headMount.mode, (int)ctx.lockRelativePositionMode,
			         (int)ctx.lockRelativePosition, (int)ctx.autoLockEffectivelyLocked, (int)ctx.autoLockHasPendingFlip,
			         (int)ctx.autoLockPendingFlipTo, autoLockHeldSec, ctx.autoLockHistory.size(),
			         spacecal::autolock::kSamplesNeeded, translMadMm, rotMadDeg, madFloorMm, enterMm,
			         settled ? "yes" : "no", settled ? secsSinceLastFlip : 0.0, errLast, errSeries.size(),
			         ctx.geometryShiftGraceUntil, g_geomShiftConsecutiveBadTicks,
			         spacecal::geometry_shift::kMinSustainedSpikes, cooldownRemaining, (int)ctx.relativePosCalibrated,
			         ctx.consecutiveHmdStalls, (int)warmRestartActive, ctx.warmRestartGraceSamples, postSnapBiasMm,
			         ctx.postSnapErrorSampleCount, madFloorSourceHb, validationStateHb,
			         (int)wkopenvr::headmount::EffectiveHeadMountMode(ctx), (int)g_reanchorNextProfileApply,
			         ctx.warmRestartReanchorCount, (unsigned long long)ctx.driverSynthFallbackTotal,
			         spacecal::witness_health::ValidPct(wh), spacecal::witness_health::LastValidSec(wh, time),
			         (unsigned long long)wh.subthresholdRelocs, (int)ctx.CustomChecksActive(),
			         calibration.LastObservabilityLambdaMin());
			Metrics::WriteLogAnnotation(hbBuf);
		}
	}
}

// One-shot session-start config dump. Fires on the first non-skipped
// CalibrationTick after the profile has been loaded, so the annotation
// reflects the user's actual saved settings. Captures every experimental
// toggle + the load-bearing tunables. Lets a session reader skip the
// "what version of the math is running" reverse-derivation from code.
void EmitSessionConfigDumpOnce(CalibrationContext& ctx)
{
	{
		static bool s_loggedConfigDump = false;
		if (!s_loggedConfigDump) {
			s_loggedConfigDump = true;
			char dumpBuf[512];
			snprintf(dumpBuf, sizeof dumpBuf,
			         "session_config_dump: ignore_outliers=%d static_recal=%d"
			         " recalibrate_on_movement=%d one_shot_speed=%.2f continuous_speed=%.2f active_speed=%.2f "
			         "jitter_threshold=%.2f",
			         (int)ctx.ignoreOutliers, (int)ctx.enableStaticRecalibration, (int)ctx.recalibrateOnMovement,
			         (double)ctx.oneShotCalibrationSpeed, (double)ctx.continuousCalibrationSpeed,
			         (double)ctx.ActiveCalibrationSpeed(), (double)ctx.jitterThreshold);
			Metrics::WriteLogAnnotation(dumpBuf);
		}
	}
}
