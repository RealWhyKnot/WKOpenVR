#include "CalibrationWatchdogs.h"

#include "CalibrationInternal.h" // calibration solver + shared auto-lock counters
#include "CalibrationMetrics.h"
#include "GravityAlignment.h"       // spacecal::gravity::TiltAngleDeg -- heartbeat tilt field
#include "HeadMountTargetBinding.h" // wkopenvr::headmount::EffectiveHeadMountMode

#include <cmath>
#include <cstdio>

#include <Eigen/Dense>

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
			char hbBuf[640];
			snprintf(hbBuf, sizeof hbBuf,
			         "[cal-heartbeat] state=%d trackingStyle=%d headMountMode=%d lockMode=%d lockRel=%d"
			         " err_last_mm=%.2f err_samples=%d"
			         " relPosCal=%d hmdStalls=%d"
			         " head_mount_eff_mode=%d synth_fallbacks=%llu"
			         " enhanced_checks=%d obs_lambda_min=%.2f tilt_deg=%.2f tilt_damping=%d",
			         (int)ctx.state, (int)ctx.trackingStyle, (int)ctx.headMount.mode, (int)ctx.lockRelativePositionMode,
			         (int)ctx.lockRelativePosition, errLast, errSeries.size(), (int)ctx.relativePosCalibrated,
			         ctx.consecutiveHmdStalls, (int)wkopenvr::headmount::EffectiveHeadMountMode(ctx),
			         (unsigned long long)ctx.driverSynthFallbackTotal, (int)ctx.CustomChecksActive(),
			         calibration.LastObservabilityLambdaMin(),
			         spacecal::gravity::TiltAngleDeg(Eigen::Quaterniond(
			             ProfileTransform(ctx.calibratedRotation, ctx.calibratedTranslation).rotation())),
			         (int)ctx.gravityTiltDamping);
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
			         "jitter_threshold=%.2f gravity_tilt_damping=%d",
			         (int)ctx.ignoreOutliers, (int)ctx.enableStaticRecalibration, (int)ctx.recalibrateOnMovement,
			         (double)ctx.oneShotCalibrationSpeed, (double)ctx.continuousCalibrationSpeed,
			         (double)ctx.ActiveCalibrationSpeed(), (double)ctx.jitterThreshold, (int)ctx.gravityTiltDamping);
			Metrics::WriteLogAnnotation(dumpBuf);
		}
	}
}
