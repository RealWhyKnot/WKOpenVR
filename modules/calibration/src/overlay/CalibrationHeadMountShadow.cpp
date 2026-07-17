#include "CalibrationHeadMountShadow.h"

#include "CalibrationAutoSpeed.h"       // spacecal::calibration_speed::SelectObservedFitRmsMm
#include "CalibrationDevicePoseUtils.h" // ConvertPose
#include "CalibrationInternal.h"        // calibration solver, snap flags, shared transform helpers
#include "CalibrationMetrics.h"
#include "HeadFromTrackerSolve.h"   // wkopenvr::headmount::Solver
#include "HeadMountShadowOffset.h"  // shadow apply gate + offset deltas + freshness bound
#include "HeadMountSourceGuard.h"   // head-mount source fingerprint + transition decision
#include "HeadMountTargetBinding.h" // wkopenvr::headmount::HeadMountMatchesContinuousTarget
#include "TrackingStyle.h"          // tracking-style presets + headset-synthesis checks

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

#include <Eigen/Dense>

HeadMountSampleSource CurrentHeadMountSampleSource(const CalibrationContext& ctx)
{
	return spacecal::headmount::SelectHeadMountSampleSource(ctx.headMount.mode, ctx.headMount.offsetCalibrated);
}

static bool InContinuousFamily(CalibrationState state)
{
	return state == CalibrationState::Continuous || state == CalibrationState::ContinuousStandby;
}

struct HeadMountShadowOffsetRuntime
{
	wkopenvr::headmount::Solver solver;
	bool collecting = false;
	Eigen::Affine3d targetFromReferenceAtStart = Eigen::Affine3d::Identity();
	uint64_t windowStartFallbackTotal = 0;
	double windowStartTime = 0.0;
	bool hasPreviousMotionPose = false;
	Eigen::Affine3d previousMotionPose = Eigen::Affine3d::Identity();
	double previousMotionTime = 0.0;
	bool hasLastCandidate = false;
	Eigen::AffineCompact3d lastCandidate = Eigen::AffineCompact3d::Identity();
	int stableWindowCount = 0;
	std::string lastBlockedReason;
	double lastBlockedLogTime = -1e9;
	bool hasInvariantPrevious = false;
	Eigen::Affine3d previousInvariantHmd = Eigen::Affine3d::Identity();
	Eigen::Affine3d previousInvariantTrackerReference = Eigen::Affine3d::Identity();
	Eigen::Affine3d previousInvariantLocalOffset = Eigen::Affine3d::Identity();
	double lastInvariantLogTime = -1e9;
};

static HeadMountShadowOffsetRuntime g_headMountShadowOffset;

static void ResetHeadMountShadowOffsetRuntime(bool clearStableWindows)
{
	g_headMountShadowOffset.solver.Cancel();
	g_headMountShadowOffset.collecting = false;
	g_headMountShadowOffset.targetFromReferenceAtStart = Eigen::Affine3d::Identity();
	g_headMountShadowOffset.windowStartFallbackTotal = 0;
	g_headMountShadowOffset.windowStartTime = 0.0;
	g_headMountShadowOffset.hasPreviousMotionPose = false;
	g_headMountShadowOffset.previousMotionPose = Eigen::Affine3d::Identity();
	g_headMountShadowOffset.previousMotionTime = 0.0;
	g_headMountShadowOffset.hasInvariantPrevious = false;
	g_headMountShadowOffset.previousInvariantHmd = Eigen::Affine3d::Identity();
	g_headMountShadowOffset.previousInvariantTrackerReference = Eigen::Affine3d::Identity();
	g_headMountShadowOffset.previousInvariantLocalOffset = Eigen::Affine3d::Identity();
	if (clearStableWindows) {
		g_headMountShadowOffset.hasLastCandidate = false;
		g_headMountShadowOffset.lastCandidate = Eigen::AffineCompact3d::Identity();
		g_headMountShadowOffset.stableWindowCount = 0;
	}
}

static double RotationDeltaRad(const Eigen::Affine3d& previous, const Eigen::Affine3d& current)
{
	const Eigen::Matrix3d delta = previous.rotation().transpose() * current.rotation();
	const double cosAngle = std::clamp((delta.trace() - 1.0) * 0.5, -1.0, 1.0);
	return std::acos(cosAngle);
}

static Eigen::Affine3d DriverPoseToAffine(const vr::DriverPose_t& raw)
{
	const Pose p = ConvertPose(raw);
	return Eigen::Translation3d(p.trans) * Eigen::Quaterniond(p.rot);
}

static bool PoseFreshEnough(const CalibrationContext& ctx, int32_t deviceId, const LARGE_INTEGER& now,
                            const LARGE_INTEGER& frequency, double& ageMs)
{
	ageMs = -1.0;
	if (deviceId < 0 || deviceId >= (int32_t)vr::k_unMaxTrackedDeviceCount) {
		return false;
	}
	const LARGE_INTEGER sampleTime = ctx.devicePoseSampleTimes[deviceId];
	if (sampleTime.QuadPart <= 0 || frequency.QuadPart <= 0) {
		return false;
	}
	if (now.QuadPart < sampleTime.QuadPart) {
		return false;
	}
	ageMs =
	    (static_cast<double>(now.QuadPart - sampleTime.QuadPart) * 1000.0) / static_cast<double>(frequency.QuadPart);
	return ageMs <= spacecal::headmount::kShadowFreshPoseMaxAgeMs;
}

static void TickHeadMountInvariantShadow(const CalibrationContext& ctx, double time,
                                         const Eigen::Affine3d& hmdReference, const Eigen::Affine3d& trackerTarget,
                                         double hmdAgeMs, double trackerAgeMs)
{
	if (!ctx.validProfile) {
		g_headMountShadowOffset.hasInvariantPrevious = false;
		return;
	}

	const Eigen::Affine3d targetToReference = CalibrationTransformFromContext(ctx);
	const Eigen::Affine3d trackerReference = targetToReference * trackerTarget;
	const Eigen::Affine3d localOffset = hmdReference.inverse() * trackerReference;
	const Eigen::Affine3d savedLocalOffset = Eigen::Affine3d(ctx.headMount.headFromTracker.inverse());
	const double savedDeltaM = (localOffset.translation() - savedLocalOffset.translation()).norm();
	const double savedDeltaDeg = RotationDeltaRad(savedLocalOffset, localOffset) * (180.0 / EIGEN_PI);

	double hmdDeltaM = -1.0;
	double trackerDeltaM = -1.0;
	double mismatchM = -1.0;
	double localJumpM = -1.0;
	double localJumpDeg = -1.0;
	if (g_headMountShadowOffset.hasInvariantPrevious) {
		hmdDeltaM = (hmdReference.translation() - g_headMountShadowOffset.previousInvariantHmd.translation()).norm();
		trackerDeltaM =
		    (trackerReference.translation() - g_headMountShadowOffset.previousInvariantTrackerReference.translation())
		        .norm();
		mismatchM = std::abs(hmdDeltaM - trackerDeltaM);
		localJumpM =
		    (localOffset.translation() - g_headMountShadowOffset.previousInvariantLocalOffset.translation()).norm();
		localJumpDeg =
		    RotationDeltaRad(g_headMountShadowOffset.previousInvariantLocalOffset, localOffset) * (180.0 / EIGEN_PI);
	}

	const bool softSnap = localJumpM > 0.030 || localJumpDeg > 5.0 || mismatchM > 0.050;
	const bool hardSnap = localJumpM > 0.075 || localJumpDeg > 10.0 || mismatchM > 0.150;
	const bool shouldLog = hardSnap || softSnap || (time - g_headMountShadowOffset.lastInvariantLogTime) >= 1.0;
	if (shouldLog) {
		g_headMountShadowOffset.lastInvariantLogTime = time;
		char buf[1050];
		snprintf(buf, sizeof buf,
		         "[hm-invariant-shadow] mode=%d deviceID=%d valid_profile=%d"
		         " local_offset_cm=(%.2f,%.2f,%.2f) saved_delta_cm=%.2f saved_delta_rot_deg=%.2f"
		         " local_jump_cm=%.2f local_jump_rot_deg=%.2f"
		         " hmd_delta_cm=%.2f tracker_cal_delta_cm=%.2f mismatch_cm=%.2f"
		         " soft_snap=%d hard_snap=%d hmd_age_ms=%.1f tracker_age_ms=%.1f offset_version=%u",
		         (int)ctx.headMount.mode, (int)ctx.headMount.deviceID, (int)ctx.validProfile,
		         localOffset.translation().x() * 100.0, localOffset.translation().y() * 100.0,
		         localOffset.translation().z() * 100.0, savedDeltaM * 100.0, savedDeltaDeg, localJumpM * 100.0,
		         localJumpDeg, hmdDeltaM * 100.0, trackerDeltaM * 100.0, mismatchM * 100.0, (int)softSnap,
		         (int)hardSnap, hmdAgeMs, trackerAgeMs, (unsigned)ctx.headMountOffsetVersion);
		Metrics::WriteLogAnnotation(buf);
	}

	g_headMountShadowOffset.previousInvariantHmd = hmdReference;
	g_headMountShadowOffset.previousInvariantTrackerReference = trackerReference;
	g_headMountShadowOffset.previousInvariantLocalOffset = localOffset;
	g_headMountShadowOffset.hasInvariantPrevious = true;
}

static double CurrentProfileFitRmsMm()
{
	return spacecal::calibration_speed::SelectObservedFitRmsMm(Metrics::error_rawComputed.last(),
	                                                           Metrics::error_currentCal.last());
}

static void ClearHeadMountContinuousSourceState(CalibrationContext& ctx, const char* reason, double time,
                                                HeadMountSampleSource newSource, bool previousFingerprintValid,
                                                HeadMountSampleSource previousSource)
{
	const bool oldRelPosCal = ctx.relativePosCalibrated;
	const bool oldLockRel = ctx.lockRelativePosition;
	const size_t oldSampleCount = calibration.SampleCount();

	calibration.Clear();
	ctx.refToTargetPose = Eigen::AffineCompact3d::Identity();
	ctx.relativePosCalibrated = false;
	ctx.pairedMotionPosSeeded = false;
	ctx.pairedMotionMismatchCount = 0;
	ctx.pairedMotionPrevRefPos = Eigen::Vector3d::Zero();
	ctx.pairedMotionPrevTgtPos = Eigen::Vector3d::Zero();
	ctx.autoLockHistory.clear();
	ctx.autoLockEffectivelyLocked = false;
	ctx.autoLockHasPendingFlip = false;
	ctx.autoLockPendingFlipTo = false;
	ctx.autoLockPendingFlipFirstSeen = 0.0;
	ctx.autoLockGateHeldWarned = false;
	ctx.headMountNeedsFreshRelativePose = true;
	ctx.headMountLastSourceResetTime = time;
	ctx.lastAcceptedContinuousSnapshot = {};

	if (ctx.validProfile) {
		calibration.SeedEstimatedTransformation(ProfileTransform(ctx.calibratedRotation, ctx.calibratedTranslation),
		                                        /*annotate=*/false);
	}
	calibration.setRelativeTransformation(ctx.refToTargetPose, false);
	calibration.lockRelativePosition = false;
	ResetHeadMountShadowOffsetRuntime(/*clearStableWindows=*/true);

	char buf[960];
	std::snprintf(buf, sizeof buf,
	              "head_mount_source_reset: reason=%s previous_source=%s source=%s"
	              " previous_valid=%d mode=%d targetID=%d deviceID=%d"
	              " offset_version=%u previous_offset_version=%u relPosCal_before=%d"
	              " relPosCal_after=%d lockRel_before=%d samples_cleared=%zu"
	              " profile_valid=%d profile_mag_cm=%.2f tracker_serial='%s'"
	              " target_serial='%s' target_system='%s'",
	              (reason && reason[0]) ? reason : "unknown", HeadMountSampleSourceName(previousSource),
	              HeadMountSampleSourceName(newSource), (int)previousFingerprintValid, (int)ctx.headMount.mode,
	              ctx.targetID, ctx.headMount.deviceID, (unsigned)ctx.headMountOffsetVersion,
	              (unsigned)ctx.headMountLastSourceOffsetVersion, (int)oldRelPosCal, (int)ctx.relativePosCalibrated,
	              (int)oldLockRel, oldSampleCount, (int)ctx.validProfile, ctx.calibratedTranslation.norm(),
	              ctx.headMount.trackerSerial.c_str(), ctx.targetStandby.serial.c_str(),
	              ctx.targetStandby.trackingSystem.c_str());
	Metrics::WriteLogAnnotation(buf);
}

static void UpdateHeadMountSourceFingerprint(CalibrationContext& ctx, HeadMountSampleSource source)
{
	ctx.headMountSourceFingerprintValid = true;
	ctx.headMountLastSampleSource = source;
	ctx.headMountLastSourceMode = ctx.headMount.mode;
	ctx.headMountLastSourceOffsetVersion = ctx.headMountOffsetVersion;
	ctx.headMountLastSourceDeviceID = ctx.headMount.deviceID;
	ctx.headMountLastSourceTargetSerial = ctx.targetStandby.serial;
	ctx.headMountLastSourceTargetSystem = ctx.targetStandby.trackingSystem;
}

bool CCal_SeedHeadMountProxyRelativeLock(const char* reason)
{
	if (!TrackingStyleUsesHeadsetSynthesis(CalCtx.trackingStyle)) {
		return false;
	}
	ApplyTrackingStylePreset(CalCtx, CalCtx.trackingStyle);
	if (!CalCtx.headMount.offsetCalibrated || !CalCtx.validProfile ||
	    !wkopenvr::headmount::HeadMountMatchesContinuousTarget(CalCtx)) {
		return false;
	}

	const HeadMountSampleSource source = CurrentHeadMountSampleSource(CalCtx);
	if (source != HeadMountSampleSource::HeadProxy) {
		return false;
	}

	CalCtx.refToTargetPose = Eigen::AffineCompact3d::Identity();
	CalCtx.relativePosCalibrated = true;
	CalCtx.headMountNeedsFreshRelativePose = false;
	CalCtx.ResolveLockMode();
	calibration.setRelativeTransformation(CalCtx.refToTargetPose, true);
	calibration.lockRelativePosition = CalCtx.lockRelativePosition;
	UpdateHeadMountSourceFingerprint(CalCtx, source);
	if (CalCtx.state == CalibrationState::Continuous) {
		g_snapNextProfileApply = true;
	}

	char buf[320];
	std::snprintf(buf, sizeof buf,
	              "head_mount_relative_lock_seeded: reason=%s style=%d mode=%d"
	              " lockRel=%d offset_version=%u target_serial='%s'",
	              (reason && reason[0]) ? reason : "unknown", (int)CalCtx.trackingStyle, (int)CalCtx.headMount.mode,
	              (int)CalCtx.lockRelativePosition, (unsigned)CalCtx.headMountOffsetVersion,
	              CalCtx.targetStandby.serial.c_str());
	Metrics::WriteLogAnnotation(buf);
	return true;
}

void TickHeadMountSourceTransitionGuard(CalibrationContext& ctx, double time)
{
	if (!InContinuousFamily(ctx.state)) {
		ctx.headMountSourceFingerprintValid = false;
		return;
	}

	const HeadMountSampleSource source = CurrentHeadMountSampleSource(ctx);
	const bool previousValid = ctx.headMountSourceFingerprintValid;
	const HeadMountSampleSource previousSource = ctx.headMountLastSampleSource;

	spacecal::headmount::HeadMountSourceFingerprint previous{};
	previous.source = ctx.headMountLastSampleSource;
	previous.mode = ctx.headMountLastSourceMode;
	previous.offsetVersion = ctx.headMountLastSourceOffsetVersion;
	previous.deviceID = ctx.headMountLastSourceDeviceID;
	previous.targetSerial = ctx.headMountLastSourceTargetSerial;
	previous.targetTrackingSystem = ctx.headMountLastSourceTargetSystem;

	spacecal::headmount::HeadMountSourceFingerprint current{};
	current.source = source;
	current.mode = ctx.headMount.mode;
	current.offsetVersion = ctx.headMountOffsetVersion;
	current.deviceID = ctx.headMount.deviceID;
	current.targetSerial = ctx.targetStandby.serial;
	current.targetTrackingSystem = ctx.targetStandby.trackingSystem;

	const auto decision = spacecal::headmount::EvaluateHeadMountSourceTransition(previousValid, previous, current,
	                                                                             ctx.relativePosCalibrated);
	if (decision.reset) {
		ClearHeadMountContinuousSourceState(ctx, decision.reason, time, source, previousValid, previousSource);
	}
	UpdateHeadMountSourceFingerprint(ctx, source);
}

static void LogShadowOffsetBlockedThrottled(CalibrationContext& ctx, const char* reason, double time,
                                            double profileFitRmsMm, double hmdAgeMs, double trackerAgeMs)
{
	const bool reasonChanged = g_headMountShadowOffset.lastBlockedReason != reason;
	if (!reasonChanged && (time - g_headMountShadowOffset.lastBlockedLogTime) < 2.0) {
		return;
	}
	g_headMountShadowOffset.lastBlockedReason = reason;
	g_headMountShadowOffset.lastBlockedLogTime = time;

	char buf[640];
	std::snprintf(buf, sizeof buf,
	              "shadow_offset_apply_blocked: reason=%s source=%s"
	              " offset_version=%u relPosCal=%d needsFreshRelPose=%d"
	              " profile_valid=%d profile_mag_cm=%.2f profile_fit_rms_mm=%.3f"
	              " hmd_age_ms=%.1f tracker_age_ms=%.1f fallback_total=%llu"
	              " target_matches=%d deviceID=%d hmd_stream=driver_shmem_raw",
	              reason, HeadMountSampleSourceName(CurrentHeadMountSampleSource(ctx)),
	              (unsigned)ctx.headMountOffsetVersion, (int)ctx.relativePosCalibrated,
	              (int)ctx.headMountNeedsFreshRelativePose, (int)ctx.validProfile, ctx.calibratedTranslation.norm(),
	              profileFitRmsMm, hmdAgeMs, trackerAgeMs, (unsigned long long)ctx.driverSynthFallbackTotal,
	              (int)wkopenvr::headmount::HeadMountMatchesContinuousTarget(ctx), ctx.headMount.deviceID);
	Metrics::WriteLogAnnotation(buf);
}

void TickHeadMountShadowOffsetEstimator(CalibrationContext& ctx, double time)
{
	using namespace spacecal::headmount;

	if (ctx.state != CalibrationState::Continuous) {
		ResetHeadMountShadowOffsetRuntime(/*clearStableWindows=*/true);
		return;
	}

	const HeadMountSampleSource source = CurrentHeadMountSampleSource(ctx);
	if (source != HeadMountSampleSource::HeadProxy || ctx.headMount.mode < HeadMountMode::AutoPaired ||
	    !ctx.headMount.offsetCalibrated) {
		ResetHeadMountShadowOffsetRuntime(/*clearStableWindows=*/true);
		return;
	}

	const bool targetMatches = wkopenvr::headmount::HeadMountMatchesContinuousTarget(ctx);
	const double profileFitRmsMm = CurrentProfileFitRmsMm();
	const bool profileHealthy =
	    ctx.validProfile && std::isfinite(profileFitRmsMm) && profileFitRmsMm <= kShadowProfileHealthyRmsMm;

	LARGE_INTEGER nowQpc{};
	LARGE_INTEGER qpcFrequency{};
	QueryPerformanceCounter(&nowQpc);
	QueryPerformanceFrequency(&qpcFrequency);
	double hmdAgeMs = -1.0;
	double trackerAgeMs = -1.0;
	const bool hmdFresh = PoseFreshEnough(ctx, vr::k_unTrackedDeviceIndex_Hmd, nowQpc, qpcFrequency, hmdAgeMs);
	const bool trackerFresh = PoseFreshEnough(ctx, ctx.headMount.deviceID, nowQpc, qpcFrequency, trackerAgeMs);

	const bool trackerInRange =
	    ctx.headMount.deviceID >= 0 && ctx.headMount.deviceID < (int32_t)vr::k_unMaxTrackedDeviceCount;
	const bool trackerPoseOk =
	    trackerInRange && spacecal::headmount::PoseIsRunningOk(ctx.devicePoses[ctx.headMount.deviceID]);
	const bool hmdPoseOk = spacecal::headmount::PoseIsRunningOk(ctx.devicePoses[vr::k_unTrackedDeviceIndex_Hmd]);

	if (!targetMatches || !profileHealthy || !hmdFresh || !trackerFresh || !trackerPoseOk || !hmdPoseOk) {
		const char* reason = !targetMatches                 ? "target_mismatch"
		                     : !profileHealthy              ? "profile_unhealthy"
		                     : (!hmdFresh || !trackerFresh) ? "stale_pose"
		                     : !hmdPoseOk                   ? "hmd_pose_invalid"
		                                                    : "tracker_pose_invalid";
		LogShadowOffsetBlockedThrottled(ctx, reason, time, profileFitRmsMm, hmdAgeMs, trackerAgeMs);
		ResetHeadMountShadowOffsetRuntime(/*clearStableWindows=*/false);
		return;
	}

	if (!g_headMountShadowOffset.collecting) {
		g_headMountShadowOffset.solver.Start();
		g_headMountShadowOffset.collecting = true;
		g_headMountShadowOffset.targetFromReferenceAtStart = CalibrationTransformFromContext(ctx).inverse();
		g_headMountShadowOffset.windowStartFallbackTotal = ctx.driverSynthFallbackTotal;
		g_headMountShadowOffset.windowStartTime = time;
		g_headMountShadowOffset.hasPreviousMotionPose = false;
	}

	const Eigen::Affine3d hmdReference = DriverPoseToAffine(ctx.devicePoses[vr::k_unTrackedDeviceIndex_Hmd]);
	const Eigen::Affine3d trackerTarget = DriverPoseToAffine(ctx.devicePoses[ctx.headMount.deviceID]);
	TickHeadMountInvariantShadow(ctx, time, hmdReference, trackerTarget, hmdAgeMs, trackerAgeMs);
	const Eigen::Affine3d hmdTarget = g_headMountShadowOffset.targetFromReferenceAtStart * hmdReference;

	double derivedLinearSpeedMps = 0.0;
	double derivedAngularSpeedRadps = 0.0;
	if (g_headMountShadowOffset.hasPreviousMotionPose && time > g_headMountShadowOffset.previousMotionTime) {
		const double dt = time - g_headMountShadowOffset.previousMotionTime;
		derivedLinearSpeedMps =
		    (hmdTarget.translation() - g_headMountShadowOffset.previousMotionPose.translation()).norm() / dt;
		derivedAngularSpeedRadps = RotationDeltaRad(g_headMountShadowOffset.previousMotionPose, hmdTarget) / dt;
	}
	g_headMountShadowOffset.previousMotionPose = hmdTarget;
	g_headMountShadowOffset.previousMotionTime = time;
	g_headMountShadowOffset.hasPreviousMotionPose = true;

	constexpr double kAngularMotionToLinearMps = 0.50;
	const double reportedSpeed = ComputeHmdSpeedMps(ctx);
	const double effectiveSpeed =
	    std::max(std::isfinite(reportedSpeed) ? reportedSpeed : 0.0,
	             std::max(derivedLinearSpeedMps, derivedAngularSpeedRadps * kAngularMotionToLinearMps));
	g_headMountShadowOffset.solver.Tick(hmdTarget, trackerTarget, effectiveSpeed);

	const auto readiness = g_headMountShadowOffset.solver.readiness();
	if (!readiness.ready && readiness.samplesUsed < kShadowMaxWindowSamplesWithoutReadiness) {
		return;
	}

	if (!readiness.ready) {
		char sbuf[640];
		std::snprintf(
		    sbuf, sizeof sbuf,
		    "shadow_offset_suspect: reason=insufficient_motion source=%s"
		    " offset_version=%u samples=%zu residual_mm=%.3f"
		    " motion_coverage=(%.2f,%.2f,%.2f) profile_mag_cm=%.2f"
		    " fallback_delta=%llu",
		    HeadMountSampleSourceName(source), (unsigned)ctx.headMountOffsetVersion, readiness.samplesUsed,
		    readiness.residualMm, readiness.axisRangeDeg[0], readiness.axisRangeDeg[1], readiness.axisRangeDeg[2],
		    ctx.calibratedTranslation.norm(),
		    (unsigned long long)(ctx.driverSynthFallbackTotal - g_headMountShadowOffset.windowStartFallbackTotal));
		Metrics::WriteLogAnnotation(sbuf);
		LogShadowOffsetBlockedThrottled(ctx, "insufficient_motion", time, profileFitRmsMm, hmdAgeMs, trackerAgeMs);
		ResetHeadMountShadowOffsetRuntime(/*clearStableWindows=*/false);
		return;
	}

	g_headMountShadowOffset.solver.Finish();
	const auto& result = g_headMountShadowOffset.solver.result();
	if (!result.failReason.empty()) {
		char sbuf[640];
		std::snprintf(
		    sbuf, sizeof sbuf,
		    "shadow_offset_suspect: reason=%s source=%s offset_version=%u"
		    " samples=%d residual_mm=%.3f motion_coverage=(%.2f,%.2f,%.2f)"
		    " profile_mag_cm=%.2f fallback_delta=%llu",
		    result.failReason.c_str(), HeadMountSampleSourceName(source), (unsigned)ctx.headMountOffsetVersion,
		    result.samplesUsed, result.residualMm, readiness.axisRangeDeg[0], readiness.axisRangeDeg[1],
		    readiness.axisRangeDeg[2], ctx.calibratedTranslation.norm(),
		    (unsigned long long)(ctx.driverSynthFallbackTotal - g_headMountShadowOffset.windowStartFallbackTotal));
		Metrics::WriteLogAnnotation(sbuf);
		ResetHeadMountShadowOffsetRuntime(/*clearStableWindows=*/false);
		return;
	}

	const OffsetDelta savedDelta = ComputeOffsetDelta(ctx.headMount.headFromTracker, result.headFromTracker);
	const bool hasPrevious = g_headMountShadowOffset.hasLastCandidate;
	const OffsetDelta previousDelta =
	    hasPrevious ? ComputeOffsetDelta(g_headMountShadowOffset.lastCandidate, result.headFromTracker) : OffsetDelta{};
	const bool candidateStable = !hasPrevious || IsStableOffsetDelta(previousDelta);
	const bool residualOk = result.residualMm <= kShadowResidualMaxMm;
	const bool sourceSettled = (time - ctx.headMountLastSourceResetTime) >= kShadowSourceResetQuietSec;
	const uint64_t fallbackDelta = ctx.driverSynthFallbackTotal - g_headMountShadowOffset.windowStartFallbackTotal;
	const bool fallbackQuiet = fallbackDelta == 0;
	const bool mismatchPlausible = IsPlausibleOffsetDelta(savedDelta);
	const bool mismatchMeaningful = IsMeaningfulOffsetDelta(savedDelta);

	if (residualOk && profileHealthy && fallbackQuiet && sourceSettled && mismatchPlausible && mismatchMeaningful &&
	    candidateStable) {
		g_headMountShadowOffset.stableWindowCount += 1;
	}
	else {
		g_headMountShadowOffset.stableWindowCount = 0;
	}
	g_headMountShadowOffset.lastCandidate = result.headFromTracker;
	g_headMountShadowOffset.hasLastCandidate = true;

	ShadowGateInput gateInput{};
	// Auto-apply retired: the estimator runs for diagnostics only, so the gate
	// can report would_apply but never reaches readyToApply.
	gateInput.toggleEnabled = false;
	gateInput.windowSolved = true;
	gateInput.posesFresh = hmdFresh && trackerFresh;
	gateInput.targetMatches = targetMatches;
	gateInput.profileHealthy = profileHealthy;
	gateInput.residualOk = residualOk;
	gateInput.sourceSettled = sourceSettled;
	gateInput.fallbackQuiet = fallbackQuiet;
	gateInput.mismatchPlausible = mismatchPlausible;
	gateInput.mismatchMeaningful = mismatchMeaningful;
	gateInput.candidateStable = candidateStable;
	gateInput.stableWindowCount = g_headMountShadowOffset.stableWindowCount;
	const ShadowGateResult gate = EvaluateShadowGate(gateInput);

	char wbuf[960];
	std::snprintf(wbuf, sizeof wbuf,
	              "shadow_offset_window: source=%s offset_version=%u relPosCal=%d"
	              " needsFreshRelPose=%d samples=%d residual_mm=%.3f"
	              " saved_delta_trans_cm=%.2f saved_delta_rot_deg=%.2f"
	              " previous_delta_trans_cm=%.2f previous_delta_rot_deg=%.2f"
	              " motion_coverage=(%.2f,%.2f,%.2f) profile_mag_cm=%.2f"
	              " profile_fit_rms_mm=%.3f fallback_delta=%llu"
	              " stable_windows=%d gate=%s would_apply=%d"
	              " hmd_stream=driver_shmem_raw",
	              HeadMountSampleSourceName(source), (unsigned)ctx.headMountOffsetVersion,
	              (int)ctx.relativePosCalibrated, (int)ctx.headMountNeedsFreshRelativePose, result.samplesUsed,
	              result.residualMm, savedDelta.translationM * 100.0, savedDelta.rotationDeg,
	              hasPrevious ? previousDelta.translationM * 100.0 : -1.0,
	              hasPrevious ? previousDelta.rotationDeg : -1.0, readiness.axisRangeDeg[0], readiness.axisRangeDeg[1],
	              readiness.axisRangeDeg[2], ctx.calibratedTranslation.norm(), profileFitRmsMm,
	              (unsigned long long)fallbackDelta, g_headMountShadowOffset.stableWindowCount, gate.reason,
	              (int)gate.wouldApply);
	Metrics::WriteLogAnnotation(wbuf);

	if (gate.wouldApply) {
		char vbuf[640];
		std::snprintf(vbuf, sizeof vbuf,
		              "shadow_offset_would_apply: source=%s offset_version=%u"
		              " residual_mm=%.3f delta_trans_cm=%.2f delta_rot_deg=%.2f"
		              " stable_windows=%d profile_mag_cm=%.2f",
		              HeadMountSampleSourceName(source), (unsigned)ctx.headMountOffsetVersion, result.residualMm,
		              savedDelta.translationM * 100.0, savedDelta.rotationDeg,
		              g_headMountShadowOffset.stableWindowCount, ctx.calibratedTranslation.norm());
		Metrics::WriteLogAnnotation(vbuf);
	}
	else if (!residualOk || !mismatchPlausible || !candidateStable) {
		char sbuf[640];
		std::snprintf(sbuf, sizeof sbuf,
		              "shadow_offset_suspect: reason=%s source=%s"
		              " offset_version=%u residual_mm=%.3f"
		              " delta_trans_cm=%.2f delta_rot_deg=%.2f"
		              " previous_delta_trans_cm=%.2f previous_delta_rot_deg=%.2f"
		              " motion_coverage=(%.2f,%.2f,%.2f)",
		              gate.reason, HeadMountSampleSourceName(source), (unsigned)ctx.headMountOffsetVersion,
		              result.residualMm, savedDelta.translationM * 100.0, savedDelta.rotationDeg,
		              hasPrevious ? previousDelta.translationM * 100.0 : -1.0,
		              hasPrevious ? previousDelta.rotationDeg : -1.0, readiness.axisRangeDeg[0],
		              readiness.axisRangeDeg[1], readiness.axisRangeDeg[2]);
		Metrics::WriteLogAnnotation(sbuf);
	}
	ResetHeadMountShadowOffsetRuntime(/*clearStableWindows=*/false);
}
