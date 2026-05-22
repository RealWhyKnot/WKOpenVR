#include "CalibrationPoseSampling.h"

#include "CalibrationInternal.h"
#include "CalibrationMetrics.h"
#include "LatencyEstimator.h"
#include "RotationMatrix3.h"
#include "VRState.h"

#include <GLFW/glfw3.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <set>
#include <string>
#include <vector>


inline vr::HmdQuaternion_t operator*(const vr::HmdQuaternion_t& lhs, const vr::HmdQuaternion_t& rhs) {
	return {
		(lhs.w * rhs.w) - (lhs.x * rhs.x) - (lhs.y * rhs.y) - (lhs.z * rhs.z),
		(lhs.w * rhs.x) + (lhs.x * rhs.w) + (lhs.y * rhs.z) - (lhs.z * rhs.y),
		(lhs.w * rhs.y) + (lhs.y * rhs.w) + (lhs.z * rhs.x) - (lhs.x * rhs.z),
		(lhs.w * rhs.z) + (lhs.z * rhs.w) + (lhs.x * rhs.y) - (lhs.y * rhs.x)
	};
}

CalibrationCalc calibration;

	inline vr::HmdVector3d_t quaternionRotateVector(const vr::HmdQuaternion_t& quat, const double(&vector)[3]) {
		vr::HmdQuaternion_t vectorQuat = { 0.0, vector[0], vector[1] , vector[2] };
		vr::HmdQuaternion_t conjugate = { quat.w, -quat.x, -quat.y, -quat.z };
		auto rotatedVectorQuat = quat * vectorQuat * conjugate;
		return { rotatedVectorQuat.x, rotatedVectorQuat.y, rotatedVectorQuat.z };
	}

	inline Eigen::Matrix3d quaternionRotateMatrix(const vr::HmdQuaternion_t& quat) {
		return Eigen::Quaterniond(quat.w, quat.x, quat.y, quat.z).toRotationMatrix();
	}

	struct DSample
	{
		bool valid;
		Eigen::Vector3d ref, target;
	};

	bool StartsWith(const std::string& str, const std::string& prefix)
	{
		if (str.length() < prefix.length())
			return false;

		return str.compare(0, prefix.length(), prefix) == 0;
	}

	bool EndsWith(const std::string& str, const std::string& suffix)
	{
		if (str.length() < suffix.length())
			return false;

		return str.compare(str.length() - suffix.length(), suffix.length(), suffix) == 0;
	}

	vr::HmdQuaternion_t VRRotationQuat(const Eigen::Quaterniond& rotQuat)
	{

		vr::HmdQuaternion_t vrRotQuat;
		vrRotQuat.x = rotQuat.coeffs()[0];
		vrRotQuat.y = rotQuat.coeffs()[1];
		vrRotQuat.z = rotQuat.coeffs()[2];
		vrRotQuat.w = rotQuat.coeffs()[3];
		return vrRotQuat;
	}

	vr::HmdQuaternion_t VRRotationQuat(Eigen::Vector3d eulerdeg)
	{
		auto euler = eulerdeg * EIGEN_PI / 180.0;

		Eigen::Quaterniond rotQuat =
			Eigen::AngleAxisd(euler(0), Eigen::Vector3d::UnitZ()) *
			Eigen::AngleAxisd(euler(1), Eigen::Vector3d::UnitY()) *
			Eigen::AngleAxisd(euler(2), Eigen::Vector3d::UnitX());

		return VRRotationQuat(rotQuat);
	}

	vr::HmdVector3d_t VRTranslationVec(Eigen::Vector3d transcm)
	{
		auto trans = transcm * 0.01;
		vr::HmdVector3d_t vrTrans;
		vrTrans.v[0] = trans[0];
		vrTrans.v[1] = trans[1];
		vrTrans.v[2] = trans[2];
		return vrTrans;
	}

	DSample DeltaRotationSamples(Sample s1, Sample s2)
	{
		// Difference in rotation between samples.
		auto dref = s1.ref.rot * s2.ref.rot.transpose();
		auto dtarget = s1.target.rot * s2.target.rot.transpose();

		// When stuck together, the two tracked objects rotate as a pair,
		// therefore their axes of rotation must be equal between any given pair of samples.
		DSample ds;
		ds.ref = AxisFromRotationMatrix3(dref);
		ds.target = AxisFromRotationMatrix3(dtarget);

		// Reject samples that were too close to each other.
		auto refA = AngleFromRotationMatrix3(dref);
		auto targetA = AngleFromRotationMatrix3(dtarget);
		ds.valid = refA > 0.4 && targetA > 0.4 && ds.ref.norm() > 0.01 && ds.target.norm() > 0.01;

		// Only normalise when the magnitudes pass the gate above; a sub-1cm
		// axis would otherwise be normalised to NaN/Inf entries, and any
		// downstream consumer that forgets to check ds.valid would ingest
		// the garbage.
		if (ds.valid) {
			ds.ref.normalize();
			ds.target.normalize();
		}
		return ds;
	}

	Pose ConvertPose(const vr::DriverPose_t &driverPose) {
		Eigen::Quaterniond driverToWorldQ(
			driverPose.qWorldFromDriverRotation.w,
			driverPose.qWorldFromDriverRotation.x,
			driverPose.qWorldFromDriverRotation.y,
			driverPose.qWorldFromDriverRotation.z
		);
		Eigen::Vector3d driverToWorldV(
			driverPose.vecWorldFromDriverTranslation[0],
			driverPose.vecWorldFromDriverTranslation[1],
			driverPose.vecWorldFromDriverTranslation[2]
		);

		Eigen::Quaterniond driverRot = driverToWorldQ * Eigen::Quaterniond(
			driverPose.qRotation.w,
			driverPose.qRotation.x,
			driverPose.qRotation.y,
			driverPose.qRotation.z
		);
		// Normalise the composed quaternion. Eigen's quaternion multiplication
		// does not auto-normalise, so over a multi-minute session the cumulative
		// scale drift from non-unit-norm input quaternions (some drivers emit
		// quaternions slightly off the unit sphere -- Quest Pro's IMU fusion
		// occasionally produces tiny scale errors per frame) compounds into a
		// non-orthonormal rotation matrix at xform.linear(), which the Kabsch /
		// SVD downstream silently treats as a mild shear. The metrics-side
		// composer at the bottom of CalibrationTick already does this; the
		// calibration-side now matches.
		driverRot.normalize();

		Eigen::Vector3d driverPos = driverToWorldV + driverToWorldQ * Eigen::Vector3d(
			driverPose.vecPosition[0],
			driverPose.vecPosition[1],
			driverPose.vecPosition[2]
		);

		Eigen::AffineCompact3d xform = Eigen::Translation3d(driverPos) * driverRot;

		return Pose(xform);
	}

	// Velocity-based extrapolation of a reference pose by `dtSeconds` (positive = forward
	// in time, negative = backward). Mutates `pose` in place.
	//
	// vecVelocity / vecAngularVelocity are in the driver's local frame (the same frame
	// as vecPosition / qRotation), so we apply them directly there and let
	// qWorldFromDriverRotation do the world-space projection downstream in ConvertPose.
	// This sidesteps the trap of rotating velocity into world space and then adding it
	// to a driver-space position.
	//
	// Returns true on success, false if the velocity data looks invalid (NaN, infinite,
	// or implausibly large). On false the caller should fall back to the un-extrapolated
	// pose; we never throw.
	inline bool ExtrapolateReferencePose(vr::DriverPose_t& pose, double dtSeconds) {
		if (dtSeconds == 0.0) return true;

		// Sanity-check the velocity components. A momentary tracking glitch can produce
		// NaN/inf or wildly large velocity values; applying those to the pose would
		// teleport the reference and pollute the sample.
		const double maxLinearMps = 50.0;        // ~180 km/h, far beyond any real head/tracker motion
		const double maxAngularRadps = 50.0;     // ~8 rev/s
		for (int i = 0; i < 3; i++) {
			double v = pose.vecVelocity[i];
			double w = pose.vecAngularVelocity[i];
			if (!std::isfinite(v) || !std::isfinite(w)) return false;
			if (std::fabs(v) > maxLinearMps) return false;
			if (std::fabs(w) > maxAngularRadps) return false;
		}

		// Linear extrapolation in driver-local space.
		pose.vecPosition[0] += pose.vecVelocity[0] * dtSeconds;
		pose.vecPosition[1] += pose.vecVelocity[1] * dtSeconds;
		pose.vecPosition[2] += pose.vecVelocity[2] * dtSeconds;

		// Angular extrapolation: vecAngularVelocity is axis-angle in radians/sec in the
		// driver-local frame. Build the corresponding small rotation deltaQ_local and
		// pre-multiply qRotation by it. qWorldFromDriverRotation downstream rotates the
		// updated qRotation into world space.
		double angSpeed = std::sqrt(
			pose.vecAngularVelocity[0] * pose.vecAngularVelocity[0] +
			pose.vecAngularVelocity[1] * pose.vecAngularVelocity[1] +
			pose.vecAngularVelocity[2] * pose.vecAngularVelocity[2]);
		if (angSpeed > 1e-9) {
			double angle = angSpeed * dtSeconds;
			double half = angle * 0.5;
			double s = std::sin(half);
			double axisX = pose.vecAngularVelocity[0] / angSpeed;
			double axisY = pose.vecAngularVelocity[1] / angSpeed;
			double axisZ = pose.vecAngularVelocity[2] / angSpeed;
			vr::HmdQuaternion_t deltaQ = { std::cos(half), axisX * s, axisY * s, axisZ * s };
			pose.qRotation = deltaQ * pose.qRotation;
		}

		return true;
	}

	bool CollectSample(const CalibrationContext& ctx)
	{
		vr::DriverPose_t reference, target;
		reference.poseIsValid = false;
		reference.result = vr::ETrackingResult::TrackingResult_Uninitialized;
		target.poseIsValid = false;
		target.result = vr::ETrackingResult::TrackingResult_Uninitialized;

		reference = ctx.devicePoses[ctx.referenceID];
		target = ctx.devicePoses[ctx.targetID];

		// Defensive: detect quash-pose bleed. The +10 km X/Z offset that
		// HideList applies to hidden trackers happens in the driver AFTER
		// the shmem write of the augmented pose. The cal math reads from
		// shmem, so it should see PRE-quash positions. If a position
		// magnitude exceeds 100 m, something is feeding the cal a quash-
		// translated pose by mistake (or the driver itself is producing
		// runaway values for a different reason). Log once per (which, kind)
		// pair so a real bleed doesn't drown the log -- but log enough to
		// see the pattern.
		{
			static std::set<std::pair<int,int>> s_seenAnomalies;
			auto checkMag = [&](const vr::DriverPose_t& p, int whichKind, const char* label) {
				const double mag2 = p.vecPosition[0]*p.vecPosition[0]
					+ p.vecPosition[1]*p.vecPosition[1]
					+ p.vecPosition[2]*p.vecPosition[2];
				if (mag2 > 10000.0) { // > 100 m
					auto key = std::make_pair(whichKind, (int)p.result);
					if (s_seenAnomalies.insert(key).second) {
						char abuf[280];
						snprintf(abuf, sizeof abuf,
							"[quash-bleed-suspect] %s pose magnitude unusually large:"
							" pos=(%.1f,%.1f,%.1f) result=%d poseIsValid=%d",
							label, p.vecPosition[0], p.vecPosition[1], p.vecPosition[2],
							(int)p.result, (int)p.poseIsValid);
						Metrics::WriteLogAnnotation(abuf);
					}
				}
			};
			checkMag(reference, 0, "reference");
			checkMag(target, 1, "target");
		}

		// Validity gate. Previously this was `if (!poseIsValid && result != Running_OK)`,
		// i.e. "fail only when BOTH signals say bad" -- permissive. The case the
		// permissive form silently accepts is the one we care about: Quest Pro
		// (and other inside-out drivers) regularly report `poseIsValid == false`
		// for one tick during a relocalization while still claiming
		// `result == Running_OK`. The pose for that single tick is genuinely
		// invalid -- using it injects a phantom translation that the calibration
		// math sees as legitimate motion and tries to fit. Tighten to `||`:
		// reject if EITHER signal says bad.
		//
		// To watch the impact: every time this gate now rejects a sample that
		// the old gate would have accepted, we annotate. If those annotations
		// show up only in known-bad sessions, the change is a net win; if they
		// show up at every tick of a healthy session, the gate is too tight and
		// we revisit.
		bool ok = true;
		const bool refSilentInvalid = !reference.poseIsValid &&
			reference.result == vr::ETrackingResult::TrackingResult_Running_OK;
		const bool tgtSilentInvalid = !target.poseIsValid &&
			target.result == vr::ETrackingResult::TrackingResult_Running_OK;
		if (!reference.poseIsValid || reference.result != vr::ETrackingResult::TrackingResult_Running_OK)
		{
			CalCtx.Log("Reference device is not tracking\n"); ok = false;
		}
		if (!target.poseIsValid || target.result != vr::ETrackingResult::TrackingResult_Running_OK)
		{
			CalCtx.Log("Target device is not tracking\n"); ok = false;
		}
		if (refSilentInvalid || tgtSilentInvalid) {
			Metrics::WriteLogAnnotation(refSilentInvalid && tgtSilentInvalid
				? "silent_invalid_pose_rejected: ref+tgt"
				: refSilentInvalid
					? "silent_invalid_pose_rejected: ref"
					: "silent_invalid_pose_rejected: tgt");
		}
		if (!ok)
		{
			if (CalCtx.state != CalibrationState::Continuous) {
				CalCtx.Log("Aborting calibration!\n");
				CalCtx.state = CalibrationState::None;
			}
			return false;
		}

		// Apply tracker offsets
		if (CalCtx.state == CalibrationState::Continuous || CalCtx.state == CalibrationState::ContinuousStandby) {
			reference.vecPosition[0] += ctx.continuousCalibrationOffset.x();
			reference.vecPosition[1] += ctx.continuousCalibrationOffset.y();
			reference.vecPosition[2] += ctx.continuousCalibrationOffset.z();
		}

		// Manual inter-system latency compensation. With a non-zero offset we shift the
		// reference pose forward/backward in time to align with the *effective* moment
		// the target sample represents. We use velocity extrapolation rather than a
		// history buffer (cheaper, bounded, and good enough for the small +/-100 ms range
		// the UI exposes). If velocity data is invalid the reference pose is left
		// un-shifted for this tick -- better one bad-but-bounded sample than a thrown
		// exception or a teleporting reference.
		//
		// Bit-for-bit identical behaviour to before this feature is preserved when the
		// active offset is 0: the shift in seconds is exactly 0.0 and
		// ExtrapolateReferencePose returns immediately without touching the pose. The
		// `if` below only enters when both that AND we have valid sample-time data,
		// i.e. when there's actually work to do.
		double activeOffsetMs = GetActiveLatencyOffsetMs(ctx);
		if (activeOffsetMs != 0.0
			&& ctx.devicePoseSampleTimes[ctx.referenceID].QuadPart != 0
			&& ctx.devicePoseSampleTimes[ctx.targetID].QuadPart != 0)
		{
			LARGE_INTEGER freq;
			QueryPerformanceFrequency(&freq);
			// Delta shmem = how stale the reference pose is relative to the target pose
			// (target ahead -> positive). Then we additionally subtract the user's
			// configured offset. We extrapolate the reference forward by this delta to put
			// it on the same effective timeline as the target.
			double shmemDeltaSec =
				double(ctx.devicePoseSampleTimes[ctx.targetID].QuadPart -
					   ctx.devicePoseSampleTimes[ctx.referenceID].QuadPart)
				/ double(freq.QuadPart);
			double offsetSec = activeOffsetMs / 1000.0;
			// effectiveTargetTime = targetSampleTime - offset, so the reference needs to
			// move to (effectiveTargetTime) - referenceSampleTime = shmemDelta - offset.
			double dt = shmemDeltaSec - offsetSec;
			ExtrapolateReferencePose(reference, dt);
		}

		// Auto-detection feed: push linear-speed magnitudes for both devices into the
		// rolling history buffers. The cross-correlation in CalibrationTick consumes
		// these to estimate the lag between reference and target signal arrival.
		// Linear velocity is preferred over angular for these speed signals -- angular
		// velocity from optical trackers is often filter-shaped (low-pass), which
		// blurs the transient that the cross-correlator looks for.
		auto speedFromVel = [](const double v[3]) -> double {
			double s = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
			if (!std::isfinite(s)) return 0.0;
			return s;
		};
		const double refSpeed = speedFromVel(ctx.devicePoses[ctx.referenceID].vecVelocity);
		const double tgtSpeed = speedFromVel(ctx.devicePoses[ctx.targetID].vecVelocity);
		{
			double now = glfwGetTime();

			CalibrationContext& mctx = const_cast<CalibrationContext&>(ctx);
			mctx.refSpeedHistory.push_back(refSpeed);
			mctx.targetSpeedHistory.push_back(tgtSpeed);
			mctx.speedSampleTimes.push_back(now);
			while (mctx.refSpeedHistory.size() > CalibrationContext::kLatencyHistoryCapacity) {
				mctx.refSpeedHistory.pop_front();
			}
			while (mctx.targetSpeedHistory.size() > CalibrationContext::kLatencyHistoryCapacity) {
				mctx.targetSpeedHistory.pop_front();
			}
			while (mctx.speedSampleTimes.size() > CalibrationContext::kLatencyHistoryCapacity) {
				mctx.speedSampleTimes.pop_front();
			}
		}

		// Paired-motion validity. We want the diversity bars to reflect data
		// the math can actually use, not raw target-tracker motion. If only
		// one device moved meaningfully since the previous sample (the
		// classic case: HMD pose frozen by passthrough or a desktop overlay
		// while the target tracker keeps reporting motion), the sample is
		// tagged so TranslationDiversity / RotationDiversity exclude it.
		// A separate metric ticks up so the popup can surface a warning.
		//
		// The motion threshold is generous: 2 mm between successive samples.
		// Above that we are confident the device genuinely moved; below it
		// we treat the device as stationary regardless of velocity reports
		// (Quest passthrough emits nonzero IMU velocity even when the
		// rendered pose is locked, so velocity alone is unreliable here).
		constexpr double kPairedMotionDeltaMeters = 0.002;
		const Pose refPose = ConvertPose(reference);
		const Pose tgtPose = ConvertPose(target);
		bool pairedMotionValid = true;
		{
			CalibrationContext& mctx = const_cast<CalibrationContext&>(ctx);
			if (!mctx.pairedMotionPosSeeded) {
				mctx.pairedMotionPrevRefPos = refPose.trans;
				mctx.pairedMotionPrevTgtPos = tgtPose.trans;
				mctx.pairedMotionPosSeeded = true;
				// First sample of the run; nothing to compare against. Leave
				// pairedMotionValid = true so this sample contributes
				// normally.
			} else {
				const double refDelta = (refPose.trans - mctx.pairedMotionPrevRefPos).norm();
				const double tgtDelta = (tgtPose.trans - mctx.pairedMotionPrevTgtPos).norm();
				const bool refMoved = refDelta > kPairedMotionDeltaMeters;
				const bool tgtMoved = tgtDelta > kPairedMotionDeltaMeters;
				if (refMoved != tgtMoved) {
					// Exactly one moved: misaligned data. Don't count this
					// sample toward diversity, and bump the warning counter.
					pairedMotionValid = false;
					mctx.pairedMotionMismatchCount = std::min(
						mctx.pairedMotionMismatchCount + 1, 30);
				} else {
					// Both moved together OR both stationary -- both are fine
					// (the latter just doesn't extend diversity range). Decay
					// the warning counter so the banner clears once the user
					// fixes the mismatch.
					if (mctx.pairedMotionMismatchCount > 0) {
						--mctx.pairedMotionMismatchCount;
					}
				}
				mctx.pairedMotionPrevRefPos = refPose.trans;
				mctx.pairedMotionPrevTgtPos = tgtPose.trans;
			}
		}

		Sample collectedSample(refPose, tgtPose, glfwGetTime(), refSpeed, tgtSpeed);
		collectedSample.pairedMotionValid = pairedMotionValid;
		calibration.PushSample(collectedSample);

		// Feed the auto-lock detector with the same sample. We use the world
		// poses directly (not the post-calibration relative pose) so the
		// rigidity check is independent of the math's current solution --
		// the detector measures whether the two devices physically move
		// together, not whether the calibration thinks they do.
		Eigen::AffineCompact3d refWorld = Eigen::AffineCompact3d::Identity();
		refWorld.linear() = ConvertPose(reference).rot;
		refWorld.translation() = ConvertPose(reference).trans;
		Eigen::AffineCompact3d tgtWorld = Eigen::AffineCompact3d::Identity();
		tgtWorld.linear() = ConvertPose(target).rot;
		tgtWorld.translation() = ConvertPose(target).trans;
		const_cast<CalibrationContext&>(ctx).UpdateAutoLockDetector(refWorld, tgtWorld);

		// Push motion-coverage metrics for the live sample buffer. The Calibration
		// Progress popup reads these via Metrics:: and renders progress bars so
		// the user can see whether their motion is varied enough to fit a clean
		// calibration. Computed every CollectSample tick; cheap (linear scan).
		Metrics::translationDiversity.Push(calibration.TranslationDiversity());
		Metrics::rotationDiversity.Push(calibration.RotationDiversity());
		Metrics::translationAxisRangesCm.Push(calibration.TranslationAxisRangesCm());
		Metrics::pairedMotionWarningCount.Push((double)ctx.pairedMotionMismatchCount);

		// Push observed jitter every tick so the AUTO calibration-speed selector
		// in ResolvedCalibrationSpeed sees a fresh value -- the previous push
		// site lived in the Begin state branch alone, where the buffer is still
		// empty (just-cleared) and so always pushed 0. AUTO would then read 0
		// from Metrics::jitterRef.last() and lock on FAST forever, regardless of
		// the user's actual tracker quality. Recomputing here is cheap (Welford
		// over the deque); skipping early ticks before two valid samples exist
		// keeps the reading honest -- WelfordStdMagnitude returns 0 on n<2 and
		// we don't want to advertise that as a meaningful "jitter".
		if (calibration.SampleCount() >= 2) {
			Metrics::jitterRef.Push(calibration.ReferenceJitter());
			Metrics::jitterTarget.Push(calibration.TargetJitter());
		}

		return true;
	}

	bool AssignTargets() {
		auto state = VRState::Load();

		if (CalCtx.referenceID < 0) {
			CalCtx.referenceID = state.FindDevice(CalCtx.referenceStandby.trackingSystem, CalCtx.referenceStandby.model, CalCtx.referenceStandby.serial);
		}

		if (CalCtx.targetID < 0) {
			CalCtx.targetID = state.FindDevice(CalCtx.targetStandby.trackingSystem, CalCtx.targetStandby.model, CalCtx.targetStandby.serial);
		}

		for (int i = 0; i < CalCtx.MAX_CONTROLLERS; i++) {
			if (i < state.devices.size()
				&& state.devices[i].trackingSystem == CalCtx.targetTrackingSystem
				&& state.devices[i].deviceClass == vr::TrackedDeviceClass_Controller
				&& (state.devices[i].controllerRole == vr::TrackedControllerRole_LeftHand || state.devices[i].controllerRole == vr::TrackedControllerRole_RightHand))
			{
				CalCtx.controllerIDs[i] = state.devices[i].id;
			} else {
				CalCtx.controllerIDs[i] = -1;
			}
		}

		return CalCtx.referenceID >= 0 && CalCtx.targetID >= 0;
	}

	// External smoothing-tool detection (kKnownTools, kSubstringTools,
	// DetectExternalSmoothingTool) relocated to the Smoothing overlay on
	// 2026-05-11 (Protocol v12 migration). The Smoothing plugin scans on
	// its own Tick and surfaces the banner inside its Prediction sub-tab.

	// Discrete cross-correlation between two equal-length speed series. Returns false
	// if there isn't enough signal energy to produce a trustworthy estimate. On
	// success, *lagSamplesOut is the lag (in samples) at which the correlation is
	// maximised, with sub-sample resolution from a quadratic fit around the peak.
	// Positive lag means the target signal lags the reference signal (target arrives later).
	//
	// 2026-05-05: math extracted to spacecal::latency::EstimateLagTimeDomain in
	// LatencyEstimator.h so it can be unit-tested directly. This wrapper
	// dispatches to either the original time-domain CC (default) or to the
	// GCC-PHAT variant (Knapp & Carter 1976) based on `useGccPhat`. GCC-PHAT
	// is opt-in until real-session evidence confirms the whitened-spectrum
	// estimate is preferable; the math review pinned time-domain CC as
	// "empirically validated, not a sore point".
	bool EstimateLatencyLagSamples(
		const std::deque<double>& ref,
		const std::deque<double>& tgt,
		int maxTau,
		bool useGccPhat,
		double* lagSamplesOut)
	{
		if (useGccPhat) {
			return spacecal::latency::EstimateLagGccPhat(ref, tgt, maxTau, lagSamplesOut);
		}
		return spacecal::latency::EstimateLagTimeDomain(ref, tgt, maxTau, lagSamplesOut);
	}

double GetActiveLatencyOffsetMs(const CalibrationContext& ctx)
{
	if (ctx.useUpstreamMath) return 0.0;
	return ctx.latencyAutoDetect ? ctx.estimatedLatencyOffsetMs : ctx.targetLatencyOffsetMs;
}
