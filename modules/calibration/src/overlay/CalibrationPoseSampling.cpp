#include "CalibrationPoseSampling.h"

#include "AutoLockHysteresis.h"
#include "CalibrationInternal.h"
#include "CalibrationMetrics.h"
#include "ControllerInput.h"
#include "HeadMountPoseSampling.h"
#include "RelocGuard.h" // spacecal::reloc_guard -- post-relocalization sample quarantine.
#include "RuntimeHealthSummary.h"
#include "TargetStabilityGate.h" // spacecal::target_stability -- continuous-solve back-off.
#include "RotationMatrix3.h"
#include "VRState.h"

#include <GLFW/glfw3.h>

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <deque>
#include <set>
#include <string>
#include <vector>

inline vr::HmdQuaternion_t operator*(const vr::HmdQuaternion_t& lhs, const vr::HmdQuaternion_t& rhs)
{
	return {(lhs.w * rhs.w) - (lhs.x * rhs.x) - (lhs.y * rhs.y) - (lhs.z * rhs.z),
	        (lhs.w * rhs.x) + (lhs.x * rhs.w) + (lhs.y * rhs.z) - (lhs.z * rhs.y),
	        (lhs.w * rhs.y) + (lhs.y * rhs.w) + (lhs.z * rhs.x) - (lhs.x * rhs.z),
	        (lhs.w * rhs.z) + (lhs.z * rhs.w) + (lhs.x * rhs.y) - (lhs.y * rhs.x)};
}

CalibrationCalc calibration;

inline vr::HmdVector3d_t quaternionRotateVector(const vr::HmdQuaternion_t& quat, const double (&vector)[3])
{
	vr::HmdQuaternion_t vectorQuat = {0.0, vector[0], vector[1], vector[2]};
	vr::HmdQuaternion_t conjugate = {quat.w, -quat.x, -quat.y, -quat.z};
	auto rotatedVectorQuat = quat * vectorQuat * conjugate;
	return {rotatedVectorQuat.x, rotatedVectorQuat.y, rotatedVectorQuat.z};
}

inline Eigen::Matrix3d quaternionRotateMatrix(const vr::HmdQuaternion_t& quat)
{
	return Eigen::Quaterniond(quat.w, quat.x, quat.y, quat.z).toRotationMatrix();
}

struct DSample
{
	bool valid;
	Eigen::Vector3d ref, target;
};

bool StartsWith(const std::string& str, const std::string& prefix)
{
	if (str.length() < prefix.length()) return false;

	return str.compare(0, prefix.length(), prefix) == 0;
}

bool EndsWith(const std::string& str, const std::string& suffix)
{
	if (str.length() < suffix.length()) return false;

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

	Eigen::Quaterniond rotQuat = Eigen::AngleAxisd(euler(0), Eigen::Vector3d::UnitZ()) *
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

Pose ConvertPose(const vr::DriverPose_t& driverPose)
{
	Eigen::Quaterniond driverToWorldQ(driverPose.qWorldFromDriverRotation.w, driverPose.qWorldFromDriverRotation.x,
	                                  driverPose.qWorldFromDriverRotation.y, driverPose.qWorldFromDriverRotation.z);
	Eigen::Vector3d driverToWorldV(driverPose.vecWorldFromDriverTranslation[0],
	                               driverPose.vecWorldFromDriverTranslation[1],
	                               driverPose.vecWorldFromDriverTranslation[2]);

	Eigen::Quaterniond driverRot = driverToWorldQ * Eigen::Quaterniond(driverPose.qRotation.w, driverPose.qRotation.x,
	                                                                   driverPose.qRotation.y, driverPose.qRotation.z);
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

	Eigen::Vector3d driverPos =
	    driverToWorldV + driverToWorldQ * Eigen::Vector3d(driverPose.vecPosition[0], driverPose.vecPosition[1],
	                                                      driverPose.vecPosition[2]);

	Eigen::AffineCompact3d xform = Eigen::Translation3d(driverPos) * driverRot;

	return Pose(xform);
}

struct PoseHealthProbe
{
	double ageMs = 0.0;
	double gapMs = 0.0;
	bool unchanged = false;
	bool stale = false;
	bool jump = false;
};

struct PoseHealthState
{
	bool seeded = false;
	LARGE_INTEGER sampleTime{};
	Eigen::Vector3d pos = Eigen::Vector3d::Zero();
	Eigen::Quaterniond rot = Eigen::Quaterniond::Identity();
};

inline double QpcDeltaMs(LONGLONG newer, LONGLONG older, LONGLONG frequency)
{
	if (frequency <= 0 || newer <= older) return 0.0;
	return (static_cast<double>(newer - older) * 1000.0) / static_cast<double>(frequency);
}

double Vector3Norm(const double (&v)[3])
{
	return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

bool PoseLooksZero(const vr::DriverPose_t& pose)
{
	const double posNorm = Vector3Norm(pose.vecPosition);
	const double quatVecNorm = std::sqrt(pose.qRotation.x * pose.qRotation.x + pose.qRotation.y * pose.qRotation.y +
	                                     pose.qRotation.z * pose.qRotation.z);
	return posNorm < 1e-9 && quatVecNorm < 1e-9 && std::fabs(pose.qRotation.w - 1.0) < 1e-9;
}

void RecordReplaySampleDiagnostics(const Sample& sample, bool accepted)
{
	Metrics::ReplaySampleDiagnostics diag;
	diag.observed = true;
	diag.accepted = accepted;
	diag.pairedMotionValid = sample.pairedMotionValid;
	diag.refDeviceConnected = sample.refDeviceConnected;
	diag.targetDeviceConnected = sample.targetDeviceConnected;
	diag.refPoseValid = sample.refPoseValid;
	diag.targetPoseValid = sample.targetPoseValid;
	diag.refTrackingResult = sample.refTrackingResult;
	diag.targetTrackingResult = sample.targetTrackingResult;
	diag.refPoseAgeMs = sample.refPoseAgeMs;
	diag.targetPoseAgeMs = sample.targetPoseAgeMs;
	diag.refPoseGapMs = sample.refPoseGapMs;
	diag.targetPoseGapMs = sample.targetPoseGapMs;
	diag.refLinearSpeedMps = sample.refLinearSpeedMps;
	diag.targetLinearSpeedMps = sample.targetLinearSpeedMps;
	diag.refAngularSpeedRadps = sample.refAngularSpeedRadps;
	diag.targetAngularSpeedRadps = sample.targetAngularSpeedRadps;
	diag.refZeroPose = sample.refZeroPose;
	diag.targetZeroPose = sample.targetZeroPose;
	diag.refPoseUnchanged = sample.refPoseUnchanged;
	diag.targetPoseUnchanged = sample.targetPoseUnchanged;
	diag.trackingPoseStale = sample.trackingPoseStale;
	diag.trackingPoseJump = sample.trackingPoseJump;
	Metrics::SetTickReplaySampleDiagnostics(diag);
}

void RecordRejectedReplaySampleDiagnostics(const vr::DriverPose_t& reference, const vr::DriverPose_t& target)
{
	Metrics::ReplaySampleDiagnostics diag;
	diag.observed = true;
	diag.accepted = false;
	diag.pairedMotionValid = false;
	diag.refDeviceConnected = reference.deviceIsConnected;
	diag.targetDeviceConnected = target.deviceIsConnected;
	diag.refPoseValid = reference.poseIsValid;
	diag.targetPoseValid = target.poseIsValid;
	diag.refTrackingResult = static_cast<int>(reference.result);
	diag.targetTrackingResult = static_cast<int>(target.result);
	diag.refLinearSpeedMps = Vector3Norm(reference.vecVelocity);
	diag.targetLinearSpeedMps = Vector3Norm(target.vecVelocity);
	diag.refAngularSpeedRadps = Vector3Norm(reference.vecAngularVelocity);
	diag.targetAngularSpeedRadps = Vector3Norm(target.vecAngularVelocity);
	diag.refZeroPose = PoseLooksZero(reference);
	diag.targetZeroPose = PoseLooksZero(target);
	Metrics::SetTickReplaySampleDiagnostics(diag);
}

bool CandidateRelPoseMadMm(const CalibrationContext& ctx, const Eigen::AffineCompact3d& rel, double& outMadMm)
{
	std::deque<Eigen::AffineCompact3d> window = ctx.autoLockHistory;
	window.push_back(rel);
	while (window.size() > spacecal::autolock::kHistoryMax)
		window.pop_front();
	if (window.size() < spacecal::autolock::kSamplesNeeded) return false;
	outMadMm = spacecal::autolock::RobustTranslDeviation(window) * 1000.0;
	return true;
}

PoseHealthProbe ProbePoseHealth(int deviceId, const Pose& pose, const LARGE_INTEGER& sampleTime,
                                const LARGE_INTEGER& now, const LARGE_INTEGER& frequency)
{
	constexpr double kStalePoseAgeMs = 120.0;
	constexpr double kStalePoseGapMs = 150.0;
	constexpr double kJumpTranslationM = 0.50;
	constexpr double kJumpSpeedMps = 3.0;
	constexpr double kJumpAngleDeg = 75.0;
	constexpr double kJumpAngularSpeedDegPerSec = 360.0;

	static std::array<PoseHealthState, vr::k_unMaxTrackedDeviceCount> s_state;

	PoseHealthProbe out{};
	if (deviceId < 0 || deviceId >= static_cast<int>(s_state.size()) || sampleTime.QuadPart <= 0 ||
	    frequency.QuadPart <= 0) {
		return out;
	}

	out.ageMs = QpcDeltaMs(now.QuadPart, sampleTime.QuadPart, frequency.QuadPart);
	out.stale = out.ageMs > kStalePoseAgeMs;

	PoseHealthState& state = s_state[static_cast<size_t>(deviceId)];
	const Eigen::Quaterniond currentRot(pose.rot);
	if (state.seeded) {
		const double translationDeltaM = (pose.trans - state.pos).norm();
		const double angleDeg = currentRot.angularDistance(state.rot) * (180.0 / EIGEN_PI);
		out.unchanged = translationDeltaM < 1e-9 && angleDeg < 1e-6;
	}
	if (state.seeded && sampleTime.QuadPart > state.sampleTime.QuadPart) {
		out.gapMs = QpcDeltaMs(sampleTime.QuadPart, state.sampleTime.QuadPart, frequency.QuadPart);
		if (out.gapMs > kStalePoseGapMs) {
			out.stale = true;
		}

		const double dtSeconds = out.gapMs / 1000.0;
		if (dtSeconds > 0.0 && dtSeconds < 1.0) {
			const double translationDeltaM = (pose.trans - state.pos).norm();
			const double angleDeg = currentRot.angularDistance(state.rot) * (180.0 / EIGEN_PI);
			const double speedMps = translationDeltaM / dtSeconds;
			const double angularSpeedDegPerSec = angleDeg / dtSeconds;
			out.jump = (translationDeltaM > kJumpTranslationM && speedMps > kJumpSpeedMps) ||
			           (angleDeg > kJumpAngleDeg && angularSpeedDegPerSec > kJumpAngularSpeedDegPerSec);
		}
	}

	if (!state.seeded || sampleTime.QuadPart >= state.sampleTime.QuadPart) {
		state.seeded = true;
		state.sampleTime = sampleTime;
		state.pos = pose.trans;
		state.rot = currentRot.normalized();
	}

	return out;
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
inline bool ExtrapolateReferencePose(vr::DriverPose_t& pose, double dtSeconds)
{
	if (dtSeconds == 0.0) return true;

	// Sanity-check the velocity components. A momentary tracking glitch can produce
	// NaN/inf or wildly large velocity values; applying those to the pose would
	// teleport the reference and pollute the sample.
	const double maxLinearMps = 50.0;    // ~180 km/h, far beyond any real head/tracker motion
	const double maxAngularRadps = 50.0; // ~8 rev/s
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
	double angSpeed = std::sqrt(pose.vecAngularVelocity[0] * pose.vecAngularVelocity[0] +
	                            pose.vecAngularVelocity[1] * pose.vecAngularVelocity[1] +
	                            pose.vecAngularVelocity[2] * pose.vecAngularVelocity[2]);
	if (angSpeed > 1e-9) {
		double angle = angSpeed * dtSeconds;
		double half = angle * 0.5;
		double s = std::sin(half);
		double axisX = pose.vecAngularVelocity[0] / angSpeed;
		double axisY = pose.vecAngularVelocity[1] / angSpeed;
		double axisZ = pose.vecAngularVelocity[2] / angSpeed;
		vr::HmdQuaternion_t deltaQ = {std::cos(half), axisX * s, axisY * s, axisZ * s};
		pose.qRotation = deltaQ * pose.qRotation;
	}

	return true;
}

bool CollectSample(const CalibrationContext& ctx)
{
	vr::DriverPose_t reference, target;
	int targetSourceDeviceId = ctx.targetID;
	reference.poseIsValid = false;
	reference.result = vr::ETrackingResult::TrackingResult_Uninitialized;
	target.poseIsValid = false;
	target.result = vr::ETrackingResult::TrackingResult_Uninitialized;

	reference = ctx.devicePoses[ctx.referenceID];

	// AutoPaired: when a head-mount tracker is resolved and valid, use its
	// pose (composed with headFromTracker) as the target-side observation.
	// Gives continuous paired samples without user motion. Off mode, or any
	// tick where the tracker pose is invalid, falls through to the regular
	// ctx.targetID path. Restricted to continuous states so one-shot
	// Begin/Rotation/Translation always read the user-selected target
	// device, regardless of head-mount config.
	const bool inContinuousFamily =
	    ctx.state == CalibrationState::Continuous || ctx.state == CalibrationState::ContinuousStandby;
	const bool headMountEngaged = inContinuousFamily && ctx.headMount.mode >= HeadMountMode::AutoPaired &&
	                              ctx.headMount.offsetCalibrated && ctx.headMount.deviceID >= 0 &&
	                              ctx.headMount.deviceID < (int32_t)vr::k_unMaxTrackedDeviceCount;
	if (inContinuousFamily && ctx.headMount.mode >= HeadMountMode::AutoPaired && !ctx.headMount.offsetCalibrated) {
		static bool s_loggedNeedsOffset = false;
		if (!s_loggedNeedsOffset) {
			s_loggedNeedsOffset = true;
			Metrics::WriteLogAnnotation("[head-mount] auto-paired synthetic sampling waiting for calibrated offset");
		}
	}

	if (headMountEngaged) {
		targetSourceDeviceId = ctx.headMount.deviceID;
		const vr::DriverPose_t& trackerRaw = ctx.devicePoses[ctx.headMount.deviceID];
		if (trackerRaw.poseIsValid && trackerRaw.result == vr::ETrackingResult::TrackingResult_Running_OK) {
			// Log first use each session (edge: headMountEngaged just became true
			// or was re-entered after an invalid tick).
			{
				static bool s_loggedActive = false;
				if (!s_loggedActive) {
					s_loggedActive = true;
					char mbuf[256];
					const std::string& mdl = CalCtx.headMount.trackerModel;
					snprintf(mbuf, sizeof mbuf, "[head-mount] auto-paired sampling active: deviceID=%d model='%s'",
					         (int)ctx.headMount.deviceID, mdl.c_str());
					Metrics::WriteLogAnnotation(mbuf);
				}
			}
			// Compose tracker world pose with headFromTracker so the math sees a
			// pose at the HMD's reference point (IPD midpoint), not the tracker
			// mount point. head_pose = tracker_pose * headFromTracker.
			// Composition and conversion logic lives in HeadMountPoseSampling.h
			// so unit tests can exercise it without a live VR session.
			const Eigen::AffineCompact3d headWorld =
			    spacecal::headmount::ComputeHeadWorldPose(trackerRaw, ctx.headMount.headFromTracker);

			// Encode the derived head pose into a synthetic DriverPose_t with an
			// identity worldFromDriver. Velocity fields are zeroed intentionally --
			// this pose is only used for translation/rotation sample collection,
			// not for velocity extrapolation.
			target = vr::DriverPose_t{};
			target.poseIsValid = true;
			target.result = vr::ETrackingResult::TrackingResult_Running_OK;
			target.deviceIsConnected = true;
			target.qWorldFromDriverRotation.w = 1.0;
			target.qWorldFromDriverRotation.x = 0.0;
			target.qWorldFromDriverRotation.y = 0.0;
			target.qWorldFromDriverRotation.z = 0.0;
			Eigen::Quaterniond headQ(headWorld.linear());
			headQ.normalize();
			target.qRotation.w = headQ.w();
			target.qRotation.x = headQ.x();
			target.qRotation.y = headQ.y();
			target.qRotation.z = headQ.z();
			target.vecPosition[0] = headWorld.translation().x();
			target.vecPosition[1] = headWorld.translation().y();
			target.vecPosition[2] = headWorld.translation().z();
		}
		// Invalid tracker: target stays zero/invalid; the validity gate below
		// rejects the sample. No silent fallback to ctx.targetID.
		// Log the first invalid tick and the recovery edge.
		{
			static bool s_trackerWasValid = false;
			const bool trackerNowValid =
			    trackerRaw.poseIsValid && trackerRaw.result == vr::ETrackingResult::TrackingResult_Running_OK;
			if (s_trackerWasValid && !trackerNowValid) {
				Metrics::WriteLogAnnotation(
				    "[head-mount] tracker invalid; skipping synthetic head sample for this tick");
			}
			else if (!s_trackerWasValid && trackerNowValid) {
				Metrics::WriteLogAnnotation("[head-mount] tracker valid again");
			}
			s_trackerWasValid = trackerNowValid;
		}
	}
	else {
		target = ctx.devicePoses[ctx.targetID];
	}

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
		static std::set<std::pair<int, int>> s_seenAnomalies;
		auto checkMag = [&](const vr::DriverPose_t& p, int whichKind, const char* label) {
			const double mag2 = p.vecPosition[0] * p.vecPosition[0] + p.vecPosition[1] * p.vecPosition[1] +
			                    p.vecPosition[2] * p.vecPosition[2];
			if (mag2 > 10000.0) { // > 100 m
				auto key = std::make_pair(whichKind, (int)p.result);
				if (s_seenAnomalies.insert(key).second) {
					char abuf[280];
					snprintf(abuf, sizeof abuf,
					         "[quash-bleed-suspect] %s pose magnitude unusually large:"
					         " pos=(%.1f,%.1f,%.1f) result=%d poseIsValid=%d",
					         label, p.vecPosition[0], p.vecPosition[1], p.vecPosition[2], (int)p.result,
					         (int)p.poseIsValid);
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
	const bool refSilentInvalid =
	    !reference.poseIsValid && reference.result == vr::ETrackingResult::TrackingResult_Running_OK;
	const bool tgtSilentInvalid =
	    !target.poseIsValid && target.result == vr::ETrackingResult::TrackingResult_Running_OK;
	if (!reference.poseIsValid || reference.result != vr::ETrackingResult::TrackingResult_Running_OK) {
		CalCtx.Log("Reference device is not tracking\n");
		ok = false;
	}
	if (!target.poseIsValid || target.result != vr::ETrackingResult::TrackingResult_Running_OK) {
		CalCtx.Log("Target device is not tracking\n");
		ok = false;
	}
	// Feed this tick's target validity into the rolling EWMA the continuous solve
	// gate reads (TargetStabilityGate.h). Updated for both valid and invalid ticks
	// so it reflects the true recent dropout rate of an intermittent target link.
	if (inContinuousFamily) {
		const bool targetInvalidThisTick =
		    !target.poseIsValid || target.result != vr::ETrackingResult::TrackingResult_Running_OK;
		CalCtx.targetInvalidEwma = spacecal::target_stability::UpdateInvalidEwma(
		    CalCtx.targetInvalidEwma, targetInvalidThisTick, spacecal::target_stability::kSolveDeferEwmaAlpha);
	}
	if (refSilentInvalid || tgtSilentInvalid) {
		Metrics::WriteLogAnnotation(refSilentInvalid && tgtSilentInvalid ? "silent_invalid_pose_rejected: ref+tgt"
		                            : refSilentInvalid                   ? "silent_invalid_pose_rejected: ref"
		                                                                 : "silent_invalid_pose_rejected: tgt");
	}
	if (!ok) {
		RecordRejectedReplaySampleDiagnostics(reference, target);
		openvr_pair::common::RuntimePoseHealthSample runtimeHealth{};
		runtimeHealth.invalid = true;
		runtimeHealth.refTrackingResult = static_cast<int>(reference.result);
		runtimeHealth.targetTrackingResult = static_cast<int>(target.result);
		openvr_pair::common::RecordRuntimePoseHealth(runtimeHealth);
		static double s_lastStrictRejectLog = -1e9;
		const double nowSec = glfwGetTime();
		if (nowSec - s_lastStrictRejectLog >= 1.0) {
			s_lastStrictRejectLog = nowSec;
			char hbuf[520];
			snprintf(hbuf, sizeof hbuf,
			         "[cal-sample-health-reject] state=%d ref_connected=%d ref_pose_valid=%d ref_result=%d"
			         " target_connected=%d target_pose_valid=%d target_result=%d"
			         " ref_silent_invalid=%d target_silent_invalid=%d head_mount_source=%d",
			         (int)CalCtx.state, (int)reference.deviceIsConnected, (int)reference.poseIsValid,
			         (int)reference.result, (int)target.deviceIsConnected, (int)target.poseIsValid, (int)target.result,
			         (int)refSilentInvalid, (int)tgtSilentInvalid, (int)(headMountEngaged ? 1 : 0));
			Metrics::WriteLogAnnotation(hbuf);
		}
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
	LARGE_INTEGER nowQpc{};
	static LARGE_INTEGER qpcFrequency{};
	static bool haveQpcFrequency = false;
	QueryPerformanceCounter(&nowQpc);
	if (!haveQpcFrequency) {
		QueryPerformanceFrequency(&qpcFrequency);
		haveQpcFrequency = true;
	}
	const PoseHealthProbe refHealth =
	    ProbePoseHealth(ctx.referenceID, refPose, ctx.devicePoseSampleTimes[ctx.referenceID], nowQpc, qpcFrequency);
	const PoseHealthProbe targetHealth = ProbePoseHealth(
	    targetSourceDeviceId, tgtPose, ctx.devicePoseSampleTimes[targetSourceDeviceId], nowQpc, qpcFrequency);
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
		}
		else {
			const double refDelta = (refPose.trans - mctx.pairedMotionPrevRefPos).norm();
			const double tgtDelta = (tgtPose.trans - mctx.pairedMotionPrevTgtPos).norm();
			const bool refMoved = refDelta > kPairedMotionDeltaMeters;
			const bool tgtMoved = tgtDelta > kPairedMotionDeltaMeters;
			if (refMoved != tgtMoved) {
				// Exactly one moved: misaligned data. Don't count this
				// sample toward diversity, and bump the warning counter.
				pairedMotionValid = false;
				mctx.pairedMotionMismatchCount = std::min(mctx.pairedMotionMismatchCount + 1, 30);
			}
			else {
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

	Sample collectedSample(refPose, tgtPose, glfwGetTime());
	collectedSample.pairedMotionValid = pairedMotionValid;
	collectedSample.refDeviceConnected = reference.deviceIsConnected;
	collectedSample.targetDeviceConnected = target.deviceIsConnected;
	collectedSample.refPoseValid = reference.poseIsValid;
	collectedSample.targetPoseValid = target.poseIsValid;
	collectedSample.refTrackingResult = static_cast<int>(reference.result);
	collectedSample.targetTrackingResult = static_cast<int>(target.result);
	collectedSample.refPoseAgeMs = refHealth.ageMs;
	collectedSample.targetPoseAgeMs = targetHealth.ageMs;
	collectedSample.refPoseGapMs = refHealth.gapMs;
	collectedSample.targetPoseGapMs = targetHealth.gapMs;
	collectedSample.refLinearSpeedMps = Vector3Norm(reference.vecVelocity);
	collectedSample.targetLinearSpeedMps = Vector3Norm(target.vecVelocity);
	collectedSample.refAngularSpeedRadps = Vector3Norm(reference.vecAngularVelocity);
	collectedSample.targetAngularSpeedRadps = Vector3Norm(target.vecAngularVelocity);
	collectedSample.refZeroPose = PoseLooksZero(reference);
	collectedSample.targetZeroPose = PoseLooksZero(target);
	collectedSample.refPoseUnchanged = refHealth.unchanged;
	collectedSample.targetPoseUnchanged = targetHealth.unchanged;
	collectedSample.trackingPoseStale = refHealth.stale || targetHealth.stale;
	collectedSample.trackingPoseJump = refHealth.jump || targetHealth.jump;

	Eigen::AffineCompact3d refWorld = Eigen::AffineCompact3d::Identity();
	refWorld.linear() = refPose.rot;
	refWorld.translation() = refPose.trans;
	Eigen::AffineCompact3d tgtWorld = Eigen::AffineCompact3d::Identity();
	tgtWorld.linear() = tgtPose.rot;
	tgtWorld.translation() = tgtPose.trans;
	const Eigen::AffineCompact3d candidateRel = refWorld.inverse() * tgtWorld;

	// Post-relocalization sample quarantine (experimental, default off). After a
	// detected HMD relocalization the inside-out world frame has shifted; drop
	// samples for a short settle window so the shifted pose pairs never enter the
	// solve OR the AUTO-lock MAD window. The early-release path tests this
	// candidate against the existing relative-pose window, so a non-poisoned row
	// can resume immediately while a shifted row falls back to the time window.
	bool quarantineActive = false;
	if (ctx.experimentalRelocQuarantineEnabled) {
		double candidateMadMm = 0.0;
		const bool haveCandidateMad = CandidateRelPoseMadMm(ctx, candidateRel, candidateMadMm);
		quarantineActive = spacecal::reloc_guard::QuarantineActive(
		    glfwGetTime(), ctx.lastRelocDetectedTime, ctx.experimentalRelocQuarantineSec,
		    haveCandidateMad ? candidateMadMm : 0.0, haveCandidateMad ? ctx.autoLockMadFloor * 1000.0 : 0.0,
		    spacecal::reloc_guard::kDefaultClearMult);
	}
	if (quarantineActive) {
		RecordReplaySampleDiagnostics(collectedSample, false);
		static double s_lastQuarantineLog = -1e9;
		const double nowLog = glfwGetTime();
		if (nowLog - s_lastQuarantineLog >= 1.0) {
			s_lastQuarantineLog = nowLog;
			Metrics::WriteLogAnnotation("reloc_quarantine_sample_dropped");
		}
		return true;
	}
	RecordReplaySampleDiagnostics(collectedSample, true);
	calibration.PushSample(collectedSample);
	openvr_pair::common::RuntimePoseHealthSample runtimeHealth{};
	runtimeHealth.refPoseAgeMs = refHealth.ageMs;
	runtimeHealth.targetPoseAgeMs = targetHealth.ageMs;
	runtimeHealth.refPoseGapMs = refHealth.gapMs;
	runtimeHealth.targetPoseGapMs = targetHealth.gapMs;
	runtimeHealth.stale = collectedSample.trackingPoseStale;
	runtimeHealth.jump = collectedSample.trackingPoseJump;
	runtimeHealth.refTrackingResult = static_cast<int>(reference.result);
	runtimeHealth.targetTrackingResult = static_cast<int>(target.result);
	openvr_pair::common::RecordRuntimePoseHealth(runtimeHealth);

	// Feed the auto-lock detector with the same sample. We use the world
	// poses directly (not the post-calibration relative pose) so the
	// rigidity check is independent of the math's current solution --
	// the detector measures whether the two devices physically move
	// together, not whether the calibration thinks they do.
	const_cast<CalibrationContext&>(ctx).UpdateAutoLockDetector(refWorld, tgtWorld);

	// Push motion-coverage metrics for the live sample buffer. The Calibration
	// Progress popup reads these via Metrics:: and renders progress bars so
	// the user can see whether their motion is varied enough to fit a clean
	// calibration. Computed every CollectSample tick; cheap (linear scan).
	Metrics::translationDiversity.Push(calibration.TranslationDiversity());
	Metrics::rotationDiversity.Push(calibration.RotationDiversity());
	Metrics::translationAxisRangesCm.Push(calibration.TranslationAxisRangesCm());
	Metrics::pairedMotionWarningCount.Push((double)ctx.pairedMotionMismatchCount);

	// Push raw position spread every tick for debug plots. This is not used
	// by AUTO speed selection because normal calibration movement makes world
	// position spread large even when tracking quality is good.
	if (calibration.SampleCount() >= 2) {
		Metrics::jitterRef.Push(calibration.ReferenceJitter());
		Metrics::jitterTarget.Push(calibration.TargetJitter());
	}

	// Head-mount diagnostic metrics.
	{
		const bool hmActive = spacecal::headmount::IsTrackerValidForSampling(ctx.headMount, ctx.devicePoses,
		                                                                     vr::k_unMaxTrackedDeviceCount);
		Metrics::headMountActive.Push(hmActive);

		// questHmdVsProxyDeltaMm: distance between the HMD's reported world
		// position and the head-mount tracker's derived head position.
		// Spikes indicate a Quest SLAM snap or mount slip.
		if (hmActive && reference.poseIsValid && reference.result == vr::ETrackingResult::TrackingResult_Running_OK) {
			const Pose hmdPose = ConvertPose(reference);
			const Eigen::AffineCompact3d proxyHeadWorld = spacecal::headmount::ComputeHeadWorldPose(
			    ctx.devicePoses[ctx.headMount.deviceID], ctx.headMount.headFromTracker);
			const double deltaM = (hmdPose.trans - proxyHeadWorld.translation()).norm();
			Metrics::questHmdVsProxyDeltaMm.Push(deltaM * 1000.0);
		}
	}

	return true;
}

bool AssignTargets()
{
	auto state = VRState::Load();

	if (CalCtx.referenceID < 0) {
		CalCtx.referenceID = state.FindDevice(CalCtx.referenceStandby.trackingSystem, CalCtx.referenceStandby.model,
		                                      CalCtx.referenceStandby.serial);
	}

	if (CalCtx.targetID < 0) {
		CalCtx.targetID = state.FindDevice(CalCtx.targetStandby.trackingSystem, CalCtx.targetStandby.model,
		                                   CalCtx.targetStandby.serial);
	}

	// Resolve the head-mount tracker by serial each scan so reconnections are
	// picked up automatically. Serial is authoritative; model is used as a
	// soft hint by FindDevice. Resolution runs whenever a serial is configured
	// regardless of mode: the offset-calibration modal needs the deviceID to
	// flow samples to its solver before the user commits to a head-mount
	// mode. Downstream consumers (auto-paired CollectSample, the quash
	// decision) still gate on mode, so resolving here doesn't leak behavior
	// into Off mode.
	if (!CalCtx.headMount.trackerSerial.empty()) {
		int32_t resolvedID = state.FindDevice(CalCtx.headMount.trackerTrackingSystem, CalCtx.headMount.trackerModel,
		                                      CalCtx.headMount.trackerSerial);
		if (resolvedID < 0) {
			if (CalCtx.headMount.deviceID >= 0) {
				char lbuf[256];
				snprintf(lbuf, sizeof lbuf,
				         "[head-mount] tracker not found in VRState; deviceID reset"
				         " serial='%s' trackingSystem='%s'",
				         CalCtx.headMount.trackerSerial.c_str(), CalCtx.headMount.trackerTrackingSystem.c_str());
				Metrics::WriteLogAnnotation(lbuf);
			}
			CalCtx.headMount.deviceID = -1;
		}
		else {
			if (CalCtx.headMount.deviceID != resolvedID) {
				char lbuf[256];
				snprintf(lbuf, sizeof lbuf, "[head-mount] tracker resolved: serial='%s' deviceID=%d",
				         CalCtx.headMount.trackerSerial.c_str(), (int)resolvedID);
				Metrics::WriteLogAnnotation(lbuf);
			}
			CalCtx.headMount.deviceID = resolvedID;
		}
	}
	else {
		{
			static int s_lastLoggedMode = -999;
			const int curMode = (int)CalCtx.headMount.mode;
			if (curMode != s_lastLoggedMode) {
				s_lastLoggedMode = curMode;
				if (curMode > 0) {
					char lbuf[128];
					snprintf(lbuf, sizeof lbuf, "[head-mount] no serial configured but mode=%d; deviceID stays -1",
					         curMode);
					Metrics::WriteLogAnnotation(lbuf);
				}
			}
		}
		CalCtx.headMount.deviceID = -1;
	}

	{
		const size_t controllerCount = wkopenvr::controller_input::FillControllerIdsForTrackingSystem(
		    state.devices, CalCtx.targetTrackingSystem, CalCtx.controllerIDs, CalCtx.MAX_CONTROLLERS);
		static size_t s_lastControllerCount = SIZE_MAX;
		static std::string s_lastControllerTarget;
		if (controllerCount != s_lastControllerCount || CalCtx.targetTrackingSystem != s_lastControllerTarget) {
			s_lastControllerCount = controllerCount;
			s_lastControllerTarget = CalCtx.targetTrackingSystem;
			char lbuf[192];
			snprintf(lbuf, sizeof lbuf, "[controllers] assigned target-system controllers=%zu target='%s'",
			         controllerCount, CalCtx.targetTrackingSystem.c_str());
			Metrics::WriteLogAnnotation(lbuf);
		}
	}

	return CalCtx.referenceID >= 0 && CalCtx.targetID >= 0;
}

// External smoothing-tool detection (kKnownTools, kSubstringTools,
// DetectExternalSmoothingTool) relocated to the Smoothing overlay on
// 2026-05-11 (Protocol v12 migration). The Smoothing plugin scans on
// its own Tick and surfaces the banner inside its Prediction sub-tab.
