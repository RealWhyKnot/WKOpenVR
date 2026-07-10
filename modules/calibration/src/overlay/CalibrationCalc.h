#pragma once

#include <Eigen/Dense>
#include <openvr.h>
#include <string>
#include <vector>
#include <deque>
#include <iostream>
#include <limits>

#include "LeverArmCovariance.h"
#include "RelPoseLockGate.h"

struct Pose
{
	Eigen::Matrix3d rot;
	Eigen::Vector3d trans;

	Pose() {}
	Pose(const Eigen::AffineCompact3d& transform)
	{
		rot = transform.rotation();
		trans = transform.translation();
	}

	Pose(vr::HmdMatrix34_t hmdMatrix)
	{
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				rot(i, j) = hmdMatrix.m[i][j];
			}
		}
		trans = Eigen::Vector3d(hmdMatrix.m[0][3], hmdMatrix.m[1][3], hmdMatrix.m[2][3]);
	}
	Pose(vr::HmdQuaternion_t rot, const double* trans)
	{
		this->rot = Eigen::Matrix3d(Eigen::Quaterniond(rot.w, rot.x, rot.y, rot.z));
		this->trans = Eigen::Vector3d(trans[0], trans[1], trans[2]);
	}
	Pose(double x, double y, double z) : trans(Eigen::Vector3d(x, y, z)) {}

	Eigen::Matrix4d ToAffine() const
	{
		Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();

		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				matrix(i, j) = rot(i, j);
			}
			matrix(i, 3) = trans(i);
		}

		return matrix;
	}
};

struct Sample
{
	Pose ref, target;
	bool valid;
	double timestamp;
	// True when both the reference and target devices showed correlated
	// motion since the previous sample (or both were stationary; that is
	// not a failure mode -- it just doesn't contribute new variety). False
	// only when one device moved meaningfully while the other did not --
	// the classic "HMD frozen by passthrough/desktop overlay while target
	// tracker keeps reporting motion" case. Diversity scores filter this
	// out so the user's progress bars reflect data the math can actually
	// use, not raw target-tracker motion. Default true so existing
	// callers (tests, replay harness, the two `valid=true` constructors)
	// behave as they always did.
	bool pairedMotionValid = true;
	bool refDeviceConnected = true;
	bool targetDeviceConnected = true;
	bool refPoseValid = true;
	bool targetPoseValid = true;
	int refTrackingResult = static_cast<int>(vr::ETrackingResult::TrackingResult_Running_OK);
	int targetTrackingResult = static_cast<int>(vr::ETrackingResult::TrackingResult_Running_OK);
	double refPoseAgeMs = 0.0;
	double targetPoseAgeMs = 0.0;
	double refPoseGapMs = 0.0;
	double targetPoseGapMs = 0.0;
	double refLinearSpeedMps = 0.0;
	double targetLinearSpeedMps = 0.0;
	double refAngularSpeedRadps = 0.0;
	double targetAngularSpeedRadps = 0.0;
	bool refZeroPose = false;
	bool targetZeroPose = false;
	bool refPoseUnchanged = false;
	bool targetPoseUnchanged = false;
	bool trackingPoseStale = false;
	bool trackingPoseJump = false;
	Sample() : valid(false), timestamp(0) {}
	Sample(Pose ref, Pose target, double timestamp) : valid(true), ref(ref), target(target), timestamp(timestamp) {}
};

struct CalibrationResidualStats
{
	int count = 0;
	double meanM = std::numeric_limits<double>::infinity();
	double medianM = std::numeric_limits<double>::infinity();
	double madSigmaM = std::numeric_limits<double>::infinity();
	double p75M = std::numeric_limits<double>::infinity();
	double p90M = std::numeric_limits<double>::infinity();
	double p95M = std::numeric_limits<double>::infinity();
	double p99M = std::numeric_limits<double>::infinity();
	double maxM = std::numeric_limits<double>::infinity();
	double rmsM = std::numeric_limits<double>::infinity();
	double trimmedRmsM = std::numeric_limits<double>::infinity();
	double inlierFraction20Mm = 0.0;
	double inlierFraction50Mm = 0.0;
	double inlierFraction100Mm = 0.0;
	double outlierFraction = 0.0;
	int outlierCount = 0;
	int modifiedZOutlierCount = 0;
};

struct CalibrationQualityReport
{
	size_t sampleCount = 0;
	int validSampleCount = 0;
	int invalidSampleCount = 0;
	int pairedSampleCount = 0;
	int strictHealthySampleCount = 0;
	int refDisconnectedSampleCount = 0;
	int targetDisconnectedSampleCount = 0;
	int refPoseInvalidSampleCount = 0;
	int targetPoseInvalidSampleCount = 0;
	int refNonRunningSampleCount = 0;
	int targetNonRunningSampleCount = 0;
	int zeroPoseSampleCount = 0;
	int unchangedPoseSampleCount = 0;
	int highMotionSampleCount = 0;
	int trackingStaleSampleCount = 0;
	int trackingJumpSampleCount = 0;
	int totalRotationPairCount = 0;
	int deltaPair5DegCount = 0;
	int deltaPair10DegCount = 0;
	int deltaPair23DegCount = 0;
	int validRotationPairCount = 0;
	int translationRank = 0;
	Eigen::Vector3d refRangeM = Eigen::Vector3d::Zero();
	Eigen::Vector3d targetRangeM = Eigen::Vector3d::Zero();
	double refSpanM = 0.0;
	double targetSpanM = 0.0;
	double rotationSpanDeg = 0.0;
	double maxPoseAgeMs = 0.0;
	double maxPoseGapMs = 0.0;
	double maxLinearSpeedMps = 0.0;
	double maxAngularSpeedDegps = 0.0;
	double medianRotationDeltaDeg = 0.0;
	double maxRefRotationDeltaDeg = 0.0;
	double maxTargetRotationDeltaDeg = 0.0;
	double deltaPair23Fraction = 0.0;
	double axisVariance0 = 0.0;
	double axisVariance1 = 0.0;
	double axisVariance2 = 0.0;
	double axisVariance3 = 0.0;
	double rotationConditionRatio = 0.0;
	double rotationSingularMin = 0.0;
	double rotationSingularMax = 0.0;
	double positionConditionRatio = 0.0;
	double translationConditionRatio = 0.0;
	double translationSingularMin = 0.0;
	double translationSingularMax = 0.0;
	double targetPathLengthM = 0.0;
	double holdoutTrainRmsRatio = 0.0;
	double dynamicLimitM = std::numeric_limits<double>::infinity();
	bool legacyRmsPass = false;
	bool developAxisVariancePass = false;
	bool novaDeltaPairsPass = false;
	bool strictSamplesPass = false;
	bool geometryPass = false;
	bool robustResidualPass = false;
	bool holdoutPass = false;
	bool trackingHealthPass = true;
	bool shadowDynamicPass = false;
	Eigen::Vector3d posOffsetM = Eigen::Vector3d::Zero();
	CalibrationResidualStats residuals;
	CalibrationResidualStats holdoutResiduals;
};

struct CalibrationQualityVerdict
{
	bool wouldAccept = false;
	const char* reason = "unknown";
};

struct CalibrationQualityShadowSignals
{
	bool wouldAccept = false;
	const char* firstRejectReason = "unknown";
	bool legacyAcceptedButShadowRejected = false;
	bool lowResidualGeometryReject = false;
	bool trackingContaminated = false;
	bool novaDeltaPairsPass = false;
	bool novaWouldRejectForDeltaPairs = false;
};

CalibrationQualityVerdict EvaluateCalibrationQualityVerdict(const CalibrationQualityReport& report);
CalibrationQualityShadowSignals EvaluateCalibrationQualityShadowSignals(const CalibrationQualityReport& report);

class CalibrationCalc
{
public:
	static const double AxisVarianceThreshold;

	bool enableStaticRecalibration;
	bool lockRelativePosition = false;

	const Eigen::AffineCompact3d Transformation() const { return m_estimatedTransformation; }

	const Eigen::Vector3d EulerRotation() const
	{
		auto rot = m_estimatedTransformation.rotation();
		return rot.eulerAngles(2, 1, 0) * 180.0 / EIGEN_PI;
	}

	bool isValid() const { return m_isValid; }

	bool LastComputeUsedRelPose() const { return m_lastComputeUsedRelPose; }

	// Most recent RMS retargeting error of the applied calibration (the
	// `priorCalibrationError` computed inside ComputeIncremental and
	// pushed to Metrics::error_currentCal for the primary pair). Exposed
	// per-instance so the common-mode coherence check at the geometry-
	// shift fire site can read each AdditionalCalibration's latest error
	// without each extra needing its own global TimeSeries. Returns
	// INFINITY when no incremental compute has produced a validated
	// result yet (the value is initialized to INFINITY and only
	// overwritten by a successful ValidateCalibration path).
	double LastPriorErrorM() const { return m_lastPriorRetargetingErrorM; }

	double LastCandidateErrorM() const { return m_lastCandidateRetargetingErrorM; }

	const Eigen::AffineCompact3d RelativeTransformation() const { return m_refToTargetPose; }

	bool isRelativeTransformationCalibrated() const { return m_relativePosCalibrated; }

	void setRelativeTransformation(const Eigen::AffineCompact3d transform, bool calibrated)
	{
		m_refToTargetPose = transform;
		m_relativePosCalibrated = calibrated;
	}

	// Sample weighting for the relative-pose average (CalibrateByRelPose).
	// Uniform is the classic upstream mean; ScalarLever is the isotropic
	// 1/lever-arm^2 precision weight (replay A/B); Covariance is the full
	// anisotropic lever-arm model (LeverArmCovariance.h) the live tick uses
	// when the enhanced-tracking switch is on.
	enum class RelPoseWeightMode
	{
		Uniform,
		ScalarLever,
		Covariance
	};
	void SetRelPoseWeightMode(RelPoseWeightMode mode) { m_relPoseWeightMode = mode; }

	// Bool back-compat wrapper (replay A/B + existing tests): on maps to the
	// scalar lever weight, off to the uniform mean.
	void SetPrecisionWeightedRelPose(bool on)
	{
		m_relPoseWeightMode = on ? RelPoseWeightMode::ScalarLever : RelPoseWeightMode::Uniform;
	}

	// Noise model for the covariance weight (and the sequential validation
	// residual). Clamped to the LeverArmCovariance.h knob ranges.
	void SetLeverArmSigmas(double sigmaThetaRad, double sigmaJitM);

	// Observability gate (ObservabilityGate.h): when on, candidate updates to
	// a valid prior are projected onto the rotation-axis subspace the sample
	// window has actually excited; unobserved components hold the prior.
	// Follows the enhanced-tracking master switch live.
	void SetV2Math(bool on) { m_useV2Math = on; }

	// lambda_min of the last solve's excitation matrix (diagnostic; -1 until
	// a gated solve has run).
	double LastObservabilityLambdaMin() const { return m_lastObservabilityLambdaMin; }

	// SE(3) residual of the latest valid sample against `applied`:
	// Log(applied * estimate^-1) with estimate = R * S * T^-1, ordering
	// [rho; phi] (Se3Log.h). Outputs the sample's ref/target translations so
	// the caller can whiten with the lever-arm covariance. False when no
	// valid sample or no locked relative pose is available.
	bool LatestSe3Residual(const Eigen::AffineCompact3d& applied, Eigen::Matrix<double, 6, 1>* residual,
	                       Eigen::Vector3d* refTrans, Eigen::Vector3d* tgtTrans) const;

	// While set, the locked-relpose accept skips its step deadband/cap (the
	// quality gates still apply). The live tick arms this during warm-restart
	// grace and before the session's first accepted candidate, where a large
	// intentional move is expected; replay mirrors the same windows.
	void SetStepGateBypass(bool on) { m_stepGateBypass = on; }

	// Locked-relpose accept gate (RelPoseLockGate.h: deadband, max-error,
	// consensus escape, drift follower). Follows the enhanced-tracking master
	// switch: off means every validated locked candidate is applied directly,
	// the classic upstream behaviour.
	void SetLockedAcceptGate(bool on) { m_useLockedAcceptGate = on; }

	// Drops the locked-accept gate's rolling state (oversize-consensus streak,
	// drift-follower ring). Called when the enhanced-tracking master switch
	// flips at runtime so stale windows don't influence the first gated
	// accepts after re-enable.
	void ResetCustomGateState()
	{
		m_lockedOversizeConsensus = {};
		m_lockedDriftFollower = {};
	}

	// True when the most recent accepted locked-relpose candidate landed via
	// the oversize consensus escape (the frame moved with no observable
	// event). Callers annotate/classify that step instead of counting it as
	// wander.
	bool LastAcceptWasConsensusStep() const { return m_lastAcceptWasConsensusStep; }

	// True when the most recent accepted locked-relpose candidate landed via
	// the in-band drift follower (the candidate-cluster median departed the
	// held calibration). Bounded (<= kStepMaxCm) and deliberate; callers
	// annotate/classify it like the consensus step.
	bool LastAcceptWasDriftStep() const { return m_lastAcceptWasDriftStep; }

	// Gravity-constrained relative-pose solve: project the solved calibration
	// rotation to its yaw-about-gravity component (both universes are +Y-up,
	// so roll/pitch in C is lever-arm noise). Replay-only A/B for now -- no
	// live path sets this. See GravityAlignment.h.
	void SetGravityConstrainedRelPose(bool on) { m_useGravityConstrainedRelPose = on; }

	// Mean of (|ref.trans|^2 + |target.trans|^2) over the current sample window
	// (m^2) -- the squared lever arm that scales the relpose solve's translation
	// uncertainty. Feeds the confidence weighting of the calibration over time.
	double MeanSquaredLeverArmM2() const;

	void SeedEstimatedTransformation(const Eigen::AffineCompact3d& transform, bool annotate = true);

	void PushSample(const Sample& sample);
	void Clear();

	double ReferenceJitter() const;
	double TargetJitter() const;

	// Motion-coverage metrics on the current sample buffer. Used during one-
	// shot calibration to give the user real-time feedback on whether their
	// figure-8 motion has produced enough variety to fit a good calibration.
	// Each returns a 0.0..1.0 score where 1.0 means "you've covered enough
	// motion that the math will produce a clean fit; you can stop now".
	//
	// TranslationDiversity is the smallest per-axis position range divided by
	// a target range (~20cm). Penalises planar / single-axis motion -- you
	// want all three axes to have meaningful spread, not just the one the
	// user is comfortable swinging.
	//
	// RotationDiversity is the maximum angular distance between any two
	// sampled target rotations, divided by a target angle (~90 deg). Even
	// one wide rotation pair is enough to constrain yaw cleanly; we don't
	// need full hemispheric coverage like translation.
	double TranslationDiversity() const;
	double RotationDiversity() const;

	// Per-axis (X, Y, Z) bounding-box ranges of the target tracker across the
	// live sample buffer, in centimetres. UI uses this to point the user at
	// the limiting axis when TranslationDiversity() is below 1.0; whichever
	// component is smallest is the bottleneck (the diversity score is
	// component-min / 20 cm). Returns zero vector when fewer than two valid
	// samples are available.
	Eigen::Vector3d TranslationAxisRangesCm() const;

	bool ComputeOneshot(const bool ignoreOutliers);
	bool ComputeIncremental(bool& lerp, double threshold, double relPoseMaxError, const bool ignoreOutliers);

	size_t SampleCount() const { return m_samples.size(); }

	void ShiftSample()
	{
		if (!m_samples.empty()) m_samples.pop_front();
	}

	// Drop every sample collected before the cutoff (glfwGetTime seconds, the
	// same clock PushSample stamps). For disturbances that move the reference
	// frame itself -- a headset SLAM re-anchor, or a long off-head gap where
	// one may have happened -- older samples describe a dead coordinate frame
	// and poison every solve until they age out of the window. Keeps the seed,
	// validity, and banked relpose: the saved-profile re-apply plus the
	// warm-restart grace window carries tracking while the buffer refills.
	// Returns the number of samples dropped.
	size_t EvictSamplesBefore(double timestamp);

	// Two-phase one-shot calibration support. The Calibration.cpp state machine
	// runs Begin → Rotation → Translation; at the Rotation→Translation
	// transition we move the rotation-phase samples into m_rotationFrozen so
	// the live buffer can refill with translation-phase samples without aging
	// the rotation samples out. ComputeOneshot then prepends the frozen samples
	// onto m_samples for the duration of the solve, so all internal helpers
	// (DetectOutliers, CalibrateRotation, ComputeAxisVariance, etc.) see the
	// unified buffer without modification. Cleared by Clear().
	void FreezeRotationPhaseSamples();
	bool HasFrozenRotationSamples() const { return !m_rotationFrozen.empty(); }
	size_t FrozenRotationSampleCount() const { return m_rotationFrozen.size(); }

	CalibrationCalc() : m_isValid(false), m_calcCycle(0), enableStaticRecalibration(true) {}

	// Debug fields
	Eigen::Vector3d m_posOffset;
	double m_axisVariance = 0.0;
	long m_calcCycle;

	double m_lastSampleTime = 0.0;

private:
	bool m_isValid;
	Eigen::AffineCompact3d m_estimatedTransformation;
	bool m_relativePosCalibrated = false;
	bool m_lastComputeUsedRelPose = false;

	// Pre-allocated scratch matrices reused across solver invocations. With
	// continuous calibration ticking at ~2 Hz against a 200-sample buffer, the
	// per-call heap churn from local Eigen::MatrixXd / Eigen::VectorXd
	// allocations was on the order of ~100 KB per tick. These members are
	// resized in place; resize() is a no-op when the requested dimensions
	// already match, so the steady-state cost is just the solve itself.
	mutable Eigen::MatrixXd m_coefficientsTrans;
	mutable Eigen::VectorXd m_constantsTrans;
	mutable Eigen::VectorXd m_weightsTrans;
	mutable Eigen::MatrixXd m_outlierCoefficients;
	mutable Eigen::VectorXd m_outlierConstraints;

	// Cached priorCalibrationError from the most recent successful
	// ValidateCalibration call inside ComputeIncremental, in metres.
	// INFINITY when no successful compute has happened yet (or after a
	// Clear). Exposed via LastPriorErrorM() for the common-mode
	// coherence check.
	double m_lastPriorRetargetingErrorM = std::numeric_limits<double>::infinity();
	double m_lastCandidateRetargetingErrorM = std::numeric_limits<double>::infinity();
	int m_shadowConsecutiveImprovingCandidates = 0;

private:
	/*
	 * This affine transform estimates the pose of the target within the reference device's local pose space.
	 * That is to say, it's given by transforming the target world pose by the inverse reference pose.
	 */
	Eigen::AffineCompact3d m_refToTargetPose = Eigen::AffineCompact3d::Identity();

	// Sample-weighting mode for CalibrateByRelPose (see RelPoseWeightMode).
	// The live tick selects Covariance whenever the enhanced-tracking switch
	// plus the profile's precision_weighted_relpose setting (or the
	// experimental confidence fusion) are active; the scalar mode survives
	// for replay A/B.
	RelPoseWeightMode m_relPoseWeightMode = RelPoseWeightMode::Uniform;
	// Lever-arm noise model (LeverArmCovariance.h); config knobs live on the
	// profile and are pushed by the tick.
	double m_sigmaThetaRad = spacecal::levercov::kDefaultSigmaThetaRad;
	double m_sigmaJitM = spacecal::levercov::kDefaultSigmaJitterM;
	// Observability gate (SetV2Math) + last-solve lambda_min diagnostic.
	bool m_useV2Math = false;
	mutable double m_lastObservabilityLambdaMin = -1.0;
	// Replay-only A/B knob; never set on the live path.
	bool m_useGravityConstrainedRelPose = false;

	// Project `candidate`'s delta against `prior` onto the observed subspace
	// of the current sample window (ObservabilityGate.h); returns the gated
	// candidate. Identity pass-through when every direction is observed.
	Eigen::AffineCompact3d ApplyObservabilityGate(const Eigen::AffineCompact3d& prior,
	                                              const Eigen::AffineCompact3d& candidate) const;

	// Locked-relpose accept gating (see RelPoseLockGate.h).
	bool m_useLockedAcceptGate = true;
	bool m_stepGateBypass = false;
	bool m_lastAcceptWasConsensusStep = false;
	bool m_lastAcceptWasDriftStep = false;
	spacecal::relpose_lock::OversizeConsensusState m_lockedOversizeConsensus;
	spacecal::relpose_lock::SmallStepDriftState m_lockedDriftFollower;

	std::deque<Sample> m_samples;

	// Frozen rotation-phase samples (see FreezeRotationPhaseSamples comment in
	// the public section). Empty during continuous calibration and during the
	// one-shot Rotation phase; populated only between the Rotation→Translation
	// transition and the final ComputeOneshot. Cleared by Clear().
	std::deque<Sample> m_rotationFrozen;

	std::vector<bool> DetectOutliers() const;
	Eigen::Vector3d CalibrateRotation(const bool ignoreOutliers) const;
	Eigen::Vector3d CalibrateTranslation(const Eigen::Matrix3d& rotation) const;

	Eigen::AffineCompact3d ComputeCalibration(const bool ignoreOutliers) const;

	double RetargetingErrorRMS(const Eigen::Vector3d& hmdToTargetPos, const Eigen::AffineCompact3d& calibration) const;
	Eigen::Vector3d ComputeRefToTargetOffset(const Eigen::AffineCompact3d& calibration) const;

	Eigen::Vector4d ComputeAxisVariance(const Eigen::AffineCompact3d& calibration) const;

	void ComputeInstantOffset();

	Eigen::AffineCompact3d EstimateRefToTargetPose(const Eigen::AffineCompact3d& calibration) const;
	bool CalibrateByRelPose(Eigen::AffineCompact3d& out) const;

public:
	// Hoisted to public so the in-app replay panel + the standalone replay
	// CLI can score arbitrary candidate transforms against the current sample
	// buffer without needing to call the private ComputeIncremental flow.
	[[nodiscard]] bool ValidateCalibration(const Eigen::AffineCompact3d& calibration, double* errorOut = nullptr,
	                                       Eigen::Vector3d* posOffsetV = nullptr);
	CalibrationQualityReport EvaluateCalibrationQuality(const Eigen::AffineCompact3d& calibration,
	                                                    bool includeHoldout = true, bool ignoreOutliers = false) const;
	void LogCalibrationQualitySnapshot(const char* label, const Eigen::AffineCompact3d& calibration,
	                                   bool includeHoldout, bool ignoreOutliers) const;
};
