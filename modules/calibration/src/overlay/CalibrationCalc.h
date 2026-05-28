#pragma once

#include <Eigen/Dense>
#include <openvr.h>
#include <string>
#include <vector>
#include <deque>
#include <iostream>
#include <limits>

struct Pose
{
	Eigen::Matrix3d rot;
	Eigen::Vector3d trans;

	Pose() { }
	Pose(const Eigen::AffineCompact3d& transform) {
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
	Pose(vr::HmdQuaternion_t rot, const double *trans) {
		this->rot = Eigen::Matrix3d(Eigen::Quaterniond(rot.w, rot.x, rot.y, rot.z));
		this->trans = Eigen::Vector3d(trans[0], trans[1], trans[2]);
	}
	Pose(double x, double y, double z) : trans(Eigen::Vector3d(x, y, z)) { }

	Eigen::Matrix4d ToAffine() const {
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
	double refPoseAgeMs = 0.0;
	double targetPoseAgeMs = 0.0;
	double refPoseGapMs = 0.0;
	double targetPoseGapMs = 0.0;
	bool trackingPoseStale = false;
	bool trackingPoseJump = false;
	Sample() : valid(false), timestamp(0) { }
	Sample(Pose ref, Pose target, double timestamp) : valid(true), ref(ref), target(target), timestamp(timestamp){ }
};

struct CalibrationResidualStats {
	int count = 0;
	double medianM = std::numeric_limits<double>::infinity();
	double madSigmaM = std::numeric_limits<double>::infinity();
	double p90M = std::numeric_limits<double>::infinity();
	double p95M = std::numeric_limits<double>::infinity();
	double maxM = std::numeric_limits<double>::infinity();
	double rmsM = std::numeric_limits<double>::infinity();
	double outlierFraction = 0.0;
};

struct CalibrationQualityReport {
	size_t sampleCount = 0;
	int validSampleCount = 0;
	int pairedSampleCount = 0;
	int trackingStaleSampleCount = 0;
	int trackingJumpSampleCount = 0;
	int validRotationPairCount = 0;
	int translationRank = 0;
	Eigen::Vector3d refRangeM = Eigen::Vector3d::Zero();
	Eigen::Vector3d targetRangeM = Eigen::Vector3d::Zero();
	double refSpanM = 0.0;
	double targetSpanM = 0.0;
	double rotationSpanDeg = 0.0;
	double maxPoseAgeMs = 0.0;
	double maxPoseGapMs = 0.0;
	double rotationConditionRatio = 0.0;
	double translationConditionRatio = 0.0;
	double dynamicLimitM = std::numeric_limits<double>::infinity();
	bool legacyRmsPass = false;
	bool geometryPass = false;
	bool robustResidualPass = false;
	bool holdoutPass = false;
	bool trackingHealthPass = true;
	bool shadowDynamicPass = false;
	Eigen::Vector3d posOffsetM = Eigen::Vector3d::Zero();
	CalibrationResidualStats residuals;
	CalibrationResidualStats holdoutResiduals;
};

struct CalibrationQualityVerdict {
	bool wouldAccept = false;
	const char* reason = "unknown";
};

CalibrationQualityVerdict EvaluateCalibrationQualityVerdict(
	const CalibrationQualityReport& report);

class CalibrationCalc {
public:
	static const double AxisVarianceThreshold;

	bool enableStaticRecalibration;
	bool lockRelativePosition = false;
	
	const Eigen::AffineCompact3d Transformation() const 
	{
		return m_estimatedTransformation;
	}

	const Eigen::Vector3d EulerRotation() const {
		auto rot = m_estimatedTransformation.rotation();
		return rot.eulerAngles(2, 1, 0) * 180.0 / EIGEN_PI;
	}

	bool isValid() const {
		return m_isValid;
	}

	bool LastComputeUsedRelPose() const {
		return m_lastComputeUsedRelPose;
	}

	// Most recent RMS retargeting error of the applied calibration (the
	// `priorCalibrationError` computed inside ComputeIncremental and
	// pushed to Metrics::error_currentCal for the primary pair). Exposed
	// per-instance so the common-mode coherence check at the geometry-
	// shift fire site can read each AdditionalCalibration's latest error
	// without each extra needing its own global TimeSeries. Returns
	// INFINITY when no incremental compute has produced a validated
	// result yet (the value is initialized to INFINITY and only
	// overwritten by a successful ValidateCalibration path).
	double LastPriorErrorM() const {
		return m_lastPriorRetargetingErrorM;
	}

	double LastCandidateErrorM() const {
		return m_lastCandidateRetargetingErrorM;
	}

	const Eigen::AffineCompact3d RelativeTransformation() const
	{
		return m_refToTargetPose;
	}

	bool isRelativeTransformationCalibrated() const
	{
		return m_relativePosCalibrated;
	}

	void setRelativeTransformation(const Eigen::AffineCompact3d transform, bool calibrated)
	{
		m_refToTargetPose = transform;
		m_relativePosCalibrated = calibrated;
	}

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
	bool ComputeIncremental(bool &lerp, double threshold, double relPoseMaxError, const bool ignoreOutliers);

	size_t SampleCount() const {
		return m_samples.size();
	}

	void ShiftSample() {
		if (!m_samples.empty()) m_samples.pop_front();
	}

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

private:

	/*
	 * This affine transform estimates the pose of the target within the reference device's local pose space.
	 * That is to say, it's given by transforming the target world pose by the inverse reference pose.
	 */
	Eigen::AffineCompact3d m_refToTargetPose = Eigen::AffineCompact3d::Identity();

	std::deque<Sample> m_samples;

	// Frozen rotation-phase samples (see FreezeRotationPhaseSamples comment in
	// the public section). Empty during continuous calibration and during the
	// one-shot Rotation phase; populated only between the Rotation→Translation
	// transition and the final ComputeOneshot. Cleared by Clear().
	std::deque<Sample> m_rotationFrozen;

	std::vector<bool> DetectOutliers() const;
	Eigen::Vector3d CalibrateRotation(const bool ignoreOutliers) const;
	Eigen::Vector3d CalibrateTranslation(const Eigen::Matrix3d &rotation) const;

	Eigen::AffineCompact3d ComputeCalibration(const bool ignoreOutliers) const;

	double RetargetingErrorRMS(const Eigen::Vector3d& hmdToTargetPos, const Eigen::AffineCompact3d& calibration) const;
	Eigen::Vector3d ComputeRefToTargetOffset(const Eigen::AffineCompact3d& calibration) const;

	Eigen::Vector4d ComputeAxisVariance(const Eigen::AffineCompact3d& calibration) const;

	void ComputeInstantOffset();

	Eigen::AffineCompact3d EstimateRefToTargetPose(const Eigen::AffineCompact3d& calibration) const;
	bool CalibrateByRelPose(Eigen::AffineCompact3d &out) const;

public:
	// Hoisted to public so the in-app replay panel + the standalone replay
	// CLI can score arbitrary candidate transforms against the current sample
	// buffer without needing to call the private ComputeIncremental flow.
	[[nodiscard]] bool ValidateCalibration(const Eigen::AffineCompact3d& calibration, double *errorOut = nullptr, Eigen::Vector3d* posOffsetV = nullptr);
	CalibrationQualityReport EvaluateCalibrationQuality(
		const Eigen::AffineCompact3d& calibration,
		bool includeHoldout = true,
		bool ignoreOutliers = false) const;
	void LogCalibrationQualitySnapshot(
		const char* label,
		const Eigen::AffineCompact3d& calibration,
		bool includeHoldout,
		bool ignoreOutliers) const;
};
