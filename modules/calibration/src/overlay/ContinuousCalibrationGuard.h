#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>

namespace spacecal::continuous {

constexpr double kMaxFirstAcceptedJumpM = 1.0;
constexpr double kMaxSteadyAcceptedJumpM = 0.25;
constexpr double kMinPublishRotationRad = 1e-3;
constexpr double kMinPublishTranslationCm = 0.1;
constexpr int kStableLargeFullSolveSamples = 3;

struct CandidateGuardResult {
	bool accepted = true;
	const char* reason = "accepted";
	double jumpM = 0.0;
};

struct PublishCandidateGuardResult {
	bool accepted = true;
	const char* reason = "accepted";
	double jumpM = 0.0;
	double rotAngleRad = 0.0;
	double translationMagnitudeCm = 0.0;
	bool finiteTranslation = true;
};

struct RelPoseTrustResult {
	bool accepted = true;
	const char* reason = "accepted";
};

struct LargeFullSolveStabilityResult {
	bool stable = false;
	int nextSampleCount = 0;
	bool storeAsPendingAnchor = false;
	bool separatedFromBaseline = false;
	double uncertaintyCm = 0.0;
	double expectedDeltaCm = 0.0;
	double combinedDeltaCm = 0.0;
	double translationDeltaCm = 0.0;
	double rotationDeltaRad = 0.0;
	double rotationEquivalentCm = 0.0;
	double jumpCm = 0.0;
};

inline RelPoseTrustResult EvaluateRelPoseTrust(
	bool lockRelativePosition,
	bool relativePosCalibrated)
{
	if (!lockRelativePosition) {
		return { false, "relpose_unlocked" };
	}

	if (!relativePosCalibrated) {
		return { false, "relpose_uncalibrated" };
	}

	return {};
}

inline CandidateGuardResult EvaluateCandidate(
	bool inContinuous,
	bool hasBaseline,
	bool hasAcceptedThisSession,
	const Eigen::Vector3d& baselineTranslationCm,
	const Eigen::Vector3d& candidateTranslationCm,
	const Eigen::Matrix3d& candidateRotation,
	double firstAcceptedLimitM = kMaxFirstAcceptedJumpM)
{
	if (!candidateTranslationCm.allFinite() || !candidateRotation.allFinite()) {
		return { false, "non_finite", 0.0 };
	}

	if (!inContinuous || !hasBaseline) {
		return {};
	}

	const double jumpM =
		(candidateTranslationCm - baselineTranslationCm).norm() * 0.01;
	const double limitM = hasAcceptedThisSession
		? kMaxSteadyAcceptedJumpM
		: firstAcceptedLimitM;

	if (std::isfinite(jumpM) && jumpM > limitM) {
		return { false, "jump_exceeds_limit", jumpM };
	}

	return { true, "accepted", jumpM };
}

inline double RotationAngleRad(const Eigen::Matrix3d& rotation)
{
	if (!rotation.allFinite()) {
		return 0.0;
	}
	return std::acos(std::clamp((rotation.trace() - 1.0) * 0.5, -1.0, 1.0));
}

inline double PositiveFiniteOrZero(double value)
{
	return (std::isfinite(value) && value > 0.0) ? value : 0.0;
}

inline double UnitConditionOrOne(double condition)
{
	if (!std::isfinite(condition) || condition <= 0.0) {
		return 1.0;
	}
	return std::clamp(condition, 1e-6, 1.0);
}

inline double EstimateSolveUncertaintyCm(double candidateErrorM,
                                         double referenceJitterM,
                                         double targetJitterM,
                                         double rotationConditionRatio,
                                         double translationConditionRatio)
{
	const double fitM = PositiveFiniteOrZero(candidateErrorM);
	const double refM = PositiveFiniteOrZero(referenceJitterM);
	const double targetM = PositiveFiniteOrZero(targetJitterM);
	const double observedNoiseM = std::sqrt(fitM * fitM + refM * refM + targetM * targetM);
	const double condition = std::min(
		UnitConditionOrOne(rotationConditionRatio),
		UnitConditionOrOne(translationConditionRatio));
	return observedNoiseM * 100.0 / condition;
}

inline LargeFullSolveStabilityResult EvaluateLargeFullSolveStability(
	bool hasPending,
	int pendingSampleCount,
	const Eigen::Vector3d& pendingTranslationCm,
	const Eigen::Matrix3d& pendingRotation,
	double pendingUncertaintyCm,
	const Eigen::Vector3d& baselineTranslationCm,
	const Eigen::Vector3d& candidateTranslationCm,
	const Eigen::Matrix3d& candidateRotation,
	double candidateErrorM,
	double referenceJitterM,
	double targetJitterM,
	double rotationConditionRatio,
	double translationConditionRatio)
{
	LargeFullSolveStabilityResult result{};
	if (!candidateTranslationCm.allFinite() || !candidateRotation.allFinite()) {
		return result;
	}

	result.jumpCm = baselineTranslationCm.allFinite()
		? (candidateTranslationCm - baselineTranslationCm).norm()
		: candidateTranslationCm.norm();
	result.uncertaintyCm = EstimateSolveUncertaintyCm(
		candidateErrorM,
		referenceJitterM,
		targetJitterM,
		rotationConditionRatio,
		translationConditionRatio);
	result.separatedFromBaseline = result.jumpCm > result.uncertaintyCm;

	if (!hasPending || pendingSampleCount <= 0
		|| !pendingTranslationCm.allFinite() || !pendingRotation.allFinite())
	{
		result.nextSampleCount = 1;
		result.storeAsPendingAnchor = true;
		return result;
	}

	result.translationDeltaCm =
		(candidateTranslationCm - pendingTranslationCm).norm();
	result.rotationDeltaRad =
		RotationAngleRad(pendingRotation.transpose() * candidateRotation);
	result.rotationEquivalentCm = result.rotationDeltaRad * result.jumpCm;
	result.combinedDeltaCm =
		std::hypot(result.translationDeltaCm, result.rotationEquivalentCm);

	const double pendingUncertainty =
		PositiveFiniteOrZero(pendingUncertaintyCm);
	result.expectedDeltaCm =
		std::sqrt(pendingUncertainty * pendingUncertainty
			+ result.uncertaintyCm * result.uncertaintyCm);
	const bool closeEnough =
		result.combinedDeltaCm <= result.expectedDeltaCm;
	result.nextSampleCount = closeEnough ? (pendingSampleCount + 1) : 1;
	result.storeAsPendingAnchor = !closeEnough;
	result.stable =
		closeEnough
		&& result.separatedFromBaseline
		&& result.nextSampleCount >= kStableLargeFullSolveSamples;
	return result;
}

inline PublishCandidateGuardResult EvaluatePublishCandidate(
	bool inContinuous,
	bool hasBaseline,
	bool hasAcceptedThisSession,
	const Eigen::Vector3d& baselineTranslationCm,
	const Eigen::Vector3d& candidateTranslationCm,
	const Eigen::Matrix3d& candidateRotation)
{
	PublishCandidateGuardResult result{};
	result.finiteTranslation = candidateTranslationCm.allFinite();
	const bool finiteRotation = candidateRotation.allFinite();
	if (result.finiteTranslation) {
		result.translationMagnitudeCm = candidateTranslationCm.norm();
	}
	result.rotAngleRad = RotationAngleRad(candidateRotation);

	if (!result.finiteTranslation || !finiteRotation) {
		result.accepted = false;
		result.reason = "non_finite";
		return result;
	}

	const bool hasMeaningfulPose =
		result.rotAngleRad > kMinPublishRotationRad ||
		result.translationMagnitudeCm > kMinPublishTranslationCm;
	if (!hasMeaningfulPose) {
		result.accepted = false;
		result.reason = "identity_candidate";
		return result;
	}

	const auto jumpGuard = EvaluateCandidate(
		inContinuous,
		hasBaseline,
		hasAcceptedThisSession,
		baselineTranslationCm,
		candidateTranslationCm,
		candidateRotation,
		kMaxFirstAcceptedJumpM);
	result.accepted = jumpGuard.accepted;
	result.reason = jumpGuard.reason;
	result.jumpM = jumpGuard.jumpM;
	return result;
}

} // namespace spacecal::continuous
