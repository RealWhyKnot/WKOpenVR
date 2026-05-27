#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>

namespace spacecal::continuous {

constexpr double kMaxFirstAcceptedJumpM = 1.0;
constexpr double kMaxFirstFullSolveAcceptedJumpM = 5.0;
constexpr double kMaxSteadyAcceptedJumpM = 0.25;
constexpr double kMinPublishRotationRad = 1e-3;
constexpr double kMinPublishTranslationCm = 0.1;
constexpr int kStableLargeFullSolveSamples = 3;
constexpr double kStableLargeFullSolveTranslationDeltaCm = 15.0;
constexpr double kStableLargeFullSolveRotationDeltaRad = 0.25;

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
	double translationDeltaCm = 0.0;
	double rotationDeltaRad = 0.0;
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

inline LargeFullSolveStabilityResult EvaluateLargeFullSolveStability(
	bool hasPending,
	int pendingSampleCount,
	const Eigen::Vector3d& pendingTranslationCm,
	const Eigen::Matrix3d& pendingRotation,
	const Eigen::Vector3d& candidateTranslationCm,
	const Eigen::Matrix3d& candidateRotation)
{
	LargeFullSolveStabilityResult result{};
	if (!candidateTranslationCm.allFinite() || !candidateRotation.allFinite()) {
		return result;
	}

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

	const bool closeEnough =
		result.translationDeltaCm <= kStableLargeFullSolveTranslationDeltaCm
		&& result.rotationDeltaRad <= kStableLargeFullSolveRotationDeltaRad;
	result.nextSampleCount = closeEnough ? (pendingSampleCount + 1) : 1;
	result.storeAsPendingAnchor = !closeEnough;
	result.stable =
		closeEnough && result.nextSampleCount >= kStableLargeFullSolveSamples;
	return result;
}

inline PublishCandidateGuardResult EvaluatePublishCandidate(
	bool inContinuous,
	bool hasBaseline,
	bool hasAcceptedThisSession,
	bool candidateFromRelPose,
	bool allowLargeFirstFullSolveCorrection,
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
		(!candidateFromRelPose && allowLargeFirstFullSolveCorrection)
			? kMaxFirstFullSolveAcceptedJumpM
			: kMaxFirstAcceptedJumpM);
	result.accepted = jumpGuard.accepted;
	result.reason = jumpGuard.reason;
	result.jumpM = jumpGuard.jumpM;
	return result;
}

} // namespace spacecal::continuous
