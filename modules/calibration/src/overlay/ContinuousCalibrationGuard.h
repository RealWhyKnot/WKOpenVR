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

inline CandidateGuardResult EvaluateCandidate(
	bool inContinuous,
	bool hasBaseline,
	bool hasAcceptedThisSession,
	const Eigen::Vector3d& baselineTranslationCm,
	const Eigen::Vector3d& candidateTranslationCm,
	const Eigen::Matrix3d& candidateRotation)
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
		: kMaxFirstAcceptedJumpM;

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
		candidateRotation);
	result.accepted = jumpGuard.accepted;
	result.reason = jumpGuard.reason;
	result.jumpM = jumpGuard.jumpM;
	return result;
}

} // namespace spacecal::continuous
