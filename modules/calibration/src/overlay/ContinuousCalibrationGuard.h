#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>

namespace spacecal::continuous {

constexpr double kMaxFirstAcceptedJumpM = 1.0;
constexpr double kMaxSteadyAcceptedJumpM = 0.25;

struct CandidateGuardResult {
	bool accepted = true;
	const char* reason = "accepted";
	double jumpM = 0.0;
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

} // namespace spacecal::continuous
