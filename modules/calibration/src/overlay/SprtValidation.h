#pragma once

// Sequential (Wald SPRT) validation for the warm-restart grace window.
//
// H0: the re-applied saved profile still matches reality -- the whitened
// SE(3) residual of each fresh sample is chi^2(dof)-distributed.
// H1: the world frame moved -- residuals behave as if their variance were
// inflated by kH1VarianceRatio.
//
// Each fresh sample contributes the log-likelihood ratio
//   l_i = 0.5 * d2_i * (1 - 1/v) - (dof/2) * ln(v),   v = kH1VarianceRatio
// where d2_i is the residual's Mahalanobis-squared norm under the lever-arm
// noise model (LeverArmCovariance.h). Because d2 is normalized by the
// distance-dependent covariance, the same thresholds hold at the origin and
// at a 7 m lever arm -- the fixed-millimetre thresholds this replaces did
// not. Decide H1 when the accumulated ratio crosses log((1-beta)/alpha), H0
// below log(beta/(1-alpha)); otherwise keep sampling. A minimum-sample floor
// keeps a single lucky/unlucky sample from settling or failing the episode
// (the failure shape of the old one-bias-sample verdict).

#include "WarmRestart.h"

#include <algorithm>
#include <cmath>

namespace spacecal::sprt {

struct SprtParams
{
	double alpha = 0.01;          // P(decide moved | profile valid)
	double beta = 0.05;           // P(decide valid | frame moved)
	double h1VarianceRatio = 9.0; // H1 residual variance inflation (3x sigma)
	int dof = 6;
	int minDecisionSamples = 8; // mirrors kValidationMinBiasSamples
};

struct SprtState
{
	double llr = 0.0;
	int n = 0;
};

enum class SprtDecision
{
	Continue,
	AcceptH0,
	AcceptH1
};

inline double UpperThreshold(const SprtParams& p)
{
	return std::log((1.0 - p.beta) / p.alpha);
}
inline double LowerThreshold(const SprtParams& p)
{
	return std::log(p.beta / (1.0 - p.alpha));
}

inline SprtDecision Decide(const SprtState& s, const SprtParams& p = {})
{
	if (s.n < p.minDecisionSamples) return SprtDecision::Continue;
	if (s.llr >= UpperThreshold(p)) return SprtDecision::AcceptH1;
	if (s.llr <= LowerThreshold(p)) return SprtDecision::AcceptH0;
	return SprtDecision::Continue;
}

inline SprtDecision Step(SprtState& s, double mahalanobisSq, const SprtParams& p = {})
{
	const double v = p.h1VarianceRatio;
	const double l = 0.5 * mahalanobisSq * (1.0 - 1.0 / v) - 0.5 * static_cast<double>(p.dof) * std::log(v);
	s.llr += l;
	s.n += 1;
	return Decide(s, p);
}

inline warm_restart::ValidationOutcome ToValidationOutcome(SprtDecision d)
{
	switch (d) {
		case SprtDecision::AcceptH0:
			return warm_restart::ValidationOutcome::Settled;
		case SprtDecision::AcceptH1:
			return warm_restart::ValidationOutcome::Failed;
		case SprtDecision::Continue:
		default:
			return warm_restart::ValidationOutcome::Inconclusive;
	}
}

} // namespace spacecal::sprt
