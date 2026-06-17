#pragma once

#include <cmath>
#include <algorithm>
#include <limits>

namespace spacecal::calibration_speed {

enum class AutoSpeedBucket
{
	Fast,
	Slow,
	VerySlow,
};

enum class AutoSpeedPhase
{
	Settled,
	Converging,
};

struct AutoSpeedState
{
	AutoSpeedPhase phase = AutoSpeedPhase::Settled;
	int settleTicks = 0;
};

struct AutoSpeedDecision
{
	AutoSpeedState state;
	AutoSpeedBucket bucket = AutoSpeedBucket::Fast;
	double correctionFitMm = std::numeric_limits<double>::quiet_NaN();
	double freshFitMm = std::numeric_limits<double>::quiet_NaN();
	double reducibleMm = 0.0;
};

constexpr double kReducibleEnterMm = 5.0;
constexpr double kReducibleExitMm = 2.0;
constexpr int kSettleDwellTicks = 30;

inline bool IsUsableFitRmsMm(double valueMm)
{
	return std::isfinite(valueMm) && valueMm > 0.0;
}

inline double SelectObservedFitRmsMm(double candidateFitRmsMm, double currentFitRmsMm)
{
	if (IsUsableFitRmsMm(candidateFitRmsMm)) return candidateFitRmsMm;
	if (IsUsableFitRmsMm(currentFitRmsMm)) return currentFitRmsMm;
	return std::numeric_limits<double>::quiet_NaN();
}

inline AutoSpeedBucket BucketForObservedFitRmsMm(double fitRmsMm)
{
	if (!IsUsableFitRmsMm(fitRmsMm) || fitRmsMm < 5.0) {
		return AutoSpeedBucket::Fast;
	}
	if (fitRmsMm < 10.0) {
		return AutoSpeedBucket::Slow;
	}
	return AutoSpeedBucket::VerySlow;
}

inline const char* AutoSpeedBucketName(AutoSpeedBucket bucket)
{
	switch (bucket) {
		case AutoSpeedBucket::Fast:
			return "Fast";
		case AutoSpeedBucket::Slow:
			return "Slow";
		case AutoSpeedBucket::VerySlow:
			return "Very Slow";
	}
	return "?";
}

inline const char* AutoSpeedPhaseName(AutoSpeedPhase phase)
{
	switch (phase) {
		case AutoSpeedPhase::Settled:
			return "settled";
		case AutoSpeedPhase::Converging:
			return "converging";
	}
	return "?";
}

inline AutoSpeedDecision ResolveAutoSpeed(AutoSpeedState prior, double correctionFitMm, double freshFitMm)
{
	AutoSpeedDecision out;
	out.state = prior;
	out.correctionFitMm = correctionFitMm;
	out.freshFitMm = freshFitMm;

	const bool haveCorrection = IsUsableFitRmsMm(correctionFitMm);
	const bool haveFresh = IsUsableFitRmsMm(freshFitMm);
	if (!haveCorrection && !haveFresh) {
		out.state.phase = AutoSpeedPhase::Settled;
		out.state.settleTicks = 0;
		out.bucket = AutoSpeedBucket::Fast;
		return out;
	}

	const double noiseFloorMm = haveFresh ? freshFitMm : 0.0;
	const double correctionMm = haveCorrection ? correctionFitMm : (haveFresh ? freshFitMm : 0.0);
	out.reducibleMm = std::max(0.0, correctionMm - noiseFloorMm);

	if (out.reducibleMm >= kReducibleEnterMm) {
		out.state.phase = AutoSpeedPhase::Converging;
		out.state.settleTicks = 0;
	}
	else if (out.state.phase == AutoSpeedPhase::Converging) {
		if (out.reducibleMm <= kReducibleExitMm) {
			++out.state.settleTicks;
			if (out.state.settleTicks >= kSettleDwellTicks) {
				out.state.phase = AutoSpeedPhase::Settled;
				out.state.settleTicks = 0;
			}
		}
		else {
			out.state.settleTicks = 0;
		}
	}
	else {
		out.state.settleTicks = 0;
	}

	out.bucket = out.state.phase == AutoSpeedPhase::Converging
	                 ? AutoSpeedBucket::Fast
	                 : BucketForObservedFitRmsMm(haveFresh ? freshFitMm : correctionMm);
	return out;
}

} // namespace spacecal::calibration_speed
