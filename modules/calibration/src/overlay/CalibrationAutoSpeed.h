#pragma once

#include <cmath>

namespace spacecal::calibration_speed {

enum class AutoSpeedBucket
{
	Fast,
	Slow,
	VerySlow,
};

inline bool IsUsableFitRmsMm(double valueMm)
{
	return std::isfinite(valueMm) && valueMm >= 0.0;
}

inline double SelectObservedFitRmsMm(double candidateFitRmsMm, double currentFitRmsMm)
{
	if (IsUsableFitRmsMm(candidateFitRmsMm)) return candidateFitRmsMm;
	if (IsUsableFitRmsMm(currentFitRmsMm)) return currentFitRmsMm;
	return 0.0;
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

} // namespace spacecal::calibration_speed
