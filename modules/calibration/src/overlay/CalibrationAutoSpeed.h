#pragma once

#include <cmath>
#include <limits>

namespace spacecal::calibration_speed {

inline bool IsUsableFitRmsMm(double valueMm)
{
	return std::isfinite(valueMm) && valueMm > 0.0;
}

// Prefer the candidate fit when it carries a real reading; fall back to the
// current fit; NaN when neither is usable.
inline double SelectObservedFitRmsMm(double candidateFitRmsMm, double currentFitRmsMm)
{
	if (IsUsableFitRmsMm(candidateFitRmsMm)) return candidateFitRmsMm;
	if (IsUsableFitRmsMm(currentFitRmsMm)) return currentFitRmsMm;
	return std::numeric_limits<double>::quiet_NaN();
}

} // namespace spacecal::calibration_speed
