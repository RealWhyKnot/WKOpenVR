#pragma once

#include "Calibration.h"

#include <algorithm>
#include <cmath>

namespace spacecal::calibration_progress {

inline constexpr double kOneShotRotationReadyDiversity = 0.70;
inline constexpr double kOneShotTranslationReadyDiversity = 0.55;

inline double Clamp01(double value)
{
	if (!std::isfinite(value)) return 0.0;
	return std::clamp(value, 0.0, 1.0);
}

inline double SampleFillScore(int progress, int target)
{
	if (target <= 0) return 0.0;
	return Clamp01(static_cast<double>(progress) / static_cast<double>(target));
}

inline double OneShotReadyScore(CalibrationState state, int sampleProgress, int sampleTarget, double rotationDiversity,
                                double translationDiversity)
{
	const double sampleScore = SampleFillScore(sampleProgress, sampleTarget);
	const double rotationScore = std::min(sampleScore, Clamp01(rotationDiversity / kOneShotRotationReadyDiversity));
	const double translationScore =
	    std::min(sampleScore, Clamp01(translationDiversity / kOneShotTranslationReadyDiversity));

	switch (state) {
		case CalibrationState::Begin:
			return 0.0;
		case CalibrationState::Rotation:
			return 0.5 * rotationScore;
		case CalibrationState::Translation:
			return 0.5 + 0.5 * translationScore;
		default:
			return sampleScore;
	}
}

} // namespace spacecal::calibration_progress
