#pragma once

#include "Calibration.h"

namespace spacecal::oneshot {

void MaybeLogReadiness(
	const CalibrationContext& ctx,
	int sampleProgress,
	int sampleTarget,
	double rotationDiversity,
	double translationDiversity,
	double now);

} // namespace spacecal::oneshot
