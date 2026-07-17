#pragma once

#include "Calibration.h"

// Head-mount continuous-source guard + shadow-offset estimator, extracted
// from Calibration.cpp. The estimator is diagnostics-only: it solves a
// shadow head-from-tracker offset each motion window and logs whether the
// apply gate would accept it, without touching the live offset.
inline const char* HeadMountSampleSourceName(HeadMountSampleSource source)
{
	switch (source) {
		case HeadMountSampleSource::PhysicalTracker:
			return "physical_tracker";
		case HeadMountSampleSource::HeadProxy:
			return "head_proxy";
		case HeadMountSampleSource::Unknown:
		default:
			return "unknown";
	}
}
HeadMountSampleSource CurrentHeadMountSampleSource(const CalibrationContext& ctx);
void TickHeadMountSourceTransitionGuard(CalibrationContext& ctx, double time);
void TickHeadMountShadowOffsetEstimator(CalibrationContext& ctx, double time);

// Seed an identity relative-pose lock for head-proxy setups; called from the
// tracking-style UI actions and the offset modal save path.
bool CCal_SeedHeadMountProxyRelativeLock(const char* reason = "unknown");
