#pragma once

#include <algorithm>
#include <cmath>

// Two-sided Page-Hinkley change-point test.
//
// Reference: Page 1954 (Biometrika 41) for the original CUSUM; Hinkley 1971
// (Biometrika 58) for the slack-augmented variant this is named after.
//
// Detects a step change in the mean of a 1D signal while tolerating bounded
// noise + transient bumps. Two accumulators track positive and negative
// deviations from a running mean estimate; a change is declared when either
// crosses a threshold lambda.
//
//   m_t   = (1 - alpha) * m_{t-1} + alpha * x_t          // EWMA reference
//   PH+_t = max(0, PH+_{t-1} + (x_t - m_t - delta))      // upward drift
//   PH-_t = max(0, PH-_{t-1} - (x_t - m_t + delta))      // downward drift
//   trigger if PH+_t > lambda or PH-_t > lambda
//
// alpha   forgetting factor; for a target half-life tau in seconds at
//         observation rate dt, set alpha = 1 - exp(-dt / tau).
// delta   slack -- step magnitudes below this size are absorbed without
//         contributing to the accumulator. Typical: 1/5 of the smallest
//         shift you want to detect.
// lambda  detection threshold. Tune per-category against an Average Run
//         Length (ARL) target via Wald-SPRT thresholds (research doc Q8).
//
// Used by InputHealth for stick rest-drift detection (two-sided PH on each
// axis of the rest centroid) and trigger min-stuck detection (one-sided PH
// on the rest-min EWMA).
//
// O(1) per update. Caller owns thread safety.

namespace inputhealth {

struct PageHinkleyState
{
	double mean = 0.0;               // running EWMA reference
	double ph_pos = 0.0;             // positive drift accumulator
	double ph_neg = 0.0;             // negative drift accumulator
	bool initialized = false;        // false until first sample seeds the mean
	bool triggered = false;          // latched until PageHinkleyReset is called
	bool triggered_positive = false; // direction of the latched trigger
};

struct PageHinkleyParams
{
	double alpha = 0.05;             // EWMA forgetting; ~30s half-life at 250 Hz
	double delta = 0.002;            // slack
	double lambda = 0.05;            // detection threshold
	bool one_sided_positive = false; // when true, ignore downward drift
};

// Streaming update. Returns true on the tick that first crosses the threshold;
// the latched `triggered` flag stays set until PageHinkleyReset() so callers
// don't have to debounce around the tick boundary.
inline bool PageHinkleyUpdate(PageHinkleyState& s, const PageHinkleyParams& p, double x)
{
	if (!s.initialized) {
		s.mean = x;
		s.initialized = true;
	}
	else {
		s.mean = (1.0 - p.alpha) * s.mean + p.alpha * x;
	}

	const double dev = x - s.mean;
	s.ph_pos = std::max(0.0, s.ph_pos + (dev - p.delta));
	if (!p.one_sided_positive) {
		s.ph_neg = std::max(0.0, s.ph_neg - (dev + p.delta));
	}

	if (!s.triggered) {
		if (s.ph_pos > p.lambda) {
			s.triggered = true;
			s.triggered_positive = true;
			return true;
		}
		if (!p.one_sided_positive && s.ph_neg > p.lambda) {
			s.triggered = true;
			s.triggered_positive = false;
			return true;
		}
	}
	return false;
}

// Reset the state to "no observations yet". Use after a confirmed change so
// the detector starts looking for the next one rather than re-firing on
// every subsequent tick. Calibration-wizard "start fresh" also calls this.
inline void PageHinkleyReset(PageHinkleyState& s)
{
	s.mean = 0.0;
	s.ph_pos = 0.0;
	s.ph_neg = 0.0;
	s.initialized = false;
	s.triggered = false;
	s.triggered_positive = false;
}

// Convert a wall-clock half-life to the per-sample alpha for a given sample
// rate. dt_seconds is the sampling interval; tau_seconds is the desired
// half-life of the EWMA. The formula assumes uniform sampling; for
// bursty rates the caller should compute alpha per-sample using the actual
// elapsed dt between calls.
inline double EwmaAlphaForHalfLife(double dt_seconds, double tau_seconds)
{
	if (tau_seconds <= 0.0 || dt_seconds <= 0.0) return 1.0;
	return 1.0 - std::exp(-dt_seconds / tau_seconds);
}

} // namespace inputhealth
