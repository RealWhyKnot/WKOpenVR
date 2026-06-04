#pragma once

#include <algorithm>
#include <vector>

// Common-mode coherence score for the geometry-shift detector. When the
// primary HMD<->target pair's residual spikes, the question is whether the
// spike is pair-local (real attachment shift on the primary tracker) or
// shared across every active calibration pair (runtime relocalization,
// base-station perturbation, or tracking-origin reset affecting all pairs
// together). The coherence score answers that question directly from the
// residual streams: a spike that affects every pair coherently is common
// mode regardless of which subsystem caused it.
//
// Lives next to the existing geometry-shift detector at
// modules/calibration/src/overlay/Calibration.cpp's fire site. The fire
// decision goes through the legacy / CUSUM path first as today; if it
// votes to fire, the coherence check runs and can suppress the verdict
// when the spike was global.

namespace spacecal::coherence {

// Source identifier for head-mount tracker corroboration. When the geometry-
// shift fire site feeds the head-mount tracker's actual displacement as a
// corroborating input, it tags the source with this label so the suppression
// log line can distinguish this path from the multi-pair extras path.
//
// The value is a diagnostic label only; the coherence score formula and
// kSuppressThreshold / kMinExtrasForCoherence thresholds are unchanged.
enum class CorroborationSource : uint8_t
{
	kExtraPairs = 0,             // default: extras from additionalCalibrations
	kHeadMountCorroboration = 1, // head-mount tracker displacement vs velocity estimate
};

// Threshold above which a multi-pair spike is treated as common-mode.
// 0.70 = "the extras spiked at least 70% as hard as the primary, in the
// median." Below this, the primary's spike was substantially worse than
// the extras' -- the simplest explanation is pair-local geometry. Above,
// the spike was correlated across pairs and the primary's apparent shift
// is likely a shared-frame event.
constexpr double kSuppressThreshold = 0.70;

// Minimum extras count for the coherence check to apply. With zero
// extras there is no second opinion to confirm a common-mode
// interpretation; fall back to the existing fire decision unchanged.
// One extra is enough -- a single coherent spike on a second pair is
// already strong evidence the cause is shared.
constexpr int kMinExtrasForCoherence = 1;

// Common-mode coherence score in [0, 1] given the primary pair's spike
// ratio (current_error / rolling_median, dimensionless) and each extra
// pair's spike ratio.
//
// Score = clamp(median(extraRatios) / primaryRatio, 0, 1).
//
// 1.0 = the median extra spiked as hard as the primary -> common-mode.
// 0.0 = extras stayed near their baselines while the primary spiked
//       -> pair-local. The median is used so a single noisy extra does
//       not dominate; in the typical 1-3 extras case the median is well
//       defined as the middle sorted entry.
//
// Returns 0.0 (the conservative "no coherence evidence" value) when:
//   - extraRatios is empty (caller should also gate on
//     kMinExtrasForCoherence before calling);
//   - primaryRatio is non-positive (spike ratio undefined; e.g. the
//     rolling median was below the noise floor and the detector
//     skipped the per-pair ratio computation).
inline double ComputeCoherenceScore(double primaryRatio, const std::vector<double>& extraRatios)
{
	if (extraRatios.empty()) return 0.0;
	if (!(primaryRatio > 0.0)) return 0.0;

	std::vector<double> sorted = extraRatios;
	std::sort(sorted.begin(), sorted.end());
	const double median = sorted[sorted.size() / 2];

	double score = median / primaryRatio;
	if (score < 0.0) score = 0.0;
	if (score > 1.0) score = 1.0;
	return score;
}

// True iff coherence is high enough AND there are enough extras to
// trust the second-opinion check, so a fire vote produced by the
// primary detector should be suppressed as common-mode.
inline bool ShouldSuppressFire(double coherenceScore, int extrasCount)
{
	return extrasCount >= kMinExtrasForCoherence && coherenceScore >= kSuppressThreshold;
}

} // namespace spacecal::coherence
