#include "InputHealthObservation.h"

#include "inputhealth/LearningRules.h"

#include <algorithm>

namespace {

// Tunable Page-Hinkley parameters. Defaults from the research doc Q3:
// alpha set so the EWMA half-life is ~30s at 250 Hz observation rate;
// delta = 0.002 (1/5 of a typical "still considered rest" envelope of 0.01);
// lambda = 0.05 (gives ARL ~10 hours for a step shift of 0.01, per Q8).
// These are starting points; per-category retuning lands once the snapshot
// publish path exists and field telemetry can be sampled.
inline inputhealth::PageHinkleyParams DefaultDriftParams()
{
	inputhealth::PageHinkleyParams p;
	p.alpha = 0.0001; // ~30s half-life at 250 Hz: 1 - exp(-1/(250*30)) ~ 1.3e-4
	p.delta = 0.002;
	p.lambda = 0.05;
	p.one_sided_positive = false;
	return p;
}

// Asymmetric rolling-min decay: 24-hour half-life at 250 Hz works out to
// alpha ~3.2e-8 per sample. Picks up genuinely stuck triggers without being
// fooled by transient near-zero crossings.
constexpr double kRestMinDecay = 3.2e-8;

// Threshold below which a sample is treated as "in rest" for the purposes
// of updating rest_min. Conservative: 0.1 catches the rest band of typical
// analog triggers (factory-stocked rest near 0.0, noise typically <0.05)
// without including mid-pull samples.
constexpr float kRestThreshold = 0.1f;

} // namespace

namespace inputhealth {

void ObserveBooleanSample(ComponentStats& stats, bool newValue, uint64_t nowUs)
{
	if (newValue != stats.pending_state) {
		if (stats.last_raw_transition_us != 0) {
			const uint64_t interval = nowUs - stats.last_raw_transition_us;
			if (inputhealth::IsLikelyButtonBounceInterval(interval)) {
				++stats.bounce_transition_count;
				const uint32_t clamped =
				    static_cast<uint32_t>(std::min<uint64_t>(interval, inputhealth::kButtonBounceMaxIntervalUs));
				if (clamped > stats.bounce_max_interval_us) {
					stats.bounce_max_interval_us = clamped;
				}
			}
		}
		stats.last_raw_transition_us = nowUs;
		if (newValue) ++stats.press_count;
		stats.pending_state = newValue;
	}
}

void ObserveScalarSample(ComponentStats& stats, float newValue, uint64_t nowUs, ComponentStats* partnerStats)
{
	// Stage 1D per-tick budget keeps to the items the research
	// doc Q6 admits onto the detour thread: ring push (skipped
	// for now -- ring-buffer wiring lands with the snapshot
	// path), Welford update, Page-Hinkley update, EWMA-min
	// update for rest samples, polar bin update for paired
	// axes. Heavy work (geometric median, hull rebuild, SPRT)
	// runs on the background worker (also not yet wired).
	WelfordUpdate(stats.welford, static_cast<double>(newValue));
	if (!stats.scalar_range_initialized) {
		stats.scalar_range_initialized = true;
		stats.observed_min = newValue;
		stats.observed_max = newValue;
	}
	else {
		if (newValue < stats.observed_min) stats.observed_min = newValue;
		if (newValue > stats.observed_max) stats.observed_max = newValue;
	}
	// DefaultDriftParams is a pure-constant struct -- compute once and
	// reuse instead of building it on every observation (called at the
	// per-component update rate, hundreds of Hz total).
	static const inputhealth::PageHinkleyParams driftParams = DefaultDriftParams();
	PageHinkleyUpdate(stats.ph_drift, driftParams, static_cast<double>(newValue));
	if (newValue < kRestThreshold && newValue > -kRestThreshold) {
		EWMARollingMinUpdate(stats.rest_min, static_cast<double>(newValue), kRestMinDecay);
	}

	// Polar histogram is owned by the X-side of a paired stick.
	// On a Y-side update, look up the X partner and update its
	// histogram with the (partner_last_x, this_y) tuple. On an
	// X-side update, do the same with this value as x and the
	// partner's last value as y. The histogram thus integrates
	// samples from both axes against a slight (~1 tick) cross-
	// axis lag, which is well below the 10 deg bin resolution.
	if (stats.axis_role == inputhealth::AxisRole::StickX) {
		const float partner_y = partnerStats ? partnerStats->last_value : 0.0f;
		PolarHistogramUpdate(stats.polar, static_cast<double>(newValue), static_cast<double>(partner_y), nowUs,
		                     kRestMinDecay);
	}
	else if (stats.axis_role == inputhealth::AxisRole::StickY) {
		if (partnerStats && partnerStats->axis_role == inputhealth::AxisRole::StickX) {
			PolarHistogramUpdate(partnerStats->polar, static_cast<double>(partnerStats->last_value),
			                     static_cast<double>(newValue), nowUs, kRestMinDecay);
		}
	}

	stats.last_value = newValue;
	stats.last_update_us = nowUs;
}

} // namespace inputhealth
