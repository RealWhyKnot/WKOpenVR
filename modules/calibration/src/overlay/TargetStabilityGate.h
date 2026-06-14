#pragma once

// Pure, ImGui-free helpers for the continuous-cal target-stability back-off.
//
// The reference/target validity gate in CollectSample already drops any single
// tick whose target tracker is not reporting a valid pose, so the incremental
// full solve only ever runs on a freshly collected valid sample. But when the
// target link is *intermittently* dropping -- a streamed inside-out headset
// whose tracking flaps, for example -- the rolling sample buffer ends up
// stitched across the dropouts and the solve produces high-dispersion
// candidates that are correctly rejected: pure churn (and noisy diagnostics).
//
// These helpers track a recent invalid-pose fraction (EWMA over continuous-cal
// ticks) and decide when to defer the solve until the link steadies. Deferring
// never alters the applied offset, so a mis-fire is harmless; the threshold is
// set high enough that a healthy session (invalid fraction ~0) never trips it.

namespace spacecal::target_stability {

// EWMA smoothing factor applied once per continuous-cal tick. ~0.05 gives a
// recent-history window of roughly 20 ticks.
inline constexpr double kSolveDeferEwmaAlpha = 0.05;

// Defer the solve once the recent invalid-pose fraction exceeds this. 0.5 means
// "the target was untracked for more than half of recent ticks" -- a clear
// instability burst, not the occasional dropout a healthy link has.
inline constexpr double kSolveDeferInvalidFraction = 0.5;

// Update the exponentially-weighted recent fraction of ticks whose target was
// not tracking. prev in [0,1]; returns the new value in [0,1].
inline double UpdateInvalidEwma(double prev, bool targetInvalidThisTick, double alpha)
{
	const double sample = targetInvalidThisTick ? 1.0 : 0.0;
	return prev + alpha * (sample - prev);
}

// True when the recent invalid fraction is high enough that a new solve should
// be deferred. Free function so it can be unit-tested in isolation.
inline bool ShouldDeferSolve(double invalidEwma, double maxInvalidFraction)
{
	return invalidEwma > maxInvalidFraction;
}

} // namespace spacecal::target_stability
