#pragma once

// Pure two-stream time-offset (tau) estimation (Stage 2, item 6).
//
// The reference (Lighthouse) and target (SLAM/Quest) pose streams are
// time-misaligned; pairing samples taken at different instants injects a
// motion-dependent error that dominates the residual during head motion.
// Rotation is shared between the streams and has high SNR during head turns, so
// the lag that best aligns the two angular-speed signals estimates tau.
//
// This is the pure estimator (normalized cross-correlation over a lag window).
// Kept free of CalCtx / Eigen so it is unit-testable; the caller buffers the
// per-stream angular speeds and converts the returned sample lag to milliseconds
// using the sample period.

#include <cmath>
#include <cstddef>

namespace spacecal::timesync {

struct TauEstimate
{
	int lagSamples = 0;      // +lag means stream b lags stream a by lagSamples
	double confidence = 0.0; // peak normalized correlation in [0,1]
};

// Minimum peak correlation to trust a tau estimate; below this the streams are
// too flat / noisy (e.g. the user is stationary) to lock onto a lag.
constexpr double kMinConfidence = 0.5;

// Normalized cross-correlation of a[] and b[] (length n) over lags in
// [-maxLag, maxLag]. Returns the lag with the highest normalized correlation and
// that correlation as confidence. Returns {0, 0} when either signal has
// negligible energy (norm^2 below eps) so a flat/stationary window never
// produces a spurious lag.
inline TauEstimate EstimateLag(const double* a, const double* b, size_t n, int maxLag)
{
	TauEstimate best;
	if (!a || !b || n == 0 || maxLag < 0) return best;
	if ((size_t)maxLag >= n) maxLag = (int)n - 1;

	constexpr double kEnergyEps = 1e-12;
	double bestCorr = -2.0;
	for (int lag = -maxLag; lag <= maxLag; ++lag) {
		double dot = 0.0, ea = 0.0, eb = 0.0;
		for (size_t i = 0; i < n; ++i) {
			const long j = (long)i + lag; // +lag: b[i+lag] aligns with a[i] -> b lags a by lag
			if (j < 0 || (size_t)j >= n) continue;
			const double av = a[i];
			const double bv = b[(size_t)j];
			dot += av * bv;
			ea += av * av;
			eb += bv * bv;
		}
		if (ea < kEnergyEps || eb < kEnergyEps) continue;
		const double corr = dot / std::sqrt(ea * eb); // normalized cross-correlation in [-1,1]
		if (corr > bestCorr) {
			bestCorr = corr;
			best.lagSamples = lag;
		}
	}
	best.confidence = bestCorr < 0.0 ? 0.0 : bestCorr;
	return best;
}

} // namespace spacecal::timesync
