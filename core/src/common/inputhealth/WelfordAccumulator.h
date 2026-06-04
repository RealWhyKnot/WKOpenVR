#pragma once

#include <cmath>
#include <cstdint>

// Welford / Chan-Golub-LeVeque streaming mean + variance accumulator.
//
// Reference: Welford 1962 (Technometrics 4(3) p. 419) for the scalar form;
// Chan, Golub, LeVeque 1983 (American Statistician) for the parallel /
// pairwise update used here for one-shot insertion.
//
// Why this rather than naive sum-of-squares:
//   sum_x  = sum(x_i)
//   sum_x2 = sum(x_i * x_i)
//   var    = (sum_x2 - sum_x*sum_x / n) / (n - 1)
// suffers from catastrophic cancellation when the values are tightly
// clustered around a non-zero mean. Welford reformulates the update so the
// running variance never differences two large nearly-equal numbers.
//
// Used by InputHealth to track per-component noise floors at rest, peak
// distributions, and scalar variance for stuck-finger detection. None of the
// Stage-1 detection categories tolerate the cancellation error a sum-of-
// squares approach introduces at typical rest-noise magnitudes (sigma ~5e-3).
//
// Pure data + free functions. Caller owns thread safety; for the driver-side
// detour the accumulator lives in a per-component struct guarded by the
// component-map mutex.

namespace inputhealth {

struct WelfordState
{
	uint64_t count = 0;
	double mean = 0.0;
	double m2 = 0.0; // sum of squared deltas from running mean
};

// Streaming update. Cost: ~10 ns on x64.
inline void WelfordUpdate(WelfordState& s, double x)
{
	++s.count;
	const double delta = x - s.mean;
	s.mean += delta / static_cast<double>(s.count);
	const double delta2 = x - s.mean;
	s.m2 += delta * delta2;
}

// Population variance. Returns 0 for n < 1; use SampleVariance for n < 2 if
// you want a NaN-or-zero distinction.
inline double PopulationVariance(const WelfordState& s)
{
	if (s.count == 0) return 0.0;
	return s.m2 / static_cast<double>(s.count);
}

// Sample variance (Bessel-corrected). Returns 0 for n < 2.
inline double SampleVariance(const WelfordState& s)
{
	if (s.count < 2) return 0.0;
	return s.m2 / static_cast<double>(s.count - 1);
}

inline double PopulationStdDev(const WelfordState& s)
{
	return std::sqrt(PopulationVariance(s));
}

inline double SampleStdDev(const WelfordState& s)
{
	return std::sqrt(SampleVariance(s));
}

// Reset to initial state. Cheaper than re-constructing if the state lives
// inside a larger struct.
inline void WelfordReset(WelfordState& s)
{
	s.count = 0;
	s.mean = 0.0;
	s.m2 = 0.0;
}

} // namespace inputhealth
