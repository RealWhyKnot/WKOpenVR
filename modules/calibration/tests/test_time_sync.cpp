// Tests for the pure two-stream tau (time-offset) estimator in TimeSync.h.

#include "TimeSync.h"

#include <gtest/gtest.h>

#include <vector>

namespace ts = spacecal::timesync;

namespace {
// A reusable angular-speed-like signal: a smooth bump (head turn) over a flat
// baseline so cross-correlation has a clear peak.
std::vector<double> Bump(size_t n, size_t center, double width)
{
	std::vector<double> v(n, 0.0);
	for (size_t i = 0; i < n; ++i) {
		const double d = (double)i - (double)center;
		v[i] = std::exp(-(d * d) / (2.0 * width * width));
	}
	return v;
}

// b[i] = a[i - shift] (b lags a by `shift`), zero-padded at the edges.
std::vector<double> ShiftRight(const std::vector<double>& a, int shift)
{
	std::vector<double> b(a.size(), 0.0);
	for (size_t i = 0; i < a.size(); ++i) {
		const long j = (long)i - shift;
		if (j >= 0 && (size_t)j < a.size()) b[i] = a[(size_t)j];
	}
	return b;
}
} // namespace

TEST(TimeSync, Identical_signals_zero_lag)
{
	const auto a = Bump(40, 20, 4.0);
	const auto est = ts::EstimateLag(a.data(), a.data(), a.size(), 10);
	EXPECT_EQ(est.lagSamples, 0);
	EXPECT_GT(est.confidence, 0.99);
}

TEST(TimeSync, Recovers_positive_lag)
{
	// b lags a by 3 samples -> estimator returns +3.
	const auto a = Bump(40, 18, 4.0);
	const auto b = ShiftRight(a, 3);
	const auto est = ts::EstimateLag(a.data(), b.data(), a.size(), 10);
	EXPECT_EQ(est.lagSamples, 3);
	EXPECT_GT(est.confidence, ts::kMinConfidence);
}

TEST(TimeSync, Recovers_negative_lag)
{
	// b leads a by 2 (a lags b) -> estimator returns -2.
	const auto a = Bump(40, 20, 4.0);
	const auto b = ShiftRight(a, -2);
	const auto est = ts::EstimateLag(a.data(), b.data(), a.size(), 10);
	EXPECT_EQ(est.lagSamples, -2);
}

TEST(TimeSync, Flat_signal_no_confidence)
{
	// A stationary window (no motion energy) yields no lock.
	std::vector<double> flat(40, 0.0);
	const auto a = Bump(40, 20, 4.0);
	const auto est = ts::EstimateLag(a.data(), flat.data(), a.size(), 10);
	EXPECT_EQ(est.lagSamples, 0);
	EXPECT_DOUBLE_EQ(est.confidence, 0.0);
}

TEST(TimeSync, Empty_or_bad_inputs_safe)
{
	const auto a = Bump(10, 5, 2.0);
	EXPECT_EQ(ts::EstimateLag(nullptr, a.data(), a.size(), 4).confidence, 0.0);
	EXPECT_EQ(ts::EstimateLag(a.data(), a.data(), 0, 4).confidence, 0.0);
	EXPECT_EQ(ts::EstimateLag(a.data(), a.data(), a.size(), -1).confidence, 0.0);
}
