#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

// 36-bin polar max-radius histogram for stick perimeter envelope tracking.
//
// Each bin covers 10 degrees of angular range and stores:
//   max_r           the largest radius observed in this angular bin
//   count           samples observed in this bin (capped at kPerBinCap)
//   last_update_us  microsecond timestamp of the most recent update
//
// Used by InputHealth to detect:
//   * dead arc          per-bin max_r notably below the global max
//   * asymmetry         per-octant aggregation of bin maxes, opposite-octant
//                       ratio test
//   * deadzone radius   smallest bin index with non-zero count near rest
//
// Cost on the detour thread: one std::atan2, one std::hypot, one bin
// arithmetic update. Roughly 80 ns per call on x64 (research doc Q6).
//
// Per the research doc Q4, passive stick usage is biased toward cardinal
// angles (forward / strafe / brake) by an order of magnitude over
// diagonals. Without intervention, popular bins drown rare bins in any
// "weight by sample count" aggregator. Two mitigations live in this module:
//
//   1. Per-bin sample cap: once count reaches kPerBinCap, additional
//      samples update max_r via decay-weighted EWMA but the count stops
//      incrementing. Prevents one bin's frequency from out-voting another.
//   2. Inverse-density adoption rule: confidence in a bin's max_r is
//      min(1, count / kPerBinMinForConfidence). Detectors should consult
//      that confidence before drawing per-bin conclusions.
//
// O(1) per update. Caller owns thread safety. Total state is bounded by
// kBinCount and is small (under 1 KiB) so the structure can live inline
// inside the per-component state.

namespace inputhealth {

constexpr int kBinCount = 36;               // 10 deg per bin
constexpr int kPerBinCap = 256;             // count saturation
constexpr int kPerBinMinForConfidence = 16; // adoption rule denominator
constexpr double kBinWidthRad = 6.283185307179586 / kBinCount;

struct PolarBin
{
	float max_r = 0.0f;
	uint16_t count = 0;
	uint64_t last_update_us = 0;
};

struct PolarHistogramState
{
	PolarBin bins[kBinCount] = {};
	float global_max_r = 0.0f; // running max of all bins, for ratio tests
};

inline int PolarBinIndexForAngle(double theta_rad)
{
	// Normalize theta into [0, 2pi).
	constexpr double kTwoPi = 6.283185307179586;
	double t = std::fmod(theta_rad, kTwoPi);
	if (t < 0.0) t += kTwoPi;
	int idx = static_cast<int>(t / kBinWidthRad);
	if (idx < 0) idx = 0;
	if (idx >= kBinCount) idx = kBinCount - 1;
	return idx;
}

// Update the histogram with one sample (x, y) at the given timestamp.
// timestamp_us is microseconds since an arbitrary epoch -- the caller can
// use any monotonic clock. The decay parameter controls how fast a bin's
// max_r decays once its count has saturated; typical: alpha for 24h
// half-life at the per-component update rate.
inline void PolarHistogramUpdate(PolarHistogramState& s, double x, double y, uint64_t timestamp_us,
                                 double saturated_decay)
{
	const double r = std::hypot(x, y);
	const double theta = std::atan2(y, x);
	const int bin = PolarBinIndexForAngle(theta);

	PolarBin& b = s.bins[bin];
	const float rf = static_cast<float>(r);

	if (b.count < kPerBinCap) {
		++b.count;
		if (rf > b.max_r) b.max_r = rf;
	}
	else {
		// Cap reached: blend the running max via EWMA decay so the bin can
		// reflect a hardware change (stick that used to reach 1.0 in this
		// direction starts capping at 0.92) instead of being permanently
		// pinned to the highest historical value.
		float decay = static_cast<float>(saturated_decay);
		if (decay < 0.0f) decay = 0.0f;
		if (decay > 1.0f) decay = 1.0f;
		const float blended = (1.0f - decay) * b.max_r + decay * rf;
		// Adopt only when blending pulls upward, OR when the new sample is
		// strictly higher than the historical max (a genuine new peak should
		// always be captured even at a tiny decay).
		if (rf > b.max_r)
			b.max_r = rf;
		else
			b.max_r = blended;
	}
	b.last_update_us = timestamp_us;

	if (b.max_r > s.global_max_r) s.global_max_r = b.max_r;
}

// Confidence in a bin's reported max_r, scaled into [0, 1] by the inverse-
// density adoption rule. Detectors should multiply test statistics by this
// confidence before crossing a threshold so a single-sample bin cannot
// trigger a "dead arc" call.
inline float PolarBinConfidence(const PolarBin& b)
{
	if (b.count == 0) return 0.0f;
	float c = static_cast<float>(b.count) / static_cast<float>(kPerBinMinForConfidence);
	if (c > 1.0f) c = 1.0f;
	return c;
}

inline void PolarHistogramReset(PolarHistogramState& s)
{
	for (int i = 0; i < kBinCount; ++i)
		s.bins[i] = PolarBin{};
	s.global_max_r = 0.0f;
}

} // namespace inputhealth
