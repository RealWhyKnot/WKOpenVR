#pragma once

#include <cmath>
#include <cstddef>

// Pure helpers for reporting microphone input level. Kept header-only and free
// of Win32/COM so they can be unit-tested without a live capture device.
namespace captions {

// Peak (max absolute amplitude) of a mono float buffer, clamped to [0, 1].
// A silent buffer returns 0; this is what the overlay renders as the mic meter
// and what distinguishes "device delivers audio" from "device delivers nothing".
inline float ComputeBufferPeak(const float* data, std::size_t n)
{
	if (!data || n == 0) return 0.0f;
	float peak = 0.0f;
	for (std::size_t i = 0; i < n; ++i) {
		float a = std::fabs(data[i]);
		if (a > peak) peak = a;
	}
	if (peak > 1.0f) peak = 1.0f;
	return peak;
}

inline float ComputeBufferRms(const float* data, std::size_t n)
{
	if (!data || n == 0) return 0.0f;
	double sum = 0.0;
	for (std::size_t i = 0; i < n; ++i) {
		const double s = static_cast<double>(data[i]);
		sum += s * s;
	}
	float rms = static_cast<float>(std::sqrt(sum / static_cast<double>(n)));
	if (rms > 1.0f) rms = 1.0f;
	return rms;
}

// Smooth the displayed level: jump up to a louder peak immediately (so the
// meter is responsive to speech onset) but decay slowly toward silence (so the
// meter does not flicker between words). `decay` is the fraction of the old
// level retained when the new peak is lower.
inline float DecayLevel(float current, float newPeak, float decay)
{
	if (newPeak >= current) return newPeak;
	return current * decay;
}

} // namespace captions
