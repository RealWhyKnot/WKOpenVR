#pragma once

#include <algorithm>
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

struct AudioFrameFeatures
{
	float peak = 0.0f;
	float rms = 0.0f;
	float low_band_ratio = 0.0f;
	float speech_band_ratio = 0.0f;
	float high_band_ratio = 0.0f;
	float zero_crossing_rate = 0.0f;
	float clipping_ratio = 0.0f;
	float crest_factor = 0.0f;
	float artifact_risk = 0.0f;
};

inline float ClampAudio01(float value)
{
	if (value < 0.0f) return 0.0f;
	if (value > 1.0f) return 1.0f;
	return value;
}

inline float OnePoleLowpassAlpha(float cutoff_hz, float sample_rate_hz)
{
	if (cutoff_hz <= 0.0f || sample_rate_hz <= 0.0f) return 1.0f;
	constexpr float kPi = 3.14159265358979323846f;
	const float rc = 1.0f / (2.0f * kPi * cutoff_hz);
	const float dt = 1.0f / sample_rate_hz;
	return ClampAudio01(dt / (rc + dt));
}

inline float EstimateAudioArtifactRisk(const AudioFrameFeatures& f)
{
	if (f.rms <= 0.0001f) return 0.0f;

	float risk = 0.0f;
	if (f.speech_band_ratio < 0.22f) risk += 0.20f;
	if (f.high_band_ratio > 0.72f && f.zero_crossing_rate > 0.20f)
		risk += 0.25f;
	else if (f.high_band_ratio > 0.82f)
		risk += 0.15f;
	if (f.low_band_ratio > 0.82f) risk += 0.15f;
	if (f.zero_crossing_rate > 0.35f) risk += 0.20f;
	if (f.clipping_ratio > 0.02f) risk += 0.25f;
	if (f.crest_factor > 0.0f && f.crest_factor < 1.80f && f.rms > 0.02f) risk += 0.10f;
	return ClampAudio01(risk);
}

inline AudioFrameFeatures ComputeAudioFrameFeatures(const float* data, std::size_t n, float sample_rate_hz = 16000.0f)
{
	AudioFrameFeatures out;
	if (!data || n == 0) return out;

	const double low_alpha = OnePoleLowpassAlpha(120.0f, sample_rate_hz);
	const double high_alpha = OnePoleLowpassAlpha(4000.0f, sample_rate_hz);
	double low120 = 0.0;
	double low4000 = 0.0;
	double low_energy = 0.0;
	double low4000_energy = 0.0;
	double high_energy = 0.0;
	double sum = 0.0;
	std::size_t clipping_samples = 0;
	std::size_t zero_crossings = 0;
	bool have_previous_sign = false;
	bool previous_positive = false;

	for (std::size_t i = 0; i < n; ++i) {
		const double s = static_cast<double>(data[i]);
		const double abs_s = std::fabs(s);
		if (abs_s > out.peak) out.peak = static_cast<float>(abs_s);
		if (abs_s >= 0.98) ++clipping_samples;
		sum += s * s;

		if (abs_s > 0.000001) {
			const bool positive = s >= 0.0;
			if (have_previous_sign && positive != previous_positive) {
				++zero_crossings;
			}
			previous_positive = positive;
			have_previous_sign = true;
		}

		low120 += low_alpha * (s - low120);
		low4000 += high_alpha * (s - low4000);
		const double high = s - low4000;
		low_energy += low120 * low120;
		low4000_energy += low4000 * low4000;
		high_energy += high * high;
	}

	out.peak = ClampAudio01(out.peak);
	out.rms = ClampAudio01(static_cast<float>(std::sqrt(sum / static_cast<double>(n))));
	out.zero_crossing_rate =
	    n > 1 ? ClampAudio01(static_cast<float>(zero_crossings) / static_cast<float>(n - 1)) : 0.0f;
	out.clipping_ratio = ClampAudio01(static_cast<float>(clipping_samples) / static_cast<float>(n));
	out.crest_factor = out.rms > 0.000001f ? out.peak / out.rms : 0.0f;

	const double mid_energy = std::max(0.0, low4000_energy - low_energy);
	const double denom = low_energy + mid_energy + high_energy + 0.000000001;
	out.low_band_ratio = ClampAudio01(static_cast<float>(low_energy / denom));
	out.speech_band_ratio = ClampAudio01(static_cast<float>(mid_energy / denom));
	out.high_band_ratio = ClampAudio01(static_cast<float>(high_energy / denom));
	out.artifact_risk = EstimateAudioArtifactRisk(out);
	return out;
}

} // namespace captions
