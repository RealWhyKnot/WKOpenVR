#include "EyelidSync.h"

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include <algorithm>
#include <cmath>

namespace facetracking {

static constexpr float kWinkThreshold = 0.45f; // |lid_L - lid_R| threshold
static constexpr float kWinkConfMin = 0.6f;    // minimum confidence for wink detection
static constexpr float kWinkDwellMs = 120.f;   // ms asymmetry must hold
static constexpr float kSmoothTimeMs = 80.f;   // temporal smoothing time constant
// The 80 ms smooth makes blink onsets look natural instead of popping.
// alpha = 1 - exp(-dt/tau); at 120 Hz, dt ~ 8.3 ms, tau = 80 ms -> alpha ~ 0.098.
// We do a simplified frame-rate-independent lerp using a fixed alpha tuned for
// 120 Hz.  At 90 Hz the smoother is slightly heavier; at 144 Hz slightly lighter.
// Acceptable range for a cosmetic filter.
static constexpr float kSmoothAlpha = 0.10f; // per-frame EMA alpha at ~120 Hz

EyelidSync::EyelidSync()
    : wink_start_qpc_(0), wink_stable_(false), smooth_l_(0.5f), smooth_r_(0.5f), smooth_init_(false), qpc_freq_{}
{
}

LARGE_INTEGER EyelidSync::QpcFreq() const
{
	if (qpc_freq_.QuadPart == 0) QueryPerformanceFrequency(&qpc_freq_);
	return qpc_freq_;
}

void EyelidSync::Apply(protocol::FaceTrackingFrameBody& frame, uint8_t strength_0_to_100, bool preserve_winks)
{
	if (!(frame.flags & 1u)) return; // eye fields not valid
	if (strength_0_to_100 == 0) return;

	const float lid_L = frame.eye_openness_l;
	const float lid_R = frame.eye_openness_r;
	const float c_L = frame.eye_confidence_l;
	const float c_R = frame.eye_confidence_r;

	// Seed the smoother on first call so there is no step from 0.5 to reality.
	if (!smooth_init_) {
		smooth_l_ = lid_L;
		smooth_r_ = lid_R;
		smooth_init_ = true;
	}

	// Wink detection.
	bool intentional_wink = false;
	if (preserve_winks) {
		const float asym = std::abs(lid_L - lid_R);
		if (asym > kWinkThreshold && c_L > kWinkConfMin && c_R > kWinkConfMin) {
			// Asymmetric state -- start or continue the dwell timer.
			if (wink_start_qpc_ == 0) {
				LARGE_INTEGER now{};
				QueryPerformanceCounter(&now);
				wink_start_qpc_ = (uint64_t)now.QuadPart;
				wink_stable_ = false;
			}
			else if (!wink_stable_) {
				LARGE_INTEGER now{};
				QueryPerformanceCounter(&now);
				float elapsed_ms =
				    (float)((uint64_t)now.QuadPart - wink_start_qpc_) * 1000.f / (float)QpcFreq().QuadPart;
				if (elapsed_ms >= kWinkDwellMs) wink_stable_ = true;
			}
			intentional_wink = wink_stable_;
		}
		else {
			// Symmetry restored -- reset dwell timer.
			wink_start_qpc_ = 0;
			wink_stable_ = false;
		}
	}

	if (intentional_wink) {
		// Bypass sync entirely so the intentional wink reaches the output.
		// Keep the smoother tracking reality so there is no pop when sync
		// re-engages. Standard EMA toward the raw lid value:
		//   smooth += alpha * (raw - smooth)
		// The prior form `raw + alpha * (raw - smooth)` was an inverted-sign
		// bug -- the smoother diverged from the raw value instead of tracking
		// it, and re-engagement of sync popped exactly the gap the smoother
		// was meant to close.
		smooth_l_ = smooth_l_ + kSmoothAlpha * (lid_L - smooth_l_);
		smooth_r_ = smooth_r_ + kSmoothAlpha * (lid_R - smooth_r_);
		return;
	}

	// Confidence-weighted target.
	float denom = c_L + c_R + 1e-6f;
	float target = (c_L * lid_L + c_R * lid_R) / denom;

	float k = (float)strength_0_to_100 / 100.f;
	float blended_L = lid_L + k * (target - lid_L);
	float blended_R = lid_R + k * (target - lid_R);

	// 80 ms temporal smoothing.
	smooth_l_ = smooth_l_ + kSmoothAlpha * (blended_L - smooth_l_);
	smooth_r_ = smooth_r_ + kSmoothAlpha * (blended_R - smooth_r_);

	frame.eye_openness_l = std::max(0.f, std::min(1.f, smooth_l_));
	frame.eye_openness_r = std::max(0.f, std::min(1.f, smooth_r_));
}

} // namespace facetracking
