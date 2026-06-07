#include "EyelidSync.h"

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include <algorithm>
#include <cmath>

namespace facetracking {

static constexpr float kWinkThreshold = 0.45f; // |lid_L - lid_R| threshold
static constexpr float kWinkConfMin = 0.6f;    // minimum confidence for wink detection
static constexpr float kWinkDwellMs = 120.f;   // ms asymmetry must hold
// Frame-rate-independent asymmetric smoothing time constants. A fast attack on
// closing keeps real blinks (a ~100-150 ms event) from being averaged away,
// while a gentle release on reopening keeps the look smooth. The previous fixed
// per-frame alpha (0.10) was tuned for 120 Hz but the module delivers ~60 Hz,
// giving a ~155 ms effective constant that masked blinks.
static constexpr float kCloseTauMs = 18.f; // closing (eye shutting) -- fast attack
static constexpr float kOpenTauMs = 70.f;  // opening (eye reopening) -- cosmetic

EyelidSync::EyelidSync()
    : wink_start_qpc_(0), wink_stable_(false), smooth_l_(0.5f), smooth_r_(0.5f), smooth_init_(false), last_qpc_(0),
      qpc_freq_{}
{
}

float EyelidSync::SmoothToward(float current, float target, double dtSeconds)
{
	// A long gap (e.g. tracking paused): snap rather than crawl. A zero/negative
	// delta can happen in tight test loops or same-tick frames; use a nominal
	// frame step so a one-frame spike still smooths instead of snapping.
	if (dtSeconds > 0.5) return target;
	// Sub-millisecond deltas mean same-tick or tight-loop calls (two frames in one
	// tick, or a unit test driving Apply in a loop); real frame data never arrives
	// that fast. Snap to a nominal frame step so convergence is deterministic
	// instead of crawling by a wall-clock-tiny alpha -- the latter made the
	// convergence tests flaky across machine speed and runner load.
	if (dtSeconds < 1.0 / 1000.0) dtSeconds = 1.0 / 120.0;
	const double tauMs = (target < current) ? kCloseTauMs : kOpenTauMs;
	const double alpha = 1.0 - std::exp(-(dtSeconds * 1000.0) / tauMs);
	return current + static_cast<float>(alpha) * (target - current);
}

LARGE_INTEGER EyelidSync::QpcFreq() const
{
	if (qpc_freq_.QuadPart == 0) QueryPerformanceFrequency(&qpc_freq_);
	return qpc_freq_;
}

void EyelidSync::Apply(protocol::FaceTrackingFrameBody& frame, uint8_t strength_0_to_100, bool preserve_winks,
                       uint8_t sync_mode)
{
	if (!(frame.flags & 1u)) return; // eye fields not valid
	if (strength_0_to_100 == 0) return;

	const float lid_L = frame.eye_openness_l;
	const float lid_R = frame.eye_openness_r;
	const float c_L = frame.eye_confidence_l;
	const float c_R = frame.eye_confidence_r;

	// Frame-rate-independent dt for the asymmetric smoother below.
	LARGE_INTEGER now_dt{};
	QueryPerformanceCounter(&now_dt);
	double dt = 0.0;
	if (last_qpc_ != 0) {
		dt = (double)((uint64_t)now_dt.QuadPart - last_qpc_) / (double)QpcFreq().QuadPart;
	}
	last_qpc_ = (uint64_t)now_dt.QuadPart;

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
		smooth_l_ = SmoothToward(smooth_l_, lid_L, dt);
		smooth_r_ = SmoothToward(smooth_r_, lid_R, dt);
		return;
	}

	float target = std::min(lid_L, lid_R);
	if (sync_mode == protocol::FACETRACKING_EYELID_SYNC_MOST_OPEN) {
		target = std::max(lid_L, lid_R);
	}

	float k = (float)strength_0_to_100 / 100.f;
	float blended_L = lid_L + k * (target - lid_L);
	float blended_R = lid_R + k * (target - lid_R);

	// Asymmetric, frame-rate-independent smoothing: fast close, gentle reopen.
	smooth_l_ = SmoothToward(smooth_l_, blended_L, dt);
	smooth_r_ = SmoothToward(smooth_r_, blended_R, dt);

	frame.eye_openness_l = std::max(0.f, std::min(1.f, smooth_l_));
	frame.eye_openness_r = std::max(0.f, std::min(1.f, smooth_r_));
}

} // namespace facetracking
