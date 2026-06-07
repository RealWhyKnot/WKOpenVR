#pragma once

#include "Protocol.h"

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include <cstdint>

namespace facetracking {

// Targeted eyelid sync with wink preservation.
//
// When engaged, both eyes are pulled toward either the most closed or most open
// eye, selected by the user. Each eye is lerped toward the target by
// `strength / 100`. Temporal smoothing on the blended output prevents blink
// onsets from popping.
//
// Wink detection: if both eyes have high confidence (> 0.6) AND the asymmetry
// (|lid_L - lid_R|) exceeds 0.45 AND that asymmetry has been stable for at
// least 120 ms, the frame is treated as an intentional wink and sync is
// bypassed for that window.
//
// Hot path: no allocations, no logging.
class EyelidSync
{
public:
	EyelidSync();

	// Apply eyelid sync in-place. Strength 0 = no-op, 100 = full sync.
	void Apply(protocol::FaceTrackingFrameBody& frame, uint8_t strength_0_to_100, bool preserve_winks,
	           uint8_t sync_mode = protocol::FACETRACKING_EYELID_SYNC_MOST_CLOSED);

	// Frame-rate-independent one-pole step toward `target`. dtSeconds <= 0 or a
	// long gap snaps. Uses a short time constant while closing (target < current)
	// so blinks pass through, and a longer one while opening. Pure -- exposed for
	// tests.
	static float SmoothToward(float current, float target, double dtSeconds);

private:
	// Wink-dwell timer: QPC tick at which the current asymmetric state started.
	// 0 means no asymmetric state is in progress.
	uint64_t wink_start_qpc_;
	bool wink_stable_; // true once asymmetry has been stable >= 120 ms

	// Temporal smoothing on the synchronized output (one value per eye).
	// Updated each Apply() call so that even if wink bypass fires for a few
	// frames the smoother state doesn't jump. The smoother is frame-rate
	// independent and asymmetric: a fast attack on closing lets real blinks
	// reach the avatar, while a gentle release on reopening stays cosmetic.
	float smooth_l_;
	float smooth_r_;
	bool smooth_init_;
	uint64_t last_qpc_; // QPC tick of the previous Apply() call; 0 = none yet

	// Cached QPC frequency.
	mutable LARGE_INTEGER qpc_freq_;
	LARGE_INTEGER QpcFreq() const;
};

} // namespace facetracking
