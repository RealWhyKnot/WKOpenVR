#pragma once

#include "Protocol.h"

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include <cstdint>

namespace facetracking {

// Confidence-weighted eyelid sync with wink preservation.
//
// When engaged, the target openness is the confidence-weighted mean of both
// eyes; each eye is lerped toward the target by `strength / 100`.  An 80 ms
// temporal smoothing on the blended output prevents blink onsets from popping.
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

	// Apply eyelid sync in-place.  Strength 0 = no-op, 100 = full sync.
	void Apply(protocol::FaceTrackingFrameBody& frame, uint8_t strength_0_to_100, bool preserve_winks);

private:
	// Wink-dwell timer: QPC tick at which the current asymmetric state started.
	// 0 means no asymmetric state is in progress.
	uint64_t wink_start_qpc_;
	bool wink_stable_; // true once asymmetry has been stable >= 120 ms

	// 80 ms temporal smoothing on the synchronized output (one value per eye).
	// Updated each Apply() call so that even if wink bypass fires for a few
	// frames the smoother state doesn't jump.
	float smooth_l_;
	float smooth_r_;
	bool smooth_init_;

	// Cached QPC frequency.
	mutable LARGE_INTEGER qpc_freq_;
	LARGE_INTEGER QpcFreq() const;
};

} // namespace facetracking
