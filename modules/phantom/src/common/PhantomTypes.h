#pragma once

#include <cstdint>

namespace phantom {

// Per-device state in the dropout->synth->lost ladder. The driver publishes
// this for each tracker it observes via the per-device shmem so the overlay
// can render a status badge without re-deriving the state. Order is the
// natural progression of a dropout event:
//   REAL          -- pose stream is healthy, passthrough.
//   BLEND_OUT     -- transitioning from real to synthesised pose (80 ms).
//   SYNTH_RECKON  -- dead-reckoned synthesised pose, first 250 ms of silence.
//   SYNTH_IK      -- IK-driven synthesised pose, Phase 1.5+.
//   SYNTH_ML      -- ML-completed synthesised pose, Phase 3+.
//   OUT_OF_RANGE  -- still publishing, but ETrackingResult = Running_OutOfRange
//                    so consumers gracefully drop the tracker from IK chains.
//   LOST          -- no longer publishing for this device this session.
enum class TrackerState : uint8_t
{
	REAL = 0,
	BLEND_OUT = 1,
	SYNTH_RECKON = 2,
	SYNTH_IK = 3,
	SYNTH_ML = 4,
	BLEND_IN = 5,
	OUT_OF_RANGE = 6,
	LOST = 7,
};

// Human-readable label for diagnostics + overlay badge text.
inline const char* TrackerStateLabel(TrackerState s)
{
	switch (s) {
		case TrackerState::REAL:
			return "Real";
		case TrackerState::BLEND_OUT:
			return "Blending out";
		case TrackerState::SYNTH_RECKON:
			return "Dead reckoning";
		case TrackerState::SYNTH_IK:
			return "IK";
		case TrackerState::SYNTH_ML:
			return "ML";
		case TrackerState::BLEND_IN:
			return "Blending in";
		case TrackerState::OUT_OF_RANGE:
			return "Out of range";
		case TrackerState::LOST:
			return "Lost";
	}
	return "?";
}

} // namespace phantom
