#pragma once

#include "PoseSample.h"

#include <array>
#include <cstddef>
#include <cstdint>

namespace phantom {

// Per-device fixed-capacity ring buffer of recent real poses. Single
// producer / single consumer: the hook thread that calls Push is the same
// thread that reads samples back during synthesis, so no synchronisation
// is required. Capacity is sized to hold ~2.8 s of 90 Hz history -- enough
// to cover the longest synth-hold window plus margin for restoring context
// after a recovery.
class PoseHistory
{
public:
	static constexpr size_t kCapacity = 256;

	PoseHistory() = default;

	// Append a real pose with its observation timestamp. Drops the oldest
	// entry if the buffer is full.
	void Push(int64_t qpc_ns, const vr::DriverPose_t& pose);

	// Mark all entries as not_real so future synthesis does not regress to a
	// partial pre-recovery history. Called on entry to LOST state.
	void ClearRealFlags();

	// How many samples are currently stored (<= kCapacity).
	size_t Size() const { return count_; }

	// Newest first. index 0 = most recent push, index Size()-1 = oldest.
	// Returns nullptr if index is out of range.
	const PoseSample* GetNewest(size_t index) const;

private:
	std::array<PoseSample, kCapacity> samples_{};
	size_t head_ = 0;  // Index of slot the NEXT push will write to.
	size_t count_ = 0; // Number of valid entries (caps at kCapacity).
};

} // namespace phantom
