#include "PoseHistory.h"

namespace phantom {

void PoseHistory::Push(int64_t qpc_ns, const vr::DriverPose_t& pose)
{
	samples_[head_].qpc_ns = qpc_ns;
	samples_[head_].pose = pose;
	samples_[head_].was_real = true;
	head_ = (head_ + 1) % kCapacity;
	if (count_ < kCapacity) {
		++count_;
	}
}

void PoseHistory::ClearRealFlags()
{
	for (auto& s : samples_) {
		s.was_real = false;
	}
}

const PoseSample* PoseHistory::GetNewest(size_t index) const
{
	if (index >= count_) return nullptr;
	// head_ points one past the newest entry. Newest index 0 is at
	// (head_ + kCapacity - 1) % kCapacity.
	const size_t slot = (head_ + kCapacity - 1 - index) % kCapacity;
	return &samples_[slot];
}

} // namespace phantom
