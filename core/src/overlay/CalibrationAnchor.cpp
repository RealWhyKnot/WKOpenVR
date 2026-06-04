#include "CalibrationAnchor.h"

namespace openvr_pair::overlay {

namespace {

std::vector<openvr_pair::overlay::CalibrationDeviceLock> g_locks;

} // namespace

void SetCalibrationDeviceLocks(const std::vector<CalibrationDeviceLock>& locks)
{
	g_locks.clear();

	for (const auto& lock : locks) {
		if (lock.serial.empty()) continue;

		bool duplicate = false;
		for (const auto& existing : g_locks) {
			if (existing.serial == lock.serial) {
				duplicate = true;
				break;
			}
		}
		if (duplicate) continue;

		g_locks.push_back(lock);
	}
}

bool TryGetCalibrationDeviceLockKind(const std::string& serial, CalibrationDeviceLockKind& kind)
{
	for (const auto& lock : g_locks) {
		if (lock.serial == serial) {
			kind = lock.kind;
			return true;
		}
	}
	return false;
}

} // namespace openvr_pair::overlay
