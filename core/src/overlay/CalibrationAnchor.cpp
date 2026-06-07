#include "CalibrationAnchor.h"

namespace openvr_pair::overlay {

namespace {

std::vector<openvr_pair::overlay::CalibrationDeviceLock> g_locks;
std::string g_headsetSynthesisTrackerSerial;

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

void SetHeadsetSynthesisTrackerSerial(const std::string& serial)
{
	g_headsetSynthesisTrackerSerial = serial;
}

bool TryGetHeadsetSynthesisTrackerSerial(std::string& serial)
{
	if (g_headsetSynthesisTrackerSerial.empty()) return false;
	serial = g_headsetSynthesisTrackerSerial;
	return true;
}

bool IsHeadsetSynthesisTracker(const std::string& serial)
{
	return !serial.empty() && serial == g_headsetSynthesisTrackerSerial;
}

} // namespace openvr_pair::overlay
