#include "CalibrationAnchor.h"

namespace openvr_pair::overlay {

namespace {

std::vector<openvr_pair::overlay::CalibrationDeviceLock> g_locks;
std::string g_headsetSynthesisTrackerSerial;
int g_headsetSynthesisLockedSmoothing = 0;
HeadsetSynthesisSmoothingUpdateFn g_headsetSynthesisSmoothingUpdateFn = nullptr;

int ClampPercent(int value)
{
	if (value < 0) return 0;
	if (value > 100) return 100;
	return value;
}

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
	g_headsetSynthesisLockedSmoothing = 0;
	g_headsetSynthesisSmoothingUpdateFn = nullptr;
}

void SetHeadsetSynthesisState(const std::string& serial, int lockedHeadsetSmoothing,
                              HeadsetSynthesisSmoothingUpdateFn updateFn)
{
	g_headsetSynthesisTrackerSerial = serial;
	g_headsetSynthesisLockedSmoothing = serial.empty() ? 0 : ClampPercent(lockedHeadsetSmoothing);
	g_headsetSynthesisSmoothingUpdateFn = serial.empty() ? nullptr : updateFn;
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

bool TryGetHeadsetSynthesisLockedSmoothing(int& smoothness)
{
	if (g_headsetSynthesisTrackerSerial.empty()) return false;
	smoothness = g_headsetSynthesisLockedSmoothing;
	return true;
}

bool TrySetHeadsetSynthesisLockedSmoothing(int smoothness, const char* reason)
{
	if (g_headsetSynthesisTrackerSerial.empty() || !g_headsetSynthesisSmoothingUpdateFn) return false;
	smoothness = ClampPercent(smoothness);
	if (!g_headsetSynthesisSmoothingUpdateFn(smoothness, reason)) return false;
	g_headsetSynthesisLockedSmoothing = smoothness;
	return true;
}

} // namespace openvr_pair::overlay
