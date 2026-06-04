#pragma once

#include <string>
#include <vector>

// In-process shared state: serial strings of devices currently used by active
// continuous calibration. Written by the calibration overlay on each scan tick;
// read by the smoothing overlay to lock out per-tracker prediction suppression
// on devices that feed the calibration solve.
//
// Both overlays run in the same process (OpenVRPairOverlay). The calls are
// always made from the main thread (draw + tick share a thread), so no
// synchronisation is required beyond plain containers.

namespace openvr_pair::overlay {

enum class CalibrationDeviceLockKind
{
	Reference,
	Target,
};

struct CalibrationDeviceLock
{
	std::string serial;
	CalibrationDeviceLockKind kind = CalibrationDeviceLockKind::Target;
};

// Set the current calibration device locks. Pass an empty vector when the
// calibration module is not running continuous calibration.
void SetCalibrationDeviceLocks(const std::vector<CalibrationDeviceLock>& locks);

// Return true when the serial is currently locked by continuous calibration.
bool TryGetCalibrationDeviceLockKind(const std::string& serial, CalibrationDeviceLockKind& kind);

} // namespace openvr_pair::overlay
