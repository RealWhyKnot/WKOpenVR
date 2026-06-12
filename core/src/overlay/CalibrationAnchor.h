#pragma once

#include <string>
#include <vector>

// In-process shared state: serial strings of devices currently used by active
// continuous calibration or headset synthesis. Written by the calibration
// overlay on each scan tick; read by the smoothing overlay to explain or lock
// per-tracker prediction suppression on devices that feed calibration/headset
// pose paths.
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

using HeadsetSynthesisSmoothingUpdateFn = bool (*)(int smoothness, const char* reason);

// Set the current calibration device locks. Pass an empty vector when the
// calibration module is not running continuous calibration.
void SetCalibrationDeviceLocks(const std::vector<CalibrationDeviceLock>& locks);

// Return true when the serial is currently locked by continuous calibration.
bool TryGetCalibrationDeviceLockKind(const std::string& serial, CalibrationDeviceLockKind& kind);

// Set the physical tracker currently driving synthesized headset pose. Pass an
// empty string when headset synthesis is inactive or unresolved.
void SetHeadsetSynthesisTrackerSerial(const std::string& serial);

// Set the headset synthesis tracker and the authoritative smoothing value for
// the synthesized headset pose. The optional callback lets another UI surface
// update the calibration-owned value without depending on calibration globals.
void SetHeadsetSynthesisState(const std::string& serial, int lockedHeadsetSmoothing,
                              HeadsetSynthesisSmoothingUpdateFn updateFn);

// Return true when headset synthesis is active and the selected tracker serial
// is known.
bool TryGetHeadsetSynthesisTrackerSerial(std::string& serial);

// Return true when the serial is the physical tracker feeding headset synthesis.
bool IsHeadsetSynthesisTracker(const std::string& serial);

bool TryGetHeadsetSynthesisLockedSmoothing(int& smoothness);

bool TrySetHeadsetSynthesisLockedSmoothing(int smoothness, const char* reason);

} // namespace openvr_pair::overlay
