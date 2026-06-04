#include "DeviceFilters.h"

#include <algorithm>
#include <cctype>
#include <string>

namespace {

std::string LowerAscii(std::string value)
{
	std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) { return (char)std::tolower(c); });
	return value;
}

bool EqualsNoCase(const std::string& a, const char* b)
{
	return LowerAscii(a) == LowerAscii(b ? std::string(b) : std::string());
}

bool ContainsNoCase(const std::string& haystack, const char* needle)
{
	if (!needle || !*needle) return true;
	return LowerAscii(haystack).find(LowerAscii(needle)) != std::string::npos;
}

bool IsFaceTrackingSink(const std::string& serial, const std::string& model)
{
	if (EqualsNoCase(serial, "OpenVRPair-FaceTracking-Sink")) return true;
	if (EqualsNoCase(serial, "WKOpenVR-FaceTracking-Sink")) return true;

	const bool branded = ContainsNoCase(serial, "openvrpair") || ContainsNoCase(serial, "wkopenvr") ||
	                     ContainsNoCase(model, "openvrpair") || ContainsNoCase(model, "wkopenvr");
	const bool faceSink = ContainsNoCase(serial, "facetracking-sink") || ContainsNoCase(model, "facetracking-sink");
	return branded && faceSink;
}

bool IsUserPoseDeviceClass(vr::TrackedDeviceClass deviceClass)
{
	switch (deviceClass) {
		case vr::TrackedDeviceClass_HMD:
		case vr::TrackedDeviceClass_Controller:
		case vr::TrackedDeviceClass_GenericTracker:
			return true;
		default:
			return false;
	}
}

} // namespace

namespace openvr_pair::overlay {

bool IsInternalAuxiliaryTrackedDevice(const std::string& serial, const std::string& model)
{
	return IsFaceTrackingSink(serial, model);
}

namespace {

// WKOpenVR-emitted devices (phantom virtual trackers, future synthetic
// sources) declare themselves via Prop_TrackingSystemName_String == "wkopenvr".
// Calibrating against a synthetic source is nonsensical -- and a phantom-
// emitted virtual tracker sitting at its IK fallback position would
// otherwise appear in SC's input picker as a valid calibration anchor.
bool IsOwnTrackingSystem(const std::string& trackingSystem)
{
	return EqualsNoCase(trackingSystem, "wkopenvr");
}

} // namespace

bool ShouldShowInCalibrationDeviceList(vr::TrackedDeviceClass deviceClass, const std::string& serial,
                                       const std::string& model, const std::string& trackingSystem)
{
	if (!IsUserPoseDeviceClass(deviceClass)) return false;
	if (IsInternalAuxiliaryTrackedDevice(serial, model)) return false;
	if (IsOwnTrackingSystem(trackingSystem)) return false;
	return true;
}

bool ShouldShowInSmoothingPredictionList(vr::TrackedDeviceClass deviceClass, const std::string& serial,
                                         const std::string& model, const std::string& trackingSystem)
{
	if (!IsUserPoseDeviceClass(deviceClass)) return false;
	if (IsInternalAuxiliaryTrackedDevice(serial, model)) return false;
	if (IsOwnTrackingSystem(trackingSystem)) return false;
	return true;
}

} // namespace openvr_pair::overlay
