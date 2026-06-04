#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace inputhealth {

enum class PathFamily : uint8_t
{
	TriggerValue,
	ThumbstickAxis,
	TrackpadAxis,
	ForceSensor,
	GripValue,
	FingerCapsense,
	ControllerButton,
	DiagnosticsOnly,
	Unsupported,
};

inline bool EndsWithPathSegment(const std::string& path, const char* suffix, size_t len)
{
	const size_t n = path.size();
	return n >= len && path.compare(n - len, len, suffix) == 0;
}

inline bool HasClickOrTouchSuffix(const std::string& path)
{
	return EndsWithPathSegment(path, "/click", 6) || EndsWithPathSegment(path, "/touch", 6) ||
	       path.find("/click/") != std::string::npos || path.find("/touch/") != std::string::npos;
}

inline bool IsAxisSuffix(const std::string& path)
{
	const size_t n = path.size();
	if (n < 2 || path[n - 2] != '/') return false;
	const char axis = path[n - 1];
	return axis == 'x' || axis == 'X' || axis == 'y' || axis == 'Y';
}

inline PathFamily ClassifyPathFamily(const std::string& path)
{
	if (path.empty()) return PathFamily::Unsupported;

	if (path.find("/eye") != std::string::npos || path.find("/face") != std::string::npos) {
		return PathFamily::DiagnosticsOnly;
	}

	if (path.find("/pupil") != std::string::npos || path.find("proximity") != std::string::npos ||
	    path.find("/imu") != std::string::npos) {
		return PathFamily::Unsupported;
	}

	if (path.find("/input/finger/") != std::string::npos) {
		return PathFamily::FingerCapsense;
	}

	if (HasClickOrTouchSuffix(path)) {
		return PathFamily::ControllerButton;
	}

	if (path.find("trigger") != std::string::npos || path.find("Trigger") != std::string::npos) {
		return PathFamily::TriggerValue;
	}

	if (path.find("/input/grip/value") != std::string::npos || path.find("/input/squeeze/value") != std::string::npos) {
		return PathFamily::GripValue;
	}

	if (EndsWithPathSegment(path, "/force", 6) || EndsWithPathSegment(path, "/pressure", 9)) {
		return PathFamily::ForceSensor;
	}

	if (IsAxisSuffix(path)) {
		if (path.find("thumbstick") != std::string::npos || path.find("joystick") != std::string::npos) {
			return PathFamily::ThumbstickAxis;
		}
		if (path.find("trackpad") != std::string::npos || path.find("touchpad") != std::string::npos) {
			return PathFamily::TrackpadAxis;
		}
	}

	if (path.find("/input/") != std::string::npos) {
		return PathFamily::ControllerButton;
	}

	return PathFamily::Unsupported;
}

inline const char* PathFamilyName(PathFamily family)
{
	switch (family) {
		case PathFamily::TriggerValue:
			return "trigger_value";
		case PathFamily::ThumbstickAxis:
			return "thumbstick_axis";
		case PathFamily::TrackpadAxis:
			return "trackpad_axis";
		case PathFamily::ForceSensor:
			return "force_sensor";
		case PathFamily::GripValue:
			return "grip_value";
		case PathFamily::FingerCapsense:
			return "finger_capsense";
		case PathFamily::ControllerButton:
			return "controller_button";
		case PathFamily::DiagnosticsOnly:
			return "diagnostics_only";
		case PathFamily::Unsupported:
			return "unsupported";
	}
	return "unsupported";
}

inline bool IsTriggerRemapFamily(PathFamily family)
{
	return family == PathFamily::TriggerValue;
}

inline bool IsThumbstickAxisFamily(PathFamily family)
{
	return family == PathFamily::ThumbstickAxis;
}

inline bool IsTrackpadAxisFamily(PathFamily family)
{
	return family == PathFamily::TrackpadAxis;
}

inline bool IsIdleFloorFamily(PathFamily family)
{
	return family == PathFamily::ForceSensor || family == PathFamily::GripValue;
}

inline bool IsDiagnosticsOnlyFamily(PathFamily family)
{
	return family == PathFamily::DiagnosticsOnly || family == PathFamily::FingerCapsense ||
	       family == PathFamily::TrackpadAxis;
}

inline bool AllowsDriverCompensation(PathFamily family)
{
	return family == PathFamily::TriggerValue || family == PathFamily::ThumbstickAxis ||
	       family == PathFamily::ForceSensor || family == PathFamily::GripValue ||
	       family == PathFamily::ControllerButton;
}

inline bool AllowsPersistentScalarLearning(PathFamily family)
{
	return family == PathFamily::TriggerValue || family == PathFamily::ThumbstickAxis ||
	       family == PathFamily::ForceSensor || family == PathFamily::GripValue;
}

inline bool AllowsPersistentBooleanLearning(PathFamily family)
{
	return family == PathFamily::ControllerButton;
}

} // namespace inputhealth
