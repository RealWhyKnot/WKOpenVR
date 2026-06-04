#pragma once

#include "Calibration.h"

#include <cstdint>
#include <string>

namespace spacecal::headmount {

struct HeadMountSourceFingerprint
{
	HeadMountSampleSource source = HeadMountSampleSource::Unknown;
	HeadMountMode mode = HeadMountMode::Off;
	uint32_t offsetVersion = 0;
	int32_t deviceID = -2;
	std::string targetSerial;
	std::string targetTrackingSystem;
};

struct HeadMountSourceDecision
{
	bool reset = false;
	const char* reason = "none";
};

inline HeadMountSampleSource SelectHeadMountSampleSource(HeadMountMode mode, bool offsetCalibrated)
{
	return (mode >= HeadMountMode::AutoPaired && offsetCalibrated) ? HeadMountSampleSource::HeadProxy
	                                                               : HeadMountSampleSource::PhysicalTracker;
}

inline HeadMountSourceDecision EvaluateHeadMountSourceTransition(bool previousValid,
                                                                 const HeadMountSourceFingerprint& previous,
                                                                 const HeadMountSourceFingerprint& current,
                                                                 bool relativePosCalibrated)
{
	if (!previousValid) {
		if (current.source == HeadMountSampleSource::HeadProxy && relativePosCalibrated) {
			return {true, "initial_head_proxy"};
		}
		return {};
	}

	if (previous.source != current.source) {
		return {true, "sample_source_changed"};
	}
	if (previous.mode != current.mode) {
		return {true, "mode_changed"};
	}
	if (previous.offsetVersion != current.offsetVersion) {
		return {true, "offset_changed"};
	}
	if (previous.deviceID != current.deviceID) {
		return {true, "device_changed"};
	}
	if (previous.targetSerial != current.targetSerial ||
	    previous.targetTrackingSystem != current.targetTrackingSystem) {
		return {true, "target_identity_changed"};
	}

	return {};
}

} // namespace spacecal::headmount
