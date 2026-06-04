#pragma once

#include "VRState.h"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <openvr.h>

namespace wkopenvr::controller_input {

struct TriggerReading
{
	bool buttonPressed = false;
	bool legacyFallbackUsed = false;
	int analogAxis = -1;
	float analogValue = 0.0f;
	int triggerAxisCount = 0;
	int propertyErrors = 0;
};

struct ControllerSelectionChoice
{
	int32_t deviceId = -1;
	vr::ETrackedControllerRole role = vr::TrackedControllerRole_Invalid;
	bool poseValid = false;
};

bool IsTriggerHeldFromAxisTypes(const vr::VRControllerState_t& state, const int32_t* axisTypes, size_t axisCount,
                                float analogThreshold, TriggerReading* reading = nullptr);

bool IsTriggerHeld(vr::IVRSystem* vrs, vr::TrackedDeviceIndex_t deviceId, const vr::VRControllerState_t& state,
                   float analogThreshold = 0.75f, TriggerReading* reading = nullptr);

size_t FillControllerIdsForTrackingSystem(const std::vector<VRDevice>& devices, const std::string& trackingSystem,
                                          int32_t* outControllerIds, size_t outCount);

int32_t ChoosePreferredController(const ControllerSelectionChoice* choices, size_t choiceCount,
                                  int32_t currentDeviceId);

} // namespace wkopenvr::controller_input
