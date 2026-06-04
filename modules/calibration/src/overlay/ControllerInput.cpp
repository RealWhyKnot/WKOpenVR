#include "ControllerInput.h"

#include <algorithm>
#include <cmath>

namespace wkopenvr::controller_input {

namespace {

constexpr float kLegacyFallbackThreshold = 0.75f;

bool TriggerButtonPressed(const vr::VRControllerState_t& state)
{
	return (state.ulButtonPressed & vr::ButtonMaskFromId(vr::k_EButton_SteamVR_Trigger)) != 0;
}

void NoteAnalogValue(TriggerReading* reading, int axis, float value)
{
	if (!reading) return;
	if (axis < 0) return;
	if (reading->analogAxis < 0 || value > reading->analogValue) {
		reading->analogAxis = axis;
		reading->analogValue = value;
	}
}

bool LegacyAxisFallback(const vr::VRControllerState_t& state, float analogThreshold, TriggerReading* reading)
{
	const float threshold = std::max(analogThreshold, kLegacyFallbackThreshold);
	for (uint32_t axis = 0; axis < vr::k_unControllerStateAxisCount; ++axis) {
		const float value = state.rAxis[axis].x;
		NoteAnalogValue(reading, static_cast<int>(axis), value);
		if (value >= threshold) {
			if (reading) reading->legacyFallbackUsed = true;
			return true;
		}
	}
	return false;
}

} // namespace

bool IsTriggerHeldFromAxisTypes(const vr::VRControllerState_t& state, const int32_t* axisTypes, size_t axisCount,
                                float analogThreshold, TriggerReading* reading)
{
	if (reading) {
		*reading = TriggerReading{};
	}

	if (TriggerButtonPressed(state)) {
		if (reading) reading->buttonPressed = true;
		return true;
	}

	const size_t count = std::min<size_t>(axisCount, vr::k_unControllerStateAxisCount);
	bool sawTriggerAxis = false;
	for (size_t axis = 0; axis < count; ++axis) {
		if (axisTypes[axis] != static_cast<int32_t>(vr::k_eControllerAxis_Trigger)) {
			continue;
		}

		sawTriggerAxis = true;
		if (reading) ++reading->triggerAxisCount;
		const float value = state.rAxis[axis].x;
		NoteAnalogValue(reading, static_cast<int>(axis), value);
		if (value >= analogThreshold) {
			return true;
		}
	}

	if (!sawTriggerAxis) {
		return LegacyAxisFallback(state, analogThreshold, reading);
	}

	return false;
}

bool IsTriggerHeld(vr::IVRSystem* vrs, vr::TrackedDeviceIndex_t deviceId, const vr::VRControllerState_t& state,
                   float analogThreshold, TriggerReading* reading)
{
	if (reading) {
		*reading = TriggerReading{};
	}

	if (TriggerButtonPressed(state)) {
		if (reading) reading->buttonPressed = true;
		return true;
	}

	int32_t axisTypes[vr::k_unControllerStateAxisCount] = {};
	int propertyErrors = 0;
	if (vrs) {
		for (uint32_t axis = 0; axis < vr::k_unControllerStateAxisCount; ++axis) {
			vr::ETrackedPropertyError err = vr::TrackedProp_Success;
			const auto prop = static_cast<vr::ETrackedDeviceProperty>(static_cast<int>(vr::Prop_Axis0Type_Int32) +
			                                                          static_cast<int>(axis));
			axisTypes[axis] = vrs->GetInt32TrackedDeviceProperty(deviceId, prop, &err);
			if (err != vr::TrackedProp_Success) {
				axisTypes[axis] = static_cast<int32_t>(vr::k_eControllerAxis_None);
				++propertyErrors;
			}
		}
	}
	else {
		propertyErrors = static_cast<int>(vr::k_unControllerStateAxisCount);
	}

	TriggerReading typedReading;
	const bool held =
	    IsTriggerHeldFromAxisTypes(state, axisTypes, vr::k_unControllerStateAxisCount, analogThreshold, &typedReading);

	if (reading) {
		*reading = typedReading;
		reading->propertyErrors = propertyErrors;
	}

	if (held) {
		return true;
	}

	if (typedReading.triggerAxisCount == 0 || propertyErrors > 0) {
		return LegacyAxisFallback(state, analogThreshold, reading);
	}

	return false;
}

size_t FillControllerIdsForTrackingSystem(const std::vector<VRDevice>& devices, const std::string& trackingSystem,
                                          int32_t* outControllerIds, size_t outCount)
{
	if (!outControllerIds || outCount == 0) {
		return 0;
	}

	std::fill(outControllerIds, outControllerIds + outCount, -1);

	size_t written = 0;
	for (const auto& device : devices) {
		if (written >= outCount) {
			break;
		}
		if (device.deviceClass != vr::TrackedDeviceClass_Controller) {
			continue;
		}
		if (!trackingSystem.empty() && device.trackingSystem != trackingSystem) {
			continue;
		}
		outControllerIds[written++] = device.id;
	}

	return written;
}

int32_t ChoosePreferredController(const ControllerSelectionChoice* choices, size_t choiceCount, int32_t currentDeviceId)
{
	if (!choices || choiceCount == 0) {
		return -1;
	}

	for (size_t i = 0; i < choiceCount; ++i) {
		if (choices[i].deviceId == currentDeviceId) {
			return currentDeviceId;
		}
	}

	int32_t rightWithPose = -1;
	int32_t rightAnyPose = -1;
	int32_t anyWithPose = -1;
	for (size_t i = 0; i < choiceCount; ++i) {
		const auto& choice = choices[i];
		if (choice.deviceId < 0) {
			continue;
		}
		if (choice.role == vr::TrackedControllerRole_RightHand) {
			if (rightAnyPose < 0) rightAnyPose = choice.deviceId;
			if (choice.poseValid && rightWithPose < 0) {
				rightWithPose = choice.deviceId;
			}
		}
		if (choice.poseValid && anyWithPose < 0) {
			anyWithPose = choice.deviceId;
		}
	}
	if (rightWithPose >= 0) return rightWithPose;
	if (rightAnyPose >= 0) return rightAnyPose;
	if (anyWithPose >= 0) return anyWithPose;
	return choices[0].deviceId;
}

} // namespace wkopenvr::controller_input
