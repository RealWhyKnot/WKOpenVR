#include "ControllerInput.h"

#include <gtest/gtest.h>

#include <vector>

namespace {

vr::VRControllerState_t EmptyState()
{
	vr::VRControllerState_t state = {};
	return state;
}

} // namespace

TEST(ControllerInputTest, UsesAxisTypeMappingForTrigger)
{
	vr::VRControllerState_t state = EmptyState();
	state.rAxis[1].x = 0.82f;
	state.rAxis[static_cast<int>(vr::k_eControllerAxis_Trigger)].x = 0.0f;

	int32_t axisTypes[vr::k_unControllerStateAxisCount] = {};
	axisTypes[1] = static_cast<int32_t>(vr::k_eControllerAxis_Trigger);
	axisTypes[static_cast<int>(vr::k_eControllerAxis_Trigger)] = static_cast<int32_t>(vr::k_eControllerAxis_Joystick);

	wkopenvr::controller_input::TriggerReading reading;
	EXPECT_TRUE(wkopenvr::controller_input::IsTriggerHeldFromAxisTypes(
	    state, axisTypes, vr::k_unControllerStateAxisCount, 0.75f, &reading));
	EXPECT_EQ(reading.analogAxis, 1);
	EXPECT_FLOAT_EQ(reading.analogValue, 0.82f);
	EXPECT_EQ(reading.triggerAxisCount, 1);
}

TEST(ControllerInputTest, DoesNotTreatAxisTypeEnumAsControllerAxisIndex)
{
	vr::VRControllerState_t state = EmptyState();
	state.rAxis[static_cast<int>(vr::k_eControllerAxis_Trigger)].x = 0.95f;

	int32_t axisTypes[vr::k_unControllerStateAxisCount] = {};
	axisTypes[1] = static_cast<int32_t>(vr::k_eControllerAxis_Trigger);
	axisTypes[static_cast<int>(vr::k_eControllerAxis_Trigger)] = static_cast<int32_t>(vr::k_eControllerAxis_Joystick);

	wkopenvr::controller_input::TriggerReading reading;
	EXPECT_FALSE(wkopenvr::controller_input::IsTriggerHeldFromAxisTypes(
	    state, axisTypes, vr::k_unControllerStateAxisCount, 0.75f, &reading));
	EXPECT_EQ(reading.analogAxis, 1);
	EXPECT_FLOAT_EQ(reading.analogValue, 0.0f);
}

TEST(ControllerInputTest, ButtonMaskCountsAsTrigger)
{
	vr::VRControllerState_t state = EmptyState();
	state.ulButtonPressed = vr::ButtonMaskFromId(vr::k_EButton_SteamVR_Trigger);

	int32_t axisTypes[vr::k_unControllerStateAxisCount] = {};

	wkopenvr::controller_input::TriggerReading reading;
	EXPECT_TRUE(wkopenvr::controller_input::IsTriggerHeldFromAxisTypes(
	    state, axisTypes, vr::k_unControllerStateAxisCount, 0.75f, &reading));
	EXPECT_TRUE(reading.buttonPressed);
}

TEST(ControllerInputTest, FallsBackToHighAnalogWhenNoTriggerAxisIsReported)
{
	vr::VRControllerState_t state = EmptyState();
	state.rAxis[2].x = 0.96f;

	int32_t axisTypes[vr::k_unControllerStateAxisCount] = {};
	for (uint32_t i = 0; i < vr::k_unControllerStateAxisCount; ++i) {
		axisTypes[i] = static_cast<int32_t>(vr::k_eControllerAxis_None);
	}

	wkopenvr::controller_input::TriggerReading reading;
	EXPECT_TRUE(wkopenvr::controller_input::IsTriggerHeldFromAxisTypes(
	    state, axisTypes, vr::k_unControllerStateAxisCount, 0.75f, &reading));
	EXPECT_TRUE(reading.legacyFallbackUsed);
	EXPECT_EQ(reading.analogAxis, 2);
	EXPECT_FLOAT_EQ(reading.analogValue, 0.96f);
	EXPECT_EQ(reading.triggerAxisCount, 0);
}

TEST(ControllerInputTest, FillsControllersPastFirstEightDevices)
{
	std::vector<VRDevice> devices;
	for (int i = 0; i < 10; ++i) {
		VRDevice d;
		d.id = i;
		d.deviceClass = (i == 0) ? vr::TrackedDeviceClass_HMD : vr::TrackedDeviceClass_GenericTracker;
		d.trackingSystem = "lighthouse";
		devices.push_back(d);
	}

	VRDevice left;
	left.id = 12;
	left.deviceClass = vr::TrackedDeviceClass_Controller;
	left.trackingSystem = "lighthouse";
	left.controllerRole = vr::TrackedControllerRole_LeftHand;
	devices.push_back(left);

	VRDevice questController;
	questController.id = 14;
	questController.deviceClass = vr::TrackedDeviceClass_Controller;
	questController.trackingSystem = "oculus";
	devices.push_back(questController);

	VRDevice right;
	right.id = 18;
	right.deviceClass = vr::TrackedDeviceClass_Controller;
	right.trackingSystem = "lighthouse";
	right.controllerRole = vr::TrackedControllerRole_RightHand;
	devices.push_back(right);

	int32_t out[2] = {-1, -1};
	const size_t count = wkopenvr::controller_input::FillControllerIdsForTrackingSystem(devices, "lighthouse", out, 2);

	EXPECT_EQ(count, 2u);
	EXPECT_EQ(out[0], 12);
	EXPECT_EQ(out[1], 18);
}
