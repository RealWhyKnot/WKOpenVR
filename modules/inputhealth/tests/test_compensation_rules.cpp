#include <gtest/gtest.h>

#include "inputhealth/CompensationRules.h"

using namespace inputhealth;

TEST(InputHealthCompensationRules, ForceFloorOnlyClampsAndDoesNotScaleMax)
{
	EXPECT_FLOAT_EQ(ApplyScalarCompensationValue(protocol::InputHealthCompScalarSingle, "/input/grip/force",
	                                             kOpenVrScalarTypeAbsolute, kOpenVrScalarUnitsNormalizedOneSided, 0.03f,
	                                             0.10f, 0.0f, 0.50f, 0.0f, 0.0f, false),
	                0.0f);

	EXPECT_FLOAT_EQ(ApplyScalarCompensationValue(protocol::InputHealthCompScalarSingle, "/input/trackpad/force",
	                                             kOpenVrScalarTypeAbsolute, kOpenVrScalarUnitsNormalizedOneSided, 0.80f,
	                                             0.10f, 0.0f, 0.50f, 0.0f, 0.0f, false),
	                0.75f);
}

TEST(InputHealthCompensationRules, GripValueFloorOnlyClampsToOneSidedRange)
{
	EXPECT_FLOAT_EQ(ApplyScalarCompensationValue(protocol::InputHealthCompScalarSingle, "/input/grip/value",
	                                             kOpenVrScalarTypeAbsolute, kOpenVrScalarUnitsNormalizedOneSided, 1.0f,
	                                             0.02f, 0.0f, 0.0f, 0.0f, 0.0f, false),
	                0.98f);
}

TEST(InputHealthCompensationRules, FingerAndTrackpadAxesReturnRaw)
{
	EXPECT_FLOAT_EQ(ApplyScalarCompensationValue(protocol::InputHealthCompScalarSingle, "/input/finger/index",
	                                             kOpenVrScalarTypeAbsolute, kOpenVrScalarUnitsNormalizedOneSided, 0.42f,
	                                             0.10f, 0.0f, 0.0f, 0.0f, 0.0f, false),
	                0.42f);

	EXPECT_FLOAT_EQ(ApplyScalarCompensationValue(protocol::InputHealthCompStickX, "/input/trackpad/x",
	                                             kOpenVrScalarTypeAbsolute, kOpenVrScalarUnitsNormalizedTwoSided, 0.25f,
	                                             0.10f, 0.0f, 0.0f, 0.20f, 0.0f, false),
	                0.25f);
}

TEST(InputHealthCompensationRules, TriggerRemapDoesNotDoubleSubtractRest)
{
	EXPECT_FLOAT_EQ(ApplyScalarCompensationValue(protocol::InputHealthCompScalarSingle, "/input/trigger/value",
	                                             kOpenVrScalarTypeAbsolute, kOpenVrScalarUnitsNormalizedOneSided, 0.05f,
	                                             0.05f, 0.05f, 0.95f, 0.0f, 0.0f, false),
	                0.0f);
}

TEST(InputHealthCompensationRules, ThumbstickRadialDeadzoneUsesPartnerOffset)
{
	EXPECT_FLOAT_EQ(ApplyScalarCompensationValue(protocol::InputHealthCompStickX, "/input/thumbstick/x",
	                                             kOpenVrScalarTypeAbsolute, kOpenVrScalarUnitsNormalizedTwoSided, 0.03f,
	                                             0.02f, 0.0f, 0.0f, 0.05f, 0.02f, true, 0.02f, true),
	                0.0f);
}
