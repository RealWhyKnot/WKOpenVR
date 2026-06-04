#include <gtest/gtest.h>

#include "inputhealth/LearningRules.h"

using namespace inputhealth;

TEST(InputHealthLearningRules, StickMetadataRequiresAbsoluteTwoSided)
{
	EXPECT_TRUE(ScalarMetadataAllowsCompensation(protocol::InputHealthCompStickX, "/input/joystick/x",
	                                             kOpenVrScalarTypeAbsolute, kOpenVrScalarUnitsNormalizedTwoSided));
	EXPECT_FALSE(ScalarMetadataAllowsCompensation(protocol::InputHealthCompStickX, "/input/joystick/x",
	                                              kOpenVrScalarTypeAbsolute, kOpenVrScalarUnitsNormalizedOneSided));
	EXPECT_FALSE(ScalarMetadataAllowsCompensation(protocol::InputHealthCompStickX, "/input/joystick/x",
	                                              kOpenVrScalarTypeRelative, kOpenVrScalarUnitsNormalizedTwoSided));
}

TEST(InputHealthLearningRules, TriggerMetadataRequiresAbsoluteOneSided)
{
	EXPECT_TRUE(ScalarMetadataAllowsCompensation(protocol::InputHealthCompScalarSingle, "/input/trigger/value",
	                                             kOpenVrScalarTypeAbsolute, kOpenVrScalarUnitsNormalizedOneSided));
	EXPECT_FALSE(ScalarMetadataAllowsCompensation(protocol::InputHealthCompScalarSingle, "/input/trigger/value",
	                                              kOpenVrScalarTypeAbsolute, kOpenVrScalarUnitsNormalizedTwoSided));
	EXPECT_FALSE(ScalarMetadataAllowsCompensation(protocol::InputHealthCompScalarSingle, "/input/trigger/value",
	                                              kOpenVrScalarTypeRelative, kOpenVrScalarUnitsNormalizedOneSided));
}

TEST(InputHealthLearningRules, TrackpadAxisDoesNotLearnAsStick)
{
	EXPECT_FALSE(ScalarMetadataAllowsCompensation(protocol::InputHealthCompStickX, "/input/trackpad/x",
	                                              kOpenVrScalarTypeAbsolute, kOpenVrScalarUnitsNormalizedTwoSided));
	EXPECT_FALSE(ScalarMetadataAllowsLearning(PathFamily::TrackpadAxis, "/input/trackpad/x",
	                                          protocol::InputHealthCompStickX, kOpenVrScalarTypeAbsolute,
	                                          kOpenVrScalarUnitsNormalizedTwoSided));
}

TEST(InputHealthLearningRules, ForceAndGripAreOneSidedScalarOnly)
{
	EXPECT_TRUE(ScalarMetadataAllowsCompensation(protocol::InputHealthCompScalarSingle, "/input/grip/force",
	                                             kOpenVrScalarTypeAbsolute, kOpenVrScalarUnitsNormalizedOneSided));
	EXPECT_TRUE(ScalarMetadataAllowsCompensation(protocol::InputHealthCompScalarSingle, "/input/grip/value",
	                                             kOpenVrScalarTypeAbsolute, kOpenVrScalarUnitsNormalizedOneSided));
	EXPECT_FALSE(ScalarMetadataAllowsCompensation(protocol::InputHealthCompStickX, "/input/grip/force",
	                                              kOpenVrScalarTypeAbsolute, kOpenVrScalarUnitsNormalizedOneSided));
	EXPECT_FALSE(IsTriggerLikePath("/input/grip/force"));
	EXPECT_FALSE(IsTriggerLikePath("/input/grip/value"));
}

TEST(InputHealthLearningRules, StableStickRestAcceptsDriftAboveStrictThreshold)
{
	StableRestWindow w;
	EXPECT_FALSE(IsStrictStickRest(0.075f, 0.010f, true));
	EXPECT_TRUE(IsStableStickRestCandidate(0.075f, 0.010f, true));

	EXPECT_FALSE(UpdateStableRestWindow(w, true, 0.075f, 0.010f, 1000000ULL));
	EXPECT_FALSE(UpdateStableRestWindow(w, true, 0.077f, 0.011f, 1500000ULL));
	EXPECT_TRUE(UpdateStableRestWindow(w, true, 0.076f, 0.010f, 2000000ULL));
}

TEST(InputHealthLearningRules, StableRestWindowResetsWhenSpanGetsTooWide)
{
	StableRestWindow w;
	EXPECT_FALSE(UpdateStableRestWindow(w, true, 0.075f, 0.010f, 1000000ULL));
	EXPECT_FALSE(UpdateStableRestWindow(w, true, 0.095f, 0.010f, 1500000ULL));
	EXPECT_FALSE(UpdateStableRestWindow(w, true, 0.096f, 0.010f, 2000000ULL));
	EXPECT_TRUE(UpdateStableRestWindow(w, true, 0.095f, 0.011f, 2500000ULL));
}

TEST(InputHealthLearningRules, StableTriggerRestCoversElevatedFloorBand)
{
	EXPECT_FALSE(IsStrictTriggerRest(0.075f));
	EXPECT_TRUE(IsStableTriggerRestCandidate(0.075f, true));
	EXPECT_FALSE(IsStableTriggerRestCandidate(0.125f, true));
	EXPECT_FALSE(IsStableTriggerRestCandidate(0.075f, false));
}

TEST(InputHealthLearningRules, ButtonBounceDebounceIsPaddedAndCapped)
{
	EXPECT_TRUE(IsLikelyButtonBounceInterval(3000));
	EXPECT_FALSE(IsLikelyButtonBounceInterval(50000));
	EXPECT_EQ(DebounceFromBounceInterval(3000), 4000u);
	EXPECT_EQ(DebounceFromBounceInterval(25000), 20000u);
}

TEST(InputHealthLearningRules, SystemButtonPathIsSpecial)
{
	EXPECT_TRUE(IsSystemButtonPath("/input/system/click"));
	EXPECT_FALSE(IsSystemButtonPath("/input/a/click"));
}
