#include <gtest/gtest.h>

#include "RoleCatalog.h"

TEST(RoleCatalogTest, ControllerTypeStringMatchesValveConvention)
{
    using phantom::BodyRoleToControllerType;
    EXPECT_STREQ(BodyRoleToControllerType(phantom::BodyRole::Waist),
                 "vive_tracker_waist");
    EXPECT_STREQ(BodyRoleToControllerType(phantom::BodyRole::LeftFoot),
                 "vive_tracker_left_foot");
    EXPECT_STREQ(BodyRoleToControllerType(phantom::BodyRole::RightFoot),
                 "vive_tracker_right_foot");
    EXPECT_STREQ(BodyRoleToControllerType(phantom::BodyRole::Chest),
                 "vive_tracker_chest");
    EXPECT_STREQ(BodyRoleToControllerType(phantom::BodyRole::LeftKnee),
                 "vive_tracker_left_knee");
    EXPECT_STREQ(BodyRoleToControllerType(phantom::BodyRole::RightKnee),
                 "vive_tracker_right_knee");
    EXPECT_STREQ(BodyRoleToControllerType(phantom::BodyRole::LeftElbow),
                 "vive_tracker_left_elbow");
    EXPECT_STREQ(BodyRoleToControllerType(phantom::BodyRole::RightElbow),
                 "vive_tracker_right_elbow");
    // HMD/hand roles never publish as a generic-tracker virtual device.
    EXPECT_EQ(BodyRoleToControllerType(phantom::BodyRole::Hmd),       nullptr);
    EXPECT_EQ(BodyRoleToControllerType(phantom::BodyRole::LeftHand),  nullptr);
    EXPECT_EQ(BodyRoleToControllerType(phantom::BodyRole::RightHand), nullptr);
    // VRChat uses upper-arm/elbow trackers to drive shoulders; shoulders are
    // internal solver roles, not separate virtual trackers.
    EXPECT_EQ(BodyRoleToControllerType(phantom::BodyRole::LeftShoulder),  nullptr);
    EXPECT_EQ(BodyRoleToControllerType(phantom::BodyRole::RightShoulder), nullptr);
}

TEST(RoleCatalogTest, KeyRoundTripsThroughFromKey)
{
    for (uint8_t i = 0; i < phantom::kBodyRoleCount; ++i) {
        const auto r = static_cast<phantom::BodyRole>(i);
        const auto k = phantom::BodyRoleToKey(r);
        ASSERT_NE(k, nullptr);
        EXPECT_EQ(phantom::BodyRoleFromKey(k), r) << "round-trip failed for "
            << phantom::BodyRoleLabel(r);
    }
}

TEST(RoleCatalogTest, UnknownKeyReturnsNone)
{
    EXPECT_EQ(phantom::BodyRoleFromKey(nullptr),     phantom::BodyRole::None);
    EXPECT_EQ(phantom::BodyRoleFromKey(""),          phantom::BodyRole::None);
    EXPECT_EQ(phantom::BodyRoleFromKey("nope"),      phantom::BodyRole::None);
    EXPECT_EQ(phantom::BodyRoleFromKey("waist_no"),  phantom::BodyRole::None);
}
