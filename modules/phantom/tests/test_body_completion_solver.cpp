#include <gtest/gtest.h>

#include "BodyCompletionSolver.h"

#include <cmath>

namespace {

constexpr double kPi = 3.14159265358979323846;

phantom::BodyCompletionPose MakePose(double x, double y, double z,
                                     const double q[4])
{
    phantom::BodyCompletionPose pose;
    pose.position[0] = x;
    pose.position[1] = y;
    pose.position[2] = z;
    pose.rotation[0] = q[0];
    pose.rotation[1] = q[1];
    pose.rotation[2] = q[2];
    pose.rotation[3] = q[3];
    return pose;
}

phantom::BodyCompletionSensorPose Sensor(const phantom::BodyCompletionPose& pose)
{
    phantom::BodyCompletionSensorPose sensor;
    sensor.pose = pose;
    sensor.valid = true;
    return sensor;
}

void Enable(phantom::BodyCompletionInput& input, phantom::BodyRole role)
{
    input.enabled_roles[static_cast<uint8_t>(role)] = true;
}

} // namespace

TEST(BodyCompletionSolverTest, HmdOnlyNeutralStanceProducesCoreRoles)
{
    phantom::BodyCompletionSolver solver;
    phantom::BodyCompletionInput input;
    const double identity[4] = {1.0, 0.0, 0.0, 0.0};
    input.hmd = Sensor(MakePose(0.0, 1.70, 0.0, identity));
    Enable(input, phantom::BodyRole::Waist);
    Enable(input, phantom::BodyRole::Chest);
    Enable(input, phantom::BodyRole::LeftFoot);
    Enable(input, phantom::BodyRole::RightFoot);

    const auto result = solver.Solve(input);
    const auto& waist = result.roles[static_cast<uint8_t>(phantom::BodyRole::Waist)];
    const auto& chest = result.roles[static_cast<uint8_t>(phantom::BodyRole::Chest)];
    const auto& leftFoot = result.roles[static_cast<uint8_t>(phantom::BodyRole::LeftFoot)];
    const auto& rightFoot = result.roles[static_cast<uint8_t>(phantom::BodyRole::RightFoot)];

    ASSERT_TRUE(waist.valid);
    ASSERT_TRUE(chest.valid);
    ASSERT_TRUE(leftFoot.valid);
    ASSERT_TRUE(rightFoot.valid);
    EXPECT_LT(waist.pose.position[1], input.hmd.pose.position[1]);
    EXPECT_LT(chest.pose.position[1], input.hmd.pose.position[1]);
    EXPECT_NEAR(leftFoot.pose.position[1], input.calibration.floor_y_m + 0.04, 1e-6);
    EXPECT_NEAR(rightFoot.pose.position[1], input.calibration.floor_y_m + 0.04, 1e-6);
    EXPECT_GT(chest.confidence, leftFoot.confidence);
    EXPECT_GT(result.global_confidence, 0.0f);
}

TEST(BodyCompletionSolverTest, HmdYawRotatesStance)
{
    phantom::BodyCompletionSolver solver;
    phantom::BodyCompletionInput input;
    const double yaw90[4] = {std::cos(kPi / 4.0), 0.0, std::sin(kPi / 4.0), 0.0};
    input.hmd = Sensor(MakePose(0.0, 1.70, 0.0, yaw90));
    Enable(input, phantom::BodyRole::LeftFoot);
    Enable(input, phantom::BodyRole::RightFoot);

    const auto result = solver.Solve(input);
    const auto& leftFoot = result.roles[static_cast<uint8_t>(phantom::BodyRole::LeftFoot)];
    const auto& rightFoot = result.roles[static_cast<uint8_t>(phantom::BodyRole::RightFoot)];
    ASSERT_TRUE(leftFoot.valid);
    ASSERT_TRUE(rightFoot.valid);

    const double dx = std::abs(leftFoot.pose.position[0] - rightFoot.pose.position[0]);
    const double dz = std::abs(leftFoot.pose.position[2] - rightFoot.pose.position[2]);
    EXPECT_GT(dz, dx);
}

TEST(BodyCompletionSolverTest, CrouchLowersRootEstimate)
{
    phantom::BodyCompletionSolver solver;
    phantom::BodyCompletionInput standing;
    phantom::BodyCompletionInput crouch;
    const double identity[4] = {1.0, 0.0, 0.0, 0.0};
    standing.hmd = Sensor(MakePose(0.0, 1.70, 0.0, identity));
    crouch.hmd = Sensor(MakePose(0.0, 1.10, 0.0, identity));
    Enable(standing, phantom::BodyRole::Waist);
    Enable(crouch, phantom::BodyRole::Waist);

    const auto standingResult = solver.Solve(standing);
    const auto crouchResult = solver.Solve(crouch);
    const auto& standingWaist =
        standingResult.roles[static_cast<uint8_t>(phantom::BodyRole::Waist)];
    const auto& crouchWaist =
        crouchResult.roles[static_cast<uint8_t>(phantom::BodyRole::Waist)];
    ASSERT_TRUE(standingWaist.valid);
    ASSERT_TRUE(crouchWaist.valid);
    EXPECT_LT(crouchWaist.pose.position[1], standingWaist.pose.position[1]);
}

TEST(BodyCompletionSolverTest, ControllerPoseRaisesElbowConfidence)
{
    phantom::BodyCompletionSolver solver;
    phantom::BodyCompletionInput input;
    const double identity[4] = {1.0, 0.0, 0.0, 0.0};
    input.hmd = Sensor(MakePose(0.0, 1.70, 0.0, identity));
    input.left_controller = Sensor(MakePose(-0.35, 1.20, 0.15, identity));
    Enable(input, phantom::BodyRole::LeftElbow);

    const auto result = solver.Solve(input);
    const auto& elbow = result.roles[static_cast<uint8_t>(phantom::BodyRole::LeftElbow)];
    ASSERT_TRUE(elbow.valid);
    EXPECT_TRUE((elbow.source_mask & phantom::kBodySourceController) != 0);
    EXPECT_EQ(elbow.mode, phantom::BodyCompletionMode::ControllerIk);
    EXPECT_GT(elbow.confidence, 0.50f);
}

TEST(BodyCompletionSolverTest, MeasuredRoleOverridesInference)
{
    phantom::BodyCompletionSolver solver;
    phantom::BodyCompletionInput input;
    const double identity[4] = {1.0, 0.0, 0.0, 0.0};
    input.hmd = Sensor(MakePose(0.0, 1.70, 0.0, identity));
    input.real_roles[static_cast<uint8_t>(phantom::BodyRole::LeftFoot)] =
        Sensor(MakePose(1.0, 0.22, 3.0, identity));
    Enable(input, phantom::BodyRole::LeftFoot);

    const auto result = solver.Solve(input);
    const auto& foot = result.roles[static_cast<uint8_t>(phantom::BodyRole::LeftFoot)];
    ASSERT_TRUE(foot.valid);
    EXPECT_EQ(foot.mode, phantom::BodyCompletionMode::Measured);
    EXPECT_TRUE((foot.source_mask & phantom::kBodySourceMeasured) != 0);
    EXPECT_NEAR(foot.pose.position[0], 1.0, 1e-9);
    EXPECT_NEAR(foot.pose.position[1], 0.22, 1e-9);
    EXPECT_NEAR(foot.pose.position[2], 3.0, 1e-9);
    EXPECT_GT(foot.confidence, 0.90f);
}

TEST(BodyCompletionSolverTest, FootLockReleasesWhenHeadMotionContradictsPlant)
{
    phantom::BodyCompletionSolver solver;
    phantom::BodyCompletionInput input;
    const double identity[4] = {1.0, 0.0, 0.0, 0.0};
    input.hmd = Sensor(MakePose(0.0, 1.70, 0.0, identity));
    Enable(input, phantom::BodyRole::LeftFoot);

    auto first = solver.Solve(input);
    const auto& firstFoot = first.roles[static_cast<uint8_t>(phantom::BodyRole::LeftFoot)];
    ASSERT_TRUE(firstFoot.valid);
    EXPECT_EQ(firstFoot.mode, phantom::BodyCompletionMode::HeldContact);

    input.hmd.pose.velocity[0] = 1.0;
    auto moving = solver.Solve(input);
    const auto& movingFoot = moving.roles[static_cast<uint8_t>(phantom::BodyRole::LeftFoot)];
    ASSERT_TRUE(movingFoot.valid);
    EXPECT_EQ(movingFoot.mode, phantom::BodyCompletionMode::FloorContact);
    EXPECT_TRUE((movingFoot.source_mask & phantom::kBodySourceHeld) == 0);
}

TEST(BodyCompletionSolverTest, ImpossibleControllerReachDegradesConfidence)
{
    phantom::BodyCompletionSolver solver;
    phantom::BodyCompletionInput input;
    const double identity[4] = {1.0, 0.0, 0.0, 0.0};
    input.hmd = Sensor(MakePose(0.0, 1.70, 0.0, identity));
    input.left_controller = Sensor(MakePose(-4.0, 4.0, 4.0, identity));
    Enable(input, phantom::BodyRole::LeftElbow);

    const auto result = solver.Solve(input);
    const auto& elbow = result.roles[static_cast<uint8_t>(phantom::BodyRole::LeftElbow)];
    ASSERT_TRUE(elbow.valid);
    EXPECT_EQ(elbow.mode, phantom::BodyCompletionMode::LowConfidence);
    EXPECT_LT(elbow.confidence, 0.25f);
}
