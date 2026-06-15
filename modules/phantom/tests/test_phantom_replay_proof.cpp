#include <gtest/gtest.h>

#include "BodyCompletionSolver.h"
#include "BlendCurves.h"
#include "DeadReckoner.h"
#include "DropoutState.h"
#include "PassiveRoleInference.h"
#include "PoseHistory.h"

#include <openvr_driver.h>

#include <array>
#include <cmath>
#include <vector>

namespace {

constexpr int64_t kQpcFreq = 10000000;

int64_t Ms(int64_t ms)
{
	return ms * (kQpcFreq / 1000);
}

vr::DriverPose_t DriverPose(double x, double y, double z, double vx = 0.0, double vy = 0.0, double vz = 0.0)
{
	vr::DriverPose_t p{};
	p.qWorldFromDriverRotation = {1, 0, 0, 0};
	p.qDriverFromHeadRotation = {1, 0, 0, 0};
	p.qRotation = {1, 0, 0, 0};
	p.vecPosition[0] = x;
	p.vecPosition[1] = y;
	p.vecPosition[2] = z;
	p.vecVelocity[0] = vx;
	p.vecVelocity[1] = vy;
	p.vecVelocity[2] = vz;
	p.poseIsValid = true;
	p.deviceIsConnected = true;
	p.result = vr::TrackingResult_Running_OK;
	return p;
}

phantom::BodyCompletionPose BodyPose(double x, double y, double z)
{
	phantom::BodyCompletionPose p;
	p.position[0] = x;
	p.position[1] = y;
	p.position[2] = z;
	p.rotation[0] = 1.0;
	return p;
}

phantom::BodyCompletionSensorPose Sensor(const phantom::BodyCompletionPose& pose)
{
	phantom::BodyCompletionSensorPose s;
	s.pose = pose;
	s.valid = true;
	return s;
}

void Enable(phantom::BodyCompletionInput& input, phantom::BodyRole role)
{
	input.enabled_roles[static_cast<uint8_t>(role)] = true;
}

double PositionErrorM(const phantom::BodyCompletionRoleOutput& actual, const phantom::BodyCompletionPose& expected)
{
	const double dx = actual.pose.position[0] - expected.position[0];
	const double dy = actual.pose.position[1] - expected.position[1];
	const double dz = actual.pose.position[2] - expected.position[2];
	return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double PositionErrorM(const vr::DriverPose_t& actual, const vr::DriverPose_t& expected)
{
	const double dx = actual.vecPosition[0] - expected.vecPosition[0];
	const double dy = actual.vecPosition[1] - expected.vecPosition[1];
	const double dz = actual.vecPosition[2] - expected.vecPosition[2];
	return std::sqrt(dx * dx + dy * dy + dz * dz);
}

phantom::TrackerMotionFeatures Feature(double height, double lateral, double vertical_motion)
{
	phantom::TrackerMotionFeatures f;
	f.height_ratio = height;
	f.lateral_norm = lateral;
	f.forward_norm = 0.0;
	f.vert_motion_norm = vertical_motion;
	f.sample_count = 1200;
	f.has_data = true;
	return f;
}

phantom::RoleAssignment AssignmentFor(const std::vector<phantom::RoleAssignment>& result, int tracker)
{
	for (const auto& assignment : result) {
		if (assignment.tracker_index == tracker) return assignment;
	}
	return phantom::RoleAssignment{};
}

} // namespace

TEST(PhantomReplayProof, DropoutBridgesMovingTrackerAgainstRecordedGroundTruth)
{
	phantom::PoseHistory history;
	phantom::DropoutState ladder;
	phantom::DeadReckoner reckoner;
	ladder.SetTimings(phantom::LadderTimings::Defaults());

	const auto last_real = DriverPose(0.0, 0.08, 0.02, 0.25, 0.0, 0.0);
	history.Push(Ms(0), last_real);
	ladder.OnRealPoseObserved(Ms(0), history, last_real);

	const int64_t replay_ms = 180;
	ladder.Tick(Ms(replay_ms), kQpcFreq);
	ASSERT_EQ(ladder.state(), phantom::TrackerState::SYNTH_RECKON);

	vr::DriverPose_t synthetic{};
	ASSERT_TRUE(reckoner.Project(history, kQpcFreq, Ms(replay_ms), synthetic));

	const auto recorded_ground_truth = DriverPose(0.25 * (replay_ms / 1000.0), 0.08, 0.02, 0.25, 0.0, 0.0);
	EXPECT_LT(PositionErrorM(synthetic, recorded_ground_truth), 0.04);
	EXPECT_EQ(synthetic.result, vr::TrackingResult_Running_OK);

	ladder.Tick(Ms(phantom::DefaultTimings::kSynthHoldMs + 50), kQpcFreq);
	EXPECT_EQ(ladder.tracking_result_override(), vr::TrackingResult_Running_OutOfRange);
}

TEST(PhantomReplayProof, PassiveInferenceAutoMapsCleanFullBodyRecording)
{
	std::vector<phantom::TrackerMotionFeatures> recorded = {
	    Feature(0.53, 0.00, 0.10),  Feature(0.74, 0.00, 0.10), Feature(0.06, -0.12, 0.45), Feature(0.06, 0.12, 0.45),
	    Feature(0.28, -0.10, 0.30), Feature(0.28, 0.10, 0.30), Feature(0.63, -0.18, 0.25), Feature(0.63, 0.18, 0.25),
	};
	std::vector<phantom::BodyRole> roles = {phantom::BodyRole::Waist,     phantom::BodyRole::Chest,
	                                        phantom::BodyRole::LeftFoot,  phantom::BodyRole::RightFoot,
	                                        phantom::BodyRole::LeftKnee,  phantom::BodyRole::RightKnee,
	                                        phantom::BodyRole::LeftElbow, phantom::BodyRole::RightElbow};

	const auto assignments = phantom::InferRoles(recorded, roles);

	for (int i = 0; i < static_cast<int>(roles.size()); ++i) {
		const auto assignment = AssignmentFor(assignments, i);
		EXPECT_EQ(assignment.role, roles[static_cast<size_t>(i)]);
		EXPECT_GE(assignment.confidence, 0.70f) << "tracker " << i;
	}
}

TEST(PhantomReplayProof, HmdAndHandsFillMissingBodyTrackersNearCalibratedReference)
{
	phantom::BodyCompletionSolver solver;
	phantom::BodyCompletionInput input;
	input.calibration.height_m = 1.70;
	input.calibration.floor_y_m = 0.0;
	input.calibration.forward_calibrated = true;
	input.calibration.forward_yaw_rad = 0.0;
	input.calibration.stance_width_m = 0.30;
	input.calibration.shoulder_width_m = 0.40;
	input.calibration.pelvis_width_m = 0.30;
	input.calibration.upper_arm_m = 0.31;
	input.calibration.lower_arm_m = 0.27;
	input.calibration.upper_leg_m = 0.45;
	input.calibration.lower_leg_m = 0.45;

	input.hmd = Sensor(BodyPose(0.0, 1.70, 0.0));
	input.left_controller = Sensor(BodyPose(-0.45, 1.20, 0.18));
	input.right_controller = Sensor(BodyPose(0.45, 1.20, 0.18));

	for (phantom::BodyRole role :
	     {phantom::BodyRole::Waist, phantom::BodyRole::Chest, phantom::BodyRole::LeftFoot, phantom::BodyRole::RightFoot,
	      phantom::BodyRole::LeftKnee, phantom::BodyRole::RightKnee, phantom::BodyRole::LeftElbow,
	      phantom::BodyRole::RightElbow}) {
		Enable(input, role);
	}

	const auto result = solver.Solve(input);

	struct ExpectedRole
	{
		phantom::BodyRole role;
		phantom::BodyCompletionPose pose;
		double max_error_m;
		float min_confidence;
	};
	const std::array<ExpectedRole, 8> expected = {{
	    {phantom::BodyRole::Waist, BodyPose(0.0, 0.92, -0.05), 0.08, 0.40f},
	    {phantom::BodyRole::Chest, BodyPose(0.0, 1.33, -0.03), 0.08, 0.50f},
	    {phantom::BodyRole::LeftFoot, BodyPose(-0.15, 0.04, -0.03), 0.08, 0.20f},
	    {phantom::BodyRole::RightFoot, BodyPose(0.15, 0.04, -0.03), 0.08, 0.20f},
	    {phantom::BodyRole::LeftKnee, BodyPose(-0.15, 0.47, 0.04), 0.12, 0.20f},
	    {phantom::BodyRole::RightKnee, BodyPose(0.15, 0.47, 0.04), 0.12, 0.20f},
	    {phantom::BodyRole::LeftElbow, BodyPose(-0.32, 1.31, 0.08), 0.16, 0.50f},
	    {phantom::BodyRole::RightElbow, BodyPose(0.32, 1.31, 0.08), 0.16, 0.50f},
	}};

	for (const auto& e : expected) {
		const auto& out = result.roles[static_cast<uint8_t>(e.role)];
		ASSERT_TRUE(out.valid) << phantom::BodyRoleToKey(e.role);
		EXPECT_LT(PositionErrorM(out, e.pose), e.max_error_m) << phantom::BodyRoleToKey(e.role);
		EXPECT_GE(out.confidence, e.min_confidence) << phantom::BodyRoleToKey(e.role);
	}
	EXPECT_GT(result.global_confidence, 0.35f);
}
