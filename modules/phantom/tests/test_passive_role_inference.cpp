#include "PassiveRoleInference.h"

#include <gtest/gtest.h>

#include <vector>

using namespace phantom;

namespace {

TrackerMotionFeatures Feat(double height, double lateral, double vmotion, uint32_t samples = 1000)
{
	TrackerMotionFeatures f;
	f.height_ratio = height;
	f.lateral_norm = lateral;
	f.forward_norm = 0.0;
	f.vert_motion_norm = vmotion;
	f.sample_count = samples;
	f.has_data = true;
	return f;
}

RoleAssignment AssignmentFor(const std::vector<RoleAssignment>& result, int tracker)
{
	for (const auto& a : result) {
		if (a.tracker_index == tracker) {
			return a;
		}
	}
	return RoleAssignment{};
}

} // namespace

// A clean 3-point tracker set (waist + two feet) maps each tracker to the right role.
TEST(PassiveRoleInference, AssignsThreePointSetup)
{
	std::vector<TrackerMotionFeatures> trackers = {
	    Feat(0.53, 0.00, 0.10),  // waist
	    Feat(0.06, -0.12, 0.45), // left foot
	    Feat(0.06, 0.12, 0.45),  // right foot
	};
	std::vector<BodyRole> roles = {BodyRole::Waist, BodyRole::LeftFoot, BodyRole::RightFoot};

	auto result = InferRoles(trackers, roles);

	EXPECT_EQ(AssignmentFor(result, 0).role, BodyRole::Waist);
	EXPECT_EQ(AssignmentFor(result, 1).role, BodyRole::LeftFoot);
	EXPECT_EQ(AssignmentFor(result, 2).role, BodyRole::RightFoot);
	for (int i = 0; i < 3; ++i) {
		EXPECT_GE(AssignmentFor(result, i).confidence, 0.35f) << "tracker " << i;
	}
}

// Left/right must not be swapped: the tracker on the head's left gets the left
// role even though both feet share the same height prior.
TEST(PassiveRoleInference, DoesNotSwapLeftRight)
{
	std::vector<TrackerMotionFeatures> trackers = {
	    Feat(0.06, 0.13, 0.45),  // clearly to the right
	    Feat(0.06, -0.13, 0.45), // clearly to the left
	};
	std::vector<BodyRole> roles = {BodyRole::LeftFoot, BodyRole::RightFoot};

	auto result = InferRoles(trackers, roles);

	EXPECT_EQ(AssignmentFor(result, 0).role, BodyRole::RightFoot);
	EXPECT_EQ(AssignmentFor(result, 1).role, BodyRole::LeftFoot);
}

// A full six-point tracker set (waist, chest, two feet, two knees) all resolve.
TEST(PassiveRoleInference, AssignsSixPointSetup)
{
	std::vector<TrackerMotionFeatures> trackers = {
	    Feat(0.53, 0.00, 0.10),  // waist
	    Feat(0.74, 0.00, 0.10),  // chest
	    Feat(0.06, -0.12, 0.45), // left foot
	    Feat(0.06, 0.12, 0.45),  // right foot
	    Feat(0.28, -0.10, 0.30), // left knee
	    Feat(0.28, 0.10, 0.30),  // right knee
	};
	std::vector<BodyRole> roles = {BodyRole::Waist,     BodyRole::Chest,    BodyRole::LeftFoot,
	                               BodyRole::RightFoot, BodyRole::LeftKnee, BodyRole::RightKnee};

	auto result = InferRoles(trackers, roles);

	EXPECT_EQ(AssignmentFor(result, 0).role, BodyRole::Waist);
	EXPECT_EQ(AssignmentFor(result, 1).role, BodyRole::Chest);
	EXPECT_EQ(AssignmentFor(result, 2).role, BodyRole::LeftFoot);
	EXPECT_EQ(AssignmentFor(result, 3).role, BodyRole::RightFoot);
	EXPECT_EQ(AssignmentFor(result, 4).role, BodyRole::LeftKnee);
	EXPECT_EQ(AssignmentFor(result, 5).role, BodyRole::RightKnee);
}

TEST(PassiveRoleInference, CleanFullBodySetupClearsAutoAcceptThreshold)
{
	std::vector<TrackerMotionFeatures> trackers = {
	    Feat(0.53, 0.00, 0.10),  // waist
	    Feat(0.74, 0.00, 0.10),  // chest
	    Feat(0.06, -0.12, 0.45), // left foot
	    Feat(0.06, 0.12, 0.45),  // right foot
	    Feat(0.28, -0.10, 0.30), // left knee
	    Feat(0.28, 0.10, 0.30),  // right knee
	    Feat(0.63, -0.18, 0.25), // left elbow
	    Feat(0.63, 0.18, 0.25),  // right elbow
	};
	std::vector<BodyRole> roles = {BodyRole::Waist,    BodyRole::Chest,     BodyRole::LeftFoot,  BodyRole::RightFoot,
	                               BodyRole::LeftKnee, BodyRole::RightKnee, BodyRole::LeftElbow, BodyRole::RightElbow};

	auto result = InferRoles(trackers, roles);

	for (int i = 0; i < 8; ++i) {
		EXPECT_EQ(AssignmentFor(result, i).role, roles[static_cast<size_t>(i)]);
		EXPECT_GE(AssignmentFor(result, i).confidence, 0.70f) << "tracker " << i;
	}
}

// A foot held near the centreline is ambiguous between left and right, so it
// should come back low-confidence (reported as None) rather than guessing.
TEST(PassiveRoleInference, AmbiguousLateralIsLowConfidence)
{
	std::vector<TrackerMotionFeatures> trackers = {Feat(0.06, 0.0, 0.45)};
	std::vector<BodyRole> roles = {BodyRole::LeftFoot, BodyRole::RightFoot};

	auto result = InferRoles(trackers, roles);
	EXPECT_EQ(AssignmentFor(result, 0).role, BodyRole::None);
}

// Few samples cannot reach the confidence threshold even with a clean match.
TEST(PassiveRoleInference, FewSamplesAreLowConfidence)
{
	std::vector<TrackerMotionFeatures> few = {Feat(0.53, 0.0, 0.10, /*samples=*/10)};
	std::vector<TrackerMotionFeatures> many = {Feat(0.53, 0.0, 0.10, /*samples=*/2000)};
	std::vector<BodyRole> roles = {BodyRole::Waist, BodyRole::Chest};

	float fewConf = AssignmentFor(InferRoles(few, roles), 0).confidence;
	float manyConf = AssignmentFor(InferRoles(many, roles), 0).confidence;

	EXPECT_LT(fewConf, manyConf);
	EXPECT_LT(fewConf, 0.35f);
}

// One role cannot be assigned to two trackers; a weaker leftover match must
// not become high-confidence just because the best role was already claimed.
TEST(PassiveRoleInference, AssignmentsAreOneToOne)
{
	std::vector<TrackerMotionFeatures> trackers = {
	    Feat(0.53, 0.0, 0.10), // best matches waist
	    Feat(0.60, 0.0, 0.10), // also closest to waist, but waist is taken
	};
	std::vector<BodyRole> roles = {BodyRole::Waist, BodyRole::Chest};

	auto result = InferRoles(trackers, roles);
	BodyRole a = AssignmentFor(result, 0).role;
	BodyRole b = AssignmentFor(result, 1).role;
	EXPECT_NE(a, b);
	EXPECT_TRUE(a == BodyRole::Waist || b == BodyRole::Waist);
	if (a != BodyRole::None && b != BodyRole::None) {
		EXPECT_LT(std::min(AssignmentFor(result, 0).confidence, AssignmentFor(result, 1).confidence), 0.60f);
	}
}

// The accumulator turns raw poses into the expected normalized features and
// gets the left/right sign right off the HMD's horizontal right axis.
TEST(PassiveRoleInference, AccumulatorComputesFeatures)
{
	RoleInferenceAccumulator acc;
	const double hmd[3] = {0.0, 1.70, 0.0};
	const double right[2] = {1.0, 0.0}; // head facing -Z, right is +X
	const double fwd[2] = {0.0, -1.0};

	// A tracker 0.34 m up and 0.17 m to the head's right.
	const double tracker[3] = {0.17, 0.34, 0.0};
	for (int i = 0; i < 100; ++i) {
		acc.AddSample(hmd, right, fwd, tracker);
	}

	TrackerMotionFeatures f = acc.Compute();
	EXPECT_TRUE(f.has_data);
	EXPECT_EQ(f.sample_count, 100u);
	EXPECT_NEAR(f.height_ratio, 0.34 / 1.70, 1e-6);
	EXPECT_NEAR(f.lateral_norm, 0.17 / 1.70, 1e-6); // positive => right side
	EXPECT_NEAR(f.vert_motion_norm, 0.0, 1e-6);     // stationary
}

// A tracker on the head's left yields a negative lateral feature.
TEST(PassiveRoleInference, AccumulatorLeftSideIsNegative)
{
	RoleInferenceAccumulator acc;
	const double hmd[3] = {0.0, 1.70, 0.0};
	const double right[2] = {1.0, 0.0};
	const double fwd[2] = {0.0, -1.0};
	const double tracker[3] = {-0.20, 0.10, 0.0};
	acc.AddSample(hmd, right, fwd, tracker);

	TrackerMotionFeatures f = acc.Compute();
	EXPECT_LT(f.lateral_norm, 0.0);
}

TEST(PassiveRoleInference, WrongHeightCannotAutoAdoptAsTorso)
{
	std::vector<TrackerMotionFeatures> trackers = {
	    Feat(0.06, 0.12, 0.45), // a foot-like tracker
	};
	std::vector<BodyRole> roles = {BodyRole::Waist, BodyRole::Chest};

	auto result = InferRoles(trackers, roles);
	const auto assignment = AssignmentFor(result, 0);
	EXPECT_TRUE(assignment.role == BodyRole::None || assignment.confidence < 0.60f);
}

TEST(PassiveRoleInference, MidlineFeetDoNotBecomeHighConfidenceWrongSide)
{
	std::vector<TrackerMotionFeatures> trackers = {
	    Feat(0.06, -0.01, 0.45),
	    Feat(0.06, 0.01, 0.45),
	};
	std::vector<BodyRole> roles = {BodyRole::LeftFoot, BodyRole::RightFoot};

	auto result = InferRoles(trackers, roles);
	for (int i = 0; i < 2; ++i) {
		const auto assignment = AssignmentFor(result, i);
		EXPECT_LT(assignment.confidence, 0.60f) << "tracker " << i;
	}
}
