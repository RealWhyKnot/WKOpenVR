#include "quash/QuashPose.h"

#include <gtest/gtest.h>

namespace {

// Build a pose with non-default values across as many fields as practical
// so each test can verify ApplyQuashToPose preserves them while only adding
// the position offset. The new offset design is intentionally minimal --
// rotation, velocity, validity, tracking result all pass through untouched.
vr::DriverPose_t MakeBusyPose()
{
	vr::DriverPose_t p{};
	p.qWorldFromDriverRotation = {0.5, 0.5, 0.5, 0.5};
	p.qDriverFromHeadRotation = {0.0, 1.0, 0.0, 0.0};
	p.qRotation = {0.7, 0.0, 0.7, 0.0};
	p.vecWorldFromDriverTranslation[0] = 1.0;
	p.vecWorldFromDriverTranslation[1] = 2.0;
	p.vecWorldFromDriverTranslation[2] = 3.0;
	p.vecDriverFromHeadTranslation[0] = 4.0;
	p.vecDriverFromHeadTranslation[1] = 5.0;
	p.vecDriverFromHeadTranslation[2] = 6.0;
	p.vecPosition[0] = 10.0;
	p.vecPosition[1] = 5.0;
	p.vecPosition[2] = -3.0;
	p.vecVelocity[0] = 2.0;
	p.vecVelocity[1] = -1.0;
	p.vecVelocity[2] = 0.5;
	p.vecAcceleration[0] = 0.1;
	p.vecAcceleration[1] = 9.81;
	p.vecAcceleration[2] = -0.2;
	p.vecAngularVelocity[0] = 0.3;
	p.vecAngularVelocity[1] = 0.4;
	p.vecAngularVelocity[2] = 1.5;
	p.vecAngularAcceleration[0] = 0.01;
	p.vecAngularAcceleration[1] = 0.02;
	p.vecAngularAcceleration[2] = 0.03;
	p.poseTimeOffset = 0.05;
	p.poseIsValid = true;
	p.deviceIsConnected = true;
	p.result = vr::TrackingResult_Running_OK;
	p.shouldApplyHeadModel = true;
	p.willDriftInYaw = true;
	return p;
}

} // namespace

// Position is translated additively by the hide-offset vector. The pose's
// real X / Y / Z values are still present; we add 10 km on X and Z so the
// rendered model lives ~14 km from the play space origin without losing the
// natural per-frame motion.
TEST(QuashPoseTest, AddsFixedOffsetToPosition)
{
	vr::DriverPose_t pose = MakeBusyPose();
	const double originalX = pose.vecPosition[0];
	const double originalY = pose.vecPosition[1];
	const double originalZ = pose.vecPosition[2];

	openvr_pair::common::quash::ApplyQuashToPose(pose);

	EXPECT_DOUBLE_EQ(pose.vecPosition[0], originalX + openvr_pair::common::quash::kQuashOffsetX);
	EXPECT_DOUBLE_EQ(pose.vecPosition[1], originalY + openvr_pair::common::quash::kQuashOffsetY);
	EXPECT_DOUBLE_EQ(pose.vecPosition[2], originalZ + openvr_pair::common::quash::kQuashOffsetZ);
}

// Rotation, velocity, acceleration, angular velocity / acceleration, the
// driver-relative transform fields, the time offset, and the validity flags
// are all preserved. The old design zeroed them; the new design leaves the
// tracker's motion intact so downstream timeout / liveness gates don't
// misfire on what looks like a frozen pose.
TEST(QuashPoseTest, PreservesRotationVelocityAndValidity)
{
	vr::DriverPose_t pose = MakeBusyPose();
	const vr::DriverPose_t original = pose;

	openvr_pair::common::quash::ApplyQuashToPose(pose);

	EXPECT_DOUBLE_EQ(pose.qRotation.w, original.qRotation.w);
	EXPECT_DOUBLE_EQ(pose.qRotation.x, original.qRotation.x);
	EXPECT_DOUBLE_EQ(pose.qRotation.y, original.qRotation.y);
	EXPECT_DOUBLE_EQ(pose.qRotation.z, original.qRotation.z);
	EXPECT_DOUBLE_EQ(pose.qWorldFromDriverRotation.w, original.qWorldFromDriverRotation.w);
	EXPECT_DOUBLE_EQ(pose.qDriverFromHeadRotation.x, original.qDriverFromHeadRotation.x);
	for (int i = 0; i < 3; ++i) {
		EXPECT_DOUBLE_EQ(pose.vecVelocity[i], original.vecVelocity[i]);
		EXPECT_DOUBLE_EQ(pose.vecAcceleration[i], original.vecAcceleration[i]);
		EXPECT_DOUBLE_EQ(pose.vecAngularVelocity[i], original.vecAngularVelocity[i]);
		EXPECT_DOUBLE_EQ(pose.vecAngularAcceleration[i], original.vecAngularAcceleration[i]);
		EXPECT_DOUBLE_EQ(pose.vecWorldFromDriverTranslation[i], original.vecWorldFromDriverTranslation[i]);
		EXPECT_DOUBLE_EQ(pose.vecDriverFromHeadTranslation[i], original.vecDriverFromHeadTranslation[i]);
	}
	EXPECT_DOUBLE_EQ(pose.poseTimeOffset, original.poseTimeOffset);
	EXPECT_EQ(pose.poseIsValid, original.poseIsValid);
	EXPECT_EQ(pose.deviceIsConnected, original.deviceIsConnected);
	EXPECT_EQ(pose.result, original.result);
	EXPECT_EQ(pose.shouldApplyHeadModel, original.shouldApplyHeadModel);
	EXPECT_EQ(pose.willDriftInYaw, original.willDriftInYaw);
}

// Tracking result stays Running_OK rather than the old Calibrating_OutOfRange.
// Important: downstream consumers (TrackerLiveness, phantom dropout ladder,
// SteamVR's own disconnect timer) gate on `result != Running_OK` to decide
// the tracker is in trouble. The old sentinel triggered those gates; the new
// design must NOT.
TEST(QuashPoseTest, KeepsRunningOkSoNoDownstreamTimeoutFires)
{
	vr::DriverPose_t pose{};
	pose.deviceIsConnected = true;
	pose.poseIsValid = true;
	pose.result = vr::TrackingResult_Running_OK;

	openvr_pair::common::quash::ApplyQuashToPose(pose);

	EXPECT_TRUE(pose.deviceIsConnected);
	EXPECT_TRUE(pose.poseIsValid);
	EXPECT_EQ(pose.result, vr::TrackingResult_Running_OK);
}

// The offset is the same vector regardless of input pose. Two distinct
// starting poses end up separated by the same delta they started with --
// motion is preserved on top of the translation.
TEST(QuashPoseTest, OffsetIsTranslationOnlySoMotionIsPreserved)
{
	vr::DriverPose_t a{};
	a.vecPosition[0] = 0.0;
	a.vecPosition[1] = 0.0;
	a.vecPosition[2] = 0.0;
	vr::DriverPose_t b{};
	b.vecPosition[0] = 0.5;
	b.vecPosition[1] = 0.0;
	b.vecPosition[2] = 0.0;

	openvr_pair::common::quash::ApplyQuashToPose(a);
	openvr_pair::common::quash::ApplyQuashToPose(b);

	EXPECT_DOUBLE_EQ(b.vecPosition[0] - a.vecPosition[0], 0.5);
	EXPECT_DOUBLE_EQ(b.vecPosition[1] - a.vecPosition[1], 0.0);
	EXPECT_DOUBLE_EQ(b.vecPosition[2] - a.vecPosition[2], 0.0);
}

// Applying the hide twice doubles the offset. Confirms the operation is
// additive rather than absolute -- if the same pose passes through the hide
// path twice (e.g. a driver-side double-write bug), it goes twice as far,
// not no further. Tests in the driver assert it's only applied once.
TEST(QuashPoseTest, ApplyTwiceDoublesTheOffset)
{
	vr::DriverPose_t pose{};
	pose.vecPosition[0] = 1.0;
	pose.vecPosition[2] = 2.0;

	openvr_pair::common::quash::ApplyQuashToPose(pose);
	openvr_pair::common::quash::ApplyQuashToPose(pose);

	EXPECT_DOUBLE_EQ(pose.vecPosition[0], 1.0 + 2.0 * openvr_pair::common::quash::kQuashOffsetX);
	EXPECT_DOUBLE_EQ(pose.vecPosition[2], 2.0 + 2.0 * openvr_pair::common::quash::kQuashOffsetZ);
}

// The hide-offset constants live well outside any plausible play space. This
// pins the magnitude so a future tuning change doesn't quietly shrink the
// offset to a value the user's rendered model could overlap.
TEST(QuashPoseTest, OffsetMagnitudeIsFarOutsideAnyPlaySpace)
{
	// 1 km minimum on each axis used for the hide -- arbitrary but well above
	// the largest documented chaperone (Vive: 5 m, Quest 3: 50 m on roomscale).
	EXPECT_GE(openvr_pair::common::quash::kQuashOffsetX, 1000.0);
	EXPECT_GE(openvr_pair::common::quash::kQuashOffsetZ, 1000.0);
}
