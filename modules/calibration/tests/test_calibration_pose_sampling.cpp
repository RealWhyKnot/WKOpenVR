// Tests for the head-mount tracker pose-sampling helpers in
// HeadMountPoseSampling.h. Verifies the four contracts the plan requires:
//
// _USE_MATH_DEFINES must appear before any math header on MSVC to get M_PI.
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#include <cmath>
//   1. Off mode: target sample selection unchanged (reads from targetID).
//   2. AutoPaired: target sample comes from head-mount deviceID after offset calibration.
//   3. AutoPaired with invalid tracker: sampling refuses (no sample, no crash).
//   4. AssignTargets resolution: FindDevice matches serial -> deviceID.
//
// All tested logic lives in HeadMountPoseSampling.h (pure helpers) and
// VRState::FindDevice. No live VR runtime is required.

#include "HeadMountPoseSampling.h"

#include <gtest/gtest.h>

#include <Eigen/Geometry>

namespace hm = spacecal::headmount;

namespace {

// Build a valid DriverPose_t at a given world position with an identity
// worldFromDriver transform so ConvertPose / ComputeHeadWorldPose see the
// position directly.
vr::DriverPose_t MakeValidPoseAt(double x, double y, double z)
{
	vr::DriverPose_t p{};
	p.poseIsValid = true;
	p.deviceIsConnected = true;
	p.result = vr::ETrackingResult::TrackingResult_Running_OK;
	p.qWorldFromDriverRotation = {1.0, 0.0, 0.0, 0.0}; // identity
	p.qRotation = {1.0, 0.0, 0.0, 0.0};                // identity
	p.vecPosition[0] = x;
	p.vecPosition[1] = y;
	p.vecPosition[2] = z;
	return p;
}

// Build a DriverPose_t with poseIsValid == false.
vr::DriverPose_t MakeInvalidPose()
{
	vr::DriverPose_t p{};
	p.poseIsValid = false;
	p.deviceIsConnected = true;
	p.result = vr::ETrackingResult::TrackingResult_Running_OK;
	return p;
}

} // namespace

// ---------------------------------------------------------------------------
// IsTrackerValidForSampling
// ---------------------------------------------------------------------------

TEST(HeadMountSamplingTest, OffModeIsNeverValid)
{
	vr::DriverPose_t poses[4];
	poses[1] = MakeValidPoseAt(0, 0, 0);

	HeadMountConfig cfg;
	cfg.mode = HeadMountMode::Off;
	cfg.deviceID = 1;

	EXPECT_FALSE(hm::IsTrackerValidForSampling(cfg, poses, 4));
}

TEST(HeadMountSamplingTest, AutoPairedWithValidTrackerIsValid)
{
	vr::DriverPose_t poses[4];
	poses[2] = MakeValidPoseAt(0.1, 0.2, 0.3);

	HeadMountConfig cfg;
	cfg.mode = HeadMountMode::AutoPaired;
	cfg.deviceID = 2;
	cfg.offsetCalibrated = true;

	EXPECT_TRUE(hm::IsTrackerValidForSampling(cfg, poses, 4));
}

TEST(HeadMountSamplingTest, AutoPairedWithoutOffsetIsNotValid)
{
	vr::DriverPose_t poses[4];
	poses[2] = MakeValidPoseAt(0.1, 0.2, 0.3);

	HeadMountConfig cfg;
	cfg.mode = HeadMountMode::AutoPaired;
	cfg.deviceID = 2;
	cfg.offsetCalibrated = false;

	EXPECT_FALSE(hm::IsTrackerValidForSampling(cfg, poses, 4));
}

TEST(HeadMountSamplingTest, AutoPairedWithInvalidTrackerIsNotValid)
{
	vr::DriverPose_t poses[4];
	poses[2] = MakeInvalidPose();

	HeadMountConfig cfg;
	cfg.mode = HeadMountMode::AutoPaired;
	cfg.deviceID = 2;
	cfg.offsetCalibrated = true;

	EXPECT_FALSE(hm::IsTrackerValidForSampling(cfg, poses, 4));
}

TEST(HeadMountSamplingTest, NegativeDeviceIDIsNotValid)
{
	vr::DriverPose_t poses[4];

	HeadMountConfig cfg;
	cfg.mode = HeadMountMode::AutoPaired;
	cfg.deviceID = -1;
	cfg.offsetCalibrated = true;

	EXPECT_FALSE(hm::IsTrackerValidForSampling(cfg, poses, 4));
}

TEST(HeadMountSamplingTest, DeviceIDOutOfRangeIsNotValid)
{
	vr::DriverPose_t poses[4];

	HeadMountConfig cfg;
	cfg.mode = HeadMountMode::AutoPaired;
	cfg.deviceID = 10; // >= poseArraySize (4)
	cfg.offsetCalibrated = true;

	EXPECT_FALSE(hm::IsTrackerValidForSampling(cfg, poses, 4));
}

// ---------------------------------------------------------------------------
// ComputeHeadWorldPose -- identity offset (tracker IS the head point)
// ---------------------------------------------------------------------------

// With an identity headFromTracker, the derived head world position must equal
// the tracker's world position exactly.
TEST(HeadMountSamplingTest, IdentityOffsetPreservesTrackerPosition)
{
	vr::DriverPose_t tracker = MakeValidPoseAt(1.0, 2.0, 3.0);

	Eigen::AffineCompact3d identity = Eigen::AffineCompact3d::Identity();
	Eigen::AffineCompact3d result = hm::ComputeHeadWorldPose(tracker, identity);

	EXPECT_NEAR(result.translation().x(), 1.0, 1e-10);
	EXPECT_NEAR(result.translation().y(), 2.0, 1e-10);
	EXPECT_NEAR(result.translation().z(), 3.0, 1e-10);
}

// With a pure translation offset (0.05 m forward in the tracker's local frame),
// the head world position must be shifted by that amount when the tracker
// orientation is identity (no rotation, so local == world).
TEST(HeadMountSamplingTest, PureTranslationOffsetAppliedCorrectly)
{
	vr::DriverPose_t tracker = MakeValidPoseAt(1.0, 0.0, 0.0);

	Eigen::AffineCompact3d offset = Eigen::AffineCompact3d::Identity();
	offset.translation() = Eigen::Vector3d(0.05, 0.0, 0.0); // 5 cm forward

	Eigen::AffineCompact3d result = hm::ComputeHeadWorldPose(tracker, offset);

	EXPECT_NEAR(result.translation().x(), 1.05, 1e-10);
	EXPECT_NEAR(result.translation().y(), 0.0, 1e-10);
	EXPECT_NEAR(result.translation().z(), 0.0, 1e-10);
}

// AutoPairedReadsFromHeadMountDevice: verify the offset is composed in the
// tracker's frame (not world frame). Rotate the tracker 90 degrees about Y,
// then a local-X offset should appear as a world-Z shift.
TEST(HeadMountSamplingTest, OffsetComposedInTrackerFrame)
{
	vr::DriverPose_t tracker{};
	tracker.poseIsValid = true;
	tracker.deviceIsConnected = true;
	tracker.result = vr::ETrackingResult::TrackingResult_Running_OK;
	// worldFromDriver identity; rotate local 90 deg about Y.
	tracker.qWorldFromDriverRotation = {1.0, 0.0, 0.0, 0.0};
	// 90 deg about Y: w=cos(45deg), x=0, y=sin(45deg), z=0
	const double s = std::sin(M_PI / 4.0);
	const double c = std::cos(M_PI / 4.0);
	tracker.qRotation = {c, 0.0, s, 0.0};
	tracker.vecPosition[0] = 0.0;
	tracker.vecPosition[1] = 0.0;
	tracker.vecPosition[2] = 0.0;

	// Offset 1 m along tracker-local X.
	Eigen::AffineCompact3d offset = Eigen::AffineCompact3d::Identity();
	offset.translation() = Eigen::Vector3d(1.0, 0.0, 0.0);

	Eigen::AffineCompact3d result = hm::ComputeHeadWorldPose(tracker, offset);

	// After 90 deg Y rotation, local-X maps to world-Z (negated by right-hand rule).
	// Check that translation has ~0 on X and ~1 on Z.
	EXPECT_NEAR(result.translation().x(), 0.0, 1e-9);
	EXPECT_NEAR(result.translation().y(), 0.0, 1e-9);
	EXPECT_NEAR(std::abs(result.translation().z()), 1.0, 1e-9);
}

// ---------------------------------------------------------------------------
// ResolveHeadMountDeviceID -- FindDevice integration
// ---------------------------------------------------------------------------

// Build a minimal VRState with one tracker device and verify resolution.
TEST(HeadMountSamplingTest, AssignTargetsResolvesHeadMountSerial)
{
	VRState state;
	VRDevice dev;
	dev.id = 5;
	dev.deviceClass = vr::TrackedDeviceClass_GenericTracker;
	dev.trackingSystem = "lighthouse";
	dev.model = "vive_tracker_3";
	dev.serial = "LHR-ABCD1234";
	state.devices.push_back(dev);

	HeadMountConfig cfg;
	cfg.trackerTrackingSystem = "lighthouse";
	cfg.trackerModel = "vive_tracker_3";
	cfg.trackerSerial = "LHR-ABCD1234";

	int32_t resolved = hm::ResolveHeadMountDeviceID(cfg, state);
	EXPECT_EQ(resolved, 5);
}

// A mismatch in serial returns -1 even when tracking system and model match.
TEST(HeadMountSamplingTest, ResolveReturnsMinusOneOnSerialMismatch)
{
	VRState state;
	VRDevice dev;
	dev.id = 5;
	dev.deviceClass = vr::TrackedDeviceClass_GenericTracker;
	dev.trackingSystem = "lighthouse";
	dev.model = "vive_tracker_3";
	dev.serial = "LHR-ABCD1234";
	state.devices.push_back(dev);

	HeadMountConfig cfg;
	cfg.trackerTrackingSystem = "lighthouse";
	cfg.trackerModel = "vive_tracker_3";
	cfg.trackerSerial = "LHR-WRONGSERIAL";

	int32_t resolved = hm::ResolveHeadMountDeviceID(cfg, state);
	EXPECT_EQ(resolved, -1);
}

// Empty serial always returns -1 (guard against accidental empty-string matches).
TEST(HeadMountSamplingTest, EmptySerialNeverResolves)
{
	VRState state;
	VRDevice dev;
	dev.id = 3;
	dev.deviceClass = vr::TrackedDeviceClass_GenericTracker;
	dev.trackingSystem = "lighthouse";
	dev.model = "";
	dev.serial = "";
	state.devices.push_back(dev);

	HeadMountConfig cfg;
	cfg.trackerTrackingSystem = "lighthouse";
	cfg.trackerModel = "";
	cfg.trackerSerial = "";

	int32_t resolved = hm::ResolveHeadMountDeviceID(cfg, state);
	EXPECT_EQ(resolved, -1);
}

// ---------------------------------------------------------------------------
// AutoPairedFallsBackOnInvalidTracker -- IsTrackerValidForSampling gate
// ---------------------------------------------------------------------------

// When the tracker's result is not Running_OK, it must not be considered valid
// even if poseIsValid happens to be set (defensive: some runtimes set both).
TEST(HeadMountSamplingTest, NonRunningOkResultIsInvalid)
{
	vr::DriverPose_t poses[4];
	poses[2] = MakeValidPoseAt(0, 0, 0);
	poses[2].result = vr::ETrackingResult::TrackingResult_Calibrating_InProgress;

	HeadMountConfig cfg;
	cfg.mode = HeadMountMode::AutoPaired;
	cfg.deviceID = 2;
	cfg.offsetCalibrated = true;

	EXPECT_FALSE(hm::IsTrackerValidForSampling(cfg, poses, 4));
}

TEST(HeadMountSamplingTest, DriverSynthReadyKeepsContinuousSolverRunning)
{
	vr::DriverPose_t poses[4];
	poses[0] = MakeValidPoseAt(0.0, 1.7, 0.0);
	poses[2] = MakeValidPoseAt(0.0, 1.7, 0.0);

	HeadMountConfig cfg;
	cfg.mode = HeadMountMode::DriverSynth;
	cfg.deviceID = 2;
	cfg.offsetCalibrated = true;
	cfg.headFromTracker = Eigen::AffineCompact3d::Identity();

	const auto status = hm::EvaluateDriverSynthContinuousStatus(cfg,
	                                                            /*inContinuous=*/true,
	                                                            /*targetMatchesHeadMount=*/true, poses, 4);

	EXPECT_TRUE(status.ready);
	EXPECT_STREQ(status.reason, "driver_synth_ready");
	EXPECT_NEAR(status.hmdProxyDeltaM, 0.0, 1e-10);
}

TEST(HeadMountSamplingTest, AutoPairedNotDriverSynthStatus)
{
	vr::DriverPose_t poses[4];
	poses[0] = MakeValidPoseAt(0.0, 1.7, 0.0);
	poses[2] = MakeValidPoseAt(0.0, 1.7, 0.0);

	HeadMountConfig cfg;
	cfg.mode = HeadMountMode::AutoPaired;
	cfg.deviceID = 2;
	cfg.offsetCalibrated = true;

	const auto status = hm::EvaluateDriverSynthContinuousStatus(cfg,
	                                                            /*inContinuous=*/true,
	                                                            /*targetMatchesHeadMount=*/true, poses, 4);

	EXPECT_FALSE(status.ready);
	EXPECT_STREQ(status.reason, "mode_not_driver_synth");
}

TEST(HeadMountSamplingTest, DriverSynthTargetMismatchNotReady)
{
	vr::DriverPose_t poses[4];
	poses[0] = MakeValidPoseAt(0.0, 1.7, 0.0);
	poses[2] = MakeValidPoseAt(0.0, 1.7, 0.0);

	HeadMountConfig cfg;
	cfg.mode = HeadMountMode::DriverSynth;
	cfg.deviceID = 2;
	cfg.offsetCalibrated = true;

	const auto status = hm::EvaluateDriverSynthContinuousStatus(cfg,
	                                                            /*inContinuous=*/true,
	                                                            /*targetMatchesHeadMount=*/false, poses, 4);

	EXPECT_FALSE(status.ready);
	EXPECT_STREQ(status.reason, "target_mismatch");
}

TEST(HeadMountSamplingTest, DriverSynthLargeHeadDeltaStillKeepsSolverRunning)
{
	vr::DriverPose_t poses[4];
	poses[0] = MakeValidPoseAt(0.0, 1.7, 0.0);
	poses[2] = MakeValidPoseAt(2.0, 1.7, 0.0);

	HeadMountConfig cfg;
	cfg.mode = HeadMountMode::DriverSynth;
	cfg.deviceID = 2;
	cfg.offsetCalibrated = true;
	cfg.headFromTracker = Eigen::AffineCompact3d::Identity();

	const auto status = hm::EvaluateDriverSynthContinuousStatus(cfg,
	                                                            /*inContinuous=*/true,
	                                                            /*targetMatchesHeadMount=*/true, poses, 4);

	EXPECT_TRUE(status.ready);
	EXPECT_STREQ(status.reason, "driver_synth_ready");
	EXPECT_NEAR(status.hmdProxyDeltaM, 2.0, 1e-10);
}
