#include "quash/QuashPose.h"

#include <gtest/gtest.h>

namespace {

vr::DriverPose_t MakePose(double x, double y, double z, bool valid, bool conn,
                          vr::ETrackingResult result = vr::TrackingResult_Running_OK)
{
    vr::DriverPose_t p{};
    p.qWorldFromDriverRotation = { 1, 0, 0, 0 };
    p.qDriverFromHeadRotation  = { 1, 0, 0, 0 };
    p.qRotation                = { 1, 0, 0, 0 };
    p.vecPosition[0]           = x;
    p.vecPosition[1]           = y;
    p.vecPosition[2]           = z;
    p.poseIsValid              = valid;
    p.deviceIsConnected        = conn;
    p.result                   = result;
    return p;
}

} // namespace

// First quash with no cached last-good pose: incoming pose flows through
// (so the very first frame after a fresh device appears in a quashed slot
// has *something* to show), but the validity / connection flags are clamped
// to the OutOfRange contract.
TEST(QuashPoseTest, FirstQuashWithNoLastGoodHoldsIncomingButMarksOutOfRange)
{
    vr::DriverPose_t pose = MakePose(1.0, 1.5, 0.5, false, false,
                                     vr::TrackingResult_Running_OutOfRange);
    vr::DriverPose_t lastGood{};

    openvr_pair::common::quash::ApplyQuashToPose(pose, lastGood, /*lastGoodValid=*/false);

    EXPECT_TRUE(pose.deviceIsConnected);
    EXPECT_TRUE(pose.poseIsValid);
    EXPECT_EQ(pose.result, vr::TrackingResult_Calibrating_OutOfRange);
    EXPECT_DOUBLE_EQ(pose.vecPosition[0], 1.0);
    EXPECT_DOUBLE_EQ(pose.vecPosition[1], 1.5);
    EXPECT_DOUBLE_EQ(pose.vecPosition[2], 0.5);
}

// Steady-state quash with a cached last-good pose: the cached value
// replaces the incoming raw pose (which is what avoids the "ghost at the
// vendor origin" symptom), and the connection / validity flags are forced
// into the OutOfRange contract.
TEST(QuashPoseTest, QuashSubstitutesLastGoodAndMarksOutOfRange)
{
    // Incoming raw vendor pose -- typically a floor-level point if the
    // driver hasn't applied the calibration matrix.
    vr::DriverPose_t pose = MakePose(10.0, 0.0, -5.0, true, true);
    // Cached last-good pose -- where the user actually expects to see the
    // tracker (calibrated, in roomspace).
    vr::DriverPose_t lastGood = MakePose(0.5, 1.6, 0.2, true, true);

    openvr_pair::common::quash::ApplyQuashToPose(pose, lastGood, /*lastGoodValid=*/true);

    EXPECT_TRUE(pose.deviceIsConnected);
    EXPECT_TRUE(pose.poseIsValid);
    EXPECT_EQ(pose.result, vr::TrackingResult_Calibrating_OutOfRange);
    EXPECT_DOUBLE_EQ(pose.vecPosition[0], 0.5);
    EXPECT_DOUBLE_EQ(pose.vecPosition[1], 1.6);
    EXPECT_DOUBLE_EQ(pose.vecPosition[2], 0.2);
}

// Even when the cached last-good pose was itself reported with disconnected
// or invalid flags (it should not be, but defend against accidents), the
// quash mutation must clamp the resulting flags to keep SteamVR seeing the
// device as connected.
TEST(QuashPoseTest, QuashClampsFlagsRegardlessOfLastGoodFlags)
{
    vr::DriverPose_t pose = MakePose(0.0, 0.0, 0.0, false, false,
                                     vr::TrackingResult_Uninitialized);
    vr::DriverPose_t lastGood = MakePose(0.5, 1.6, 0.2,
                                         /*valid=*/false, /*conn=*/false,
                                         vr::TrackingResult_Uninitialized);

    openvr_pair::common::quash::ApplyQuashToPose(pose, lastGood, /*lastGoodValid=*/true);

    EXPECT_TRUE(pose.deviceIsConnected);
    EXPECT_TRUE(pose.poseIsValid);
    EXPECT_EQ(pose.result, vr::TrackingResult_Calibrating_OutOfRange);
}
