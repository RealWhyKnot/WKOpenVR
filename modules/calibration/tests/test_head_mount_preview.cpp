#include "HeadMountPreview.h"

#include <gtest/gtest.h>
#include <Eigen/Geometry>

using namespace wkopenvr::headmount;

TEST(HeadMountPreviewTest, UsesStandingUniverseForVisibleMarker)
{
	EXPECT_EQ(HeadMountPreviewTrackingOrigin(), vr::TrackingUniverseStanding);
}

TEST(HeadMountPreviewTest, PlacesMarkerForwardOfComputedHeadPose)
{
	Eigen::Affine3d tracker = Eigen::Affine3d::Identity();
	tracker.translation() = Eigen::Vector3d(1.0, 2.0, 3.0);

	Eigen::AffineCompact3d headFromTracker = Eigen::AffineCompact3d::Identity();
	headFromTracker.translation() = Eigen::Vector3d(0.10, 0.20, 0.30);

	const auto mat = HeadMountPreviewTransform(tracker, headFromTracker, HeadMountPreviewForwardMeters());

	EXPECT_FLOAT_EQ(mat.m[0][3], 1.10f);
	EXPECT_FLOAT_EQ(mat.m[1][3], 2.20f);
	EXPECT_FLOAT_EQ(mat.m[2][3], static_cast<float>(3.30 - HeadMountPreviewForwardMeters()));
}

TEST(HeadMountPreviewTest, ForwardDistanceIsClampedAwayFromEyePoint)
{
	const auto mat = HeadMountPreviewTransform(Eigen::Affine3d::Identity(), Eigen::AffineCompact3d::Identity(), 0.0);

	EXPECT_FLOAT_EQ(mat.m[2][3], -0.05f);
}

TEST(HeadMountPreviewTest, HmdReferenceMarkerUsesTrackedDeviceRelativeOffset)
{
	const auto mat = HeadMountPreviewHmdReferenceTransform();

	EXPECT_FLOAT_EQ(mat.m[0][0], 1.0f);
	EXPECT_FLOAT_EQ(mat.m[1][1], 1.0f);
	EXPECT_FLOAT_EQ(mat.m[2][2], 1.0f);
	EXPECT_FLOAT_EQ(mat.m[0][3], 0.12f);
	EXPECT_FLOAT_EQ(mat.m[1][3], 0.08f);
	EXPECT_FLOAT_EQ(mat.m[2][3], -0.45f);
}
