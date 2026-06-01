// Tests for the safety boundary backend: Douglas-Peucker simplification,
// polygon area, coordinate transform, and CaptureSession state machine.
// Chaperone IVRChaperoneSetup calls are not exercised here -- they require
// a running SteamVR instance.

#include "Boundary.h"
#include "BoundaryCapture.h"
#include "BoundaryFloorCapture.h"
#include "BoundaryPreview.h"
#include "BoundaryRePush.h"
#include "BoundarySpatial.h"
#include "ControllerInput.h"
#include "boundary_test_helpers.h"

#include <Eigen/Geometry>

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

using namespace wkopenvr::boundary;
using boundary_test::CountAlphaPixels;
using boundary_test::ExpectBoundsNear;
using boundary_test::ExpectMatrixRotationUnchanged;
using boundary_test::ExpectMatrixTranslationNear;
using boundary_test::ExpectVertexNear;
using boundary_test::MakeFloorPointerPose;
using boundary_test::MakeIdentityMatrix34;
using boundary_test::MakePose;
using boundary_test::Matrix34ToAffine;
using boundary_test::SetTranslation;
using boundary_test::SquareBoundary;
using boundary_test::TransformStandingPointToRaw;

namespace {

struct FloorOffsetCase {
    const char* name;
    vr::HmdMatrix34_t pose;
    double measuredFloorYStanding;
    std::array<float, 3> expectedTranslation;
};

struct ControllerContactCase {
    const char* name;
    const char* controllerType;
    Eigen::Affine3d pose;
    double expectedOffsetMeters;
};

SteamVrFloorSnapshot ValidSteamVrFloorSnapshot()
{
    SteamVrFloorSnapshot snapshot;
    snapshot.chaperoneAvailable = true;
    snapshot.calibrationState = vr::ChaperoneCalibrationState_OK;
    snapshot.playAreaSizeValid = true;
    snapshot.playAreaX = 2.0f;
    snapshot.playAreaZ = 2.0f;
    snapshot.playAreaRectValid = true;
    snapshot.playAreaRect.vCorners[0].v[0] = -1.0f;
    snapshot.playAreaRect.vCorners[0].v[1] = 0.0f;
    snapshot.playAreaRect.vCorners[0].v[2] = -1.0f;
    snapshot.playAreaRect.vCorners[1].v[0] = 1.0f;
    snapshot.playAreaRect.vCorners[1].v[1] = 0.0f;
    snapshot.playAreaRect.vCorners[1].v[2] = -1.0f;
    snapshot.playAreaRect.vCorners[2].v[0] = 1.0f;
    snapshot.playAreaRect.vCorners[2].v[1] = 0.0f;
    snapshot.playAreaRect.vCorners[2].v[2] = 1.0f;
    snapshot.playAreaRect.vCorners[3].v[0] = -1.0f;
    snapshot.playAreaRect.vCorners[3].v[1] = 0.0f;
    snapshot.playAreaRect.vCorners[3].v[2] = 1.0f;
    snapshot.chaperoneSetupAvailable = true;
    snapshot.standingZeroValid = true;
    snapshot.standingZeroToRaw = MakeIdentityMatrix34();
    return snapshot;
}

size_t RasterPixelIndexForWorld(
    const BoundaryPreviewRaster& raster,
    double x,
    double z,
    size_t channel)
{
    const double half = raster.plane.spanMeters * 0.5;
    const double minX = raster.plane.centerX - half;
    const double maxZ = raster.plane.centerZ + half;
    const double scale = static_cast<double>(BoundaryPreviewRaster::kTextureSize - 1) /
        raster.plane.spanMeters;
    const int px = std::clamp(
        static_cast<int>(std::lround((x - minX) * scale)),
        0,
        BoundaryPreviewRaster::kTextureSize - 1);
    const int py = std::clamp(
        static_cast<int>(std::lround((maxZ - z) * scale)),
        0,
        BoundaryPreviewRaster::kTextureSize - 1);
    return static_cast<size_t>(py * BoundaryPreviewRaster::kTextureSize + px) * 4u + channel;
}

uint8_t RasterChannelAtWorld(
    const BoundaryPreviewRaster& raster,
    double x,
    double z,
    size_t channel)
{
    const size_t idx = RasterPixelIndexForWorld(raster, x, z, channel);
    EXPECT_LT(idx, raster.rgba.size());
    return raster.rgba[idx];
}

Eigen::Affine3d FaceDownPose()
{
    Eigen::Affine3d pose = Eigen::Affine3d::Identity();
    pose.linear() = Eigen::AngleAxisd(
        EIGEN_PI,
        Eigen::Vector3d::UnitZ()).toRotationMatrix();
    return pose;
}

std::vector<FloorOffsetCase> FloorOffsetCases()
{
    vr::HmdMatrix34_t identity = MakeIdentityMatrix34();
    SetTranslation(identity, 1.25f, -0.40f, 2.50f);

    vr::HmdMatrix34_t tilted = MakeIdentityMatrix34();
    tilted.m[0][1] = 0.10f;
    tilted.m[1][1] = 0.95f;
    tilted.m[2][1] = -0.20f;
    SetTranslation(tilted, 1.0f, 2.0f, 3.0f);

    vr::HmdMatrix34_t yawed = MakeIdentityMatrix34();
    yawed.m[0][0] = 0.8660254f;
    yawed.m[0][1] = 0.0500000f;
    yawed.m[0][2] = 0.5000000f;
    yawed.m[1][0] = 0.0000000f;
    yawed.m[1][1] = 0.9950000f;
    yawed.m[1][2] = 0.0000000f;
    yawed.m[2][0] = -0.5000000f;
    yawed.m[2][1] = -0.0800000f;
    yawed.m[2][2] = 0.8660254f;
    SetTranslation(yawed, -1.0f, 0.5f, 2.0f);

    return {
        { "identity_y_only", identity, 0.125, { 1.25f, -0.275f, 2.50f } },
        { "tilted_up_basis", tilted, 0.50, { 1.05f, 2.475f, 2.90f } },
        { "negative_offset_keeps_rotation", yawed, -0.25, { -1.0125f, 0.25125f, 2.0200f } },
    };
}

std::vector<ControllerContactCase> ControllerContactCases()
{
    const Eigen::Affine3d faceUp = Eigen::Affine3d::Identity();
    const Eigen::Affine3d faceDown = FaceDownPose();
    return {
        { "knuckles_face_up", "knuckles", faceUp, 0.0285 },
        { "vive_face_up", "vive_controller", faceUp, 0.0620 },
        { "knuckles_face_down", "knuckles", faceDown, 0.0310 },
        { "vive_face_down", "vive_controller", faceDown, 0.0060 },
        { "unknown_controller", "unknown_controller", faceUp, 0.0 },
    };
}

vr::HmdQuad_t MakeWallQuad(double ax, double az, double bx, double bz, double floorY, double ceilingY)
{
    vr::HmdQuad_t q{};
    q.vCorners[0].v[0] = static_cast<float>(ax);
    q.vCorners[0].v[1] = static_cast<float>(floorY);
    q.vCorners[0].v[2] = static_cast<float>(az);
    q.vCorners[1].v[0] = static_cast<float>(bx);
    q.vCorners[1].v[1] = static_cast<float>(floorY);
    q.vCorners[1].v[2] = static_cast<float>(bz);
    q.vCorners[2].v[0] = static_cast<float>(bx);
    q.vCorners[2].v[1] = static_cast<float>(ceilingY);
    q.vCorners[2].v[2] = static_cast<float>(bz);
    q.vCorners[3].v[0] = static_cast<float>(ax);
    q.vCorners[3].v[1] = static_cast<float>(ceilingY);
    q.vCorners[3].v[2] = static_cast<float>(az);
    return q;
}

} // namespace

// ---------------------------------------------------------------------------
// Douglas-Peucker
// ---------------------------------------------------------------------------

// 100 points all on the X-axis -> 2 kept (endpoints only).
TEST(BoundaryDouglasPeuckerTest, SimplifiesCollinear) {
    std::vector<BoundaryVertex> path;
    for (int i = 0; i < 100; ++i) {
        path.push_back({ (double)i * 0.1, 0.0, 0.0 });
    }
    auto kept = SimplifyDouglasPeucker(path, 0.05);
    EXPECT_EQ(kept.size(), 2u);
    EXPECT_EQ(kept.front(), 0u);
    EXPECT_EQ(kept.back(), 99u);
}

// L-shape: (0,0,0) -> (1,0,0) -> (1,0,1) -> (2,0,1). The corner at (1,0,0)
// and (1,0,1) must survive; a straight line from start to end would be
// > 0.05 m away from both those points.
TEST(BoundaryDouglasPeuckerTest, PreservesCorners) {
    std::vector<BoundaryVertex> path = {
        { 0.0, 0.0, 0.0 },
        { 0.5, 0.0, 0.0 },   // collinear -- should be dropped
        { 1.0, 0.0, 0.0 },   // corner A
        { 1.0, 0.0, 0.5 },   // collinear -- should be dropped
        { 1.0, 0.0, 1.0 },   // corner B
        { 1.5, 0.0, 1.0 },   // collinear -- should be dropped
        { 2.0, 0.0, 1.0 },
    };
    auto kept = SimplifyDouglasPeucker(path, 0.05);
    // Kept indices must include the two corners (index 2 and 4).
    bool hasCornerA = false, hasCornerB = false;
    for (size_t idx : kept) {
        if (idx == 2) hasCornerA = true;
        if (idx == 4) hasCornerB = true;
    }
    EXPECT_TRUE(hasCornerA) << "corner at index 2 should be preserved";
    EXPECT_TRUE(hasCornerB) << "corner at index 4 should be preserved";
    // Collinear mid-points should be gone.
    EXPECT_LE(kept.size(), 5u);
}

// Empty and single-point inputs should not crash.
TEST(BoundaryDouglasPeuckerTest, HandlesEmpty) {
    std::vector<BoundaryVertex> empty;
    auto kept = SimplifyDouglasPeucker(empty, 0.05);
    EXPECT_TRUE(kept.empty());
}

TEST(BoundaryDouglasPeuckerTest, HandlesSinglePoint) {
    std::vector<BoundaryVertex> one = { { 1.0, 0.0, 2.0 } };
    auto kept = SimplifyDouglasPeucker(one, 0.05);
    ASSERT_EQ(kept.size(), 1u);
    EXPECT_EQ(kept[0], 0u);
}

// ---------------------------------------------------------------------------
// Polygon area
// ---------------------------------------------------------------------------

// Counter-clockwise unit square on XZ plane: area should be +1.0.
TEST(BoundaryAreaTest, SignedAreaCorrectForUnitSquare) {
    std::vector<XZPoint> square = {
        { 0.0, 0.0 },
        { 1.0, 0.0 },
        { 1.0, 1.0 },
        { 0.0, 1.0 },
    };
    // Signed area is negative for CW winding. We care that AbsoluteArea = 1.
    double area = AbsoluteAreaXZ(square);
    EXPECT_NEAR(area, 1.0, 1e-9);
}

TEST(BoundaryAreaTest, AbsoluteAreaIgnoresWinding) {
    // CW winding
    std::vector<XZPoint> squareCW = {
        { 0.0, 0.0 },
        { 0.0, 1.0 },
        { 1.0, 1.0 },
        { 1.0, 0.0 },
    };
    double area = AbsoluteAreaXZ(squareCW);
    EXPECT_NEAR(area, 1.0, 1e-9);
}

// ---------------------------------------------------------------------------
// Transform to standing universe
// ---------------------------------------------------------------------------

// Identity transform: output equals input.
TEST(BoundaryTransformTest, IdentityPassesThrough) {
    Eigen::AffineCompact3d identity = Eigen::AffineCompact3d::Identity();
    std::vector<BoundaryVertex> verts = {
        { 1.0, 2.0, 3.0 },
        { -0.5, 0.1, 4.2 },
    };
    auto out = TransformToStandingUniverse(verts, identity);
    ASSERT_EQ(out.size(), verts.size());
    for (size_t i = 0; i < verts.size(); ++i) {
        EXPECT_NEAR(out[i].x, verts[i].x, 1e-9);
        EXPECT_NEAR(out[i].y, verts[i].y, 1e-9);
        EXPECT_NEAR(out[i].z, verts[i].z, 1e-9);
    }
}

// Known translation applied to vertices matches Eigen-direct multiplication.
TEST(BoundaryTransformTest, KnownTranslationApplied) {
    Eigen::AffineCompact3d xf = Eigen::AffineCompact3d::Identity();
    xf.translation() = Eigen::Vector3d(1.0, 2.0, 3.0);

    std::vector<BoundaryVertex> verts = { { 0.0, 0.0, 0.0 }, { 1.0, 0.0, 0.0 } };
    auto out = TransformToStandingUniverse(verts, xf);
    ASSERT_EQ(out.size(), 2u);

    // First point (0,0,0) translated by (1,2,3) -> (1,2,3).
    EXPECT_NEAR(out[0].x, 1.0, 1e-9);
    EXPECT_NEAR(out[0].y, 2.0, 1e-9);
    EXPECT_NEAR(out[0].z, 3.0, 1e-9);

    // Second point (1,0,0) translated -> (2,2,3).
    EXPECT_NEAR(out[1].x, 2.0, 1e-9);
    EXPECT_NEAR(out[1].y, 2.0, 1e-9);
    EXPECT_NEAR(out[1].z, 3.0, 1e-9);
}

TEST(BoundaryTransformTest, HeightUsesSameTargetToStandingTransform) {
    Eigen::AffineCompact3d xf = Eigen::AffineCompact3d::Identity();
    xf.linear() = Eigen::AngleAxisd(
        15.0 * EIGEN_PI / 180.0,
        Eigen::Vector3d::UnitX()).toRotationMatrix();
    xf.translation() = Eigen::Vector3d(1.0, 0.75, -0.25);

    const double targetY = 0.40;
    const double standingY = TransformHeightToStandingUniverse(targetY, xf);
    const Eigen::Vector3d expected = xf * Eigen::Vector3d(0.0, targetY, 0.0);

    EXPECT_NEAR(standingY, expected.y(), 1e-9);
}

TEST(BoundaryTransformTest, BoundaryHeightUsesPolygonCenter) {
    Eigen::AffineCompact3d xf = Eigen::AffineCompact3d::Identity();
    xf.linear() = Eigen::AngleAxisd(
        10.0 * EIGEN_PI / 180.0,
        Eigen::Vector3d::UnitZ()).toRotationMatrix();
    xf.translation() = Eigen::Vector3d(0.0, 0.25, 0.0);

    std::vector<BoundaryVertex> verts = {
        { 10.0, 0.0, -1.0 },
        { 12.0, 0.0, -1.0 },
        { 12.0, 0.0,  1.0 },
        { 10.0, 0.0,  1.0 },
    };
    const double targetY = 0.30;

    const double standingY = TransformHeightToStandingUniverse(verts, targetY, xf);
    const Eigen::Vector3d expected = xf * Eigen::Vector3d(11.0, targetY, 0.0);

    EXPECT_NEAR(standingY, expected.y(), 1e-9);
    EXPECT_NE(standingY, TransformHeightToStandingUniverse(targetY, xf));
}

TEST(BoundaryTransformTest, TargetFloorYMapsToStandingFloorAtPolygonCenter) {
    Eigen::AffineCompact3d xf = Eigen::AffineCompact3d::Identity();
    xf.linear() = Eigen::AngleAxisd(
        10.0 * EIGEN_PI / 180.0,
        Eigen::Vector3d::UnitZ()).toRotationMatrix();
    xf.translation() = Eigen::Vector3d(0.25, 1.50, -0.10);

    std::vector<BoundaryVertex> verts = {
        { 2.0, 0.0, -1.0 },
        { 4.0, 0.0, -1.0 },
        { 4.0, 0.0,  1.0 },
        { 2.0, 0.0,  1.0 },
    };

    const double floorY = TargetFloorYForStandingFloor(verts, xf);
    EXPECT_NEAR(TransformHeightToStandingUniverse(verts, floorY, xf), 0.0, 1e-9);
}

TEST(BoundaryTransformTest, ProfileTransformAppliesToControllerPose) {
    const Eigen::AffineCompact3d xf = ProfileTransformFromCalibration(
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d(100.0, 0.0, -50.0));

    Eigen::Affine3d raw = Eigen::Affine3d::Identity();
    raw.translation() = Eigen::Vector3d(0.25, 1.0, 0.75);

    const Eigen::Affine3d standing = TransformPoseToStandingUniverse(raw, xf);
    EXPECT_NEAR(standing.translation().x(), 1.25, 1e-9);
    EXPECT_NEAR(standing.translation().y(), 1.0, 1e-9);
    EXPECT_NEAR(standing.translation().z(), 0.25, 1e-9);
}

TEST(BoundaryTransformTest, FloorOffsetCasesPreserveBasisAndMoveAlongUpVector) {
    for (const auto& c : FloorOffsetCases()) {
        SCOPED_TRACE(c.name);
        const auto adjusted =
            OffsetStandingZeroPoseForFloor(c.pose, c.measuredFloorYStanding);

        ExpectMatrixRotationUnchanged(adjusted, c.pose);
        ExpectMatrixTranslationNear(
            adjusted,
            c.expectedTranslation[0],
            c.expectedTranslation[1],
            c.expectedTranslation[2]);
    }
}

TEST(BoundaryTransformTest, FloorOffsetMakesMeasuredStandingFloorBecomeZero) {
    for (const auto& c : FloorOffsetCases()) {
        SCOPED_TRACE(c.name);
        const Eigen::Vector3d rawFloorPoint =
            TransformStandingPointToRaw(c.pose, 0.0, c.measuredFloorYStanding, 0.0);
        const auto adjusted =
            OffsetStandingZeroPoseForFloor(c.pose, c.measuredFloorYStanding);
        const Eigen::Vector3d adjustedStanding =
            Matrix34ToAffine(adjusted).inverse() * rawFloorPoint;

        EXPECT_NEAR(adjustedStanding.x(), 0.0, 1e-5);
        EXPECT_NEAR(adjustedStanding.y(), 0.0, 1e-5);
        EXPECT_NEAR(adjustedStanding.z(), 0.0, 1e-5);
    }
}

TEST(BoundaryTransformTest, ControllerContactOffsetCasesMatchKnownControllerTypes) {
    for (const auto& c : ControllerContactCases()) {
        SCOPED_TRACE(c.name);
        EXPECT_NEAR(
            ControllerFloorContactOffsetMeters(c.controllerType, c.pose),
            c.expectedOffsetMeters,
            1e-9);
        EXPECT_NEAR(
            AdjustControllerFloorYForContact(0.4694, c.controllerType, c.pose),
            0.4694 - c.expectedOffsetMeters,
            1e-9);
    }
}

TEST(BoundaryTransformTest, FloorApplyDefaultsToBoundaryLocalHeight) {
    EXPECT_NEAR(BoundaryFloorYAfterApply(0.42, false), 0.42, 1e-9);
    EXPECT_NEAR(BoundaryFloorYAfterApply(-0.12, false), -0.12, 1e-9);
    EXPECT_NEAR(BoundaryFloorYAfterApply(0.42, true), 0.0, 1e-9);
}

TEST(BoundaryTransformTest, SteamVrFloorVerificationRequiresObservedPoseShift) {
    StandingYSample before;
    before.valid = true;
    before.y = 0.53;
    StandingYSample afterMoved;
    afterMoved.valid = true;
    afterMoved.y = 0.03;

    const auto verified =
        EvaluateSteamVrFloorVerification(before, afterMoved, 0.50);
    EXPECT_TRUE(verified.verified);
    EXPECT_NEAR(verified.expectedAfterY, 0.03, 1e-9);
    EXPECT_NEAR(verified.residualY, 0.0, 1e-9);

    StandingYSample afterUnchanged = afterMoved;
    afterUnchanged.y = 0.53;
    const auto stale =
        EvaluateSteamVrFloorVerification(before, afterUnchanged, 0.50);
    EXPECT_FALSE(stale.verified);
    EXPECT_NEAR(stale.residualY, 0.50, 1e-9);
}

TEST(BoundaryTransformTest, SteamVrFloorVerificationFailsWithoutValidSamples) {
    StandingYSample before;
    before.valid = true;
    before.y = 0.40;
    StandingYSample after;
    after.valid = false;
    after.y = 0.0;

    const auto result =
        EvaluateSteamVrFloorVerification(before, after, 0.40);
    EXPECT_FALSE(result.verified);
    EXPECT_TRUE(result.beforeValid);
    EXPECT_FALSE(result.afterValid);
}

TEST(BoundaryTransformTest, ControllerTargetMatchRequiresConfiguredTargetSystem) {
    EXPECT_FALSE(BoundaryControllerMatchesTargetTrackingSystem("lighthouse", ""));
    EXPECT_FALSE(BoundaryControllerMatchesTargetTrackingSystem("", "lighthouse"));
    EXPECT_FALSE(BoundaryControllerMatchesTargetTrackingSystem("oculus", "lighthouse"));
    EXPECT_TRUE(BoundaryControllerMatchesTargetTrackingSystem("lighthouse", "lighthouse"));
}

TEST(BoundaryTransformTest, BoundaryCaptureUsesStandingSpaceEvenForTargetControllers) {
    EXPECT_FALSE(BoundaryCaptureShouldUseTargetSpace(false, false));
    EXPECT_FALSE(BoundaryCaptureShouldUseTargetSpace(false, true));
    EXPECT_FALSE(BoundaryCaptureShouldUseTargetSpace(true, false));
    EXPECT_FALSE(BoundaryCaptureShouldUseTargetSpace(true, true));
}

TEST(BoundaryFloorSourceTest, SteamVrFloorTakesPriorityOverSavedBoundary) {
    BoundaryFloorSourceRequest request;
    request.boundaryStandingSpace = true;
    request.hasSavedBoundaryFloor = true;
    request.savedBoundaryFloorY = 0.35;

    const auto decision =
        ResolveBoundaryFloorSource(ValidSteamVrFloorSnapshot(), request);

    ASSERT_TRUE(decision.valid);
    EXPECT_EQ(decision.source, BoundaryFloorSourceKind::SteamVrStanding);
    EXPECT_NEAR(decision.standingFloorY, 0.0, 1e-9);
    EXPECT_NEAR(decision.boundaryFloorY, 0.0, 1e-9);
}

TEST(BoundaryFloorSourceTest, SteamVrFloorIsUsedWhenSavedFloorIsUnavailable) {
    BoundaryFloorSourceRequest request;
    request.boundaryStandingSpace = true;

    const auto decision =
        ResolveBoundaryFloorSource(ValidSteamVrFloorSnapshot(), request);

    ASSERT_TRUE(decision.valid);
    EXPECT_EQ(decision.source, BoundaryFloorSourceKind::SteamVrStanding);
    EXPECT_NEAR(decision.standingFloorY, 0.0, 1e-9);
    EXPECT_NEAR(decision.boundaryFloorY, 0.0, 1e-9);
}

TEST(BoundaryFloorSourceTest, SteamVrFloorTakesPriorityOverControllerContact) {
    BoundaryFloorSourceRequest request;
    request.boundaryStandingSpace = true;
    request.controllerContactValid = true;
    request.controllerContactStandingY = 0.47;
    request.hasSavedBoundaryFloor = true;
    request.savedBoundaryFloorY = 0.35;

    const auto decision =
        ResolveBoundaryFloorSource(ValidSteamVrFloorSnapshot(), request);

    ASSERT_TRUE(decision.valid);
    EXPECT_EQ(decision.source, BoundaryFloorSourceKind::SteamVrStanding);
    EXPECT_NEAR(decision.boundaryFloorY, 0.0, 1e-9);
}

TEST(BoundaryFloorSourceTest, InvalidSteamVrFallsBackToSavedBoundary) {
    SteamVrFloorSnapshot snapshot = ValidSteamVrFloorSnapshot();
    snapshot.calibrationState = vr::ChaperoneCalibrationState_Error_PlayAreaInvalid;
    snapshot.playAreaSizeValid = false;
    snapshot.playAreaRectValid = false;

    BoundaryFloorSourceRequest request;
    request.boundaryStandingSpace = true;
    request.hasSavedBoundaryFloor = true;
    request.savedBoundaryFloorY = -0.18;

    const auto decision = ResolveBoundaryFloorSource(snapshot, request);

    ASSERT_TRUE(decision.valid);
    EXPECT_EQ(decision.source, BoundaryFloorSourceKind::SavedBoundary);
    EXPECT_NEAR(decision.standingFloorY, -0.18, 1e-9);
    EXPECT_NEAR(decision.boundaryFloorY, -0.18, 1e-9);
    EXPECT_FALSE(decision.rejectedReasons.empty());
}

TEST(BoundaryFloorSourceTest, ControllerContactUsedWhenSteamVrAndSavedUnavailable) {
    SteamVrFloorSnapshot snapshot = ValidSteamVrFloorSnapshot();
    snapshot.calibrationState = vr::ChaperoneCalibrationState_Error_PlayAreaInvalid;
    snapshot.playAreaSizeValid = false;
    snapshot.playAreaRectValid = false;

    BoundaryFloorSourceRequest request;
    request.boundaryStandingSpace = true;
    request.controllerContactValid = true;
    request.controllerContactStandingY = 0.47;

    const auto decision = ResolveBoundaryFloorSource(snapshot, request);

    ASSERT_TRUE(decision.valid);
    EXPECT_EQ(decision.source, BoundaryFloorSourceKind::ControllerContact);
    EXPECT_NEAR(decision.boundaryFloorY, 0.47, 1e-9);
}

TEST(BoundaryFloorSourceTest, TargetSpaceSavedFloorMapsToStandingHeight) {
    // Standing space is the path going forward; this exercises the legacy
    // target-space height mapping used when migrating an old saved boundary,
    // so SteamVR must be unavailable for the saved value to be selected.
    SteamVrFloorSnapshot snapshot = ValidSteamVrFloorSnapshot();
    snapshot.calibrationState = vr::ChaperoneCalibrationState_Error_PlayAreaInvalid;
    snapshot.playAreaSizeValid = false;
    snapshot.playAreaRectValid = false;

    Eigen::AffineCompact3d targetToStanding = Eigen::AffineCompact3d::Identity();
    targetToStanding.translation() = Eigen::Vector3d(1.0, 2.50, -1.0);

    BoundaryFloorSourceRequest request;
    request.boundaryStandingSpace = false;
    request.boundaryVertices = SquareBoundary(-1.0, -1.0, 1.0, 1.0, 0.0);
    request.targetTransformValid = true;
    request.targetToStanding = targetToStanding;
    request.hasSavedBoundaryFloor = true;
    request.savedBoundaryFloorY = -2.0;

    const auto decision = ResolveBoundaryFloorSource(snapshot, request);

    ASSERT_TRUE(decision.valid);
    EXPECT_EQ(decision.source, BoundaryFloorSourceKind::SavedBoundary);
    EXPECT_NEAR(decision.standingFloorY, 0.50, 1e-9);
    EXPECT_NEAR(decision.boundaryFloorY, -2.0, 1e-9);
}

// ---------------------------------------------------------------------------
// CaptureSession
// ---------------------------------------------------------------------------

// 100 ticks with the same pose -> only 1 vertex (debounce suppresses duplicates).
TEST(CaptureSessionTest, DebouncesVertices) {
    CaptureSession session;
    session.Start();
    EXPECT_EQ(session.state(), CaptureState::Active);

    Eigen::Affine3d pose = MakeFloorPointerPose(0.5, 0.5);

    for (int i = 0; i < 100; ++i) {
        session.Tick(pose, /*triggerHeld=*/true);
    }

    EXPECT_EQ(session.rawVertexCount(), 1u);
}

// Walking an L-shape, then Finish -> simplified to ~3 vertices.
TEST(CaptureSessionTest, FinishSimplifies) {
    CaptureSession session;
    session.Start();

    // Walk along +X axis: 20 steps of 0.1 m -> 2 m total
    for (int i = 0; i <= 20; ++i) {
        Eigen::Affine3d pose = MakeFloorPointerPose(i * 0.1, 0.0);
        session.Tick(pose, true);
    }
    // Turn: walk along +Z axis: 20 steps of 0.1 m
    for (int i = 1; i <= 20; ++i) {
        Eigen::Affine3d pose = MakeFloorPointerPose(2.0, i * 0.1);
        session.Tick(pose, true);
    }

    session.Finish();
    EXPECT_EQ(session.state(), CaptureState::Finished);
    // Simplified result should have the two endpoints and the corner, ~3 points.
    const auto& simplified = session.vertices();
    EXPECT_GE(simplified.size(), 2u);
    EXPECT_LE(simplified.size(), 5u);
}

// Cancel -> state Idle, buffer empty.
TEST(CaptureSessionTest, CancelClearsBuffer) {
    CaptureSession session;
    session.Start();

    for (int i = 0; i < 5; ++i) {
        Eigen::Affine3d pose = MakeFloorPointerPose(i * 0.2, 0.0);
        session.Tick(pose, true);
    }
    EXPECT_GT(session.rawVertexCount(), 0u);

    session.Cancel();
    EXPECT_EQ(session.state(), CaptureState::Idle);
    EXPECT_EQ(session.vertices().size(), 0u);
    EXPECT_EQ(session.rawVertexCount(), 0u);
}

// Trigger not held -> no vertices accumulated.
TEST(CaptureSessionTest, IgnoresTicksWhenTriggerNotHeld) {
    CaptureSession session;
    session.Start();

    Eigen::Affine3d pose = MakeFloorPointerPose(0.0, 0.0);
    for (int i = 0; i < 10; ++i) {
        pose.translation() = Eigen::Vector3d(i * 0.1, 0.0, 0.0);
        session.Tick(pose, /*triggerHeld=*/false);
    }
    EXPECT_EQ(session.rawVertexCount(), 0u);
}

TEST(CaptureSessionTest, UsesControllerAimRayFloorHit) {
    CaptureSession session;
    session.Start();

    Eigen::Affine3d pose = MakeFloorPointerPose(0.25, -0.5, 1.25);
    EXPECT_TRUE(session.Tick(pose, true, 0.25));

    ASSERT_EQ(session.rawVertexCount(), 1u);
    const auto& verts = session.vertices();
    ASSERT_EQ(verts.size(), 1u);
    EXPECT_NEAR(verts[0].x, 0.25, 1e-9);
    EXPECT_NEAR(verts[0].y, 0.25, 1e-9);
    EXPECT_NEAR(verts[0].z, -0.5, 1e-9);
}

TEST(CaptureSessionTest, AcceptsOppositeZPointerRayFallback) {
    CaptureSession session;
    session.Start();

    Eigen::Affine3d pose = Eigen::Affine3d::Identity();
    pose.translation() = Eigen::Vector3d(0.25, 1.25, -0.5);
    const Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(
        Eigen::Vector3d::UnitZ(), -Eigen::Vector3d::UnitY());
    pose.linear() = q.toRotationMatrix();

    EXPECT_TRUE(session.Tick(pose, true, 0.25));
    ASSERT_EQ(session.rawVertexCount(), 1u);
    const auto& verts = session.vertices();
    ASSERT_EQ(verts.size(), 1u);
    EXPECT_NEAR(verts[0].x, 0.25, 1e-9);
    EXPECT_NEAR(verts[0].y, 0.25, 1e-9);
    EXPECT_NEAR(verts[0].z, -0.5, 1e-9);
}

TEST(CaptureSessionTest, AcceptsAlternateAxisPointerRayFallback) {
    CaptureSession session;
    session.Start();

    Eigen::Affine3d pose = Eigen::Affine3d::Identity();
    pose.translation() = Eigen::Vector3d(0.0, 1.0, 0.0);
    EXPECT_TRUE(session.Tick(pose, true));

    EXPECT_EQ(session.rawVertexCount(), 1u);
}

TEST(CaptureSessionTest, ChoosesSteepestValidFloorRay) {
    CaptureSession session;
    session.Start();

    Eigen::Affine3d pose = Eigen::Affine3d::Identity();
    pose.translation() = Eigen::Vector3d(0.0, 1.0, 0.0);
    pose.linear() = Eigen::AngleAxisd(
        -20.0 * EIGEN_PI / 180.0,
        Eigen::Vector3d::UnitX()).toRotationMatrix();

    EXPECT_TRUE(session.Tick(pose, true, 0.0));
    ASSERT_EQ(session.rawVertexCount(), 1u);
    const auto& verts = session.vertices();
    ASSERT_EQ(verts.size(), 1u);

    const Eigen::Vector3d steepRay =
        pose.rotation() * Eigen::Vector3d(0.0, -1.0, 0.0);
    const double distance = -pose.translation().y() / steepRay.y();
    const Eigen::Vector3d expected = pose.translation() + steepRay * distance;
    EXPECT_NEAR(verts[0].x, expected.x(), 1e-9);
    EXPECT_NEAR(verts[0].y, 0.0, 1e-9);
    EXPECT_NEAR(verts[0].z, expected.z(), 1e-9);
}

TEST(CaptureSessionTest, PointerPoseUsesTipMinusZOnly) {
    CaptureSession session;
    session.Start();

    Eigen::Affine3d pointerPose = Eigen::Affine3d::Identity();
    pointerPose.translation() = Eigen::Vector3d(0.50, 1.00, -0.25);
    pointerPose.linear() = Eigen::AngleAxisd(
        -45.0 * EIGEN_PI / 180.0,
        Eigen::Vector3d::UnitX()).toRotationMatrix();

    ASSERT_TRUE(session.TickPointerPose(pointerPose, true, 0.0));
    ASSERT_EQ(session.rawVertexCount(), 1u);
    const auto& verts = session.vertices();
    ASSERT_EQ(verts.size(), 1u);

    const Eigen::Vector3d ray =
        pointerPose.rotation() * Eigen::Vector3d(0.0, 0.0, -1.0);
    const double distance = -pointerPose.translation().y() / ray.y();
    const Eigen::Vector3d expected = pointerPose.translation() + ray * distance;

    EXPECT_NEAR(verts[0].x, expected.x(), 1e-9);
    EXPECT_NEAR(verts[0].y, 0.0, 1e-9);
    EXPECT_NEAR(verts[0].z, expected.z(), 1e-9);
}

TEST(CaptureSessionTest, PreviewPointerHitDoesNotAppend) {
    CaptureSession session;
    session.Start();

    Eigen::Affine3d pointerPose = Eigen::Affine3d::Identity();
    pointerPose.translation() = Eigen::Vector3d(0.50, 1.00, -0.25);
    pointerPose.linear() = Eigen::AngleAxisd(
        -45.0 * EIGEN_PI / 180.0,
        Eigen::Vector3d::UnitX()).toRotationMatrix();

    const auto preview = session.PreviewPointerFloorHit(pointerPose, 0.0);
    ASSERT_TRUE(preview.valid);
    EXPECT_STREQ(preview.rayName, "tip:-Z");
    EXPECT_EQ(session.rawVertexCount(), 0u);

    ASSERT_TRUE(session.TickPointerPose(pointerPose, true, 0.0));
    EXPECT_EQ(session.rawVertexCount(), 1u);
    EXPECT_NEAR(session.vertices()[0].x, preview.hit.x, 1e-9);
    EXPECT_NEAR(session.vertices()[0].y, preview.hit.y, 1e-9);
    EXPECT_NEAR(session.vertices()[0].z, preview.hit.z, 1e-9);
}

TEST(CaptureSessionTest, ProjectedPositionUsesControllerXZAndFloorY) {
    CaptureSession session;
    session.Start();

    Eigen::Affine3d pose = Eigen::Affine3d::Identity();
    pose.translation() = Eigen::Vector3d(1.25, 1.0, -0.75);

    ASSERT_TRUE(session.TickProjectedPosition(pose, true, -2.50));
    ASSERT_EQ(session.rawVertexCount(), 1u);
    const auto& verts = session.vertices();
    ASSERT_EQ(verts.size(), 1u);
    EXPECT_NEAR(verts[0].x, 1.25, 1e-9);
    EXPECT_NEAR(verts[0].y, -2.50, 1e-9);
    EXPECT_NEAR(verts[0].z, -0.75, 1e-9);
}

TEST(CaptureSessionTest, ProjectedPositionUsesRawControllerPointNotDisplaySmoothing) {
    CaptureSession session;
    session.Start();

    Eigen::Affine3d pose = Eigen::Affine3d::Identity();
    pose.translation() = Eigen::Vector3d(0.0, 1.0, 0.0);
    ASSERT_TRUE(session.TickProjectedPosition(pose, true, 0.0));

    pose.translation() = Eigen::Vector3d(0.50, 1.0, 0.0);
    ASSERT_TRUE(session.TickProjectedPosition(pose, true, 0.0));

    ASSERT_EQ(session.rawVertexCount(), 2u);
    const auto& verts = session.vertices();
    ASSERT_EQ(verts.size(), 2u);
    EXPECT_NEAR(verts.back().x, 0.50, 1e-9);
    EXPECT_NEAR(verts.back().y, 0.0, 1e-9);
    EXPECT_NEAR(verts.back().z, 0.0, 1e-9);
}

TEST(CaptureSessionTest, ProjectedPositionDebouncesDuplicateSamples) {
    CaptureSession session;
    session.Start();

    Eigen::Affine3d pose = Eigen::Affine3d::Identity();
    pose.translation() = Eigen::Vector3d(1.25, 1.0, -0.75);

    EXPECT_TRUE(session.TickProjectedPosition(pose, true, 0.0));
    EXPECT_FALSE(session.TickProjectedPosition(pose, true, 0.0));
    EXPECT_EQ(session.rawVertexCount(), 1u);
}

TEST(FloorCaptureSessionTest, SmoothsLowPercentileFloorAndReportsReadiness) {
    FloorCaptureSession floor;
    floor.Begin(0.10, 2.60);

    Eigen::Affine3d first = Eigen::Affine3d::Identity();
    first.translation() = Eigen::Vector3d(0.0, 0.40, 0.0);
    EXPECT_TRUE(floor.Observe(first, 4, "lighthouse"));

    ASSERT_TRUE(floor.candidate().valid);
    EXPECT_NEAR(floor.candidate().floorY, 0.40, 1e-9);
    EXPECT_EQ(floor.candidate().deviceId, 4);
    EXPECT_FALSE(floor.candidate().ready);

    for (int i = 0; i < 7; ++i) {
        Eigen::Affine3d stable = Eigen::Affine3d::Identity();
        stable.translation() = Eigen::Vector3d(0.01 * i, 0.405, -0.01 * i);
        floor.Observe(stable, 4, "lighthouse");
    }

    EXPECT_TRUE(floor.candidate().ready);
    EXPECT_TRUE(floor.candidate().stable);
    EXPECT_LT(floor.candidate().jitterMeters, 0.02);
    EXPECT_NEAR(floor.candidate().floorY, 0.40275, 0.01);

    Eigen::Affine3d lower = Eigen::Affine3d::Identity();
    lower.translation() = Eigen::Vector3d(0.25, -0.02, -0.5);
    EXPECT_TRUE(floor.Observe(lower, 6, "lighthouse"));

    ASSERT_TRUE(floor.candidate().valid);
    EXPECT_LT(floor.candidate().floorY, 0.20);
    EXPECT_EQ(floor.candidate().deviceId, 6);
    EXPECT_EQ(floor.candidate().sampleCount, 9u);
}

TEST(FloorCaptureSessionTest, SamplesControllerContactPointInsteadOfOrigin) {
    FloorCaptureSession floor;
    floor.Begin(0.0, 2.4);

    Eigen::Affine3d pose = Eigen::Affine3d::Identity();
    pose.translation() = Eigen::Vector3d(0.0, 0.50, 0.0);
    pose.translation().y() = AdjustControllerFloorYForContact(
        pose.translation().y(),
        "knuckles",
        pose);

    ASSERT_TRUE(floor.Observe(pose, 4, "lighthouse"));
    ASSERT_TRUE(floor.candidate().valid);
    EXPECT_NEAR(floor.candidate().floorY, 0.4715, 1e-9);
}

TEST(FloorCaptureSessionTest, ReportsUnstableWhenFloorSamplesAreStillMoving) {
    FloorCaptureSession floor;
    floor.Begin(0.10, 2.60);

    for (int i = 0; i < 8; ++i) {
        Eigen::Affine3d moving = Eigen::Affine3d::Identity();
        moving.translation() = Eigen::Vector3d(0.0, 0.40 - 0.06 * i, 0.0);
        EXPECT_TRUE(floor.Observe(moving, 4, "lighthouse"));
    }

    ASSERT_TRUE(floor.candidate().valid);
    EXPECT_TRUE(floor.candidate().ready);
    EXPECT_FALSE(floor.candidate().stable);
    EXPECT_GT(floor.candidate().jitterMeters, 0.02);
}

TEST(FloorCaptureSessionTest, RejectsLowJitterButDriftingLatestWindow) {
    FloorCaptureSession floor;
    floor.Begin(0.10, 2.60);

    for (int i = 0; i < 8; ++i) {
        Eigen::Affine3d moving = Eigen::Affine3d::Identity();
        moving.translation() = Eigen::Vector3d(0.0, 0.400 + 0.0015 * i, 0.0);
        floor.Observe(moving, 4, "lighthouse");
    }

    ASSERT_TRUE(floor.candidate().valid);
    EXPECT_TRUE(floor.candidate().ready);
    EXPECT_LT(floor.candidate().jitterMeters, 0.02);
    EXPECT_GT(floor.candidate().recentDriftMeters, 0.008);
    EXPECT_FALSE(floor.candidate().stable);
}

TEST(FloorCaptureSessionTest, IgnoresInvalidSamplesAndHighOutliers) {
    FloorCaptureSession floor;
    floor.Begin(0.10, 2.60);

    Eigen::Affine3d invalid = Eigen::Affine3d::Identity();
    invalid.translation() = Eigen::Vector3d(0.0, std::numeric_limits<double>::quiet_NaN(), 0.0);
    EXPECT_FALSE(floor.Observe(invalid, 4, "lighthouse"));
    EXPECT_FALSE(floor.candidate().valid);

    for (int i = 0; i < 10; ++i) {
        Eigen::Affine3d stable = Eigen::Affine3d::Identity();
        stable.translation() = Eigen::Vector3d(0.01 * i, 0.42, -0.01 * i);
        EXPECT_TRUE(floor.Observe(stable, 4, "lighthouse"));
    }
    ASSERT_TRUE(floor.candidate().valid);
    const double beforeOutlier = floor.candidate().floorY;

    Eigen::Affine3d highOutlier = Eigen::Affine3d::Identity();
    highOutlier.translation() = Eigen::Vector3d(0.0, 1.75, 0.0);
    EXPECT_TRUE(floor.Observe(highOutlier, 4, "lighthouse"));

    EXPECT_NEAR(floor.candidate().floorY, beforeOutlier, 0.01);
}

TEST(FloorCaptureSessionTest, BuildsFloorMarkerAtControllerXZ) {
    Eigen::Affine3d pose = Eigen::Affine3d::Identity();
    pose.translation() = Eigen::Vector3d(1.25, 0.08, -0.75);

    const auto marker = BuildFloorMarkerVertices(pose, -0.03, 0.20);
    ASSERT_EQ(marker.size(), 4u);
    EXPECT_NEAR(marker[0].x, 1.05, 1e-9);
    EXPECT_NEAR(marker[0].y, -0.03, 1e-9);
    EXPECT_NEAR(marker[0].z, -0.95, 1e-9);
    EXPECT_NEAR(marker[2].x, 1.45, 1e-9);
    EXPECT_NEAR(marker[2].y, -0.03, 1e-9);
    EXPECT_NEAR(marker[2].z, -0.55, 1e-9);
}

TEST(FloorCaptureSessionTest, FloorNeverRisesWhenControllerIsLifted) {
    FloorCaptureSession floor;
    floor.Begin(0.10, 2.60);

    // Rest the controller on the floor and hold.
    for (int i = 0; i < 10; ++i) {
        Eigen::Affine3d down = Eigen::Affine3d::Identity();
        down.translation() = Eigen::Vector3d(0.0, 0.02, 0.0);
        floor.Observe(down, 4, "lighthouse");
    }
    ASSERT_TRUE(floor.candidate().valid);
    const double latched = floor.candidate().floorY;
    EXPECT_LT(latched, 0.10);

    // Lift the controller far above the floor; the latched floor must hold.
    double maxFloorAfterLift = latched;
    for (int i = 0; i < 25; ++i) {
        Eigen::Affine3d up = Eigen::Affine3d::Identity();
        up.translation() = Eigen::Vector3d(0.0, 0.90, 0.0);
        floor.Observe(up, 4, "lighthouse");
        maxFloorAfterLift = std::max(maxFloorAfterLift, floor.candidate().floorY);
    }
    EXPECT_LE(floor.candidate().floorY, latched + 1e-9);
    EXPECT_LE(maxFloorAfterLift, latched + 1e-9);
}

TEST(BoundarySpatialTest, AgeShadeRampFadesNewestWhiteToOldestGray) {
    EXPECT_EQ(BoundaryAgeShade(0, 1), 255);
    EXPECT_EQ(BoundaryAgeShade(9, 10), 255);
    EXPECT_LT(BoundaryAgeShade(0, 10), BoundaryAgeShade(5, 10));
    EXPECT_LT(BoundaryAgeShade(5, 10), BoundaryAgeShade(9, 10));
    EXPECT_GE(BoundaryAgeShade(0, 10), 100);
    EXPECT_LE(BoundaryAgeShade(0, 10), 110);
}

TEST(CaptureSessionTest, RejectsAimRayWhenControllerIsBelowFloor) {
    CaptureSession session;
    session.Start();

    Eigen::Affine3d pose = Eigen::Affine3d::Identity();
    pose.translation() = Eigen::Vector3d(1.25, -1.0, -0.5);
    EXPECT_FALSE(session.Tick(pose, true, 0.0));

    EXPECT_EQ(session.rawVertexCount(), 0u);
}

TEST(CaptureSessionTest, RejectsWhenControllerIsFarBelowFloor) {
    CaptureSession session;
    session.Start();

    Eigen::Affine3d pose = Eigen::Affine3d::Identity();
    pose.translation() = Eigen::Vector3d(0.0, -3.5, 0.0);
    EXPECT_FALSE(session.Tick(pose, true, 0.0));

    EXPECT_EQ(session.rawVertexCount(), 0u);
}

TEST(CaptureSessionTest, FinishCleansPaintedSquareToEdges) {
    CaptureSession session;
    session.Start();

    for (int i = 0; i <= 20; ++i) {
        session.Tick(MakeFloorPointerPose(i * 0.1, 0.0), true);
    }
    for (int i = 1; i <= 20; ++i) {
        session.Tick(MakeFloorPointerPose(2.0, i * 0.1), true);
    }
    for (int i = 1; i <= 20; ++i) {
        session.Tick(MakeFloorPointerPose(2.0 - i * 0.1, 2.0), true);
    }
    for (int i = 1; i <= 20; ++i) {
        session.Tick(MakeFloorPointerPose(0.0, 2.0 - i * 0.1), true);
    }

    session.Finish();

    const auto& verts = session.vertices();
    EXPECT_EQ(verts.size(), 4u);
    EXPECT_NEAR(AbsoluteAreaXZ(ProjectXZ(verts)), 4.0, 1e-9);
}

TEST(CaptureSessionTest, FinishUsesOuterHullForSelfIntersectingLoop) {
    CaptureSession session;
    session.Start();

    const std::vector<Eigen::Vector3d> points = {
        { 0.0, 1.0, 0.0 },
        { 2.0, 1.0, 2.0 },
        { 0.0, 1.0, 2.0 },
        { 2.0, 1.0, 0.0 },
        { 0.0, 1.0, 0.0 },
    };

    for (const auto& p : points) {
        Eigen::Affine3d pose = Eigen::Affine3d::Identity();
        pose.translation() = p;
        session.TickProjectedPosition(pose, true, 0.0);
    }

    session.Finish();
    const auto& verts = session.vertices();
    EXPECT_EQ(verts.size(), 4u);
    EXPECT_NEAR(AbsoluteAreaXZ(ProjectXZ(verts)), 4.0, 1e-9);
}

TEST(CaptureSessionTest, FinishClosesAtNearestEarlierPoint) {
    CaptureSession session;
    session.Start();

    const std::vector<Eigen::Vector3d> points = {
        { -5.0, 1.0, -5.0 },
        { -4.0, 1.0, -5.0 },
        {  0.0, 1.0,  0.0 },
        {  2.0, 1.0,  0.0 },
        {  2.0, 1.0,  2.0 },
        {  0.0, 1.0,  2.0 },
        {  0.05, 1.0, 0.03 },
    };

    for (const auto& p : points) {
        Eigen::Affine3d pose = Eigen::Affine3d::Identity();
        pose.translation() = p;
        session.TickProjectedPosition(pose, true, 0.0);
    }

    session.Finish();
    const auto& verts = session.vertices();
    EXPECT_EQ(verts.size(), 4u);
    EXPECT_NEAR(AbsoluteAreaXZ(ProjectXZ(verts)), 4.0, 0.25);
    for (const auto& v : verts) {
        EXPECT_GT(v.x, -1.0);
        EXPECT_GT(v.z, -1.0);
    }
}

TEST(CaptureSessionTest, FinishImplicitlyClosesOpenWalkedBoundary) {
    CaptureSession session;
    session.Start();

    const std::vector<Eigen::Vector3d> points = {
        { 0.0, 1.0, 0.0 },
        { 2.0, 1.0, 0.0 },
        { 2.0, 1.0, 2.0 },
        { 0.0, 1.0, 2.0 },
    };
    for (const auto& p : points) {
        Eigen::Affine3d pose = Eigen::Affine3d::Identity();
        pose.translation() = p;
        session.TickProjectedPosition(pose, true, 0.0);
    }

    session.Finish();
    const auto& verts = session.vertices();
    ASSERT_EQ(verts.size(), 4u);
    EXPECT_NEAR(AbsoluteAreaXZ(ProjectXZ(verts)), 4.0, 1e-9);
}

TEST(CaptureSessionTest, FinishHandlesDenseWalkWithoutFreezingOrExploding) {
    CaptureSession session;
    session.Start();

    for (int side = 0; side < 4; ++side) {
        for (int i = 0; i <= 160; ++i) {
            const double t = static_cast<double>(i) / 160.0;
            Eigen::Affine3d pose = Eigen::Affine3d::Identity();
            if (side == 0) pose.translation() = Eigen::Vector3d(2.0 * t, 1.0, 0.0);
            if (side == 1) pose.translation() = Eigen::Vector3d(2.0, 1.0, 2.0 * t);
            if (side == 2) pose.translation() = Eigen::Vector3d(2.0 - 2.0 * t, 1.0, 2.0);
            if (side == 3) pose.translation() = Eigen::Vector3d(0.0, 1.0, 2.0 - 2.0 * t);
            session.TickProjectedPosition(pose, true, 0.0);
        }
    }

    EXPECT_GT(session.rawVertexCount(), 32u);
    EXPECT_LT(session.rawVertexCount(), 260u);

    session.Finish();
    const auto& verts = session.vertices();
    EXPECT_EQ(verts.size(), 4u);
    EXPECT_NEAR(AbsoluteAreaXZ(ProjectXZ(verts)), 4.0, 0.05);
}

// ---------------------------------------------------------------------------
// ComputePolygonBoundsXZ
// ---------------------------------------------------------------------------

TEST(BoundaryBoundsTest, ComputesBoundsForCommonShapes) {
    struct BoundsCase {
        const char* name;
        std::vector<BoundaryVertex> vertices;
        std::array<double, 4> expected;
    };

    const std::vector<BoundsCase> cases = {
        { "empty", {}, { 0.0, 0.0, 0.0, 0.0 } },
        { "single_point", { { 3.0, 0.0, -2.0 } }, { 3.0, 3.0, -2.0, -2.0 } },
        { "unit_square", SquareBoundary(0.0, 0.0, 1.0, 1.0), { 0.0, 1.0, 0.0, 1.0 } },
        {
            "negative_coordinates",
            {
                { -3.0, 1.0, -4.0 },
                {  2.0, 0.5,  5.0 },
                {  0.0, 0.0,  0.0 },
            },
            { -3.0, 2.0, -4.0, 5.0 },
        },
    };

    for (const auto& c : cases) {
        SCOPED_TRACE(c.name);
        ExpectBoundsNear(
            ComputePolygonBoundsXZ(c.vertices),
            c.expected[0],
            c.expected[1],
            c.expected[2],
            c.expected[3]);
    }
}

// ---------------------------------------------------------------------------
// Boundary drawing preview
// ---------------------------------------------------------------------------

TEST(BoundaryPreviewTest, EmptyPathHasNoPlane) {
    std::vector<BoundaryVertex> empty;
    auto plane = ComputeBoundaryPreviewPlane(empty);
    EXPECT_FALSE(plane.valid);
}

TEST(BoundaryPreviewTest, PlaneCentersOnCapturedPath) {
    std::vector<BoundaryVertex> pts = {
        { -1.0, 0.0, -2.0 },
        {  3.0, 0.0,  2.0 },
    };

    auto plane = ComputeBoundaryPreviewPlane(pts);

    EXPECT_TRUE(plane.valid);
    EXPECT_NEAR(plane.centerX, 1.0, 1e-9);
    EXPECT_NEAR(plane.centerZ, 0.0, 1e-9);
    EXPECT_GT(plane.spanMeters, 4.0);
}

TEST(BoundaryPreviewTest, RasterMarksLivePathPixels) {
    std::vector<BoundaryVertex> pts = {
        { 0.0, 0.0, 0.0 },
        { 1.0, 0.0, 0.0 },
        { 1.0, 0.0, 1.0 },
    };

    auto openRaster = BuildBoundaryPreviewRaster(pts, false);
    auto closedRaster = BuildBoundaryPreviewRaster(pts, true);

    ASSERT_EQ(openRaster.rgba.size(), closedRaster.rgba.size());
    EXPECT_TRUE(openRaster.plane.valid);
    EXPECT_TRUE(closedRaster.plane.valid);
    EXPECT_NE(openRaster.hash, closedRaster.hash);

    const size_t openAlpha = CountAlphaPixels(openRaster);
    const size_t closedAlpha = CountAlphaPixels(closedRaster);

    EXPECT_GT(openAlpha, 0u);
    EXPECT_GT(closedAlpha, openAlpha);
}

TEST(SpatialFrameworkTest, TargetPrimitiveBuildsStandingRenderCommand) {
    Eigen::AffineCompact3d targetToStanding = Eigen::AffineCompact3d::Identity();
    targetToStanding.translation() = Eigen::Vector3d(1.0, 0.25, -2.0);
    const SpatialSpace target =
        TargetSpace("lighthouse", targetToStanding, 17);
    const SpatialSession session = BoundaryCaptureSessionDescriptor(
        target,
        4,
        "lighthouse",
        0.10,
        true,
        99);

    const std::vector<BoundaryVertex> targetPath = {
        { 0.0, 0.10, 0.0 },
        { 1.0, 0.10, 0.0 },
        { 1.0, 0.10, 1.0 },
    };
    const SpatialPrimitive primitive =
        BoundaryPathPrimitive(session, targetPath, true);
    const auto commands = BuildSpatialRenderCommands({ primitive });

    ASSERT_EQ(commands.size(), 1u);
    EXPECT_EQ(commands[0].kind, SpatialPrimitiveKind::PolygonFloorRegion);
    EXPECT_TRUE(commands[0].closeLoop);
    EXPECT_NEAR(commands[0].floorY, 0.35, 1e-9);
    ASSERT_EQ(commands[0].standingVertices.size(), targetPath.size());
    ExpectVertexNear(commands[0].standingVertices[0], 1.0, 0.35, -2.0);
    ExpectVertexNear(commands[0].standingVertices[2], 2.0, 0.35, -1.0);
}

TEST(SpatialFrameworkTest, RasterCanRenderMultipleCommands) {
    SpatialSession session = BoundaryCaptureSessionDescriptor(
        StandingSpace("lighthouse"),
        4,
        "lighthouse",
        0.0,
        false,
        1);
    const auto boundary = BoundaryPathPrimitive(
        session,
        SquareBoundary(-1.0, -1.0, 1.0, 1.0),
        true);
    const auto marker = FloorMarkerPrimitive(
        StandingSpace("lighthouse"),
        SquareBoundary(-0.15, -0.15, 0.15, 0.15),
        0.0);

    const auto raster = BuildBoundaryPreviewRaster(
        BuildSpatialRenderCommands({ boundary, marker }));

    EXPECT_TRUE(raster.plane.valid);
    EXPECT_GT(CountAlphaPixels(raster), 0u);
}

TEST(BoundaryPreviewTest, PerVertexDotsAddCoverageToTheDrawnPath) {
    std::vector<BoundaryVertex> verts = {
        { -1.0, 0.0, -1.0 },
        {  0.0, 0.0,  1.0 },
        {  1.0, 0.0, -1.0 },
    };

    SpatialRenderCommand lineOnly;
    lineOnly.kind = SpatialPrimitiveKind::PolylinePath;
    lineOnly.standingVertices = verts;
    lineOnly.ageFade = true;
    lineOnly.style.a = 235;
    lineOnly.style.strokeMeters = 0.03;
    lineOnly.style.dotMeters = 0.0;

    SpatialRenderCommand withDots = lineOnly;
    withDots.style.dotMeters = 0.05;

    const auto rasterLine = BuildBoundaryPreviewRaster({ lineOnly });
    const auto rasterDots = BuildBoundaryPreviewRaster({ withDots });

    ASSERT_TRUE(rasterDots.plane.valid);
    // A dot at every captured point paints more pixels than the bare line.
    EXPECT_GT(CountAlphaPixels(rasterDots), CountAlphaPixels(rasterLine));
}

TEST(ControllerSelectionTest, DefaultsToRightControllerWithValidPose) {
    using wkopenvr::controller_input::ChoosePreferredController;
    using wkopenvr::controller_input::ControllerSelectionChoice;

    const ControllerSelectionChoice choices[] = {
        { 3, vr::TrackedControllerRole_LeftHand, true },
        { 4, vr::TrackedControllerRole_RightHand, true },
    };

    EXPECT_EQ(ChoosePreferredController(choices, 2, -1), 4);
}

TEST(ControllerSelectionTest, PreservesCurrentSelection) {
    using wkopenvr::controller_input::ChoosePreferredController;
    using wkopenvr::controller_input::ControllerSelectionChoice;

    const ControllerSelectionChoice choices[] = {
        { 3, vr::TrackedControllerRole_LeftHand, true },
        { 4, vr::TrackedControllerRole_RightHand, true },
        { 5, vr::TrackedControllerRole_Invalid, true },
    };

    EXPECT_EQ(ChoosePreferredController(choices, 3, 5), 5);
}

TEST(ControllerSelectionTest, PrefersRightEvenBeforePoseThenAnyValidPose) {
    using wkopenvr::controller_input::ChoosePreferredController;
    using wkopenvr::controller_input::ControllerSelectionChoice;

    const ControllerSelectionChoice rightWaiting[] = {
        { 3, vr::TrackedControllerRole_LeftHand, true },
        { 4, vr::TrackedControllerRole_RightHand, false },
    };
    EXPECT_EQ(ChoosePreferredController(rightWaiting, 2, -1), 4);

    const ControllerSelectionChoice noRight[] = {
        { 6, vr::TrackedControllerRole_LeftHand, false },
        { 7, vr::TrackedControllerRole_Invalid, true },
    };
    EXPECT_EQ(ChoosePreferredController(noRight, 2, -1), 7);
}

TEST(BoundaryPreviewTest, RasterUsesModerateTextureForFastUploads) {
    EXPECT_EQ(BoundaryPreviewRaster::kTextureSize, 512);
}

TEST(BoundaryPreviewTest, ClosedRasterFillsInterior) {
    std::vector<BoundaryVertex> pts = {
        { -1.0, 0.0, -1.0 },
        {  1.0, 0.0, -1.0 },
        {  1.0, 0.0,  1.0 },
        { -1.0, 0.0,  1.0 },
    };

    auto openRaster = BuildBoundaryPreviewRaster(pts, false);
    auto closedRaster = BuildBoundaryPreviewRaster(pts, true);
    ASSERT_TRUE(openRaster.plane.valid);
    ASSERT_TRUE(closedRaster.plane.valid);

    const int center = BoundaryPreviewRaster::kTextureSize / 2;
    const size_t centerAlpha =
        static_cast<size_t>(center * BoundaryPreviewRaster::kTextureSize + center) * 4u + 3u;
    ASSERT_LT(centerAlpha, openRaster.rgba.size());

    EXPECT_EQ(openRaster.rgba[centerAlpha], 0u);
    EXPECT_GT(closedRaster.rgba[centerAlpha], 0u);
}

TEST(BoundaryPreviewTest, StyledMarkersRenderNewestWhiteAndOlderGray) {
    SpatialRenderCommand oldPoint;
    oldPoint.kind = SpatialPrimitiveKind::Marker;
    oldPoint.standingVertices = { { -0.30, 0.0, 0.0 } };
    oldPoint.style.r = 120;
    oldPoint.style.g = 120;
    oldPoint.style.b = 120;
    oldPoint.style.a = 210;
    oldPoint.style.fillA = 40;
    oldPoint.style.strokeMeters = 0.0;
    oldPoint.style.dotMeters = 0.080;
    oldPoint.layer = 1;

    SpatialRenderCommand newestPoint;
    newestPoint.kind = SpatialPrimitiveKind::Marker;
    newestPoint.standingVertices = { { 0.30, 0.0, 0.0 } };
    newestPoint.style.r = 255;
    newestPoint.style.g = 255;
    newestPoint.style.b = 255;
    newestPoint.style.a = 255;
    newestPoint.style.fillA = 80;
    newestPoint.style.strokeMeters = 0.0;
    newestPoint.style.dotMeters = 0.105;
    newestPoint.layer = 2;

    const auto raster = BuildBoundaryPreviewRaster({ oldPoint, newestPoint });
    ASSERT_TRUE(raster.plane.valid);

    const uint8_t oldR = RasterChannelAtWorld(raster, -0.30, 0.0, 0);
    const uint8_t oldG = RasterChannelAtWorld(raster, -0.30, 0.0, 1);
    const uint8_t oldB = RasterChannelAtWorld(raster, -0.30, 0.0, 2);
    const uint8_t newR = RasterChannelAtWorld(raster, 0.30, 0.0, 0);
    const uint8_t newG = RasterChannelAtWorld(raster, 0.30, 0.0, 1);
    const uint8_t newB = RasterChannelAtWorld(raster, 0.30, 0.0, 2);

    EXPECT_NEAR(oldR, oldG, 2);
    EXPECT_NEAR(oldG, oldB, 2);
    EXPECT_GT(newR, oldR);
    EXPECT_GT(newG, oldG);
    EXPECT_GT(newB, oldB);
    EXPECT_NEAR(newR, newG, 2);
    EXPECT_NEAR(newG, newB, 2);
}

TEST(BoundaryPreviewTest, CommandFillCanShowClosureWithoutPathMutation) {
    SpatialRenderCommand fill;
    fill.kind = SpatialPrimitiveKind::PolygonFloorRegion;
    fill.standingVertices = SquareBoundary(-1.0, -1.0, 1.0, 1.0);
    fill.closeLoop = true;
    fill.style.r = 210;
    fill.style.g = 210;
    fill.style.b = 210;
    fill.style.a = 0;
    fill.style.fillA = 72;
    fill.style.strokeMeters = 0.0;
    fill.style.dotMeters = 0.0;
    fill.style.fill = true;

    const auto raster = BuildBoundaryPreviewRaster({ fill });
    ASSERT_TRUE(raster.plane.valid);

    EXPECT_GT(RasterChannelAtWorld(raster, 0.0, 0.0, 3), 0u);
    EXPECT_EQ(fill.standingVertices.size(), 4u);
    EXPECT_TRUE(fill.closeLoop);
}

TEST(BoundaryPreviewTest, UsesStandingTrackingUniverseForFloorOverlay) {
    EXPECT_EQ(BoundaryPreviewTrackingOrigin(), vr::TrackingUniverseStanding);

    const auto mat = BoundaryPreviewTransform(1.25, -0.10, -2.50);
    ExpectMatrixTranslationNear(mat, 1.25f, -0.075f, -2.50f);

    // Local overlay X lies on world X, and local Y lies on world -Z.
    EXPECT_NEAR(mat.m[0][0], 1.0f, 1e-6f);
    EXPECT_NEAR(mat.m[2][1], -1.0f, 1e-6f);
}

TEST(BoundaryPreviewTest, FloorOverlayTransformIsWorldLockedAndOrthonormal) {
    struct TransformCase {
        const char* name;
        float x;
        float floorY;
        float z;
        std::array<float, 3> expectedTranslation;
    };

    const std::vector<TransformCase> cases = {
        { "origin", 0.0f, 0.0f, 0.0f, { 0.0f, 0.025f, 0.0f } },
        { "offset_positive", 1.25f, -0.10f, -2.50f, { 1.25f, -0.075f, -2.50f } },
        { "offset_negative", -0.50f, 0.0f, 1.75f, { -0.50f, 0.025f, 1.75f } },
    };

    for (const auto& c : cases) {
        SCOPED_TRACE(c.name);
        const auto mat = BoundaryPreviewTransform(c.x, c.floorY, c.z);

        const Eigen::Vector3d xAxis(mat.m[0][0], mat.m[1][0], mat.m[2][0]);
        const Eigen::Vector3d yAxis(mat.m[0][1], mat.m[1][1], mat.m[2][1]);
        const Eigen::Vector3d zAxis(mat.m[0][2], mat.m[1][2], mat.m[2][2]);

        EXPECT_NEAR(xAxis.norm(), 1.0, 1e-6);
        EXPECT_NEAR(yAxis.norm(), 1.0, 1e-6);
        EXPECT_NEAR(zAxis.norm(), 1.0, 1e-6);
        EXPECT_NEAR(xAxis.dot(yAxis), 0.0, 1e-6);
        EXPECT_NEAR(xAxis.dot(zAxis), 0.0, 1e-6);
        EXPECT_NEAR(yAxis.dot(zAxis), 0.0, 1e-6);
        EXPECT_NEAR(xAxis.cross(yAxis).dot(zAxis), 1.0, 1e-6);
        ExpectMatrixTranslationNear(
            mat,
            c.expectedTranslation[0],
            c.expectedTranslation[1],
            c.expectedTranslation[2]);
    }
}

TEST(BoundaryPreviewTest, StrokeWidthScalesToWorldSpace) {
    const auto compactRaster = BuildBoundaryPreviewRaster({
        { -0.5, 0.0, 0.0 },
        {  0.5, 0.0, 0.0 },
    }, false);
    const auto wideRaster = BuildBoundaryPreviewRaster({
        { -2.0, 0.0, 0.0 },
        {  2.0, 0.0, 0.0 },
    }, false);
    ASSERT_TRUE(compactRaster.plane.valid);
    ASSERT_TRUE(wideRaster.plane.valid);

    auto verticalAlphaRun = [](const BoundaryPreviewRaster& raster) {
        const int x = BoundaryPreviewRaster::kTextureSize / 2;
        int count = 0;
        for (int y = 0; y < BoundaryPreviewRaster::kTextureSize; ++y) {
            const size_t alpha =
                static_cast<size_t>(y * BoundaryPreviewRaster::kTextureSize + x) * 4u + 3u;
            if (raster.rgba[alpha] != 0) ++count;
        }
        return count;
    };

    EXPECT_GT(verticalAlphaRun(compactRaster), verticalAlphaRun(wideRaster));
}

TEST(BoundaryPreviewTest, UploadFailuresDisableAfterThreshold) {
    EXPECT_EQ(BoundaryPreviewUploadFailureDisableThreshold(), 3);
    EXPECT_FALSE(BoundaryPreviewShouldDisableUploadsAfterFailureCount(0));
    EXPECT_FALSE(BoundaryPreviewShouldDisableUploadsAfterFailureCount(2));
    EXPECT_TRUE(BoundaryPreviewShouldDisableUploadsAfterFailureCount(3));
    EXPECT_TRUE(BoundaryPreviewShouldDisableUploadsAfterFailureCount(4));
}

TEST(BoundaryPreviewTest, UsesRawRgbaUploadForHmdPreview) {
    // The preview pushes a CPU RGBA buffer via SetOverlayRaw (no GL context),
    // so the upload works from the capture-tick thread.
    EXPECT_FALSE(BoundaryPreviewUsesOpenGlTextureUpload());
}

TEST(BoundaryPreviewTest, StatusExposesInitialUploadDiagnostics) {
    ResetBoundaryPreviewUploadFailures();

    const auto status = GetBoundaryPreviewStatus();

    EXPECT_FALSE(status.uploadsDisabled);
    EXPECT_FALSE(status.fileMarkersVisible);
    EXPECT_EQ(status.uploadFailureCount, 0);
    EXPECT_EQ(status.fileMarkerFailureCount, 0);
    EXPECT_EQ(status.lastError, 0);
    EXPECT_EQ(status.fileMarkerLastError, 0);
    ASSERT_STREQ(status.lastErrorName, "None");
    ASSERT_STREQ(status.fileMarkerLastErrorName, "None");
    EXPECT_EQ(status.renderSize, BoundaryPreviewRaster::kTextureSize);
}

TEST(BoundaryPreviewTest, FileMarkersPreferExplicitMarkerCommands) {
    SpatialRenderCommand path;
    path.kind = SpatialPrimitiveKind::PolylinePath;
    path.standingVertices = {
        { 0.0, 0.0, 0.0 },
        { 1.0, 0.0, 1.0 },
    };
    path.style.dotMeters = 0.0;

    SpatialRenderCommand marker;
    marker.kind = SpatialPrimitiveKind::Marker;
    marker.standingVertices = {
        { 3.0, 0.1, 4.0 },
    };
    marker.style.r = 255;
    marker.style.g = 70;
    marker.style.b = 70;
    marker.style.dotMeters = 0.120;

    const auto markers = BuildBoundaryPreviewFileMarkers({ path, marker });

    ASSERT_EQ(markers.size(), 1u);
    EXPECT_DOUBLE_EQ(markers[0].vertex.x, 3.0);
    EXPECT_DOUBLE_EQ(markers[0].vertex.y, 0.1);
    EXPECT_DOUBLE_EQ(markers[0].vertex.z, 4.0);
    EXPECT_EQ(markers[0].style.r, 255);
    EXPECT_DOUBLE_EQ(markers[0].style.dotMeters, 0.120);
}

TEST(BoundaryPreviewTest, FileMarkersDownsampleWithoutDroppingEndpoints) {
    SpatialRenderCommand markersCommand;
    markersCommand.kind = SpatialPrimitiveKind::Marker;
    for (int i = 0; i < BoundaryPreviewFileMarkerLimit() + 8; ++i) {
        markersCommand.standingVertices.push_back(
            { static_cast<double>(i), 0.0, static_cast<double>(i) });
    }

    const auto markers = BuildBoundaryPreviewFileMarkers({ markersCommand });

    ASSERT_EQ(markers.size(), static_cast<size_t>(BoundaryPreviewFileMarkerLimit()));
    EXPECT_DOUBLE_EQ(markers.front().vertex.x, 0.0);
    EXPECT_DOUBLE_EQ(
        markers.back().vertex.x,
        static_cast<double>(BoundaryPreviewFileMarkerLimit() + 7));
}

TEST(BoundaryPreviewTest, PersistentBoundaryCommandsAreFilledOutlineNoMarkers) {
    const std::vector<BoundaryVertex> square = {
        { 0.0, 0.0, 0.0 },
        { 2.0, 0.0, 0.0 },
        { 2.0, 0.0, 2.0 },
        { 0.0, 0.0, 2.0 },
    };

    const auto commands = BuildPersistentBoundaryCommands(square, 0.0);

    int fillCount = 0;
    int outlineCount = 0;
    int markerCount = 0;
    for (const auto& command : commands) {
        if (command.kind == SpatialPrimitiveKind::Marker) {
            ++markerCount;
        } else if (command.kind == SpatialPrimitiveKind::PolygonFloorRegion) {
            ++fillCount;
            EXPECT_TRUE(command.style.fill);
            EXPECT_GT(command.style.fillA, 0);
            EXPECT_TRUE(command.closeLoop);
        } else if (command.kind == SpatialPrimitiveKind::PolylinePath) {
            ++outlineCount;
            EXPECT_FALSE(command.style.fill);
            EXPECT_GT(command.style.strokeMeters, 0.0);
            EXPECT_DOUBLE_EQ(command.style.dotMeters, 0.0);
            EXPECT_TRUE(command.closeLoop);
        }
    }
    EXPECT_EQ(fillCount, 1);
    EXPECT_EQ(outlineCount, 1);
    EXPECT_EQ(markerCount, 0);
}

TEST(BoundaryPreviewTest, PersistentBoundaryCommandsEmptyBelowThreeVertices) {
    const std::vector<BoundaryVertex> twoPoints = {
        { 0.0, 0.0, 0.0 },
        { 1.0, 0.0, 1.0 },
    };
    EXPECT_TRUE(BuildPersistentBoundaryCommands(twoPoints, 0.0).empty());
}

TEST(BoundaryPreviewTest, UploadBackoffGrowsAndCaps) {
    // No failures -> no backoff.
    EXPECT_DOUBLE_EQ(BoundaryUploadBackoffSeconds(0), 0.0);
    EXPECT_DOUBLE_EQ(BoundaryUploadBackoffSeconds(-3), 0.0);
    // Grows ~2x per consecutive failure.
    EXPECT_NEAR(BoundaryUploadBackoffSeconds(1), 0.1, 1e-9);
    EXPECT_NEAR(BoundaryUploadBackoffSeconds(2), 0.2, 1e-9);
    EXPECT_NEAR(BoundaryUploadBackoffSeconds(3), 0.4, 1e-9);
    EXPECT_NEAR(BoundaryUploadBackoffSeconds(4), 0.8, 1e-9);
    // Caps at 1.0s and never decreases.
    EXPECT_DOUBLE_EQ(BoundaryUploadBackoffSeconds(5), 1.0);
    EXPECT_DOUBLE_EQ(BoundaryUploadBackoffSeconds(100), 1.0);
    EXPECT_GE(BoundaryUploadBackoffSeconds(3), BoundaryUploadBackoffSeconds(2));
}

TEST(BoundaryPreviewTest, FileMarkerTransformUsesFloorPlaneTransform) {
    const BoundaryVertex marker{ 1.25, -0.10, -2.50 };

    const auto mat = BoundaryPreviewFileMarkerTransform(marker);

    EXPECT_FLOAT_EQ(mat.m[0][3], 1.25f);
    EXPECT_FLOAT_EQ(mat.m[1][3], -0.075f);
    EXPECT_FLOAT_EQ(mat.m[2][3], -2.50f);
    EXPECT_FLOAT_EQ(mat.m[1][2], 1.0f);
    EXPECT_FLOAT_EQ(mat.m[2][1], -1.0f);
}

TEST(ChaperoneWorkingSetTest, BuildsPerimeterQuadsAndPlayArea) {
    const std::vector<BoundaryVertex> verts = {
        { -1.0, 0.0, -2.0 },
        {  2.0, 0.0, -2.0 },
        {  2.0, 0.0,  1.0 },
        { -1.0, 0.0,  1.0 },
    };

    const auto workingSet = BuildChaperoneWorkingSet(verts, -0.10, 2.40);

    ASSERT_TRUE(workingSet.valid);
    EXPECT_NEAR(workingSet.playAreaX, 2.0f, 1e-6f);
    EXPECT_NEAR(workingSet.playAreaZ, 2.0f, 1e-6f);
    ASSERT_EQ(workingSet.perimeter.size(), 4u);
    EXPECT_NEAR(workingSet.perimeter[0].v[0], -1.0f, 1e-6f);
    EXPECT_NEAR(workingSet.perimeter[0].v[1], -2.0f, 1e-6f);
    ASSERT_EQ(workingSet.collisionBounds.size(), 4u);
    const auto& q = workingSet.collisionBounds[0];
    EXPECT_NEAR(q.vCorners[0].v[0], -1.0f, 1e-6f);
    EXPECT_NEAR(q.vCorners[0].v[1], -0.10f, 1e-6f);
    EXPECT_NEAR(q.vCorners[0].v[2], -2.0f, 1e-6f);
    EXPECT_NEAR(q.vCorners[1].v[0], 2.0f, 1e-6f);
    EXPECT_NEAR(q.vCorners[1].v[1], -0.10f, 1e-6f);
    EXPECT_NEAR(q.vCorners[2].v[1], 2.40f, 1e-6f);
}

TEST(ChaperoneWorkingSetTest, MatchesTransformedCapturedBoundaryNumbers) {
    const std::vector<BoundaryVertex> targetVerts = {
        { -1.0, 0.0, -0.5 },
        {  1.0, 0.0, -0.5 },
        {  1.0, 0.0,  0.5 },
        { -1.0, 0.0,  0.5 },
    };
    Eigen::AffineCompact3d targetToStanding = Eigen::AffineCompact3d::Identity();
    targetToStanding.linear() = Eigen::AngleAxisd(
        EIGEN_PI * 0.5,
        Eigen::Vector3d::UnitY()).toRotationMatrix();

    const auto standingVerts =
        TransformToStandingUniverse(targetVerts, targetToStanding);
    const auto workingSet = BuildChaperoneWorkingSet(standingVerts, 0.0, 2.4);

    ASSERT_TRUE(workingSet.valid);
    ASSERT_EQ(workingSet.perimeter.size(), 4u);
    EXPECT_NEAR(workingSet.perimeter[0].v[0], -0.5f, 1e-6f);
    EXPECT_NEAR(workingSet.perimeter[0].v[1], 1.0f, 1e-6f);
    EXPECT_NEAR(workingSet.perimeter[1].v[0], -0.5f, 1e-6f);
    EXPECT_NEAR(workingSet.perimeter[1].v[1], -1.0f, 1e-6f);
    EXPECT_NEAR(workingSet.perimeter[2].v[0], 0.5f, 1e-6f);
    EXPECT_NEAR(workingSet.perimeter[2].v[1], -1.0f, 1e-6f);
    EXPECT_NEAR(workingSet.perimeter[3].v[0], 0.5f, 1e-6f);
    EXPECT_NEAR(workingSet.perimeter[3].v[1], 1.0f, 1e-6f);
    EXPECT_NEAR(workingSet.playAreaX, 1.0f, 1e-6f);
    EXPECT_NEAR(workingSet.playAreaZ, 2.0f, 1e-6f);
    ASSERT_EQ(workingSet.collisionBounds.size(), 4u);
    EXPECT_NEAR(workingSet.collisionBounds[0].vCorners[0].v[1], 0.0f, 1e-6f);
    EXPECT_NEAR(workingSet.collisionBounds[0].vCorners[2].v[1], 2.4f, 1e-6f);
}

TEST(ChaperoneWorkingSetTest, DynamicRectanglesProduceExpectedWorkingSets) {
    struct WorkingSetCase {
        const char* name;
        std::vector<BoundaryVertex> vertices;
        double floorY;
        double ceilingY;
        float playX;
        float playZ;
    };

    const std::vector<WorkingSetCase> cases = {
        { "centered_square", SquareBoundary(-1.0, -1.0, 1.0, 1.0), 0.0, 2.4, 2.0f, 2.0f },
        { "offset_rectangle", SquareBoundary(-1.0, -2.0, 2.0, 1.0), -0.10, 2.40, 2.0f, 2.0f },
        { "thin_room", SquareBoundary(-0.75, -2.50, 0.75, 2.50), 0.05, 2.10, 1.5f, 5.0f },
    };

    for (const auto& c : cases) {
        SCOPED_TRACE(c.name);
        const auto workingSet =
            BuildChaperoneWorkingSet(c.vertices, c.floorY, c.ceilingY);

        ASSERT_TRUE(workingSet.valid);
        EXPECT_NEAR(workingSet.playAreaX, c.playX, 1e-6f);
        EXPECT_NEAR(workingSet.playAreaZ, c.playZ, 1e-6f);
        ASSERT_EQ(workingSet.perimeter.size(), c.vertices.size());
        ASSERT_EQ(workingSet.collisionBounds.size(), c.vertices.size());

        for (size_t i = 0; i < c.vertices.size(); ++i) {
            const auto& vertex = c.vertices[i];
            const auto& perimeter = workingSet.perimeter[i];
            EXPECT_NEAR(perimeter.v[0], static_cast<float>(vertex.x), 1e-6f);
            EXPECT_NEAR(perimeter.v[1], static_cast<float>(vertex.z), 1e-6f);

            const auto& quad = workingSet.collisionBounds[i];
            EXPECT_NEAR(quad.vCorners[0].v[1], static_cast<float>(c.floorY), 1e-6f);
            EXPECT_NEAR(quad.vCorners[1].v[1], static_cast<float>(c.floorY), 1e-6f);
            EXPECT_NEAR(quad.vCorners[2].v[1], static_cast<float>(c.ceilingY), 1e-6f);
            EXPECT_NEAR(quad.vCorners[3].v[1], static_cast<float>(c.ceilingY), 1e-6f);
        }
    }
}

TEST(ChaperoneWorkingSetTest, RemovesDuplicateClosingVertexBeforeOpenVrPerimeter) {
    std::vector<BoundaryVertex> verts = SquareBoundary(-1.0, -1.0, 1.0, 1.0);
    verts.push_back(verts.front());

    const auto workingSet = BuildChaperoneWorkingSet(verts, 0.0, 2.4);

    ASSERT_TRUE(workingSet.valid);
    EXPECT_EQ(workingSet.perimeter.size(), 4u);
    EXPECT_EQ(workingSet.collisionBounds.size(), 4u);
}

TEST(ChaperoneWorkingSetTest, PlayAreaStaysCenteredOnOpenVrTrackingOrigin) {
    const auto workingSet = BuildChaperoneWorkingSet(
        SquareBoundary(-0.60, -1.25, 1.80, 2.25),
        0.0,
        2.4);

    ASSERT_TRUE(workingSet.valid);
    EXPECT_NEAR(workingSet.playAreaX, 1.20f, 1e-6f);
    EXPECT_NEAR(workingSet.playAreaZ, 2.50f, 1e-6f);
}

TEST(ChaperoneWorkingSetTest, RejectsBoundaryThatDoesNotContainStandingOrigin) {
    const auto workingSet = BuildChaperoneWorkingSet(
        SquareBoundary(1.0, 1.0, 3.0, 3.0),
        0.0,
        2.4);

    EXPECT_FALSE(workingSet.valid);

    const auto output = BuildChaperoneOutput(
        SquareBoundary(1.0, 1.0, 3.0, 3.0),
        0.0,
        2.4);
    EXPECT_FALSE(output.ready());
    EXPECT_EQ(output.status, ChaperoneOutputStatus::VisualOnlyNoStandingOrigin);
    ASSERT_STREQ(output.reason, "standing_origin_outside_polygon");
    EXPECT_FALSE(output.diagnostics.originInsidePolygon);
    EXPECT_NEAR(output.diagnostics.originDistanceMeters, std::sqrt(2.0), 1e-9);
    EXPECT_NEAR(output.diagnostics.centroidX, 2.0, 1e-9);
    EXPECT_NEAR(output.diagnostics.centroidZ, 2.0, 1e-9);
    EXPECT_NEAR(output.diagnostics.areaMetersSq, 4.0, 1e-9);
}

TEST(ChaperoneWorkingSetTest, RejectsDegenerateAndNonFiniteGeometry) {
    EXPECT_FALSE(BuildChaperoneWorkingSet({
        { 0.0, 0.0, 0.0 },
        { 0.2, 0.0, 0.0 },
        { 0.4, 0.0, 0.0 },
    }, 0.0, 2.4).valid);

    EXPECT_FALSE(BuildChaperoneWorkingSet({
        { -1.0, 0.0, -1.0 },
        {  1.0, 0.0, -1.0 },
        {  std::numeric_limits<double>::quiet_NaN(), 0.0, 1.0 },
    }, 0.0, 2.4).valid);
}

TEST(ChaperoneWorkingSetTest, PipelineMapsTargetFloorToOpenVrStandingFloor) {
    const std::vector<BoundaryVertex> targetVerts = SquareBoundary(-1.0, -1.0, 1.0, 1.0, 0.30);

    Eigen::AffineCompact3d targetToStanding = Eigen::AffineCompact3d::Identity();
    targetToStanding.linear() = Eigen::AngleAxisd(
        12.0 * EIGEN_PI / 180.0,
        Eigen::Vector3d::UnitZ()).toRotationMatrix();
    targetToStanding.translation() = Eigen::Vector3d(0.15, 0.85, -0.20);

    const double targetFloor =
        TargetFloorYForStandingFloor(targetVerts, targetToStanding, 0.0);
    const double standingFloor =
        TransformHeightToStandingUniverse(targetVerts, targetFloor, targetToStanding);
    const auto standingVerts =
        TransformToStandingUniverse(targetVerts, targetToStanding);
    const auto workingSet = BuildChaperoneWorkingSet(
        standingVerts,
        standingFloor,
        standingFloor + 2.4);

    EXPECT_NEAR(standingFloor, 0.0, 1e-9);
    ASSERT_TRUE(workingSet.valid);
    for (const auto& quad : workingSet.collisionBounds) {
        EXPECT_NEAR(quad.vCorners[0].v[1], 0.0f, 1e-6f);
        EXPECT_NEAR(quad.vCorners[1].v[1], 0.0f, 1e-6f);
    }
}

TEST(ChaperoneWorkingSetTest, RejectsInvalidGeometry) {
    EXPECT_FALSE(BuildChaperoneWorkingSet({}, 0.0, 2.4).valid);
    EXPECT_FALSE(BuildChaperoneWorkingSet({
        { 0.0, 0.0, 0.0 },
        { 1.0, 0.0, 0.0 },
        { 1.0, 0.0, 1.0 },
    }, 2.4, 0.0).valid);
}

TEST(ChaperoneSnapshotTest, RoundTripsFullSnapshotFormat) {
    ChaperoneSnapshot snapshot;
    snapshot.hasPlayArea = true;
    snapshot.playAreaX = 2.5f;
    snapshot.playAreaZ = 3.5f;
    snapshot.hasStandingZero = true;
    snapshot.standingZeroToRaw = MakeIdentityMatrix34();
    SetTranslation(snapshot.standingZeroToRaw, 0.1f, 0.2f, 0.3f);
    snapshot.hasSeatedZero = true;
    snapshot.seatedZeroToRaw = MakeIdentityMatrix34();
    SetTranslation(snapshot.seatedZeroToRaw, -0.1f, -0.2f, -0.3f);
    vr::HmdVector2_t p{};
    p.v[0] = -1.0f;
    p.v[1] = 2.0f;
    snapshot.perimeter.push_back(p);
    snapshot.collisionBounds.push_back(MakeWallQuad(-1.0, -1.0, 1.0, -1.0, 0.0, 2.4));

    const auto bytes = SerializeChaperoneSnapshot(snapshot);
    ChaperoneSnapshot parsed;
    std::string error;

    ASSERT_TRUE(DeserializeChaperoneSnapshot(bytes, parsed, &error)) << error;
    EXPECT_FALSE(parsed.legacyCollisionBoundsOnly);
    EXPECT_TRUE(parsed.hasPlayArea);
    EXPECT_NEAR(parsed.playAreaX, 2.5f, 1e-6f);
    EXPECT_NEAR(parsed.playAreaZ, 3.5f, 1e-6f);
    EXPECT_TRUE(parsed.hasStandingZero);
    ExpectMatrixTranslationNear(parsed.standingZeroToRaw, 0.1f, 0.2f, 0.3f);
    EXPECT_TRUE(parsed.hasSeatedZero);
    ExpectMatrixTranslationNear(parsed.seatedZeroToRaw, -0.1f, -0.2f, -0.3f);
    ASSERT_EQ(parsed.perimeter.size(), 1u);
    EXPECT_NEAR(parsed.perimeter[0].v[0], -1.0f, 1e-6f);
    EXPECT_NEAR(parsed.perimeter[0].v[1], 2.0f, 1e-6f);
    ASSERT_EQ(parsed.collisionBounds.size(), 1u);
    EXPECT_NEAR(parsed.collisionBounds[0].vCorners[2].v[1], 2.4f, 1e-6f);
}

TEST(ChaperoneSnapshotTest, ReadsLegacyCollisionOnlySnapshot) {
    const vr::HmdQuad_t quad = MakeWallQuad(-1.0, -2.0, 1.0, -2.0, 0.0, 2.4);
    std::vector<uint8_t> bytes;
    bytes.push_back(1u);
    bytes.push_back(0u);
    bytes.push_back(0u);
    bytes.push_back(0u);
    const uint8_t* raw = reinterpret_cast<const uint8_t*>(&quad);
    bytes.insert(bytes.end(), raw, raw + sizeof(quad));

    ChaperoneSnapshot parsed;
    std::string error;
    ASSERT_TRUE(DeserializeChaperoneSnapshot(bytes, parsed, &error)) << error;

    EXPECT_TRUE(parsed.legacyCollisionBoundsOnly);
    EXPECT_FALSE(parsed.hasPlayArea);
    ASSERT_EQ(parsed.collisionBounds.size(), 1u);
    ASSERT_EQ(parsed.perimeter.size(), 1u);
    EXPECT_NEAR(parsed.perimeter[0].v[0], -1.0f, 1e-6f);
    EXPECT_NEAR(parsed.perimeter[0].v[1], -2.0f, 1e-6f);
}

// ---------------------------------------------------------------------------
// BoundaryRePush startup-push countdown. Regression pins for the boundary being
// pushed with a stale (identity) transform: the countdown must be HELD while
// the boundary is not pushable and fire only after kStartupGraceTicks pushable
// ticks, so the one push lands on a converged transform.
// ---------------------------------------------------------------------------

TEST(BoundaryRePushStartupTest, HeldWhileNotPushable) {
    using boundary_repush::StepStartupCountdown;
    int pending = 30;
    for (int i = 0; i < 100; ++i) {
        const auto sc = StepStartupCountdown(/*pushable=*/false, pending);
        pending = sc.pending;
        EXPECT_FALSE(sc.fire);
        EXPECT_EQ(pending, 30) << "countdown must not be consumed while not pushable";
    }
}

TEST(BoundaryRePushStartupTest, FiresAfterGraceOfPushableTicks) {
    using boundary_repush::StepStartupCountdown;
    int pending = 30;
    int fireTick = -1;
    for (int i = 0; i < 30; ++i) {
        const auto sc = StepStartupCountdown(/*pushable=*/true, pending);
        pending = sc.pending;
        if (sc.fire) { fireTick = i; break; }
    }
    EXPECT_EQ(fireTick, 29) << "should fire on the 30th pushable tick";
    EXPECT_EQ(pending, 0);
}

TEST(BoundaryRePushStartupTest, OnlyPushableTicksCountTowardGrace) {
    using boundary_repush::StepStartupCountdown;
    int pending = 3;
    auto step = [&](bool pushable) {
        const auto sc = StepStartupCountdown(pushable, pending);
        pending = sc.pending;
        return sc.fire;
    };
    EXPECT_FALSE(step(false)); EXPECT_EQ(pending, 3);
    EXPECT_FALSE(step(true));  EXPECT_EQ(pending, 2);
    EXPECT_FALSE(step(false)); EXPECT_EQ(pending, 2);
    EXPECT_FALSE(step(true));  EXPECT_EQ(pending, 1);
    EXPECT_FALSE(step(false)); EXPECT_EQ(pending, 1);
    EXPECT_TRUE(step(true));   EXPECT_EQ(pending, 0);  // 3rd pushable tick fires
}

TEST(BoundaryRePushStartupTest, ZeroPendingNeverFires) {
    using boundary_repush::StepStartupCountdown;
    auto sc = StepStartupCountdown(/*pushable=*/true, 0);
    EXPECT_FALSE(sc.fire);
    EXPECT_EQ(sc.pending, 0);
    sc = StepStartupCountdown(/*pushable=*/false, 0);
    EXPECT_FALSE(sc.fire);
    EXPECT_EQ(sc.pending, 0);
}

// The persistent boundary overlay skips its 512x512 raster rebuild when the
// content hash is unchanged, so the hash must be deterministic for identical
// geometry and must change when the geometry moves.
TEST(BoundaryPreviewRasterTest, HashIsStableAndContentSensitive) {
    std::vector<BoundaryVertex> verts = {
        {0.0, 0.0, 0.0}, {1.5, 0.0, 0.0}, {1.5, 0.0, 1.2}, {0.0, 0.0, 1.2},
    };
    const BoundaryPreviewRaster a = BuildBoundaryPreviewRaster(verts, true);
    const BoundaryPreviewRaster b = BuildBoundaryPreviewRaster(verts, true);
    EXPECT_EQ(a.hash, b.hash);

    std::vector<BoundaryVertex> moved = verts;
    moved[2].x += 0.30;
    const BoundaryPreviewRaster c = BuildBoundaryPreviewRaster(moved, true);
    EXPECT_NE(a.hash, c.hash);
}

// Floor on/off applies +offset then -offset to the standing-zero pose. Toggling
// off must return exactly to the pre-floor pose so the saved offset can be
// re-applied later without drift.
TEST(FloorToggleTest, StandingZeroOffsetRoundTrips) {
    vr::HmdMatrix34_t base = MakeIdentityMatrix34();
    const double offset = 0.42;
    const vr::HmdMatrix34_t lowered = OffsetStandingZeroPoseForFloor(base, offset);
    const vr::HmdMatrix34_t restored = OffsetStandingZeroPoseForFloor(lowered, -offset);
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 4; ++c) {
            EXPECT_NEAR(restored.m[r][c], base.m[r][c], 1e-6);
        }
    }
}
