// Tests for the safety boundary backend: Douglas-Peucker simplification,
// polygon area, coordinate transform, and CaptureSession state machine.
// Chaperone IVRChaperoneSetup calls are not exercised here -- they require
// a running SteamVR instance.

#include "Boundary.h"
#include "BoundaryCapture.h"
#include "BoundaryPreview.h"

#include <Eigen/Geometry>

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

using namespace wkopenvr::boundary;

namespace {

Eigen::Affine3d MakeFloorPointerPose(double x, double z, double y = 1.0) {
    Eigen::Affine3d pose = Eigen::Affine3d::Identity();
    pose.translation() = Eigen::Vector3d(x, y, z);
    pose.linear() = Eigen::AngleAxisd(
        -3.14159265358979323846 / 2.0,
        Eigen::Vector3d::UnitX()).toRotationMatrix();
    return pose;
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

TEST(CaptureSessionTest, FallsBackToControllerPositionWhenControllerIsBelowFloor) {
    CaptureSession session;
    session.Start();

    Eigen::Affine3d pose = Eigen::Affine3d::Identity();
    pose.translation() = Eigen::Vector3d(1.25, -1.0, -0.5);
    EXPECT_TRUE(session.Tick(pose, true, 0.0));

    ASSERT_EQ(session.rawVertexCount(), 1u);
    const auto& verts = session.vertices();
    ASSERT_EQ(verts.size(), 1u);
    EXPECT_NEAR(verts[0].x, 1.25, 1e-9);
    EXPECT_NEAR(verts[0].y, 0.0, 1e-9);
    EXPECT_NEAR(verts[0].z, -0.5, 1e-9);
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

// ---------------------------------------------------------------------------
// ComputePolygonBoundsXZ
// ---------------------------------------------------------------------------

TEST(BoundaryBoundsTest, EmptyReturnsZero) {
    std::vector<BoundaryVertex> empty;
    auto b = ComputePolygonBoundsXZ(empty);
    EXPECT_EQ(b.xMin, 0.0);
    EXPECT_EQ(b.xMax, 0.0);
    EXPECT_EQ(b.zMin, 0.0);
    EXPECT_EQ(b.zMax, 0.0);
}

TEST(BoundaryBoundsTest, SinglePoint) {
    std::vector<BoundaryVertex> v = { { 3.0, 0.0, -2.0 } };
    auto b = ComputePolygonBoundsXZ(v);
    EXPECT_NEAR(b.xMin, 3.0, 1e-9);
    EXPECT_NEAR(b.xMax, 3.0, 1e-9);
    EXPECT_NEAR(b.zMin, -2.0, 1e-9);
    EXPECT_NEAR(b.zMax, -2.0, 1e-9);
}

TEST(BoundaryBoundsTest, UnitSquareXZ) {
    std::vector<BoundaryVertex> square = {
        { 0.0, 0.0, 0.0 },
        { 1.0, 0.0, 0.0 },
        { 1.0, 0.0, 1.0 },
        { 0.0, 0.0, 1.0 },
    };
    auto b = ComputePolygonBoundsXZ(square);
    EXPECT_NEAR(b.xMin, 0.0, 1e-9);
    EXPECT_NEAR(b.xMax, 1.0, 1e-9);
    EXPECT_NEAR(b.zMin, 0.0, 1e-9);
    EXPECT_NEAR(b.zMax, 1.0, 1e-9);
}

TEST(BoundaryBoundsTest, NegativeCoordinates) {
    std::vector<BoundaryVertex> pts = {
        { -3.0, 1.0, -4.0 },
        {  2.0, 0.5,  5.0 },
        {  0.0, 0.0,  0.0 },
    };
    auto b = ComputePolygonBoundsXZ(pts);
    EXPECT_NEAR(b.xMin, -3.0, 1e-9);
    EXPECT_NEAR(b.xMax,  2.0, 1e-9);
    EXPECT_NEAR(b.zMin, -4.0, 1e-9);
    EXPECT_NEAR(b.zMax,  5.0, 1e-9);
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

    size_t openAlpha = 0;
    size_t closedAlpha = 0;
    for (size_t i = 3; i < openRaster.rgba.size(); i += 4) {
        if (openRaster.rgba[i] != 0) ++openAlpha;
        if (closedRaster.rgba[i] != 0) ++closedAlpha;
    }

    EXPECT_GT(openAlpha, 0u);
    EXPECT_GT(closedAlpha, openAlpha);
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

TEST(BoundaryPreviewTest, UsesStandingTrackingUniverseForFloorOverlay) {
    EXPECT_EQ(BoundaryPreviewTrackingOrigin(), vr::TrackingUniverseStanding);

    const auto mat = BoundaryPreviewTransform(1.25, -0.10, -2.50);
    EXPECT_NEAR(mat.m[0][3], 1.25f, 1e-6f);
    EXPECT_NEAR(mat.m[1][3], -0.075f, 1e-6f);
    EXPECT_NEAR(mat.m[2][3], -2.50f, 1e-6f);

    // Local overlay X lies on world X, and local Y lies on world -Z.
    EXPECT_NEAR(mat.m[0][0], 1.0f, 1e-6f);
    EXPECT_NEAR(mat.m[2][1], -1.0f, 1e-6f);
}
