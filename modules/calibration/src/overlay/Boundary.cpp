#include "Boundary.h"
#include "CalibrationMetrics.h"

#include <openvr.h>

#include <Eigen/Dense>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

namespace wkopenvr::boundary {

static bool ChaperoneSetupOk();

// ---------------------------------------------------------------------------
// Area helpers
// ---------------------------------------------------------------------------

double SignedAreaXZ(const std::vector<XZPoint>& poly) {
    const size_t n = poly.size();
    if (n < 3) return 0.0;
    double area = 0.0;
    for (size_t i = 0, j = n - 1; i < n; j = i++) {
        area += (poly[j].x + poly[i].x) * (poly[j].z - poly[i].z);
    }
    return area * 0.5;
}

double AbsoluteAreaXZ(const std::vector<XZPoint>& poly) {
    double a = SignedAreaXZ(poly);
    return a < 0.0 ? -a : a;
}

// ---------------------------------------------------------------------------
// Projection
// ---------------------------------------------------------------------------

std::vector<XZPoint> ProjectXZ(const std::vector<BoundaryVertex>& v) {
    std::vector<XZPoint> out;
    out.reserve(v.size());
    for (const auto& bv : v) {
        out.push_back({ bv.x, bv.z });
    }
    return out;
}

// ---------------------------------------------------------------------------
// Douglas-Peucker simplification
// ---------------------------------------------------------------------------

// Squared perpendicular distance from point p to the segment (a, b) in 3D.
static double PerpendicularDistanceSq(const BoundaryVertex& p,
                                      const BoundaryVertex& a,
                                      const BoundaryVertex& b) {
    double dx = b.x - a.x, dy = b.y - a.y, dz = b.z - a.z;
    double lenSq = dx*dx + dy*dy + dz*dz;
    double px = p.x - a.x, py = p.y - a.y, pz = p.z - a.z;
    if (lenSq < 1e-20) {
        // Degenerate segment: use point-to-point distance.
        return px*px + py*py + pz*pz;
    }
    double t = (px*dx + py*dy + pz*dz) / lenSq;
    t = std::max(0.0, std::min(1.0, t));
    double rx = px - t*dx, ry = py - t*dy, rz = pz - t*dz;
    return rx*rx + ry*ry + rz*rz;
}

static void DouglasPeuckerRec(const std::vector<BoundaryVertex>& path,
                               size_t lo, size_t hi,
                               double epsilonSq,
                               std::vector<bool>& keep) {
    if (hi <= lo + 1) return;
    double maxDist = 0.0;
    size_t maxIdx = lo + 1;
    for (size_t i = lo + 1; i < hi; ++i) {
        double d = PerpendicularDistanceSq(path[i], path[lo], path[hi]);
        if (d > maxDist) {
            maxDist = d;
            maxIdx = i;
        }
    }
    if (maxDist > epsilonSq) {
        keep[maxIdx] = true;
        DouglasPeuckerRec(path, lo, maxIdx, epsilonSq, keep);
        DouglasPeuckerRec(path, maxIdx, hi, epsilonSq, keep);
    }
}

std::vector<size_t> SimplifyDouglasPeucker(const std::vector<BoundaryVertex>& path,
                                            double epsilonMeters) {
    const size_t n = path.size();
    if (n <= 2) {
        std::vector<size_t> indices;
        for (size_t i = 0; i < n; ++i) indices.push_back(i);
        return indices;
    }
    std::vector<bool> keep(n, false);
    keep[0] = true;
    keep[n - 1] = true;
    const double epsilonSq = epsilonMeters * epsilonMeters;
    DouglasPeuckerRec(path, 0, n - 1, epsilonSq, keep);
    std::vector<size_t> out;
    out.reserve(n);
    for (size_t i = 0; i < n; ++i) {
        if (keep[i]) out.push_back(i);
    }
    return out;
}

// ---------------------------------------------------------------------------
// Transform
// ---------------------------------------------------------------------------

std::vector<BoundaryVertex> TransformToStandingUniverse(
    const std::vector<BoundaryVertex>& targetSpace,
    const Eigen::AffineCompact3d& targetToStanding) {
    std::vector<BoundaryVertex> out;
    out.reserve(targetSpace.size());
    for (const auto& bv : targetSpace) {
        Eigen::Vector3d p(bv.x, bv.y, bv.z);
        Eigen::Vector3d t = targetToStanding * p;
        out.push_back({ t.x(), t.y(), t.z() });
    }
    return out;
}

double TransformHeightToStandingUniverse(
    double targetY,
    const Eigen::AffineCompact3d& targetToStanding)
{
    const Eigen::Vector3d p = targetToStanding * Eigen::Vector3d(0.0, targetY, 0.0);
    return p.y();
}

double TransformHeightToStandingUniverse(
    const std::vector<BoundaryVertex>& targetSpace,
    double targetY,
    const Eigen::AffineCompact3d& targetToStanding)
{
    if (targetSpace.empty()) {
        return TransformHeightToStandingUniverse(targetY, targetToStanding);
    }

    double minX = targetSpace[0].x;
    double maxX = targetSpace[0].x;
    double minZ = targetSpace[0].z;
    double maxZ = targetSpace[0].z;
    for (const auto& v : targetSpace) {
        if (v.x < minX) minX = v.x;
        if (v.x > maxX) maxX = v.x;
        if (v.z < minZ) minZ = v.z;
        if (v.z > maxZ) maxZ = v.z;
    }

    const Eigen::Vector3d p = targetToStanding * Eigen::Vector3d(
        (minX + maxX) * 0.5,
        targetY,
        (minZ + maxZ) * 0.5);
    return p.y();
}

double TargetFloorYForStandingFloor(
    const std::vector<BoundaryVertex>& targetSpace,
    const Eigen::AffineCompact3d& targetToStanding,
    double standingFloorY)
{
    double centerX = 0.0;
    double centerZ = 0.0;
    if (!targetSpace.empty()) {
        double minX = targetSpace[0].x;
        double maxX = targetSpace[0].x;
        double minZ = targetSpace[0].z;
        double maxZ = targetSpace[0].z;
        for (const auto& v : targetSpace) {
            if (v.x < minX) minX = v.x;
            if (v.x > maxX) maxX = v.x;
            if (v.z < minZ) minZ = v.z;
            if (v.z > maxZ) maxZ = v.z;
        }
        centerX = (minX + maxX) * 0.5;
        centerZ = (minZ + maxZ) * 0.5;
    }

    const auto& m = targetToStanding.matrix();
    const double yCoeff = m(1, 1);
    if (std::fabs(yCoeff) < 1e-9) {
        return 0.0;
    }

    return (standingFloorY - m(1, 0) * centerX - m(1, 2) * centerZ - m(1, 3))
        / yCoeff;
}

Eigen::AffineCompact3d ProfileTransformFromCalibration(
    Eigen::Vector3d eulerDeg,
    Eigen::Vector3d transCm)
{
    const Eigen::Vector3d euler = eulerDeg * EIGEN_PI / 180.0;
    const Eigen::Quaterniond rot =
        Eigen::AngleAxisd(euler(0), Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(euler(1), Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(euler(2), Eigen::Vector3d::UnitX());

    Eigen::AffineCompact3d transform = Eigen::AffineCompact3d::Identity();
    transform.linear() = rot.toRotationMatrix();
    transform.translation() = transCm * 0.01;
    return transform;
}

Eigen::Affine3d TransformPoseToStandingUniverse(
    const Eigen::Affine3d& rawPose,
    const Eigen::AffineCompact3d& targetToStanding)
{
    Eigen::Affine3d transform = Eigen::Affine3d::Identity();
    transform.linear() = targetToStanding.linear();
    transform.translation() = targetToStanding.translation();
    return transform * rawPose;
}

vr::HmdMatrix34_t OffsetStandingZeroPoseForFloor(
    const vr::HmdMatrix34_t& standingZeroToRaw,
    double measuredFloorYStanding)
{
    vr::HmdMatrix34_t out = standingZeroToRaw;
    const float y = static_cast<float>(measuredFloorYStanding);
    out.m[0][3] += standingZeroToRaw.m[0][1] * y;
    out.m[1][3] += standingZeroToRaw.m[1][1] * y;
    out.m[2][3] += standingZeroToRaw.m[2][1] * y;
    return out;
}

double ControllerFloorContactOffsetMeters(
    const std::string& controllerType,
    const Eigen::Affine3d& standingPose)
{
    if (!standingPose.matrix().allFinite()) {
        return 0.0;
    }

    const double roll = std::atan2(
        standingPose.linear()(1, 0),
        standingPose.linear()(1, 1));
    const bool faceUp = std::fabs(roll) <= (EIGEN_PI * 0.5);

    if (controllerType == "knuckles") {
        return faceUp ? 0.0285 : 0.0310;
    }
    if (controllerType == "vive_controller") {
        return faceUp ? 0.0620 : 0.0060;
    }
    return 0.0;
}

double AdjustControllerFloorYForContact(
    double controllerOriginYStanding,
    const std::string& controllerType,
    const Eigen::Affine3d& standingPose)
{
    if (!std::isfinite(controllerOriginYStanding)) {
        return controllerOriginYStanding;
    }
    return controllerOriginYStanding
        - ControllerFloorContactOffsetMeters(controllerType, standingPose);
}

bool ApplySteamVrFloorOffset(
    double measuredFloorYStanding,
    char* errorBuffer,
    size_t errorBufferSize)
{
    auto writeError = [&](const char* message) {
        if (errorBuffer && errorBufferSize > 0) {
            snprintf(errorBuffer, errorBufferSize, "%s", message ? message : "");
        }
    };

    if (!std::isfinite(measuredFloorYStanding)) {
        writeError("Measured floor height was not finite.");
        Metrics::WriteLogAnnotation("[boundary-floor] steamvr floor apply failed: non_finite");
        return false;
    }
    if (!ChaperoneSetupOk()) {
        writeError("SteamVR chaperone setup is unavailable.");
        Metrics::WriteLogAnnotation("[boundary-floor] steamvr floor apply failed: setup_unavailable");
        return false;
    }

    auto* setup = vr::VRChaperoneSetup();
    setup->RevertWorkingCopy();

    vr::HmdMatrix34_t before{};
    if (!setup->GetWorkingStandingZeroPoseToRawTrackingPose(&before)) {
        writeError("SteamVR did not return the current standing-zero pose.");
        Metrics::WriteLogAnnotation("[boundary-floor] steamvr floor apply failed: get_pose_failed");
        return false;
    }

    const vr::HmdMatrix34_t after =
        OffsetStandingZeroPoseForFloor(before, measuredFloorYStanding);
    setup->SetWorkingStandingZeroPoseToRawTrackingPose(&after);
    const bool committed = setup->CommitWorkingCopy(vr::EChaperoneConfigFile_Live);
    if (committed && vr::VRChaperone()) {
        vr::VRChaperone()->ReloadInfo();
    }
    if (committed) {
        setup->RevertWorkingCopy();
    }

    char lbuf[384];
    snprintf(lbuf, sizeof lbuf,
        "[boundary-floor] steamvr floor apply: measured_standing_y=%.4f before_raw=(%.4f,%.4f,%.4f) after_raw=(%.4f,%.4f,%.4f) up=(%.4f,%.4f,%.4f) commit=%d",
        measuredFloorYStanding,
        static_cast<double>(before.m[0][3]),
        static_cast<double>(before.m[1][3]),
        static_cast<double>(before.m[2][3]),
        static_cast<double>(after.m[0][3]),
        static_cast<double>(after.m[1][3]),
        static_cast<double>(after.m[2][3]),
        static_cast<double>(before.m[0][1]),
        static_cast<double>(before.m[1][1]),
        static_cast<double>(before.m[2][1]),
        committed ? 1 : 0);
    Metrics::WriteLogAnnotation(lbuf);

    if (!committed) {
        writeError("SteamVR rejected the floor commit.");
        return false;
    }

    writeError("");
    return true;
}

bool ApplySteamVrFloorOffsetFromDevice(
    vr::TrackedDeviceIndex_t deviceId,
    char* errorBuffer,
    size_t errorBufferSize)
{
    auto writeError = [&](const char* message) {
        if (errorBuffer && errorBufferSize > 0) {
            snprintf(errorBuffer, errorBufferSize, "%s", message ? message : "");
        }
    };

    auto* system = vr::VRSystem();
    if (!system) {
        writeError("SteamVR system is unavailable.");
        Metrics::WriteLogAnnotation("[boundary-floor] steamvr floor apply failed: system_unavailable");
        return false;
    }
    if (deviceId >= vr::k_unMaxTrackedDeviceCount) {
        writeError("Selected controller device id is outside the SteamVR device range.");
        Metrics::WriteLogAnnotation("[boundary-floor] steamvr floor apply failed: invalid_device");
        return false;
    }

    vr::TrackedDevicePose_t poses[vr::k_unMaxTrackedDeviceCount]{};
    system->GetDeviceToAbsoluteTrackingPose(
        vr::TrackingUniverseStanding,
        0.0f,
        poses,
        vr::k_unMaxTrackedDeviceCount);

    const vr::TrackedDevicePose_t& pose = poses[deviceId];
    if (!pose.bDeviceIsConnected || !pose.bPoseIsValid) {
        writeError("Selected controller does not have a valid SteamVR standing pose.");
        char lbuf[192];
        snprintf(lbuf, sizeof lbuf,
            "[boundary-floor] steamvr floor apply failed: invalid_pose device=%d connected=%d valid=%d result=%d",
            static_cast<int>(deviceId),
            pose.bDeviceIsConnected ? 1 : 0,
            pose.bPoseIsValid ? 1 : 0,
            static_cast<int>(pose.eTrackingResult));
        Metrics::WriteLogAnnotation(lbuf);
        return false;
    }

    const double standingY =
        static_cast<double>(pose.mDeviceToAbsoluteTracking.m[1][3]);
    if (!std::isfinite(standingY) || standingY < -2.0 || standingY > 2.0) {
        writeError("Selected controller floor sample was outside the expected SteamVR standing range.");
        char lbuf[192];
        snprintf(lbuf, sizeof lbuf,
            "[boundary-floor] steamvr floor apply failed: standing_y_out_of_range device=%d standing_y=%.4f",
            static_cast<int>(deviceId),
            standingY);
        Metrics::WriteLogAnnotation(lbuf);
        return false;
    }

    char lbuf[192];
    snprintf(lbuf, sizeof lbuf,
        "[boundary-floor] steamvr floor device sample: device=%d standing_y=%.4f",
        static_cast<int>(deviceId),
        standingY);
    Metrics::WriteLogAnnotation(lbuf);

    return ApplySteamVrFloorOffset(standingY, errorBuffer, errorBufferSize);
}

double BoundaryFloorYAfterApply(
    double measuredFloorYStanding,
    bool moveSteamVrFloor)
{
    if (!std::isfinite(measuredFloorYStanding)) {
        return 0.0;
    }
    return moveSteamVrFloor ? 0.0 : measuredFloorYStanding;
}

bool BoundaryControllerMatchesTargetTrackingSystem(
    const std::string& controllerTrackingSystem,
    const std::string& targetTrackingSystem)
{
    return !targetTrackingSystem.empty()
        && controllerTrackingSystem == targetTrackingSystem;
}

// ---------------------------------------------------------------------------
// Chaperone push
// ---------------------------------------------------------------------------

static bool ChaperoneSetupOk() {
    return vr::VRChaperoneSetup() != nullptr;
}

static std::vector<vr::HmdQuad_t> BuildWallQuads(
    const std::vector<BoundaryVertex>& standingUniverseVertices,
    double floorY,
    double ceilingY)
{
    const size_t n = standingUniverseVertices.size();
    std::vector<vr::HmdQuad_t> quads;
    quads.reserve(n);
    const float fy = static_cast<float>(floorY);
    const float cy = static_cast<float>(ceilingY);
    for (size_t i = 0; i < n; ++i) {
        const auto& a = standingUniverseVertices[i];
        const auto& b = standingUniverseVertices[(i + 1) % n];
        vr::HmdQuad_t q;
        // Bottom-left
        q.vCorners[0].v[0] = static_cast<float>(a.x);
        q.vCorners[0].v[1] = fy;
        q.vCorners[0].v[2] = static_cast<float>(a.z);
        // Bottom-right
        q.vCorners[1].v[0] = static_cast<float>(b.x);
        q.vCorners[1].v[1] = fy;
        q.vCorners[1].v[2] = static_cast<float>(b.z);
        // Top-right
        q.vCorners[2].v[0] = static_cast<float>(b.x);
        q.vCorners[2].v[1] = cy;
        q.vCorners[2].v[2] = static_cast<float>(b.z);
        // Top-left
        q.vCorners[3].v[0] = static_cast<float>(a.x);
        q.vCorners[3].v[1] = cy;
        q.vCorners[3].v[2] = static_cast<float>(a.z);
        quads.push_back(q);
    }
    return quads;
}

static double SignedAreaVerticesXZ(const std::vector<BoundaryVertex>& vertices)
{
    if (vertices.size() < 3) return 0.0;
    double twiceArea = 0.0;
    for (size_t i = 0, j = vertices.size() - 1; i < vertices.size(); j = i++) {
        twiceArea += vertices[j].x * vertices[i].z - vertices[i].x * vertices[j].z;
    }
    return twiceArea * 0.5;
}

static bool SamePointXZ(
    const BoundaryVertex& a,
    const BoundaryVertex& b,
    double toleranceMeters)
{
    const double dx = a.x - b.x;
    const double dz = a.z - b.z;
    return (dx * dx + dz * dz) <= toleranceMeters * toleranceMeters;
}

static std::vector<BoundaryVertex> NormalizeWorkingSetVertices(
    const std::vector<BoundaryVertex>& vertices)
{
    constexpr double kDuplicateToleranceMeters = 0.005;

    std::vector<BoundaryVertex> out;
    out.reserve(vertices.size());
    for (const auto& v : vertices) {
        if (!std::isfinite(v.x) || !std::isfinite(v.y) || !std::isfinite(v.z)) {
            return {};
        }
        if (!out.empty() && SamePointXZ(out.back(), v, kDuplicateToleranceMeters)) {
            continue;
        }
        out.push_back(v);
    }

    while (out.size() > 3 && SamePointXZ(out.front(), out.back(), kDuplicateToleranceMeters)) {
        out.pop_back();
    }
    return out;
}

static bool PointOnSegmentXZ(
    double px,
    double pz,
    const BoundaryVertex& a,
    const BoundaryVertex& b)
{
    const double abx = b.x - a.x;
    const double abz = b.z - a.z;
    const double apx = px - a.x;
    const double apz = pz - a.z;
    const double cross = abx * apz - abz * apx;
    if (std::fabs(cross) > 1e-8) return false;

    const double dot = apx * abx + apz * abz;
    if (dot < -1e-8) return false;

    const double lenSq = abx * abx + abz * abz;
    return dot <= lenSq + 1e-8;
}

static bool PointInsideOrOnPolygonXZ(
    const std::vector<BoundaryVertex>& vertices,
    double x,
    double z)
{
    bool inside = false;
    for (size_t i = 0, j = vertices.size() - 1; i < vertices.size(); j = i++) {
        const auto& a = vertices[j];
        const auto& b = vertices[i];
        if (PointOnSegmentXZ(x, z, a, b)) {
            return true;
        }

        const bool crosses = ((a.z > z) != (b.z > z));
        if (crosses) {
            const double xAtZ = (b.x - a.x) * (z - a.z) / (b.z - a.z) + a.x;
            if (x < xAtZ) {
                inside = !inside;
            }
        }
    }
    return inside;
}

static bool CenteredRectangleInsidePolygonXZ(
    const std::vector<BoundaryVertex>& vertices,
    double sizeX,
    double sizeZ)
{
    if (sizeX <= 0.0 || sizeZ <= 0.0) return false;

    const double hx = sizeX * 0.5;
    const double hz = sizeZ * 0.5;
    const double corners[4][2] = {
        { -hx, -hz },
        {  hx, -hz },
        {  hx,  hz },
        { -hx,  hz },
    };
    for (const auto& corner : corners) {
        if (!PointInsideOrOnPolygonXZ(vertices, corner[0], corner[1])) {
            return false;
        }
    }

    return true;
}

static bool ComputeCenteredPlayAreaSize(
    const std::vector<BoundaryVertex>& vertices,
    const PolygonBounds& bounds,
    float& playAreaX,
    float& playAreaZ)
{
    if (!PointInsideOrOnPolygonXZ(vertices, 0.0, 0.0)) {
        return false;
    }

    const double maxCenteredX = 2.0 * std::min(std::fabs(bounds.xMin), std::fabs(bounds.xMax));
    const double maxCenteredZ = 2.0 * std::min(std::fabs(bounds.zMin), std::fabs(bounds.zMax));
    if (!std::isfinite(maxCenteredX) || !std::isfinite(maxCenteredZ) ||
        maxCenteredX <= 0.0 || maxCenteredZ <= 0.0) {
        return false;
    }

    double candidateX = maxCenteredX;
    double candidateZ = maxCenteredZ;
    for (int i = 0; i < 48; ++i) {
        if (CenteredRectangleInsidePolygonXZ(vertices, candidateX, candidateZ)) {
            playAreaX = static_cast<float>(candidateX);
            playAreaZ = static_cast<float>(candidateZ);
            return playAreaX > 0.0f && playAreaZ > 0.0f;
        }
        candidateX *= 0.95;
        candidateZ *= 0.95;
    }
    return false;
}

ChaperoneWorkingSet BuildChaperoneWorkingSet(
    const std::vector<BoundaryVertex>& standingUniverseVertices,
    double floorY,
    double ceilingY)
{
    ChaperoneWorkingSet out;
    std::vector<BoundaryVertex> vertices =
        NormalizeWorkingSetVertices(standingUniverseVertices);
    if (vertices.size() < 3 ||
        !std::isfinite(floorY) ||
        !std::isfinite(ceilingY) ||
        ceilingY <= floorY) {
        return out;
    }

    if (std::fabs(SignedAreaVerticesXZ(vertices)) < 0.05) {
        return ChaperoneWorkingSet{};
    }

    const PolygonBounds bounds = ComputePolygonBoundsXZ(vertices);
    const double spanX = bounds.xMax - bounds.xMin;
    const double spanZ = bounds.zMax - bounds.zMin;
    if (!std::isfinite(spanX) || !std::isfinite(spanZ) ||
        spanX <= 0.0 || spanZ <= 0.0 ||
        !ComputeCenteredPlayAreaSize(vertices, bounds, out.playAreaX, out.playAreaZ)) {
        return ChaperoneWorkingSet{};
    }

    out.perimeter.reserve(vertices.size());
    for (const auto& v : vertices) {
        vr::HmdVector2_t p{};
        p.v[0] = static_cast<float>(v.x);
        p.v[1] = static_cast<float>(v.z);
        out.perimeter.push_back(p);
    }
    out.collisionBounds = BuildWallQuads(vertices, floorY, ceilingY);
    out.valid = out.perimeter.size() >= 3 && out.collisionBounds.size() >= 3;
    return out;
}

static bool SetWorkingBoundary(
    vr::IVRChaperoneSetup* setup,
    const std::vector<BoundaryVertex>& standingUniverseVertices,
    double floorY,
    double ceilingY)
{
    ChaperoneWorkingSet workingSet =
        BuildChaperoneWorkingSet(standingUniverseVertices, floorY, ceilingY);
    if (!workingSet.valid) {
        return false;
    }
    setup->SetWorkingPlayAreaSize(
        workingSet.playAreaX,
        workingSet.playAreaZ);
    setup->SetWorkingPerimeter(
        workingSet.perimeter.data(),
        static_cast<uint32_t>(workingSet.perimeter.size()));
    setup->SetWorkingCollisionBoundsInfo(
        workingSet.collisionBounds.data(),
        static_cast<uint32_t>(workingSet.collisionBounds.size()));
    return true;
}

bool PushToChaperone(const std::vector<BoundaryVertex>& standingUniverseVertices,
                     double floorY, double ceilingY) {
    if (!ChaperoneSetupOk()) {
        Metrics::WriteLogAnnotation(
            "[boundary] chaperone push failed: vrchap setup returned error / not initialized");
        return false;
    }
    const size_t n = standingUniverseVertices.size();
    if (n < 3) {
        Metrics::WriteLogAnnotation("[boundary] chaperone push failed: fewer_than_3_vertices");
        return false;
    }

    auto* setup = vr::VRChaperoneSetup();
    setup->RevertWorkingCopy();

    const double area = AbsoluteAreaXZ(ProjectXZ(standingUniverseVertices));
    const auto bounds = ComputePolygonBoundsXZ(standingUniverseVertices);
    {
        char lbuf[384];
        snprintf(lbuf, sizeof lbuf,
            "[boundary] chaperone push begin: vertices=%zu area=%.3f floor=%.3f ceiling=%.3f bounds_x=(%.3f,%.3f) bounds_z=(%.3f,%.3f)",
            n,
            area,
            floorY,
            ceilingY,
            bounds.xMin,
            bounds.xMax,
            bounds.zMin,
            bounds.zMax);
        Metrics::WriteLogAnnotation(lbuf);
    }

    if (!SetWorkingBoundary(setup, standingUniverseVertices, floorY, ceilingY)) {
        Metrics::WriteLogAnnotation("[boundary] chaperone push failed: invalid_working_set");
        return false;
    }
    const bool committed = setup->CommitWorkingCopy(vr::EChaperoneConfigFile_Live);
    if (committed && vr::VRChaperone()) {
        vr::VRChaperone()->ReloadInfo();
    }
    if (committed) {
        setup->RevertWorkingCopy();
    }
    if (!committed) {
        Metrics::WriteLogAnnotation("[boundary] chaperone push failed: commit_failed");
        return false;
    }

    // Log on edge: vertex count changed or first push.
    {
        static size_t s_lastN      = SIZE_MAX;
        static double s_lastArea   = -1.0;
        if (n != s_lastN || std::fabs(area - s_lastArea) > 0.01) {
            s_lastN    = n;
            s_lastArea = area;
            char lbuf[192];
            snprintf(lbuf, sizeof lbuf,
                "[boundary] chaperone pushed: vertices=%zu area=%.2fm2"
                " floor=%.2f ceiling=%.2f",
                n, area, floorY, ceilingY);
            Metrics::WriteLogAnnotation(lbuf);
        }
    }

    return true;
}

bool ShowWorkingChaperonePreview(
    const std::vector<BoundaryVertex>& standingUniverseVertices,
    double floorY,
    double ceilingY)
{
    static bool s_loggedUnavailable = false;
    if (!ChaperoneSetupOk()) {
        if (!s_loggedUnavailable) {
            Metrics::WriteLogAnnotation(
                "[boundary] working preview failed: vrchap setup returned error / not initialized");
            s_loggedUnavailable = true;
        }
        return false;
    }
    s_loggedUnavailable = false;
    if (standingUniverseVertices.size() < 3) return false;

    auto* setup = vr::VRChaperoneSetup();
    if (!SetWorkingBoundary(setup, standingUniverseVertices, floorY, ceilingY)) {
        return false;
    }
    setup->ShowWorkingSetPreview();
    return true;
}

void HideWorkingChaperonePreview()
{
    if (!ChaperoneSetupOk()) return;
    auto* setup = vr::VRChaperoneSetup();
    setup->HideWorkingSetPreview();
    setup->RevertWorkingCopy();
}

// ---------------------------------------------------------------------------
// Snapshot serialization
//
// Format: 4-byte little-endian quad count, then each HmdQuad_t verbatim
// (48 bytes: 4 corners x 3 floats each).
// ---------------------------------------------------------------------------

static constexpr size_t kQuadBytes = sizeof(vr::HmdQuad_t); // 48

static void Write32LE(std::vector<uint8_t>& buf, uint32_t v) {
    buf.push_back(static_cast<uint8_t>(v));
    buf.push_back(static_cast<uint8_t>(v >> 8));
    buf.push_back(static_cast<uint8_t>(v >> 16));
    buf.push_back(static_cast<uint8_t>(v >> 24));
}

static bool Read32LE(const std::vector<uint8_t>& buf, size_t offset, uint32_t& out) {
    if (offset + 4 > buf.size()) return false;
    out = static_cast<uint32_t>(buf[offset])
        | (static_cast<uint32_t>(buf[offset + 1]) << 8)
        | (static_cast<uint32_t>(buf[offset + 2]) << 16)
        | (static_cast<uint32_t>(buf[offset + 3]) << 24);
    return true;
}

std::vector<uint8_t> SnapshotCurrentChaperone() {
    if (!ChaperoneSetupOk()) return {};

    auto* setup = vr::VRChaperoneSetup();
    uint32_t count = 0;
    setup->GetLiveCollisionBoundsInfo(nullptr, &count);
    if (count == 0) return {};

    std::vector<vr::HmdQuad_t> quads(count);
    setup->GetLiveCollisionBoundsInfo(quads.data(), &count);

    std::vector<uint8_t> buf;
    buf.reserve(4 + count * kQuadBytes);
    Write32LE(buf, count);
    for (const auto& q : quads) {
        const uint8_t* raw = reinterpret_cast<const uint8_t*>(&q);
        buf.insert(buf.end(), raw, raw + kQuadBytes);
    }

    {
        char lbuf[96];
        snprintf(lbuf, sizeof lbuf,
            "[boundary] snapshotted prior chaperone: bytes=%zu", buf.size());
        Metrics::WriteLogAnnotation(lbuf);
    }

    return buf;
}

bool RestoreChaperoneFromSnapshot(const std::vector<uint8_t>& snapshot) {
    if (snapshot.empty()) {
        char fbuf[80];
        snprintf(fbuf, sizeof fbuf, "[boundary] restore failed: bytes=0");
        Metrics::WriteLogAnnotation(fbuf);
        return false;
    }
    if (!ChaperoneSetupOk()) return false;

    uint32_t count = 0;
    if (!Read32LE(snapshot, 0, count)) return false;
    if (count == 0) return false;
    const size_t expected = 4 + static_cast<size_t>(count) * kQuadBytes;
    if (snapshot.size() < expected) {
        char fbuf[128];
        snprintf(fbuf, sizeof fbuf,
            "[boundary] restore failed: bytes=%zu (truncated; expected %zu)",
            snapshot.size(), expected);
        Metrics::WriteLogAnnotation(fbuf);
        return false;
    }

    std::vector<vr::HmdQuad_t> quads(count);
    std::memcpy(quads.data(), snapshot.data() + 4, count * kQuadBytes);

    auto* setup = vr::VRChaperoneSetup();
    setup->SetWorkingCollisionBoundsInfo(quads.data(), count);
    setup->CommitWorkingCopy(vr::EChaperoneConfigFile_Live);
    Metrics::WriteLogAnnotation("[boundary] restored prior chaperone");
    return true;
}

// ---------------------------------------------------------------------------
// Polygon bounding rect
// ---------------------------------------------------------------------------

PolygonBounds ComputePolygonBoundsXZ(const std::vector<BoundaryVertex>& v)
{
    if (v.empty()) return {0.0, 0.0, 0.0, 0.0};
    PolygonBounds b{v[0].x, v[0].x, v[0].z, v[0].z};
    for (const auto& p : v) {
        if (p.x < b.xMin) b.xMin = p.x;
        if (p.x > b.xMax) b.xMax = p.x;
        if (p.z < b.zMin) b.zMin = p.z;
        if (p.z > b.zMax) b.zMax = p.z;
    }
    return b;
}

}  // namespace wkopenvr::boundary
