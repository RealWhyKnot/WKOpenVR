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

// ---------------------------------------------------------------------------
// Chaperone push
// ---------------------------------------------------------------------------

static bool ChaperoneSetupOk() {
    return vr::VRChaperoneSetup() != nullptr;
}

bool PushToChaperone(const std::vector<BoundaryVertex>& standingUniverseVertices,
                     double floorY, double ceilingY) {
    if (!ChaperoneSetupOk()) {
        Metrics::WriteLogAnnotation(
            "[boundary] chaperone push failed: vrchap setup returned error / not initialized");
        return false;
    }
    const size_t n = standingUniverseVertices.size();
    if (n < 3) return false;

    auto* setup = vr::VRChaperoneSetup();

    // Bounding box for the play area size.
    double minX = standingUniverseVertices[0].x, maxX = minX;
    double minZ = standingUniverseVertices[0].z, maxZ = minZ;
    for (const auto& v : standingUniverseVertices) {
        if (v.x < minX) minX = v.x;
        if (v.x > maxX) maxX = v.x;
        if (v.z < minZ) minZ = v.z;
        if (v.z > maxZ) maxZ = v.z;
    }
    setup->SetWorkingPlayAreaSize(
        static_cast<float>(maxX - minX),
        static_cast<float>(maxZ - minZ));

    // Build wall quads: one HmdQuad_t per edge, spanning floorY to ceilingY.
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

    setup->SetWorkingCollisionBoundsInfo(quads.data(), static_cast<uint32_t>(quads.size()));
    setup->CommitWorkingCopy(vr::EChaperoneConfigFile_Live);

    // Log on edge: vertex count changed or first push.
    {
        static size_t s_lastN      = SIZE_MAX;
        static double s_lastArea   = -1.0;
        const double area = AbsoluteAreaXZ(ProjectXZ(standingUniverseVertices));
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
