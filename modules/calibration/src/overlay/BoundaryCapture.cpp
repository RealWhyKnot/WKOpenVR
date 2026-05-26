#include "BoundaryCapture.h"
#include "Boundary.h"
#include "CalibrationMetrics.h"

#include <cmath>
#include <cstddef>
#include <cstdio>

namespace wkopenvr::boundary {

namespace {

double DistanceSq(const BoundaryVertex& a, const BoundaryVertex& b) {
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    const double dz = a.z - b.z;
    return dx * dx + dy * dy + dz * dz;
}

double SegmentDistanceSq(const BoundaryVertex& p,
                         const BoundaryVertex& a,
                         const BoundaryVertex& b)
{
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double dz = b.z - a.z;
    const double lenSq = dx * dx + dy * dy + dz * dz;
    const double px = p.x - a.x;
    const double py = p.y - a.y;
    const double pz = p.z - a.z;
    if (lenSq < 1e-20) {
        return px * px + py * py + pz * pz;
    }
    double t = (px * dx + py * dy + pz * dz) / lenSq;
    if (t < 0.0) t = 0.0;
    if (t > 1.0) t = 1.0;
    const double rx = px - t * dx;
    const double ry = py - t * dy;
    const double rz = pz - t * dz;
    return rx * rx + ry * ry + rz * rz;
}

bool TryProjectAimToFloor(const Eigen::Affine3d& controllerPose,
                          double floorY,
                          BoundaryVertex& out)
{
    const Eigen::Vector3d origin = controllerPose.translation();
    const Eigen::Vector3d aim =
        (controllerPose.rotation() * Eigen::Vector3d(0.0, 0.0, -1.0)).normalized();

    // Require a real downward aim. A nearly horizontal controller would throw
    // the floor hit far across the room and make the painted boundary jump.
    if (!origin.allFinite() || !aim.allFinite() || aim.y() > -0.20) {
        return false;
    }

    const double distanceMeters = (floorY - origin.y()) / aim.y();
    if (!std::isfinite(distanceMeters) ||
        distanceMeters < 0.05 ||
        distanceMeters > 5.0) {
        return false;
    }

    const Eigen::Vector3d hit = origin + aim * distanceMeters;
    if (!hit.allFinite()) {
        return false;
    }

    out = { hit.x(), floorY, hit.z() };
    return true;
}

std::vector<BoundaryVertex> RemoveNearDuplicates(
    const std::vector<BoundaryVertex>& raw,
    double minDistanceMeters)
{
    std::vector<BoundaryVertex> out;
    out.reserve(raw.size());

    const double minDistSq = minDistanceMeters * minDistanceMeters;
    for (const auto& v : raw) {
        if (!out.empty() && DistanceSq(out.back(), v) < minDistSq) {
            continue;
        }
        out.push_back(v);
    }
    return out;
}

std::vector<BoundaryVertex> RemoveClosedCollinearVertices(
    const std::vector<BoundaryVertex>& raw,
    double toleranceMeters)
{
    std::vector<BoundaryVertex> out = raw;
    const double toleranceSq = toleranceMeters * toleranceMeters;

    bool changed = true;
    while (changed && out.size() > 3) {
        changed = false;
        for (size_t i = 0; i < out.size(); ++i) {
            const BoundaryVertex& prev = out[(i + out.size() - 1) % out.size()];
            const BoundaryVertex& cur = out[i];
            const BoundaryVertex& next = out[(i + 1) % out.size()];
            if (SegmentDistanceSq(cur, prev, next) <= toleranceSq) {
                out.erase(out.begin() + static_cast<std::ptrdiff_t>(i));
                changed = true;
                break;
            }
        }
    }

    return out;
}

std::vector<BoundaryVertex> CleanPaintedLoop(
    const std::vector<BoundaryVertex>& raw,
    double debounceMeters,
    double closeLoopMeters,
    double simplifyMeters)
{
    std::vector<BoundaryVertex> path = RemoveNearDuplicates(raw, debounceMeters);
    if (path.size() < 3) {
        return path;
    }

    const double closeSq = closeLoopMeters * closeLoopMeters;
    while (path.size() >= 3 && DistanceSq(path.front(), path.back()) < closeSq) {
        path.pop_back();
    }

    if (path.size() < 3) {
        return path;
    }

    auto kept = SimplifyDouglasPeucker(path, simplifyMeters);
    std::vector<BoundaryVertex> simplified;
    simplified.reserve(kept.size());
    for (size_t idx : kept) {
        simplified.push_back(path[idx]);
    }

    simplified = RemoveNearDuplicates(simplified, debounceMeters);
    if (simplified.size() >= 3 &&
        DistanceSq(simplified.front(), simplified.back()) < closeSq) {
        simplified.pop_back();
    }
    return RemoveClosedCollinearVertices(simplified, simplifyMeters);
}

}  // namespace

void CaptureSession::Start() {
    m_raw.clear();
    m_simplified.clear();
    m_state = CaptureState::Active;
    Metrics::WriteLogAnnotation("[boundary-capture] started");
}

void CaptureSession::Cancel() {
    m_raw.clear();
    m_simplified.clear();
    m_state = CaptureState::Idle;
    Metrics::WriteLogAnnotation("[boundary-capture] cancelled");
}

void CaptureSession::Finish() {
    if (m_state != CaptureState::Active) return;
    m_simplified = CleanPaintedLoop(
        m_raw,
        kVertexDebounceMeters,
        kCloseLoopMeters,
        kSimplifyEpsilonMeters);
    m_state = CaptureState::Finished;
    {
        char lbuf[96];
        snprintf(lbuf, sizeof lbuf,
            "[boundary-capture] finished: raw=%zu simplified=%zu",
            m_raw.size(), m_simplified.size());
        Metrics::WriteLogAnnotation(lbuf);
    }
}

void CaptureSession::Tick(const Eigen::Affine3d& controllerPose,
                          bool triggerHeld,
                          double floorY) {
    if (m_state != CaptureState::Active) return;
    if (!triggerHeld) return;

    BoundaryVertex candidate;
    if (!TryProjectAimToFloor(controllerPose, floorY, candidate)) {
        return;
    }

    if (!m_raw.empty()) {
        const BoundaryVertex& last = m_raw.back();
        const double dist = std::sqrt(DistanceSq(candidate, last));
        if (dist < kVertexDebounceMeters) return;
    }

    if (m_raw.empty()) {
        char lbuf[128];
        snprintf(lbuf, sizeof lbuf,
            "[boundary-capture] first vertex: pos=(%.3f,%.3f,%.3f)",
            candidate.x, candidate.y, candidate.z);
        Metrics::WriteLogAnnotation(lbuf);
    }
    m_raw.push_back(candidate);
}

const std::vector<BoundaryVertex>& CaptureSession::vertices() const {
    if (m_state == CaptureState::Finished) return m_simplified;
    return m_raw;
}

}  // namespace wkopenvr::boundary
