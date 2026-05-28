#include "BoundaryFloorCapture.h"

#include <cmath>

namespace wkopenvr::boundary {

void FloorCaptureSession::Begin(double originalFloorY, double originalCeilingY)
{
    m_active = true;
    m_originalFloorY = originalFloorY;
    m_originalCeilingY = originalCeilingY;
    m_candidate = {};
}

void FloorCaptureSession::Reset()
{
    m_active = false;
    m_candidate = {};
}

bool FloorCaptureSession::Observe(
    const Eigen::Affine3d& rawPose,
    int deviceId,
    const std::string& trackingSystem)
{
    if (!m_active) return false;

    const Eigen::Vector3d p = rawPose.translation();
    if (!p.allFinite()) return false;

    const double y = p.y();
    if (!std::isfinite(y) || y < -5.0 || y > 5.0) return false;

    const size_t nextCount = m_candidate.sampleCount + 1;
    const bool updated = !m_candidate.valid || y < m_candidate.floorY;
    if (updated) {
        m_candidate.valid = true;
        m_candidate.floorY = y;
        m_candidate.deviceId = deviceId;
        m_candidate.trackingSystem = trackingSystem;
        m_candidate.pose = rawPose;
    }
    m_candidate.sampleCount = nextCount;
    return updated;
}

std::vector<BoundaryVertex> BuildFloorMarkerVertices(
    const Eigen::Affine3d& pose,
    double floorY,
    double halfSizeMeters)
{
    const Eigen::Vector3d p = pose.translation();
    if (!p.allFinite() || !std::isfinite(floorY) || halfSizeMeters <= 0.0) {
        return {};
    }

    const double x = p.x();
    const double z = p.z();
    const double h = halfSizeMeters;
    return {
        { x - h, floorY, z - h },
        { x + h, floorY, z - h },
        { x + h, floorY, z + h },
        { x - h, floorY, z + h },
    };
}

} // namespace wkopenvr::boundary
