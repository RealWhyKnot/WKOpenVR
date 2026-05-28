#include "BoundaryFloorCapture.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <vector>

namespace wkopenvr::boundary {

void FloorCaptureSession::Begin(double originalFloorY, double originalCeilingY)
{
    m_active = true;
    m_originalFloorY = originalFloorY;
    m_originalCeilingY = originalCeilingY;
    m_recentFloorSamples.clear();
    m_candidate = {};
}

void FloorCaptureSession::Reset()
{
    m_active = false;
    m_recentFloorSamples.clear();
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

    constexpr size_t kRecentSampleLimit = 36;
    constexpr double kSmoothing = 0.55;
    constexpr double kChangeMeters = 0.002;
    constexpr double kMarkerMoveMeters = 0.01;

    m_recentFloorSamples.push_back(y);
    while (m_recentFloorSamples.size() > kRecentSampleLimit) {
        m_recentFloorSamples.pop_front();
    }

    std::vector<double> sorted(m_recentFloorSamples.begin(), m_recentFloorSamples.end());
    std::sort(sorted.begin(), sorted.end());
    const size_t percentileIndex = std::min(
        sorted.size() - 1,
        static_cast<size_t>(std::floor(static_cast<double>(sorted.size() - 1) * 0.15)));
    const size_t highPercentileIndex = std::min(
        sorted.size() - 1,
        static_cast<size_t>(std::ceil(static_cast<double>(sorted.size() - 1) * 0.85)));
    const bool wasValid = m_candidate.valid;
    double estimate = sorted[percentileIndex];
    if (wasValid && y < m_candidate.floorY) {
        estimate = std::min(estimate, y);
    }
    const double jitterMeters = sorted[highPercentileIndex] - sorted[percentileIndex];

    const double previousFloorY = m_candidate.floorY;
    const Eigen::Vector3d previousPos = m_candidate.pose.translation();
    const double floorY = wasValid
        ? previousFloorY + (estimate - previousFloorY) * kSmoothing
        : estimate;
    const double dx = p.x() - previousPos.x();
    const double dz = p.z() - previousPos.z();
    const bool markerMoved = !wasValid || (dx * dx + dz * dz) > (kMarkerMoveMeters * kMarkerMoveMeters);
    const bool floorMoved = !wasValid || std::fabs(floorY - previousFloorY) > kChangeMeters;
    const bool deviceChanged = !wasValid || deviceId != m_candidate.deviceId;

    const size_t nextCount = m_candidate.sampleCount + 1;
    m_candidate.valid = true;
    m_candidate.floorY = floorY;
    m_candidate.deviceId = deviceId;
    m_candidate.trackingSystem = trackingSystem;
    m_candidate.pose = rawPose;
    m_candidate.sampleCount = nextCount;
    m_candidate.jitterMeters = jitterMeters;
    m_candidate.ready = nextCount >= 4;
    m_candidate.stable = nextCount >= 8 && jitterMeters <= 0.02;
    return floorMoved || markerMoved || deviceChanged;
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
