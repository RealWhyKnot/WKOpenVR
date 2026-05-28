#pragma once

#include "Calibration.h"

#include <Eigen/Geometry>

#include <cstddef>
#include <string>
#include <vector>

namespace wkopenvr::boundary {

struct FloorCaptureCandidate {
    bool valid = false;
    double floorY = 0.0;
    size_t sampleCount = 0;
    int deviceId = -1;
    std::string trackingSystem;
    Eigen::Affine3d pose = Eigen::Affine3d::Identity();
};

class FloorCaptureSession {
public:
    void Begin(double originalFloorY, double originalCeilingY);
    void Reset();

    bool active() const { return m_active; }
    double originalFloorY() const { return m_originalFloorY; }
    double originalCeilingY() const { return m_originalCeilingY; }
    const FloorCaptureCandidate& candidate() const { return m_candidate; }

    bool Observe(
        const Eigen::Affine3d& rawPose,
        int deviceId,
        const std::string& trackingSystem);

private:
    bool m_active = false;
    double m_originalFloorY = 0.0;
    double m_originalCeilingY = 2.5;
    FloorCaptureCandidate m_candidate;
};

std::vector<BoundaryVertex> BuildFloorMarkerVertices(
    const Eigen::Affine3d& pose,
    double floorY,
    double halfSizeMeters = 0.25);

} // namespace wkopenvr::boundary
