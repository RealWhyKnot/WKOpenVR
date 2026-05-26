#pragma once

#include "Calibration.h"

#include <Eigen/Geometry>
#include <cstddef>
#include <vector>

namespace wkopenvr::boundary {

enum class CaptureState { Idle, Active, Finished };

// Streams controller aim rays while the trigger is held and builds a boundary
// polygon in lighthouse (pre-transform) space. Call Finish() to clean the raw
// painted loop down to edge vertices. Cancel() discards the buffer and resets
// to Idle.
class CaptureSession {
public:
    void Start();
    void Cancel();
    void Finish();

    // Called each overlay tick. controllerPose is the raw lighthouse-space
    // pose. The controller's pointer ray is intersected with the floor plane;
    // a new vertex is appended when triggerHeld AND that floor point has moved
    // at least kVertexDebounceMeters from the last recorded position.
    void Tick(const Eigen::Affine3d& controllerPose, bool triggerHeld, double floorY = 0.0);

    CaptureState state() const { return m_state; }
    // Simplified vertices after Finish(); raw vertices while Active.
    const std::vector<BoundaryVertex>& vertices() const;
    size_t rawVertexCount() const { return m_raw.size(); }

private:
    CaptureState m_state = CaptureState::Idle;
    std::vector<BoundaryVertex> m_raw;
    std::vector<BoundaryVertex> m_simplified;

    // Minimum motion required to record a new vertex (avoids duplicate points
    // while the user holds still with the trigger pressed).
    static constexpr double kVertexDebounceMeters = 0.05;
    // Perpendicular-distance tolerance for Douglas-Peucker.
    static constexpr double kSimplifyEpsilonMeters = 0.05;
    static constexpr double kCloseLoopMeters = 0.15;
};

}  // namespace wkopenvr::boundary
