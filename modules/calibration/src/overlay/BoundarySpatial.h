#pragma once

#include "Calibration.h"

#include <Eigen/Geometry>

#include <cstdint>
#include <string>
#include <vector>

namespace wkopenvr::boundary {

enum class SpatialSpaceKind {
    Standing,
    Target,
};

enum class SpatialToolKind {
    None,
    BoundaryPolygon,
    FloorMarker,
};

enum class SpatialSamplingMode {
    ControllerXZContact,
    PointerRayFloorHit,
    RenderModelTipRay,
};

enum class SpatialPrimitiveKind {
    PolygonFloorRegion,
    PolylinePath,
    Marker,
    Label,
    ExtrudedWall,
};

struct SpatialSpace {
    SpatialSpaceKind kind = SpatialSpaceKind::Standing;
    std::string trackingSystem;
    Eigen::AffineCompact3d targetToStanding = Eigen::AffineCompact3d::Identity();
    bool transformValid = false;
    uint64_t transformEpoch = 0;
};

struct SpatialSession {
    SpatialToolKind tool = SpatialToolKind::None;
    SpatialSpace authoringSpace;
    int32_t sourceDeviceId = -1;
    std::string sourceTrackingSystem;
    SpatialSamplingMode samplingMode = SpatialSamplingMode::ControllerXZContact;
    bool requireTrigger = true;
    double floorY = 0.0;
    double ceilingY = 2.4;
    uint64_t sessionId = 0;
};

struct SpatialStyle {
    uint8_t r = 0;
    uint8_t g = 255;
    uint8_t b = 190;
    uint8_t a = 245;
    uint8_t fillA = 72;
    double strokeMeters = 0.035;
    double dotMeters = 0.070;
    bool fill = false;
};

struct SpatialPrimitive {
    SpatialPrimitiveKind kind = SpatialPrimitiveKind::PolylinePath;
    SpatialSpace space;
    std::vector<BoundaryVertex> vertices;
    double floorY = 0.0;
    double ceilingY = 2.4;
    bool closeLoop = false;
    int layer = 0;
    SpatialStyle style;
    // When set, a polyline path is stroked with a newest-white -> oldest-gray
    // age gradient instead of a single flat color.
    bool ageFade = false;
};

struct SpatialRenderCommand {
    SpatialPrimitiveKind kind = SpatialPrimitiveKind::PolylinePath;
    std::vector<BoundaryVertex> standingVertices;
    double floorY = 0.0;
    double ceilingY = 2.4;
    bool closeLoop = false;
    int layer = 0;
    SpatialStyle style;
    bool ageFade = false;
};

SpatialSpace StandingSpace(std::string trackingSystem = {});

SpatialSpace TargetSpace(
    std::string trackingSystem,
    const Eigen::AffineCompact3d& targetToStanding,
    uint64_t transformEpoch = 0);

SpatialSession BoundaryCaptureSessionDescriptor(
    const SpatialSpace& authoringSpace,
    int32_t sourceDeviceId,
    std::string sourceTrackingSystem,
    double floorY,
    bool requireTrigger,
    uint64_t sessionId);

SpatialPrimitive BoundaryPathPrimitive(
    const SpatialSession& session,
    const std::vector<BoundaryVertex>& vertices,
    bool closeLoop);

SpatialPrimitive FloorMarkerPrimitive(
    const SpatialSpace& space,
    const std::vector<BoundaryVertex>& vertices,
    double floorY);

SpatialPrimitive TransformPrimitiveToStanding(
    const SpatialPrimitive& primitive);

std::vector<SpatialRenderCommand> BuildSpatialRenderCommands(
    const std::vector<SpatialPrimitive>& primitives);

// Maps a path vertex's position-in-time to a grayscale shade: the newest
// vertex is bright white (255), older vertices fade toward mid-gray. Pure so
// it can be unit tested without a renderer.
uint8_t BoundaryAgeShade(size_t vertexIndex, size_t vertexCount);

} // namespace wkopenvr::boundary
