#include "BoundarySpatial.h"

#include "Boundary.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <utility>

namespace wkopenvr::boundary {

SpatialSpace StandingSpace(std::string trackingSystem)
{
    SpatialSpace space;
    space.kind = SpatialSpaceKind::Standing;
    space.trackingSystem = std::move(trackingSystem);
    space.targetToStanding = Eigen::AffineCompact3d::Identity();
    space.transformValid = true;
    return space;
}

SpatialSpace TargetSpace(
    std::string trackingSystem,
    const Eigen::AffineCompact3d& targetToStanding,
    uint64_t transformEpoch)
{
    SpatialSpace space;
    space.kind = SpatialSpaceKind::Target;
    space.trackingSystem = std::move(trackingSystem);
    space.targetToStanding = targetToStanding;
    space.transformValid = true;
    space.transformEpoch = transformEpoch;
    return space;
}

SpatialSession BoundaryCaptureSessionDescriptor(
    const SpatialSpace& authoringSpace,
    int32_t sourceDeviceId,
    std::string sourceTrackingSystem,
    double floorY,
    bool requireTrigger,
    uint64_t sessionId)
{
    SpatialSession session;
    session.tool = SpatialToolKind::BoundaryPolygon;
    session.authoringSpace = authoringSpace;
    session.sourceDeviceId = sourceDeviceId;
    session.sourceTrackingSystem = std::move(sourceTrackingSystem);
    session.samplingMode = SpatialSamplingMode::ControllerXZContact;
    session.requireTrigger = requireTrigger;
    session.floorY = floorY;
    session.ceilingY = floorY + 2.4;
    session.sessionId = sessionId;
    return session;
}

SpatialPrimitive BoundaryPathPrimitive(
    const SpatialSession& session,
    const std::vector<BoundaryVertex>& vertices,
    bool closeLoop)
{
    SpatialPrimitive primitive;
    primitive.kind = closeLoop
        ? SpatialPrimitiveKind::PolygonFloorRegion
        : SpatialPrimitiveKind::PolylinePath;
    primitive.space = session.authoringSpace;
    primitive.vertices = vertices;
    primitive.floorY = session.floorY;
    primitive.ceilingY = session.ceilingY;
    primitive.closeLoop = closeLoop;
    primitive.style.fill = closeLoop;
    return primitive;
}

SpatialPrimitive FloorMarkerPrimitive(
    const SpatialSpace& space,
    const std::vector<BoundaryVertex>& vertices,
    double floorY)
{
    SpatialPrimitive primitive;
    primitive.kind = SpatialPrimitiveKind::Marker;
    primitive.space = space;
    primitive.vertices = vertices;
    primitive.floorY = floorY;
    primitive.ceilingY = floorY;
    primitive.closeLoop = true;
    primitive.style.r = 255;
    primitive.style.g = 220;
    primitive.style.b = 80;
    primitive.style.a = 235;
    primitive.style.fillA = 36;
    primitive.style.fill = false;
    primitive.layer = 10;
    return primitive;
}

SpatialPrimitive TransformPrimitiveToStanding(
    const SpatialPrimitive& primitive)
{
    SpatialPrimitive out = primitive;
    if (primitive.space.kind == SpatialSpaceKind::Standing) {
        out.space = StandingSpace(primitive.space.trackingSystem);
        return out;
    }
    if (!primitive.space.transformValid) {
        out.vertices.clear();
        return out;
    }

    out.vertices = TransformToStandingUniverse(
        primitive.vertices,
        primitive.space.targetToStanding);
    out.floorY = TransformHeightToStandingUniverse(
        primitive.vertices,
        primitive.floorY,
        primitive.space.targetToStanding);
    out.ceilingY = TransformHeightToStandingUniverse(
        primitive.vertices,
        primitive.ceilingY,
        primitive.space.targetToStanding);
    out.space = StandingSpace(primitive.space.trackingSystem);
    out.space.transformEpoch = primitive.space.transformEpoch;
    return out;
}

std::vector<SpatialRenderCommand> BuildSpatialRenderCommands(
    const std::vector<SpatialPrimitive>& primitives)
{
    std::vector<SpatialRenderCommand> commands;
    commands.reserve(primitives.size());
    for (const SpatialPrimitive& primitive : primitives) {
        SpatialPrimitive standing = TransformPrimitiveToStanding(primitive);
        if (standing.vertices.empty()) {
            continue;
        }

        SpatialRenderCommand command;
        command.kind = standing.kind;
        command.standingVertices = std::move(standing.vertices);
        command.floorY = standing.floorY;
        command.ceilingY = standing.ceilingY;
        command.closeLoop = standing.closeLoop;
        command.style = standing.style;
        command.layer = standing.layer;
        command.ageFade = standing.ageFade;
        commands.push_back(std::move(command));
    }

    std::stable_sort(commands.begin(), commands.end(),
        [](const SpatialRenderCommand& a, const SpatialRenderCommand& b) {
            return a.layer < b.layer;
        });
    return commands;
}

uint8_t BoundaryAgeShade(size_t vertexIndex, size_t vertexCount)
{
    if (vertexCount <= 1 || vertexIndex + 1 >= vertexCount) {
        return 255;
    }
    const double t =
        static_cast<double>(vertexIndex) / static_cast<double>(vertexCount - 1);
    const int shade = static_cast<int>(std::lround(100.0 + 105.0 * t));
    return static_cast<uint8_t>(std::clamp(shade, 0, 255));
}

} // namespace wkopenvr::boundary
