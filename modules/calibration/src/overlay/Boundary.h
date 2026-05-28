#pragma once

#include "Calibration.h"

#include <Eigen/Geometry>
#include <cstdint>
#include <string>
#include <vector>

namespace wkopenvr::boundary {

// Top-down projection: drop Y, keep X and Z.
struct XZPoint { double x, z; };

// Polygon area on the XZ plane (signed). Clockwise winding gives negative.
double SignedAreaXZ(const std::vector<XZPoint>& poly);
double AbsoluteAreaXZ(const std::vector<XZPoint>& poly);

// Project boundary vertices to the XZ plane.
std::vector<XZPoint> ProjectXZ(const std::vector<BoundaryVertex>& v);

// Douglas-Peucker simplification of a 3D path. Returns the kept indices in
// input order. epsilonMeters is the max perpendicular deviation permitted.
std::vector<size_t> SimplifyDouglasPeucker(const std::vector<BoundaryVertex>& path,
                                            double epsilonMeters);

// Apply the continuous-cal SE(3) transform to convert target-space boundary
// vertices into standing-universe coordinates.
std::vector<BoundaryVertex> TransformToStandingUniverse(
    const std::vector<BoundaryVertex>& targetSpace,
    const Eigen::AffineCompact3d& targetToStanding);

// Convert a target-space horizontal height to the standing-universe height
// used by SteamVR chaperone wall quads.
double TransformHeightToStandingUniverse(
    double targetY,
    const Eigen::AffineCompact3d& targetToStanding);

double TransformHeightToStandingUniverse(
    const std::vector<BoundaryVertex>& targetSpace,
    double targetY,
    const Eigen::AffineCompact3d& targetToStanding);

// Return the target-space horizontal Y value that maps to standing-space floor
// height at the polygon center. This keeps saved target-space boundaries and
// SteamVR standing-space floor output aligned.
double TargetFloorYForStandingFloor(
    const std::vector<BoundaryVertex>& targetSpace,
    const Eigen::AffineCompact3d& targetToStanding,
    double standingFloorY = 0.0);

// Build/apply the same profile transform sent to the driver for the calibrated
// target tracking system. Boundary drawing stores raw target-space points and
// applies this transform only for preview and chaperone push.
Eigen::AffineCompact3d ProfileTransformFromCalibration(
    Eigen::Vector3d eulerDeg,
    Eigen::Vector3d transCm);

Eigen::Affine3d TransformPoseToStandingUniverse(
    const Eigen::Affine3d& rawPose,
    const Eigen::AffineCompact3d& targetToStanding);

vr::HmdMatrix34_t OffsetStandingZeroPoseForFloor(
    const vr::HmdMatrix34_t& standingZeroToRaw,
    double measuredFloorYStanding);

double ControllerFloorContactOffsetMeters(
    const std::string& controllerType,
    const Eigen::Affine3d& standingPose);

double AdjustControllerFloorYForContact(
    double controllerOriginYStanding,
    const std::string& controllerType,
    const Eigen::Affine3d& standingPose);

bool ApplySteamVrFloorOffset(
    double measuredFloorYStanding,
    char* errorBuffer = nullptr,
    size_t errorBufferSize = 0);

bool ApplySteamVrFloorOffsetFromDevice(
    vr::TrackedDeviceIndex_t deviceId,
    char* errorBuffer = nullptr,
    size_t errorBufferSize = 0);

double BoundaryFloorYAfterApply(
    double measuredFloorYStanding,
    bool moveSteamVrFloor);

bool BoundaryControllerMatchesTargetTrackingSystem(
    const std::string& controllerTrackingSystem,
    const std::string& targetTrackingSystem);

struct ChaperoneWorkingSet {
    bool valid = false;
    float playAreaX = 0.0f;
    float playAreaZ = 0.0f;
    std::vector<vr::HmdVector2_t> perimeter;
    std::vector<vr::HmdQuad_t> collisionBounds;
};

ChaperoneWorkingSet BuildChaperoneWorkingSet(
    const std::vector<BoundaryVertex>& standingUniverseVertices,
    double floorY,
    double ceilingY);

// Push the polygon to SteamVR chaperone. Returns false if VRChaperoneSetup
// is unavailable or any call fails. Builds vertical wall quads from floorY
// to ceilingY, pushes a conservative centered play area size, and commits
// to Live.
bool PushToChaperone(const std::vector<BoundaryVertex>& standingUniverseVertices,
                     double floorY, double ceilingY);

bool ShowWorkingChaperonePreview(
    const std::vector<BoundaryVertex>& standingUniverseVertices,
    double floorY,
    double ceilingY);

void HideWorkingChaperonePreview();

// Capture the live chaperone collision bounds for later restore.
// Returns serialized bytes suitable for storage in BoundaryConfig::priorChaperone.
std::vector<uint8_t> SnapshotCurrentChaperone();

// Restore chaperone from bytes produced by SnapshotCurrentChaperone.
// Returns false if snapshot bytes don't parse.
bool RestoreChaperoneFromSnapshot(const std::vector<uint8_t>& snapshot);

// Axis-aligned bounding rectangle of the boundary polygon on the XZ plane.
struct PolygonBounds { double xMin, xMax, zMin, zMax; };
PolygonBounds ComputePolygonBoundsXZ(const std::vector<BoundaryVertex>& v);

}  // namespace wkopenvr::boundary
