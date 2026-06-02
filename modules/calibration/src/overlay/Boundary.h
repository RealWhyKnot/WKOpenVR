#pragma once

#include "Calibration.h"

#include <Eigen/Geometry>
#include <cstdint>
#include <string>
#include <vector>

namespace wkopenvr::boundary {

// Top-down projection: drop Y, keep X and Z.
struct XZPoint { double x, z; };

// Axis-aligned bounding rectangle of the boundary polygon on the XZ plane.
struct PolygonBounds { double xMin, xMax, zMin, zMax; };

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

struct StandingYSample {
    bool valid = false;
    bool connected = false;
    vr::ETrackingResult trackingResult = vr::TrackingResult_Uninitialized;
    double y = 0.0;
};

struct SteamVrFloorVerification {
    bool beforeValid = false;
    bool afterValid = false;
    bool verified = false;
    double measuredFloorY = 0.0;
    double beforeY = 0.0;
    double afterY = 0.0;
    double expectedAfterY = 0.0;
    double residualY = 0.0;
};

StandingYSample SampleDeviceStandingY(vr::TrackedDeviceIndex_t deviceId);

SteamVrFloorVerification EvaluateSteamVrFloorVerification(
    const StandingYSample& before,
    const StandingYSample& after,
    double measuredFloorYStanding,
    double toleranceMeters = 0.05);

enum class BoundaryFloorSourceKind {
    None,
    SteamVrStanding,
    SavedBoundary,
    ControllerContact,
    Manual,
};

const char* BoundaryFloorSourceKindName(BoundaryFloorSourceKind kind);

struct SteamVrFloorSnapshot {
    bool chaperoneAvailable = false;
    vr::ChaperoneCalibrationState calibrationState = vr::ChaperoneCalibrationState_Error;
    bool playAreaSizeValid = false;
    float playAreaX = 0.0f;
    float playAreaZ = 0.0f;
    bool playAreaRectValid = false;
    vr::HmdQuad_t playAreaRect = {};
    bool chaperoneSetupAvailable = false;
    bool standingZeroValid = false;
    vr::HmdMatrix34_t standingZeroToRaw = {};
};

struct BoundaryFloorSourceRequest {
    bool boundaryStandingSpace = true;
    std::vector<BoundaryVertex> boundaryVertices;
    double savedBoundaryFloorY = 0.0;
    bool hasSavedBoundaryFloor = false;
    bool targetTransformValid = false;
    Eigen::AffineCompact3d targetToStanding = Eigen::AffineCompact3d::Identity();
    bool controllerContactValid = false;
    double controllerContactStandingY = 0.0;
    bool manualFloorValid = false;
    double manualStandingFloorY = 0.0;
};

struct BoundaryFloorSourceDecision {
    bool valid = false;
    BoundaryFloorSourceKind source = BoundaryFloorSourceKind::None;
    double boundaryFloorY = 0.0;
    double standingFloorY = 0.0;
    std::vector<std::string> rejectedReasons;
};

SteamVrFloorSnapshot QuerySteamVrFloorSnapshot();

BoundaryFloorSourceDecision ResolveBoundaryFloorSource(
    const SteamVrFloorSnapshot& steamVr,
    const BoundaryFloorSourceRequest& request);

bool BoundaryControllerMatchesTargetTrackingSystem(
    const std::string& controllerTrackingSystem,
    const std::string& targetTrackingSystem);

bool BoundaryCaptureShouldUseTargetSpace(
    bool controllerMatchesTarget,
    bool transformReady);

struct ChaperoneWorkingSet {
    bool valid = false;
    float playAreaX = 0.0f;
    float playAreaZ = 0.0f;
    std::vector<vr::HmdVector2_t> perimeter;
    std::vector<vr::HmdQuad_t> collisionBounds;
};

enum class ChaperoneOutputStatus {
    Ready,
    Empty,
    InvalidGeometry,
    VisualOnlyNoStandingOrigin,
};

struct ChaperonePreflightDiagnostics {
    size_t inputVertexCount = 0;
    size_t normalizedVertexCount = 0;
    double areaMetersSq = 0.0;
    PolygonBounds bounds{0.0, 0.0, 0.0, 0.0};
    bool originInsidePolygon = false;
    double originDistanceMeters = 0.0;
    double centroidX = 0.0;
    double centroidZ = 0.0;
    double floorY = 0.0;
    double ceilingY = 0.0;
    float playAreaX = 0.0f;
    float playAreaZ = 0.0f;
};

struct ChaperoneOutput {
    ChaperoneOutputStatus status = ChaperoneOutputStatus::Empty;
    ChaperoneWorkingSet workingSet;
    const char* reason = "empty";
    ChaperonePreflightDiagnostics diagnostics;

    bool ready() const { return status == ChaperoneOutputStatus::Ready && workingSet.valid; }
};

ChaperoneWorkingSet BuildChaperoneWorkingSet(
    const std::vector<BoundaryVertex>& standingUniverseVertices,
    double floorY,
    double ceilingY);

ChaperoneOutput BuildChaperoneOutput(
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

struct ChaperoneSnapshot {
    bool legacyCollisionBoundsOnly = false;
    bool hasPlayArea = false;
    float playAreaX = 0.0f;
    float playAreaZ = 0.0f;
    bool hasStandingZero = false;
    vr::HmdMatrix34_t standingZeroToRaw = {};
    bool hasSeatedZero = false;
    vr::HmdMatrix34_t seatedZeroToRaw = {};
    std::vector<vr::HmdVector2_t> perimeter;
    std::vector<vr::HmdQuad_t> collisionBounds;
};

std::vector<uint8_t> SerializeChaperoneSnapshot(
    const ChaperoneSnapshot& snapshot);

bool DeserializeChaperoneSnapshot(
    const std::vector<uint8_t>& bytes,
    ChaperoneSnapshot& snapshot,
    std::string* error = nullptr);

// Capture the live chaperone working-set fields for later restore.
// Returns serialized bytes suitable for storage in BoundaryConfig::priorChaperone.
std::vector<uint8_t> SnapshotCurrentChaperone();

// Restore chaperone from bytes produced by SnapshotCurrentChaperone.
// Returns false if snapshot bytes don't parse.
bool RestoreChaperoneFromSnapshot(const std::vector<uint8_t>& snapshot);

// Floor-protection: hold the SteamVR standing-zero floor against runtime resets.
// On a Quest + lighthouse (space-cal) rig the Oculus runtime re-asserts its own
// zeroed standing-zero after a chaperone reload (e.g. the ReloadInfo a boundary
// push triggers), wiping a floor the user set. SetFloorStandingZeroTarget records
// the standing-zero we committed for the floor; PushToChaperone folds that
// re-assert into the boundary push, and TickFloorStandingZeroWatchdog re-commits
// it whenever the live standing-zero drifts away. Keeps the standing-zero
// mechanism (the same call OpenVR Advanced Settings uses).
void SetFloorStandingZeroTarget(const vr::HmdMatrix34_t& standingZeroToRaw);
void ClearFloorStandingZeroTarget();
bool GetFloorStandingZeroTarget(vr::HmdMatrix34_t* out);
void TickFloorStandingZeroWatchdog();

PolygonBounds ComputePolygonBoundsXZ(const std::vector<BoundaryVertex>& v);

}  // namespace wkopenvr::boundary
