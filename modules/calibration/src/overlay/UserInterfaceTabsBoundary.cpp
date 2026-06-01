#include "Calibration.h"
#include "CalibrationMetrics.h"
#include "CalibrationProfileApply.h"
#include "Configuration.h"
#include "ControllerInput.h"
#include "DiagnosticsLog.h"
#include "Boundary.h"
#include "BoundaryCapture.h"
#include "BoundaryFloorCapture.h"
#include "BoundaryPreview.h"
#include "BoundaryRePush.h"
#include "UserInterfaceHeadMount.h"
#include "UiHelpers.h"

#include <openvr.h>
#include <algorithm>
#include <cmath>
#include <string>
#include <utility>
#include <vector>
#include <cstddef>
#include <cstdio>
#include <imgui/imgui.h>
#include "imgui_extensions.h"

extern CalibrationContext CalCtx;

void SaveProfile(CalibrationContext& ctx);
void CCal_TickBoundaryCapture();

// User journey for this tab:
//   1. Stabilize SLAM drift with the continuous target.
//   2. Draw your own play-space safety polygon.
//   3. Hand off Quest-specific headset setup to the Quest App module.

namespace {

// ---------------------------------------------------------------------------
// Section B: Safety boundary
// ---------------------------------------------------------------------------

wkopenvr::boundary::CaptureSession s_capture;
wkopenvr::boundary::FloorCaptureSession s_floorCapture;
wkopenvr::boundary::SpatialSession s_spatialSession;
std::string s_boundaryError;
std::string s_boundaryNativeStatus;
uint64_t s_floorSessionId = 0;
bool s_steamVrWorkingPreviewVisible = false;
bool s_floorPreviewVisible = false;
Eigen::AffineCompact3d s_capturePreviewTransform = Eigen::AffineCompact3d::Identity();
bool s_capturePreviewTransformValid = false;
bool s_captureUsesStandingSpace = false;
double s_captureFloorY = 0.0;
bool s_captureRequireTrigger = true;
bool s_captureLastTriggerHeld = false;
bool s_captureLastTriggerReadable = false;
wkopenvr::controller_input::TriggerReading s_captureLastTriggerReading = {};
BoundaryVertex s_smoothedPreviewCursor = {};
uint64_t s_smoothedPreviewCursorSession = 0;
bool s_smoothedPreviewCursorValid = false;
int32_t s_boundaryControllerDeviceId = -1;
int32_t s_identifyBoundaryControllerDeviceId = -1;
double s_nextIdentifyPulseAt = 0.0;
double s_identifyUntil = 0.0;
wkopenvr::boundary::BoundaryFloorSourceDecision s_cachedFloorSourceDecision;
double s_nextFloorSourceRefreshAt = 0.0;
bool s_cachedFloorSourceInitialized = false;

constexpr double kAutoBoundaryWallHeightMeters = 2.4;
constexpr double kMinimumBoundaryAreaMetersSq = 0.25;

struct BoundaryControllerChoice {
    int32_t deviceId = -1;
    std::string trackingSystem;
    std::string model;
    std::string serial;
    vr::ETrackedControllerRole role = vr::TrackedControllerRole_Invalid;
    bool poseValid = false;
    bool matchesTarget = false;
};

struct NativeBoundaryPushResult {
    bool attempted = false;
    bool applied = false;
    bool visualOnly = false;
    wkopenvr::boundary::ChaperoneOutput preflight;
    std::string message;
};

std::string TrackingSystemName(vr::IVRSystem* vrs, vr::TrackedDeviceIndex_t deviceId);
bool ShouldUseTargetSpaceForBoundaryController(const BoundaryControllerChoice* choice);

void HideBoundaryPreviewOverlay() {
    wkopenvr::boundary::TickBoundaryPreview(false, {}, CalCtx.boundary.floorY, false, "hide");
    s_floorPreviewVisible = false;
    if (s_steamVrWorkingPreviewVisible) {
        wkopenvr::boundary::HideWorkingChaperonePreview();
        s_steamVrWorkingPreviewVisible = false;
    }
}

bool BoundaryTransformReady()
{
    return CalCtx.validProfile;
}

bool TrackingSystemMatchesBoundaryTarget(const std::string& trackingSystem)
{
    return wkopenvr::boundary::BoundaryControllerMatchesTargetTrackingSystem(
        trackingSystem,
        CalCtx.targetTrackingSystem);
}

Eigen::AffineCompact3d BoundaryTargetToStandingTransform()
{
    return wkopenvr::boundary::ProfileTransformFromCalibration(
        CalCtx.calibratedRotation,
        CalCtx.calibratedTranslation);
}

double BoundaryHeightToStanding(
    const std::vector<BoundaryVertex>& targetVertices,
    double targetY)
{
    if (CalCtx.boundary.standingSpace) {
        return targetY;
    }
    return wkopenvr::boundary::TransformHeightToStandingUniverse(
        targetVertices,
        targetY,
        BoundaryTargetToStandingTransform());
}

std::vector<BoundaryVertex> BoundaryVerticesToStanding(
    const std::vector<BoundaryVertex>& targetVertices,
    const Eigen::AffineCompact3d& targetToStanding)
{
    return wkopenvr::boundary::TransformToStandingUniverse(
        targetVertices,
        targetToStanding);
}

const Eigen::AffineCompact3d& PreviewTargetToStandingTransform()
{
    if (s_capturePreviewTransformValid
        && s_capture.state() != wkopenvr::boundary::CaptureState::Idle)
    {
        return s_capturePreviewTransform;
    }

    s_capturePreviewTransform = BoundaryTargetToStandingTransform();
    return s_capturePreviewTransform;
}

std::vector<BoundaryVertex> BoundaryVerticesToStanding(
    const std::vector<BoundaryVertex>& targetVertices)
{
    return BoundaryVerticesToStanding(
        targetVertices,
        BoundaryTargetToStandingTransform());
}

void TickBoundaryPreviewForTargetVerticesAtFloor(
    bool wantVisible,
    const std::vector<BoundaryVertex>& targetVertices,
    double targetFloorY,
    bool closeLoop)
{
    if (!wantVisible || targetVertices.empty() || !BoundaryTransformReady()) {
        HideBoundaryPreviewOverlay();
        return;
    }

    const auto& targetToStanding = PreviewTargetToStandingTransform();
    wkopenvr::boundary::TickBoundaryPreview(
        true,
        BoundaryVerticesToStanding(targetVertices, targetToStanding),
        wkopenvr::boundary::TransformHeightToStandingUniverse(
            targetVertices,
            targetFloorY,
            targetToStanding),
        closeLoop,
        "target_vertices");

    if (s_steamVrWorkingPreviewVisible) {
        wkopenvr::boundary::HideWorkingChaperonePreview();
        s_steamVrWorkingPreviewVisible = false;
    }
}

void TickBoundaryPreviewForStandingVerticesAtFloor(
    bool wantVisible,
    const std::vector<BoundaryVertex>& standingVertices,
    double floorY,
    bool closeLoop)
{
    if (!wantVisible || standingVertices.empty()) {
        HideBoundaryPreviewOverlay();
        return;
    }
    wkopenvr::boundary::TickBoundaryPreview(
        true,
        standingVertices,
        floorY,
        closeLoop,
        "standing_vertices");
}

void LogNativeBoundaryPreflight(
    const char* source,
    const wkopenvr::boundary::ChaperoneOutput& output,
    const Eigen::AffineCompact3d& targetToStanding,
    bool standingSpace)
{
    const auto& d = output.diagnostics;
    const Eigen::Vector3d t = targetToStanding.translation();
    char lbuf[1024];
    snprintf(lbuf, sizeof lbuf,
        "[boundary_native_preflight] source=%s status=%d reason=%s ready=%d space=%s input_vertices=%zu normalized_vertices=%zu area=%.3f floor=%.3f ceiling=%.3f origin_inside=%d origin_distance=%.3f centroid=(%.3f,%.3f) bounds_x=(%.3f,%.3f) bounds_z=(%.3f,%.3f) play=(%.3f,%.3f) transform_trans=(%.3f,%.3f,%.3f)",
        source ? source : "unknown",
        static_cast<int>(output.status),
        output.reason ? output.reason : "",
        output.ready() ? 1 : 0,
        standingSpace ? "standing" : "target",
        d.inputVertexCount,
        d.normalizedVertexCount,
        d.areaMetersSq,
        d.floorY,
        d.ceilingY,
        d.originInsidePolygon ? 1 : 0,
        d.originDistanceMeters,
        d.centroidX,
        d.centroidZ,
        d.bounds.xMin,
        d.bounds.xMax,
        d.bounds.zMin,
        d.bounds.zMax,
        d.playAreaX,
        d.playAreaZ,
        t.x(), t.y(), t.z());
    Metrics::WriteLogAnnotation(lbuf);
    openvr_pair::common::DiagnosticLog("boundary_native_preflight", "%s", lbuf);
}

NativeBoundaryPushResult PushTargetBoundaryToChaperoneWithTransform(
    const Eigen::AffineCompact3d& targetToStanding,
    const char* source)
{
    NativeBoundaryPushResult result;
    if (!CalCtx.boundary.standingSpace && !BoundaryTransformReady()) {
        result.message = "Run calibration first so the boundary can be mapped into SteamVR space.";
        return result;
    }
    if (CalCtx.boundary.vertices.size() < 3) {
        result.message = "Boundary needs at least three floor points.";
        return result;
    }

    const auto standingVertices = CalCtx.boundary.standingSpace
        ? CalCtx.boundary.vertices
        : BoundaryVerticesToStanding(CalCtx.boundary.vertices, targetToStanding);
    const double standingFloor = CalCtx.boundary.standingSpace
        ? CalCtx.boundary.floorY
        : wkopenvr::boundary::TransformHeightToStandingUniverse(
            CalCtx.boundary.vertices,
            CalCtx.boundary.floorY,
            targetToStanding);
    const double standingCeiling = CalCtx.boundary.standingSpace
        ? CalCtx.boundary.ceilingY
        : wkopenvr::boundary::TransformHeightToStandingUniverse(
            CalCtx.boundary.vertices,
            CalCtx.boundary.floorY + kAutoBoundaryWallHeightMeters,
            targetToStanding);
    const auto bounds = wkopenvr::boundary::ComputePolygonBoundsXZ(standingVertices);
    {
        const Eigen::Vector3d t = targetToStanding.translation();
        char lbuf[512];
        snprintf(lbuf, sizeof lbuf,
            "[boundary] target push request: source=%s space=%s target_vertices=%zu standing_floor=%.3f standing_ceiling=%.3f transform_trans=(%.3f,%.3f,%.3f) bounds_x=(%.3f,%.3f) bounds_z=(%.3f,%.3f)",
            source ? source : "unknown",
            CalCtx.boundary.standingSpace ? "standing" : "target",
            CalCtx.boundary.vertices.size(),
            standingFloor,
            standingCeiling,
            t.x(), t.y(), t.z(),
            bounds.xMin, bounds.xMax,
            bounds.zMin, bounds.zMax);
        Metrics::WriteLogAnnotation(lbuf);
        openvr_pair::common::DiagnosticLog("boundary", "%s", lbuf);
    }

    const auto output = wkopenvr::boundary::BuildChaperoneOutput(
        standingVertices,
        standingFloor,
        standingCeiling);
    result.preflight = output;
    LogNativeBoundaryPreflight(
        source,
        output,
        targetToStanding,
        CalCtx.boundary.standingSpace);
    if (!output.ready()) {
        result.visualOnly =
            output.status == wkopenvr::boundary::ChaperoneOutputStatus::VisualOnlyNoStandingOrigin;
        result.message = result.visualOnly
            ? "Boundary can be previewed, but SteamVR native chaperone requires the standing origin inside the polygon."
            : "Boundary geometry is not valid for SteamVR native chaperone.";
        char obuf[224];
        snprintf(obuf, sizeof obuf,
            "[boundary] target push rejected before commit: status=%d reason=%s",
            static_cast<int>(output.status),
            output.reason ? output.reason : "");
        Metrics::WriteLogAnnotation(obuf);
        openvr_pair::common::DiagnosticLog("boundary", "%s", obuf);
        openvr_pair::common::DiagnosticLog(
            "boundary_apply_native_skipped",
            "source=%s status=%d reason=%s origin_inside=%d origin_distance=%.3f visual_only=%d",
            source ? source : "unknown",
            static_cast<int>(output.status),
            output.reason ? output.reason : "",
            output.diagnostics.originInsidePolygon ? 1 : 0,
            output.diagnostics.originDistanceMeters,
            result.visualOnly ? 1 : 0);
        return result;
    }

    result.attempted = true;
    const bool ok = wkopenvr::boundary::PushToChaperone(
        standingVertices,
        standingFloor,
        standingCeiling);
    if (!ok) {
        result.message = "PushToChaperone failed. Is SteamVR running?";
        openvr_pair::common::DiagnosticLog(
            "boundary_apply_native_skipped",
            "source=%s status=%d reason=push_failed origin_inside=%d origin_distance=%.3f visual_only=0",
            source ? source : "unknown",
            static_cast<int>(output.status),
            output.diagnostics.originInsidePolygon ? 1 : 0,
            output.diagnostics.originDistanceMeters);
    } else {
        NoteBoundaryPushedForTransform(targetToStanding);
        result.applied = true;
        result.message = "SteamVR native boundary updated.";
        openvr_pair::common::DiagnosticLog(
            "boundary_apply_native_applied",
            "source=%s vertices=%zu area=%.3f play=(%.3f,%.3f) floor=%.3f ceiling=%.3f",
            source ? source : "unknown",
            output.diagnostics.normalizedVertexCount,
            output.diagnostics.areaMetersSq,
            output.diagnostics.playAreaX,
            output.diagnostics.playAreaZ,
            output.diagnostics.floorY,
            output.diagnostics.ceilingY);
    }
    return result;
}

bool PushTargetBoundaryToChaperone(std::string& error)
{
    const auto result = PushTargetBoundaryToChaperoneWithTransform(
        BoundaryTargetToStandingTransform(),
        "current");
    if (!result.applied) {
        error = result.message;
    }
    return result.applied;
}

void RepushActiveBoundaryAfterEdit()
{
    if (!CalCtx.boundary.enabled) return;

    std::string pushError;
    if (!PushTargetBoundaryToChaperone(pushError)) {
        s_boundaryNativeStatus = "WKOpenVR boundary saved locally; SteamVR native boundary was not updated.";
        if (!pushError.empty()) {
            s_boundaryNativeStatus += " ";
            s_boundaryNativeStatus += pushError;
        }
    }
}

void ApplyBoundaryFloorY(double floorY)
{
    if (!std::isfinite(floorY)) return;

    CalCtx.boundary.floorY = floorY;
    CalCtx.boundary.ceilingY = floorY + kAutoBoundaryWallHeightMeters;
    for (auto& v : CalCtx.boundary.vertices) {
        v.y = floorY;
    }
    s_cachedFloorSourceInitialized = false;
}

// Shift the SteamVR standing-zero pose vertically by deltaY metres and commit it
// to the live chaperone. This is the correct, runtime-consistent way to set floor
// height (the same mechanism OpenVR Advanced Settings uses): moving the universe
// origin keeps the runtime, compositor, chaperone bounds and our boundary all in
// agreement, unlike a per-device world-Y shift which fights the runtime. The
// offset is applied along the standing-zero's own up axis so the rotation is
// untouched and the play space can't tilt. Returns true on a committed write.
bool AdjustStandingZeroFloorY(double deltaY)
{
    if (!std::isfinite(deltaY)) return false;
    if (std::fabs(deltaY) < 1e-6) return true;
    auto* setup = vr::VRChaperoneSetup();
    if (!setup) return false;

    setup->RevertWorkingCopy();
    vr::HmdMatrix34_t zero;
    if (!setup->GetWorkingStandingZeroPoseToRawTrackingPose(&zero)) {
        openvr_pair::common::DiagnosticLog("boundary-floor", "standing_zero get failed");
        return false;
    }

    const double beforeY = zero.m[1][3];
    zero.m[0][3] += zero.m[0][1] * deltaY;
    zero.m[1][3] += zero.m[1][1] * deltaY;
    zero.m[2][3] += zero.m[2][1] * deltaY;
    setup->SetWorkingStandingZeroPoseToRawTrackingPose(&zero);
    const bool committed = setup->CommitWorkingCopy(vr::EChaperoneConfigFile_Live);

    char lbuf[224];
    snprintf(lbuf, sizeof lbuf,
        "[boundary-floor] standing_zero shift dY=%.4f beforeY=%.4f afterY=%.4f committed=%d",
        deltaY, beforeY, zero.m[1][3], committed ? 1 : 0);
    Metrics::WriteLogAnnotation(lbuf);
    openvr_pair::common::DiagnosticLog("boundary-floor", "%s", lbuf);
    return committed;
}

wkopenvr::boundary::BoundaryFloorSourceRequest BuildFloorSourceRequest(
    bool controllerContactValid = false,
    double controllerContactStandingY = 0.0)
{
    wkopenvr::boundary::BoundaryFloorSourceRequest request;
    request.boundaryStandingSpace = CalCtx.boundary.standingSpace;
    request.boundaryVertices = CalCtx.boundary.vertices;
    request.savedBoundaryFloorY = CalCtx.boundary.floorY;
    request.hasSavedBoundaryFloor = std::isfinite(CalCtx.boundary.floorY);
    request.targetTransformValid = BoundaryTransformReady();
    if (request.targetTransformValid) {
        request.targetToStanding = BoundaryTargetToStandingTransform();
    }
    request.controllerContactValid = controllerContactValid &&
        std::isfinite(controllerContactStandingY);
    request.controllerContactStandingY = controllerContactStandingY;
    return request;
}

std::string FloorSourceRejectedSummary(
    const wkopenvr::boundary::BoundaryFloorSourceDecision& decision)
{
    if (decision.rejectedReasons.empty()) return "";

    std::string out;
    constexpr size_t kMaxReasons = 5;
    const size_t count = std::min(kMaxReasons, decision.rejectedReasons.size());
    for (size_t i = 0; i < count; ++i) {
        if (!out.empty()) out += ",";
        out += decision.rejectedReasons[i];
    }
    if (decision.rejectedReasons.size() > count) {
        out += ",more";
    }
    return out;
}

void PlayAreaRectYRange(
    const wkopenvr::boundary::SteamVrFloorSnapshot& snapshot,
    double& minY,
    double& maxY)
{
    minY = 0.0;
    maxY = 0.0;
    if (!snapshot.playAreaRectValid) return;

    minY = snapshot.playAreaRect.vCorners[0].v[1];
    maxY = snapshot.playAreaRect.vCorners[0].v[1];
    for (const auto& p : snapshot.playAreaRect.vCorners) {
        minY = std::min(minY, static_cast<double>(p.v[1]));
        maxY = std::max(maxY, static_cast<double>(p.v[1]));
    }
}

void LogFloorSourceDecision(
    const char* context,
    const wkopenvr::boundary::SteamVrFloorSnapshot& snapshot,
    const wkopenvr::boundary::BoundaryFloorSourceRequest& request,
    const wkopenvr::boundary::BoundaryFloorSourceDecision& decision,
    int deviceId = -1,
    const std::string& trackingSystem = {},
    bool rawPoseValid = false)
{
    double rectMinY = 0.0;
    double rectMaxY = 0.0;
    PlayAreaRectYRange(snapshot, rectMinY, rectMaxY);
    const auto transformT = request.targetToStanding.translation();
    const std::string rejects = FloorSourceRejectedSummary(decision);
    char lbuf[1400];
    snprintf(lbuf, sizeof lbuf,
        "[boundary-floor-source] context=%s valid=%d source='%s' boundary_floor=%.4f standing_floor=%.4f rejects='%s' chaperone=%d state=%d play_size=%d play=(%.3f,%.3f) play_rect=%d rect_y=(%.3f,%.3f) setup=%d standing_zero=%d standing_zero_t=(%.3f,%.3f,%.3f) target_transform=%d target_t=(%.3f,%.3f,%.3f) saved_floor=%.4f controller_valid=%d controller_y=%.4f controller_device=%d controller_system='%s' raw_pose=%d head_mode=%d head_device=%d head_target=%d driver_synth=%d",
        context ? context : "unknown",
        decision.valid ? 1 : 0,
        wkopenvr::boundary::BoundaryFloorSourceKindName(decision.source),
        decision.boundaryFloorY,
        decision.standingFloorY,
        rejects.c_str(),
        snapshot.chaperoneAvailable ? 1 : 0,
        static_cast<int>(snapshot.calibrationState),
        snapshot.playAreaSizeValid ? 1 : 0,
        snapshot.playAreaX,
        snapshot.playAreaZ,
        snapshot.playAreaRectValid ? 1 : 0,
        rectMinY,
        rectMaxY,
        snapshot.chaperoneSetupAvailable ? 1 : 0,
        snapshot.standingZeroValid ? 1 : 0,
        snapshot.standingZeroToRaw.m[0][3],
        snapshot.standingZeroToRaw.m[1][3],
        snapshot.standingZeroToRaw.m[2][3],
        request.targetTransformValid ? 1 : 0,
        transformT.x(),
        transformT.y(),
        transformT.z(),
        request.savedBoundaryFloorY,
        request.controllerContactValid ? 1 : 0,
        request.controllerContactStandingY,
        deviceId,
        trackingSystem.c_str(),
        rawPoseValid ? 1 : 0,
        static_cast<int>(CalCtx.headMount.mode),
        CalCtx.headMount.deviceID,
        CalCtx.targetID,
        CalCtx.headMount.mode == HeadMountMode::DriverSynth ? 1 : 0);
    Metrics::WriteLogAnnotation(lbuf);
    openvr_pair::common::DiagnosticLog("boundary-floor-source", "%s", lbuf);
}

wkopenvr::boundary::BoundaryFloorSourceDecision CurrentFloorSourceDecision(
    bool controllerContactValid = false,
    double controllerContactStandingY = 0.0,
    bool logDecision = false,
    const char* logContext = "ui",
    bool ignoreSavedBoundary = false)
{
    const auto snapshot = wkopenvr::boundary::QuerySteamVrFloorSnapshot();
    auto request = BuildFloorSourceRequest(
        controllerContactValid,
        controllerContactStandingY);
    if (ignoreSavedBoundary) {
        request.hasSavedBoundaryFloor = false;
    }
    auto decision = wkopenvr::boundary::ResolveBoundaryFloorSource(
        snapshot,
        request);
    if (logDecision) {
        LogFloorSourceDecision(
            logContext ? logContext : (controllerContactValid ? "controller_sample" : "ui"),
            snapshot,
            request,
            decision);
    }
    return decision;
}

const wkopenvr::boundary::BoundaryFloorSourceDecision& CachedFloorSourceDecision(
    bool force = false)
{
    const double now = CalCtx.timeLastTick;
    if (force || !s_cachedFloorSourceInitialized ||
        now >= s_nextFloorSourceRefreshAt)
    {
        s_cachedFloorSourceDecision = CurrentFloorSourceDecision();
        s_cachedFloorSourceInitialized = true;
        s_nextFloorSourceRefreshAt = now + 0.50;
    }
    return s_cachedFloorSourceDecision;
}

void RestoreFloorCaptureOriginal()
{
    if (!s_floorCapture.active()) return;
    CalCtx.boundary.floorY = s_floorCapture.originalFloorY();
    CalCtx.boundary.ceilingY = CalCtx.boundary.floorY + kAutoBoundaryWallHeightMeters;
    for (auto& v : CalCtx.boundary.vertices) {
        v.y = CalCtx.boundary.floorY;
    }
}

void TickBoundaryPreviewForTargetVertices(
    bool wantVisible,
    const std::vector<BoundaryVertex>& targetVertices,
    bool closeLoop)
{
    if (CalCtx.boundary.standingSpace) {
        TickBoundaryPreviewForStandingVerticesAtFloor(
            wantVisible,
            targetVertices,
            CalCtx.boundary.floorY,
            closeLoop);
        return;
    }
    TickBoundaryPreviewForTargetVerticesAtFloor(
        wantVisible,
        targetVertices,
        CalCtx.boundary.floorY,
        closeLoop);
}

void StartBoundaryCapture(const BoundaryControllerChoice* selectedController)
{
    const bool useTargetSpace =
        ShouldUseTargetSpaceForBoundaryController(selectedController);
    s_captureUsesStandingSpace = !useTargetSpace;
    s_capturePreviewTransform = useTargetSpace
        ? BoundaryTargetToStandingTransform()
        : Eigen::AffineCompact3d::Identity();
    s_capturePreviewTransformValid = true;
    if (useTargetSpace) {
        s_captureFloorY = (!CalCtx.boundary.standingSpace && std::isfinite(CalCtx.boundary.floorY))
            ? CalCtx.boundary.floorY
            : wkopenvr::boundary::TargetFloorYForStandingFloor(
                CalCtx.boundary.vertices,
                s_capturePreviewTransform,
                0.0);
    } else {
        s_captureFloorY = CalCtx.boundary.standingSpace ? CalCtx.boundary.floorY : 0.0;
    }
    s_smoothedPreviewCursorValid = false;
    s_captureLastTriggerHeld = false;
    s_captureLastTriggerReadable = false;
    s_captureLastTriggerReading = {};
    s_boundaryNativeStatus.clear();
    wkopenvr::boundary::ResetBoundaryPreviewUploadFailures();
    s_capture.Start();
    s_spatialSession =
        wkopenvr::boundary::BoundaryCaptureSessionDescriptor(
            useTargetSpace
                ? wkopenvr::boundary::TargetSpace(
                    selectedController ? selectedController->trackingSystem : CalCtx.targetTrackingSystem,
                    s_capturePreviewTransform,
                    s_capture.sessionId())
                : wkopenvr::boundary::StandingSpace(
                    selectedController ? selectedController->trackingSystem : std::string()),
            selectedController ? selectedController->deviceId : -1,
            selectedController ? selectedController->trackingSystem : std::string(),
            s_captureFloorY,
            s_captureRequireTrigger,
            s_capture.sessionId());

    char lbuf[256];
    snprintf(lbuf, sizeof lbuf,
        "[boundary-capture] mode: space=%s floor_y=%.3f require_trigger=%d controller=%d system='%s' target_match=%d transform_ready=%d",
        s_captureUsesStandingSpace ? "standing" : "target",
        s_captureFloorY,
        s_captureRequireTrigger ? 1 : 0,
        selectedController ? selectedController->deviceId : -1,
        selectedController ? selectedController->trackingSystem.c_str() : "",
        selectedController && selectedController->matchesTarget ? 1 : 0,
        BoundaryTransformReady() ? 1 : 0);
    Metrics::WriteLogAnnotation(lbuf);
    openvr_pair::common::DiagnosticLog("boundary-capture", "%s", lbuf);
}

void StopBoundaryCapturePreview()
{
    s_spatialSession = {};
    s_capturePreviewTransformValid = false;
    s_smoothedPreviewCursorValid = false;
    HideBoundaryPreviewOverlay();
}

std::string DeviceStringProperty(
    vr::IVRSystem* vrs,
    vr::TrackedDeviceIndex_t deviceId,
    vr::ETrackedDeviceProperty property)
{
    if (!vrs) return {};
    char buffer[vr::k_unMaxPropertyStringSize] = {};
    vr::ETrackedPropertyError err = vr::TrackedProp_Success;
    vrs->GetStringTrackedDeviceProperty(
        deviceId,
        property,
        buffer,
        vr::k_unMaxPropertyStringSize,
        &err);
    return err == vr::TrackedProp_Success ? std::string(buffer) : std::string();
}

const char* ControllerRoleLabel(vr::ETrackedControllerRole role)
{
    switch (role) {
    case vr::TrackedControllerRole_LeftHand: return "Left";
    case vr::TrackedControllerRole_RightHand: return "Right";
    case vr::TrackedControllerRole_OptOut: return "Opt-out";
    case vr::TrackedControllerRole_Treadmill: return "Treadmill";
    default: return "Controller";
    }
}

std::vector<BoundaryControllerChoice> EnumerateBoundaryControllers(vr::IVRSystem* vrs)
{
    std::vector<BoundaryControllerChoice> choices;
    if (!vrs) return choices;

    vr::TrackedDevicePose_t standingPoses[vr::k_unMaxTrackedDeviceCount]{};
    vrs->GetDeviceToAbsoluteTrackingPose(
        vr::TrackingUniverseStanding,
        0.0f,
        standingPoses,
        vr::k_unMaxTrackedDeviceCount);

    for (vr::TrackedDeviceIndex_t deviceId = 0;
         deviceId < vr::k_unMaxTrackedDeviceCount;
         ++deviceId)
    {
        if (vrs->GetTrackedDeviceClass(deviceId) != vr::TrackedDeviceClass_Controller) {
            continue;
        }

        BoundaryControllerChoice choice;
        choice.deviceId = static_cast<int32_t>(deviceId);
        choice.trackingSystem = TrackingSystemName(vrs, deviceId);
        choice.matchesTarget = TrackingSystemMatchesBoundaryTarget(choice.trackingSystem);

        choice.model = DeviceStringProperty(vrs, deviceId, vr::Prop_ModelNumber_String);
        choice.serial = DeviceStringProperty(vrs, deviceId, vr::Prop_SerialNumber_String);
        vr::ETrackedPropertyError err = vr::TrackedProp_Success;
        choice.role = static_cast<vr::ETrackedControllerRole>(
            vrs->GetInt32TrackedDeviceProperty(
                deviceId,
                vr::Prop_ControllerRoleHint_Int32,
                &err));
        if (err != vr::TrackedProp_Success) {
            choice.role = vr::TrackedControllerRole_Invalid;
        }

        const auto& pose = standingPoses[deviceId];
        choice.poseValid = pose.bDeviceIsConnected
            && pose.bPoseIsValid
            && pose.eTrackingResult == vr::ETrackingResult::TrackingResult_Running_OK;
        choices.push_back(std::move(choice));
    }

    return choices;
}

void ResolveBoundaryControllerSelection(
    const std::vector<BoundaryControllerChoice>& choices)
{
    auto choose = [&](bool targetOnly) -> int32_t {
        std::vector<wkopenvr::controller_input::ControllerSelectionChoice> selection;
        selection.reserve(choices.size());
        for (const auto& choice : choices) {
            if (targetOnly && !choice.matchesTarget) {
                continue;
            }
            selection.push_back({
                choice.deviceId,
                choice.role,
                choice.poseValid
            });
        }
        return wkopenvr::controller_input::ChoosePreferredController(
            selection.data(),
            selection.size(),
            s_boundaryControllerDeviceId);
    };

    int32_t selected = choose(true);
    if (selected < 0) {
        selected = choose(false);
    }
    s_boundaryControllerDeviceId = selected;
}

const BoundaryControllerChoice* SelectedBoundaryController(
    const std::vector<BoundaryControllerChoice>& choices)
{
    for (const auto& choice : choices) {
        if (choice.deviceId == s_boundaryControllerDeviceId) {
            return &choice;
        }
    }
    return nullptr;
}

bool ShouldUseTargetSpaceForBoundaryController(const BoundaryControllerChoice* choice)
{
    return wkopenvr::boundary::BoundaryCaptureShouldUseTargetSpace(
        choice && choice->matchesTarget,
        BoundaryTransformReady());
}

std::string BoundaryControllerLabel(const BoundaryControllerChoice& choice)
{
    char prefix[64];
    snprintf(prefix, sizeof prefix, "%s controller %d",
        ControllerRoleLabel(choice.role),
        choice.deviceId);

    std::string label(prefix);
    if (!choice.model.empty()) {
        label += " - ";
        label += choice.model;
    }
    if (!choice.serial.empty()) {
        label += " (";
        label += choice.serial;
        label += ")";
    }
    if (!choice.poseValid) {
        label += " - waiting for pose";
    }
    if (!choice.trackingSystem.empty()) {
        label += " - ";
        label += choice.trackingSystem;
    }
    if (!CalCtx.targetTrackingSystem.empty() && !choice.matchesTarget) {
        label += " - non-target";
    }
    return label;
}

void StartBoundaryControllerIdentify(int32_t deviceId)
{
    if (deviceId < 0) return;
    s_identifyBoundaryControllerDeviceId = deviceId;
    const double now = ImGui::GetTime();
    s_nextIdentifyPulseAt = now;
    s_identifyUntil = now + 1.25;
}

void TickBoundaryControllerIdentify()
{
    if (s_identifyBoundaryControllerDeviceId < 0) return;
    auto* vrs = vr::VRSystem();
    if (!vrs) {
        s_identifyBoundaryControllerDeviceId = -1;
        return;
    }

    const double now = ImGui::GetTime();
    if (now > s_identifyUntil) {
        s_identifyBoundaryControllerDeviceId = -1;
        return;
    }
    if (now < s_nextIdentifyPulseAt) return;

    vrs->TriggerHapticPulse(
        static_cast<vr::TrackedDeviceIndex_t>(s_identifyBoundaryControllerDeviceId),
        0,
        2400);
    s_nextIdentifyPulseAt = now + 0.075;
}

void DrawBoundaryControllerSelector(
    const std::vector<BoundaryControllerChoice>& choices)
{
    ImGui::TextDisabled("Controller");
    ImGui::SameLine();

    if (choices.empty()) {
        int empty = 0;
        const char* items[] = { "No target-system controllers detected" };
        ImGui::BeginDisabled();
        ImGui::Combo("##boundary_controller", &empty, items, 1);
        ImGui::EndDisabled();
        ImGui::SameLine();
        ImGui::BeginDisabled();
        ImGui::SmallButton("Identify");
        ImGui::EndDisabled();
        return;
    }

    std::vector<std::string> labels;
    std::vector<const char*> labelPtrs;
    labels.reserve(choices.size());
    labelPtrs.reserve(choices.size());
    int current = 0;
    for (size_t i = 0; i < choices.size(); ++i) {
        labels.push_back(BoundaryControllerLabel(choices[i]));
        labelPtrs.push_back(labels.back().c_str());
        if (choices[i].deviceId == s_boundaryControllerDeviceId) {
            current = static_cast<int>(i);
        }
    }

    ImGui::PushItemWidth(std::min(420.0f, ImGui::GetContentRegionAvail().x - 90.0f));
    if (ImGui::Combo(
            "##boundary_controller",
            &current,
            labelPtrs.data(),
            static_cast<int>(labelPtrs.size())))
    {
        if (current >= 0 && current < static_cast<int>(choices.size())) {
            s_boundaryControllerDeviceId = choices[static_cast<size_t>(current)].deviceId;
            s_smoothedPreviewCursorValid = false;
        }
    }
    ImGui::PopItemWidth();
    ImGui::SameLine();
    if (ImGui::SmallButton("Identify")) {
        StartBoundaryControllerIdentify(s_boundaryControllerDeviceId);
    }
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Vibrates the selected controller so you can confirm which one will set the floor and draw the boundary.");
    }
}

bool BoundaryLoopNearlyClosed(const std::vector<BoundaryVertex>& verts)
{
    if (verts.size() < 3) return false;
    const auto& first = verts.front();
    const auto& last = verts.back();
    const double dx = first.x - last.x;
    const double dz = first.z - last.z;
    return (dx * dx + dz * dz) <= (0.25 * 0.25);
}

double BoundaryDistanceSq(const BoundaryVertex& a, const BoundaryVertex& b)
{
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    const double dz = a.z - b.z;
    return dx * dx + dy * dy + dz * dz;
}

std::vector<BoundaryVertex> BoundaryPreviewPathWithCursor(
    const std::vector<BoundaryVertex>& rawVertices,
    const wkopenvr::boundary::FloorHitPreview* cursor)
{
    std::vector<BoundaryVertex> preview = rawVertices;
    if (cursor && cursor->valid) {
        if (preview.empty() ||
            BoundaryDistanceSq(preview.back(), cursor->hit) > (0.01 * 0.01))
        {
            preview.push_back(cursor->hit);
        }
    }
    return preview;
}

wkopenvr::boundary::SpatialPrimitive CapturePreviewPrimitive(
    wkopenvr::boundary::SpatialPrimitiveKind kind,
    const std::vector<BoundaryVertex>& vertices,
    bool closeLoop,
    wkopenvr::boundary::SpatialStyle style,
    int layer,
    bool ageFade = false)
{
    wkopenvr::boundary::SpatialPrimitive primitive;
    primitive.kind = kind;
    primitive.space = s_spatialSession.authoringSpace;
    primitive.vertices = vertices;
    primitive.floorY = s_spatialSession.floorY;
    primitive.ceilingY = s_spatialSession.ceilingY;
    primitive.closeLoop = closeLoop;
    primitive.style = style;
    primitive.layer = layer;
    primitive.ageFade = ageFade;
    return primitive;
}

std::vector<wkopenvr::boundary::SpatialRenderCommand> BuildCapturePreviewCommands(
    const std::vector<BoundaryVertex>& rawVertices,
    const wkopenvr::boundary::FloorHitPreview* cursor)
{
    using namespace wkopenvr::boundary;
    std::vector<SpatialPrimitive> primitives;
    const auto preview = BoundaryPreviewPathWithCursor(rawVertices, cursor);
    const bool closePreview = BoundaryLoopNearlyClosed(preview);

    if (closePreview && preview.size() >= 3) {
        SpatialStyle fillStyle;
        fillStyle.r = 210;
        fillStyle.g = 210;
        fillStyle.b = 210;
        fillStyle.a = 0;
        fillStyle.fillA = 48;
        fillStyle.strokeMeters = 0.0;
        fillStyle.dotMeters = 0.0;
        fillStyle.fill = true;
        primitives.push_back(CapturePreviewPrimitive(
            SpatialPrimitiveKind::PolygonFloorRegion,
            preview,
            true,
            fillStyle,
            0));
    }

    if (rawVertices.size() >= 1) {
        // The drawn path is stroked as a line AND a dot is drawn at every point
        // the user laid down, so they can see each captured point as they walk.
        // Both line and dots fade newest-white -> oldest-gray (per-vertex shade in
        // the rasterizer) so recency reads at a glance.
        SpatialStyle pathStyle;
        pathStyle.r = 235;
        pathStyle.g = 235;
        pathStyle.b = 235;
        pathStyle.a = 235;
        pathStyle.fillA = 0;
        pathStyle.strokeMeters = 0.030;
        pathStyle.dotMeters = 0.045;
        pathStyle.fill = false;
        primitives.push_back(CapturePreviewPrimitive(
            SpatialPrimitiveKind::PolylinePath,
            rawVertices,
            false,
            pathStyle,
            2,
            /*ageFade=*/true));
    }

    if (cursor && cursor->valid) {
        SpatialStyle ghostStyle;
        ghostStyle.r = 245;
        ghostStyle.g = 245;
        ghostStyle.b = 245;
        ghostStyle.a = 170;
        ghostStyle.fillA = 0;
        ghostStyle.strokeMeters = 0.020;
        ghostStyle.dotMeters = 0.0;
        ghostStyle.fill = false;
        if (!rawVertices.empty()) {
            primitives.push_back(CapturePreviewPrimitive(
                SpatialPrimitiveKind::PolylinePath,
                { rawVertices.back(), cursor->hit },
                false,
                ghostStyle,
                4));
        }
        if (closePreview && rawVertices.size() >= 3) {
            primitives.push_back(CapturePreviewPrimitive(
                SpatialPrimitiveKind::PolylinePath,
                { cursor->hit, rawVertices.front() },
                false,
                ghostStyle,
                4));
        }

        SpatialStyle cursorStyle;
        cursorStyle.r = 255;
        cursorStyle.g = 255;
        cursorStyle.b = 255;
        cursorStyle.a = 235;
        cursorStyle.fillA = 60;
        cursorStyle.strokeMeters = 0.0;
        cursorStyle.dotMeters = 0.080;
        cursorStyle.fill = false;
        primitives.push_back(CapturePreviewPrimitive(
            SpatialPrimitiveKind::Marker,
            { cursor->hit },
            false,
            cursorStyle,
            12));
    }

    const double standingFloorY = (!s_captureUsesStandingSpace && BoundaryTransformReady())
        ? TransformHeightToStandingUniverse(
            preview.empty() ? rawVertices : preview,
            s_captureFloorY,
            PreviewTargetToStandingTransform())
        : s_captureFloorY;
    auto pushStandingMarker = [&](double x,
                                  double z,
                                  SpatialStyle style,
                                  int layer) {
        SpatialPrimitive marker;
        marker.kind = SpatialPrimitiveKind::Marker;
        marker.space = StandingSpace();
        marker.vertices = { { x, standingFloorY, z } };
        marker.floorY = standingFloorY;
        marker.ceilingY = standingFloorY + 2.4;
        marker.closeLoop = false;
        marker.style = style;
        marker.layer = layer;
        primitives.push_back(std::move(marker));
    };

    SpatialStyle originStyle;
    originStyle.r = 255;
    originStyle.g = 70;
    originStyle.b = 70;
    originStyle.a = 245;
    originStyle.fillA = 0;
    originStyle.strokeMeters = 0.0;
    originStyle.dotMeters = 0.120;
    originStyle.fill = false;
    pushStandingMarker(0.0, 0.0, originStyle, 14);

    if (auto* vrs = vr::VRSystem()) {
        vr::TrackedDevicePose_t poses[vr::k_unMaxTrackedDeviceCount]{};
        vrs->GetDeviceToAbsoluteTrackingPose(
            vr::TrackingUniverseStanding,
            0.0f,
            poses,
            vr::k_unMaxTrackedDeviceCount);
        const auto& hmdPose = poses[vr::k_unTrackedDeviceIndex_Hmd];
        if (hmdPose.bDeviceIsConnected &&
            hmdPose.bPoseIsValid &&
            hmdPose.eTrackingResult == vr::TrackingResult_Running_OK)
        {
            SpatialStyle hmdStyle;
            hmdStyle.r = 100;
            hmdStyle.g = 180;
            hmdStyle.b = 255;
            hmdStyle.a = 235;
            hmdStyle.fillA = 0;
            hmdStyle.strokeMeters = 0.0;
            hmdStyle.dotMeters = 0.100;
            hmdStyle.fill = false;
            pushStandingMarker(
                hmdPose.mDeviceToAbsoluteTracking.m[0][3],
                hmdPose.mDeviceToAbsoluteTracking.m[2][3],
                hmdStyle,
                15);
        }
    }

    return BuildSpatialRenderCommands(primitives);
}

void TickBoundaryPreviewForCapture(
    const wkopenvr::boundary::FloorHitPreview* cursor = nullptr)
{
    const auto commands = BuildCapturePreviewCommands(s_capture.vertices(), cursor);
    if (commands.empty()) {
        HideBoundaryPreviewOverlay();
        return;
    }
    wkopenvr::boundary::TickBoundaryPreview(
        true,
        commands,
        commands.front().floorY,
        "capture",
        /*showFileMarkers=*/false);

    if (s_steamVrWorkingPreviewVisible) {
        wkopenvr::boundary::HideWorkingChaperonePreview();
        s_steamVrWorkingPreviewVisible = false;
    }
}

bool ApplyFloorFromStandingContactPose(
    const Eigen::Affine3d& standingContactPose,
    vr::TrackedDeviceIndex_t deviceId,
    const std::string& trackingSystem)
{
    s_boundaryNativeStatus.clear();
    const double measuredStandingY = standingContactPose.translation().y();
    if (!std::isfinite(measuredStandingY) || measuredStandingY < -5.0 || measuredStandingY > 5.0) {
        s_boundaryError = "Controller floor sample was outside the expected tracking range.";
        char fbuf[256];
        snprintf(fbuf, sizeof fbuf,
            "[boundary-floor] apply failed: standing_y_out_of_range device=%d system='%s' standing_y=%.4f",
            static_cast<int>(deviceId),
            trackingSystem.c_str(),
            measuredStandingY);
        Metrics::WriteLogAnnotation(fbuf);
        openvr_pair::common::DiagnosticLog("boundary-floor", "%s", fbuf);
        return false;
    }

    // Move the SteamVR standing-zero so the controller resting on the floor maps
    // to floor level. This is the upstream, runtime-consistent way to set floor
    // height (same as OpenVR Advanced Settings) -- runtime, compositor, chaperone
    // bounds and our boundary all stay aligned. measuredStandingY is the
    // controller's height above the current floor, so shifting by it is cumulative
    // and converges to zero over repeated applies.
    const double previousOffset = CalCtx.floorOffsetMetersY;
    if (!AdjustStandingZeroFloorY(measuredStandingY)) {
        s_boundaryError = "Couldn't write floor height to SteamVR's chaperone.";
        char fbuf[256];
        snprintf(fbuf, sizeof fbuf,
            "[boundary-floor] apply failed: chaperone_write device=%d system='%s' standing_y=%.4f",
            static_cast<int>(deviceId), trackingSystem.c_str(), measuredStandingY);
        Metrics::WriteLogAnnotation(fbuf);
        openvr_pair::common::DiagnosticLog("boundary-floor", "%s", fbuf);
        return false;
    }
    // Track the cumulative shift applied to the standing-zero so Reset undoes it.
    CalCtx.floorOffsetMetersY = previousOffset + measuredStandingY;
    CalCtx.floorEnabled = true;
    ApplyBoundaryFloorY(0.0);
    s_boundaryError.clear();
    SaveProfile(CalCtx);

    if (CalCtx.boundary.enabled) {
        // Bounds are stored relative to the standing-zero, so re-push after moving it.
        std::string pushError;
        PushTargetBoundaryToChaperone(pushError);
    }

    char lbuf[256];
    snprintf(lbuf, sizeof lbuf,
        "[boundary-floor] set from controller: device=%d system='%s' measured_standing_y=%.4f floor_offset_prev=%.4f floor_offset_now=%.4f vertices=%zu",
        static_cast<int>(deviceId),
        trackingSystem.c_str(),
        measuredStandingY,
        previousOffset,
        CalCtx.floorOffsetMetersY,
        CalCtx.boundary.vertices.size());
    Metrics::WriteLogAnnotation(lbuf);
    openvr_pair::common::DiagnosticLog("boundary-floor", "%s", lbuf);

    s_cachedFloorSourceInitialized = false;
    return true;
}

bool ApplyCapturedBoundary()
{
    s_boundaryNativeStatus.clear();
    s_capture.Finish();
    const auto& verts = s_capture.vertices();
    const auto pts = wkopenvr::boundary::ProjectXZ(verts);
    const double area = wkopenvr::boundary::AbsoluteAreaXZ(pts);
    if (verts.size() < 3 || area < kMinimumBoundaryAreaMetersSq) {
        s_boundaryError = "Boundary needs at least three floor points and a usable area.";
        return false;
    }
    for (const auto& v : verts) {
        if (!std::isfinite(v.x) || !std::isfinite(v.y) || !std::isfinite(v.z)) {
            s_boundaryError = "Boundary contains an invalid point. Cancel and draw it again.";
            return false;
        }
    }

    const auto bounds = wkopenvr::boundary::ComputePolygonBoundsXZ(verts);
    if (!std::isfinite(bounds.xMin) ||
        !std::isfinite(bounds.xMax) ||
        !std::isfinite(bounds.zMin) ||
        !std::isfinite(bounds.zMax) ||
        (bounds.xMax - bounds.xMin) < 0.35 ||
        (bounds.zMax - bounds.zMin) < 0.35)
    {
        s_boundaryError = "Boundary is too narrow to use safely. Cancel and draw a wider area.";
        return false;
    }

    auto nextBoundary = CalCtx.boundary;
    if (!nextBoundary.priorChaperoneCaptured) {
        auto snapshot = wkopenvr::boundary::SnapshotCurrentChaperone();
        if (!snapshot.empty()) {
            nextBoundary.priorChaperone = std::move(snapshot);
            nextBoundary.priorChaperoneCaptured = true;
        }
    }
    nextBoundary.vertices = verts;
    nextBoundary.standingSpace = s_captureUsesStandingSpace;
    nextBoundary.floorY = s_captureFloorY;
    nextBoundary.ceilingY = nextBoundary.floorY + kAutoBoundaryWallHeightMeters;
    for (auto& v : nextBoundary.vertices) {
        v.y = nextBoundary.floorY;
    }
    nextBoundary.enabled = true;

    CalCtx.boundary = std::move(nextBoundary);
    const Eigen::AffineCompact3d applyTransform = s_capturePreviewTransformValid
        ? s_capturePreviewTransform
        : BoundaryTargetToStandingTransform();
    {
        const Eigen::Vector3d t = applyTransform.translation();
        char lbuf[384];
        snprintf(lbuf, sizeof lbuf,
            "[boundary] apply captured: raw=%zu cleaned=%zu area=%.3f floor_y=%.3f space=%s transform_trans=(%.3f,%.3f,%.3f)",
            s_capture.rawVertexCount(),
            verts.size(),
            area,
            CalCtx.boundary.floorY,
            CalCtx.boundary.standingSpace ? "standing" : "target",
            t.x(), t.y(), t.z());
        Metrics::WriteLogAnnotation(lbuf);
        openvr_pair::common::DiagnosticLog("boundary", "%s", lbuf);
    }
    SaveProfile(CalCtx);
    openvr_pair::common::DiagnosticLog(
        "boundary_apply_local_saved",
        "source=capture_apply raw=%zu cleaned=%zu area=%.3f floor_y=%.3f space=%s enabled=%d",
        s_capture.rawVertexCount(),
        verts.size(),
        area,
        CalCtx.boundary.floorY,
        CalCtx.boundary.standingSpace ? "standing" : "target",
        CalCtx.boundary.enabled ? 1 : 0);

    const auto nativeResult = PushTargetBoundaryToChaperoneWithTransform(
        applyTransform,
        "capture_apply");
    s_boundaryNativeStatus.clear();
    if (!nativeResult.applied) {
        s_boundaryNativeStatus = nativeResult.visualOnly
            ? "Saved locally; SteamVR native boundary skipped because the standing origin is outside the drawn area."
            : "Saved locally; SteamVR native boundary was not updated.";
        if (!nativeResult.message.empty()) {
            s_boundaryNativeStatus += " ";
            s_boundaryNativeStatus += nativeResult.message;
        }
    } else {
        s_boundaryNativeStatus = nativeResult.message;
    }

    s_capture.Cancel();
    StopBoundaryCapturePreview();
    s_boundaryError.clear();
    return true;
}

void DrawPolygonPreview(const std::vector<BoundaryVertex>& verts,
                        bool closeLoop = true,
                        bool activePath = false) {
    if (verts.empty()) return;

    const bool authoringStanding = activePath
        ? s_captureUsesStandingSpace
        : CalCtx.boundary.standingSpace;
    const double authoringFloorY = activePath
        ? s_captureFloorY
        : CalCtx.boundary.floorY;
    const Eigen::AffineCompact3d targetToStanding = activePath
        ? (s_capturePreviewTransformValid ? s_capturePreviewTransform : BoundaryTargetToStandingTransform())
        : BoundaryTargetToStandingTransform();

    auto standingMarkerToAuthoring = [&](double standingX, double standingZ, BoundaryVertex& out) {
        if (authoringStanding) {
            out = { standingX, authoringFloorY, standingZ };
            return true;
        }
        if (!BoundaryTransformReady()) return false;
        const Eigen::Vector3d target =
            targetToStanding.inverse() * Eigen::Vector3d(standingX, 0.0, standingZ);
        if (!target.allFinite()) return false;
        out = { target.x(), authoringFloorY, target.z() };
        return true;
    };

    BoundaryVertex originMarker{};
    const bool originMarkerValid = standingMarkerToAuthoring(0.0, 0.0, originMarker);
    BoundaryVertex hmdMarker{};
    bool hmdMarkerValid = false;
    if (auto* vrs = vr::VRSystem()) {
        vr::TrackedDevicePose_t poses[vr::k_unMaxTrackedDeviceCount]{};
        vrs->GetDeviceToAbsoluteTrackingPose(
            vr::TrackingUniverseStanding,
            0.0f,
            poses,
            vr::k_unMaxTrackedDeviceCount);
        const auto& hmdPose = poses[vr::k_unTrackedDeviceIndex_Hmd];
        if (hmdPose.bDeviceIsConnected &&
            hmdPose.bPoseIsValid &&
            hmdPose.eTrackingResult == vr::TrackingResult_Running_OK)
        {
            hmdMarkerValid = standingMarkerToAuthoring(
                hmdPose.mDeviceToAbsoluteTracking.m[0][3],
                hmdPose.mDeviceToAbsoluteTracking.m[2][3],
                hmdMarker);
        }
    }
    const bool cursorMarkerValid = activePath && s_smoothedPreviewCursorValid;

    double xMin = verts[0].x, xMax = verts[0].x;
    double zMin = verts[0].z, zMax = verts[0].z;
    auto includePoint = [&](const BoundaryVertex& v) {
        if (v.x < xMin) xMin = v.x;
        if (v.x > xMax) xMax = v.x;
        if (v.z < zMin) zMin = v.z;
        if (v.z > zMax) zMax = v.z;
    };
    for (const auto& v : verts) includePoint(v);
    if (originMarkerValid) includePoint(originMarker);
    if (hmdMarkerValid) includePoint(hmdMarker);
    if (cursorMarkerValid) includePoint(s_smoothedPreviewCursor);
    const double rangeX = xMax - xMin;
    const double rangeZ = zMax - zMin;
    const double range = (rangeX > rangeZ ? rangeX : rangeZ);
    const double pad = range * 0.1 + 0.5;
    const double drawSpan = std::max(1.0, range + pad * 2.0);

    const float canvasW = ImGui::GetContentRegionAvail().x;
    const float canvasH = canvasW * 0.6f;
    const ImVec2 p0 = ImGui::GetCursorScreenPos();
    const ImVec2 p1(p0.x + canvasW, p0.y + canvasH);
    ImDrawList* dl = ImGui::GetWindowDrawList();
    dl->AddRectFilled(p0, p1, IM_COL32(30, 30, 30, 255));
    dl->AddRect(p0, p1, IM_COL32(80, 80, 80, 255));
    ImGui::Dummy(ImVec2(canvasW, canvasH));

    auto toCanvas = [&](double wx, double wz) -> ImVec2 {
        float cx = p0.x + (float)(((wx - xMin + pad) / drawSpan) * canvasW);
        float cy = p1.y - (float)(((wz - zMin + pad) / drawSpan) * canvasW * 0.6f);
        return ImVec2(cx, cy);
    };

    const ImU32 lineColor = activePath ? IM_COL32(0, 245, 160, 255) : IM_COL32(0, 200, 100, 255);
    const size_t segmentCount = closeLoop ? verts.size() : (verts.size() > 1 ? verts.size() - 1 : 0);
    if (closeLoop && verts.size() >= 3 && !activePath) {
        std::vector<ImVec2> points;
        points.reserve(verts.size());
        for (const auto& v : verts) {
            points.push_back(toCanvas(v.x, v.z));
        }
        dl->AddConcavePolyFilled(points.data(), static_cast<int>(points.size()), IM_COL32(0, 180, 255, 34));
    }
    for (size_t i = 0; i < segmentCount; ++i) {
        const auto& a = verts[i];
        const auto& b = verts[(i + 1) % verts.size()];
        dl->AddLine(toCanvas(a.x, a.z), toCanvas(b.x, b.z), lineColor, activePath ? 2.5f : 1.5f);
    }
    if (activePath && verts.size() >= 3) {
        dl->AddCircle(toCanvas(verts.front().x, verts.front().z), 8.0f, IM_COL32(0, 180, 255, 220), 24, 2.0f);
    }
    for (size_t i = 0; i < verts.size(); ++i) {
        const auto& v = verts[i];
        const bool last = i + 1 == verts.size();
        const ImU32 dotColor = activePath && last ? IM_COL32(255, 245, 90, 255) : lineColor;
        dl->AddCircleFilled(toCanvas(v.x, v.z), activePath && last ? 5.0f : 3.0f, dotColor);
    }
    if (originMarkerValid) {
        const ImVec2 p = toCanvas(originMarker.x, originMarker.z);
        dl->AddLine(ImVec2(p.x - 7.0f, p.y), ImVec2(p.x + 7.0f, p.y), IM_COL32(255, 90, 90, 255), 2.0f);
        dl->AddLine(ImVec2(p.x, p.y - 7.0f), ImVec2(p.x, p.y + 7.0f), IM_COL32(255, 90, 90, 255), 2.0f);
        dl->AddText(ImVec2(p.x + 8.0f, p.y - 7.0f), IM_COL32(255, 150, 150, 230), "origin");
    }
    if (hmdMarkerValid) {
        const ImVec2 p = toCanvas(hmdMarker.x, hmdMarker.z);
        dl->AddCircleFilled(p, 4.5f, IM_COL32(100, 180, 255, 255), 18);
        dl->AddText(ImVec2(p.x + 7.0f, p.y - 7.0f), IM_COL32(150, 205, 255, 230), "HMD");
    }
    if (cursorMarkerValid) {
        const ImVec2 p = toCanvas(s_smoothedPreviewCursor.x, s_smoothedPreviewCursor.z);
        dl->AddCircle(p, 7.0f, IM_COL32(255, 255, 255, 245), 20, 2.0f);
    }

    {
        const float barPxLen = (float)(1.0 / drawSpan * canvasW);
        const ImVec2 barA(p0.x + 8.0f, p1.y - 12.0f);
        const ImVec2 barB(p0.x + 8.0f + barPxLen, p1.y - 12.0f);
        dl->AddLine(barA, barB, IM_COL32(200, 200, 200, 200), 2.0f);
        dl->AddText(ImVec2(barA.x, barA.y - 14.0f), IM_COL32(200, 200, 200, 200), "1 m");
    }
}

wkopenvr::boundary::ChaperoneOutput NativePreflightForAuthoringVertices(
    const std::vector<BoundaryVertex>& vertices,
    double authoringFloorY,
    bool standingSpace,
    const Eigen::AffineCompact3d& targetToStanding)
{
    if (vertices.size() < 3) {
        return {};
    }
    if (!standingSpace && !BoundaryTransformReady()) {
        return {};
    }

    const auto standingVertices = standingSpace
        ? vertices
        : BoundaryVerticesToStanding(vertices, targetToStanding);
    const double standingFloor = standingSpace
        ? authoringFloorY
        : wkopenvr::boundary::TransformHeightToStandingUniverse(
            vertices,
            authoringFloorY,
            targetToStanding);
    const double standingCeiling = standingSpace
        ? authoringFloorY + kAutoBoundaryWallHeightMeters
        : wkopenvr::boundary::TransformHeightToStandingUniverse(
            vertices,
            authoringFloorY + kAutoBoundaryWallHeightMeters,
            targetToStanding);
    return wkopenvr::boundary::BuildChaperoneOutput(
        standingVertices,
        standingFloor,
        standingCeiling);
}

void DrawNativePreflightStatus(
    const std::vector<BoundaryVertex>& vertices,
    double authoringFloorY,
    bool standingSpace,
    const Eigen::AffineCompact3d& targetToStanding)
{
    if (vertices.size() < 3) return;
    const auto output = NativePreflightForAuthoringVertices(
        vertices,
        authoringFloorY,
        standingSpace,
        targetToStanding);
    const auto& pal = openvr_pair::overlay::ui::GetPalette();
    if (output.ready()) {
        ImGui::TextColored(
            pal.statusOk,
            "SteamVR native eligible: origin inside, play area %.2f x %.2f m",
            output.diagnostics.playAreaX,
            output.diagnostics.playAreaZ);
    } else if (output.status == wkopenvr::boundary::ChaperoneOutputStatus::VisualOnlyNoStandingOrigin) {
        ImGui::TextColored(
            pal.statusWarn,
            "Local boundary OK; SteamVR native origin is %.2f m outside the drawn area.",
            output.diagnostics.originDistanceMeters);
    } else {
        ImGui::TextColored(
            pal.statusWarn,
            "SteamVR native pending: %s",
            output.reason ? output.reason : "not ready");
    }
}

void DrawBoundaryPreviewStatus()
{
    const auto status = wkopenvr::boundary::GetBoundaryPreviewStatus();
    const auto& pal = openvr_pair::overlay::ui::GetPalette();
    const bool previewUnavailable =
        status.uploadsDisabled ||
        (status.created && status.uploadFailureCount > 0 && !status.visible);
    const ImVec4 color = previewUnavailable
        ? pal.statusWarn
        : (status.visible ? pal.statusOk : pal.statusIdle);
    if (previewUnavailable) {
        ImGui::TextColored(
            color,
            "In-VR preview unavailable - draw still works; the floor line just won't show in the headset.");
    } else {
        ImGui::TextColored(
            color,
            "In-VR preview: %s",
            status.visible ? "showing in headset" : "idle");
    }
    if (status.created || status.uploadFailureCount > 0 || status.lastRasterHash != 0) {
        ImGui::TextDisabled(
            "Preview detail: source=%s mode=%s vertices=%zu failures=%d last_error=%d/%s hash=%llu",
            status.lastSource,
            status.uploadMode,
            status.lastVertexCount,
            status.uploadFailureCount,
            status.lastError,
            status.lastErrorName,
            static_cast<unsigned long long>(status.lastRasterHash));
    }
    if (status.fileMarkersVisible || status.fileMarkerFailureCount > 0 ||
        status.fileMarkerTextureReady)
    {
        ImGui::TextDisabled(
            "Preview markers: %s count=%zu texture=%s failures=%d last_error=%d/%s",
            status.fileMarkersVisible ? "visible" : "hidden",
            status.fileMarkerCount,
            status.fileMarkerTextureReady ? "ready" : "missing",
            status.fileMarkerFailureCount,
            status.fileMarkerLastError,
            status.fileMarkerLastErrorName);
    }
}

void DrawBoundarySection(ImVec2 panelSize) {
    {
    openvr_pair::overlay::ui::PanelScope panel("Step 2: Safety boundary", panelSize);
    if (s_floorCapture.active()
        || s_capture.state() == wkopenvr::boundary::CaptureState::Active)
    {
        ::CCal_TickBoundaryCapture();
    }
    const auto& pal = openvr_pair::overlay::ui::GetPalette();
    const auto state = s_capture.state();
    const bool transformReady = BoundaryTransformReady();
    TickBoundaryControllerIdentify();
    const auto controllerChoices = EnumerateBoundaryControllers(vr::VRSystem());
    ResolveBoundaryControllerSelection(controllerChoices);
    const BoundaryControllerChoice* selectedController =
        SelectedBoundaryController(controllerChoices);
    const bool controllerReady = selectedController && selectedController->poseValid;
    const auto& floorSourceDecision = CachedFloorSourceDecision();

    ImGui::TextWrapped(
        "Set the floor from the selected controller, then walk it around the play-space edge.");
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip(
            "Floor and boundary work in SteamVR standing space, independent of space calibration.\n"
            "If a controller is virtual or unstable, choose a tracked controller.");
    }
    if (!CalCtx.boundary.standingSpace && !transformReady) {
        ImGui::TextColored(pal.statusWarn,
            "Waiting for a valid calibration profile before preview or SteamVR chaperone push.");
    }
    DrawBoundaryControllerSelector(controllerChoices);
    if (!controllerReady) {
        ImGui::TextColored(pal.statusWarn,
            selectedController
                ? "Selected controller is detected but does not have a valid pose yet."
                : "Select a controller before setting the floor or drawing.");
    }
    ImGui::Spacing();

    if (state == wkopenvr::boundary::CaptureState::Idle) {
        const bool hasVerts = !CalCtx.boundary.vertices.empty();
        if (s_floorCapture.active()) {
            const auto& floor = s_floorCapture.candidate();
            ImGui::TextColored(pal.statusWarn,
                floor.valid
                    ? "Floor preview is live. Hold still on the floor until it is stable, then apply."
                    : "Move the selected controller down to the floor.");
            if (floor.valid) {
                ImGui::TextDisabled("Measured floor %.2f m from device %d (%s)",
                    floor.floorY,
                    floor.deviceId,
                    floor.trackingSystem.c_str());
                ImGui::SameLine();
                ImGui::TextDisabled("wall height auto");
                ImGui::TextDisabled("%zu samples, %.0f mm floor jitter, %s",
                    floor.sampleCount,
                    floor.jitterMeters * 1000.0,
                    floor.stable ? "stable" : (floor.ready ? "ready" : "settling"));
                ImGui::TextDisabled("Recent drift %.0f mm. Apply lowers the rig so the controller rests on the headset floor.",
                    floor.recentDriftMeters * 1000.0);
            }
            if (!floor.valid || !floor.stable) ImGui::BeginDisabled();
            if (ImGui::Button("Apply floor")) {
                Eigen::Affine3d floorPose = floor.pose;
                floorPose.translation().y() = floor.floorY;
                const bool applied = ApplyFloorFromStandingContactPose(
                    floorPose,
                    static_cast<vr::TrackedDeviceIndex_t>(floor.deviceId),
                    floor.trackingSystem);
                if (applied) {
                    s_floorCapture.Reset();
                    s_boundaryError.clear();
                    HideBoundaryPreviewOverlay();
                }
            }
            if (!floor.valid || !floor.stable) ImGui::EndDisabled();
            ImGui::SameLine();
            if (ImGui::SmallButton("Cancel floor set")) {
                RestoreFloorCaptureOriginal();
                RepushActiveBoundaryAfterEdit();
                s_floorCapture.Reset();
                s_boundaryError.clear();
                s_boundaryNativeStatus.clear();
                HideBoundaryPreviewOverlay();
            }
        } else {
            if (!controllerReady) ImGui::BeginDisabled();
            if (ImGui::Button("Set floor from controller")) {
                s_floorCapture.Begin(CalCtx.boundary.floorY, CalCtx.boundary.ceilingY);
                ++s_floorSessionId;
                s_boundaryError.clear();
                s_boundaryNativeStatus.clear();
                HideBoundaryPreviewOverlay();
                Metrics::WriteLogAnnotation("[boundary-floor] preview started");
            }
            if (!controllerReady) ImGui::EndDisabled();
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Rest the controller on the floor and apply. Moves SteamVR's floor (standing-zero) so the controller sits at floor level.");
            }
            ImGui::SameLine();
            const bool hasFloorOffset = std::fabs(CalCtx.floorOffsetMetersY) > 1e-6;
            if (hasFloorOffset) {
                bool floorOn = CalCtx.floorEnabled;
                if (ImGui::Checkbox("Floor height applied", &floorOn)) {
                    AdjustStandingZeroFloorY(floorOn
                        ? CalCtx.floorOffsetMetersY
                        : -CalCtx.floorOffsetMetersY);
                    CalCtx.floorEnabled = floorOn;
                    SaveProfile(CalCtx);
                }
                if (ImGui::IsItemHovered()) {
                    ImGui::SetTooltip(
                        "Lower the rig to the headset floor, or turn it off to lift "
                        "back without losing the saved height. Reset clears it.");
                }
            }
            if (!hasFloorOffset) ImGui::BeginDisabled();
            if (ImGui::Button("Reset floor")) {
                AdjustStandingZeroFloorY(-CalCtx.floorOffsetMetersY);
                CalCtx.floorOffsetMetersY = 0.0;
                CalCtx.floorEnabled = false;
                ApplyBoundaryFloorY(0.0);
                SaveProfile(CalCtx);
                s_cachedFloorSourceInitialized = false;
                s_boundaryError.clear();
                s_boundaryNativeStatus.clear();
                Metrics::WriteLogAnnotation("[boundary-floor] floor reset (standing-zero restored)");
            }
            if (!hasFloorOffset) ImGui::EndDisabled();
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Undoes the floor adjustment, restoring SteamVR's previous floor height.");
            }
        }

        ImGui::Spacing();

        ImGui::TextDisabled(
            "Floor adjustment %.0f mm. Boundary wall height is automatic.",
            CalCtx.floorOffsetMetersY * 1000.0);
        if (floorSourceDecision.valid) {
            ImGui::TextDisabled("Detected floor: %s, boundary %.2f m, SteamVR %.2f m",
                wkopenvr::boundary::BoundaryFloorSourceKindName(floorSourceDecision.source),
                floorSourceDecision.boundaryFloorY,
                floorSourceDecision.standingFloorY);
        } else {
            ImGui::TextDisabled("Detected floor: unavailable");
        }
        if (ImGui::IsItemHovered()) {
            const std::string rejected = FloorSourceRejectedSummary(floorSourceDecision);
            if (!rejected.empty()) {
                ImGui::SetTooltip("Skipped: %s", rejected.c_str());
            }
        }
        if (transformReady) {
            ImGui::TextDisabled("Preview maps floor to SteamVR Y %.2f m",
                BoundaryHeightToStanding(CalCtx.boundary.vertices, CalCtx.boundary.floorY));
        }
        DrawBoundaryPreviewStatus();

        ImGui::Spacing();

        ImGui::Checkbox("Draw only while trigger is held", &s_captureRequireTrigger);
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("On matches Quest-style drawing. Turn it off if the dashboard is not reporting controller trigger input.");
        }

        ImGui::Spacing();

        if (!hasVerts) {
            if (s_floorCapture.active() || !controllerReady) ImGui::BeginDisabled();
            if (ImGui::Button("Draw boundary")) {
                StartBoundaryCapture(selectedController);
                s_boundaryError.clear();
            }
            if (s_floorCapture.active() || !controllerReady) ImGui::EndDisabled();
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("After this starts, move the lighthouse controller around the floor edge. Apply or cancel when done.");
            }
        } else {
            if (s_floorCapture.active() || !controllerReady) ImGui::BeginDisabled();
            if (ImGui::Button("Draw new boundary")) {
                StartBoundaryCapture(selectedController);
                s_boundaryError.clear();
            }
            if (s_floorCapture.active() || !controllerReady) ImGui::EndDisabled();
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Replaces the current boundary with a new draw-and-apply pass.");
            }

            ImGui::Spacing();

            // Summary.
            const auto pts = wkopenvr::boundary::ProjectXZ(CalCtx.boundary.vertices);
            const double area = wkopenvr::boundary::AbsoluteAreaXZ(pts);
            ImGui::TextDisabled("%d pts, %.2f m^2",
                (int)CalCtx.boundary.vertices.size(), area);
            ImGui::SameLine();
            ImGui::TextColored(
                CalCtx.boundary.enabled ? ImVec4(0.0f, 0.85f, 0.45f, 1.0f) : pal.statusWarn,
                CalCtx.boundary.enabled ? "applied" : "not applied");

            bool boundaryOn = CalCtx.boundary.enabled;
            if (ImGui::Checkbox("Boundary enabled", &boundaryOn)) {
                CalCtx.boundary.enabled = boundaryOn;
                if (boundaryOn) {
                    ScheduleBoundaryStartupPush();
                    std::string pushError;
                    PushTargetBoundaryToChaperone(pushError);
                } else if (CalCtx.boundary.priorChaperoneCaptured) {
                    wkopenvr::boundary::RestoreChaperoneFromSnapshot(
                        CalCtx.boundary.priorChaperone);
                }
                SaveProfile(CalCtx);
            }
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip(
                    "Turn the drawn safety boundary off or back on without "
                    "redrawing it. Off restores your previous room boundary.");
            }

            DrawPolygonPreview(CalCtx.boundary.vertices);
            DrawNativePreflightStatus(
                CalCtx.boundary.vertices,
                CalCtx.boundary.floorY,
                CalCtx.boundary.standingSpace,
                BoundaryTargetToStandingTransform());
        }
    } else if (state == wkopenvr::boundary::CaptureState::Active) {
        const bool closePreview = BoundaryLoopNearlyClosed(s_capture.vertices());
        const bool enoughPoints = s_capture.rawVertexCount() >= 3;
        ImGui::TextColored(pal.statusWarn,
            s_captureRequireTrigger
                ? "Hold trigger and move the selected controller around the floor edge."
                : "Drawing live. Move the selected controller around the floor edge.");
        if (s_captureRequireTrigger) {
            ImGui::TextDisabled("Trigger %s%s",
                s_captureLastTriggerHeld ? "held" : "released",
                s_captureLastTriggerReadable ? "" : " (input unavailable)");
        }
        if (closePreview && enoughPoints) {
            ImGui::TextDisabled("Loop is close to the start point. Apply when the preview looks right.");
        } else if (enoughPoints) {
            ImGui::TextDisabled("Apply will close the drawn path. Raw vertices: %d",
                (int)s_capture.rawVertexCount());
        } else {
            ImGui::TextDisabled("Need at least three floor points. Raw vertices: %d",
                (int)s_capture.rawVertexCount());
        }
        DrawPolygonPreview(s_capture.vertices(), closePreview, true);
        wkopenvr::boundary::FloorHitPreview statusCursor;
        const wkopenvr::boundary::FloorHitPreview* statusCursorPtr = nullptr;
        if (s_smoothedPreviewCursorValid) {
            statusCursor.valid = true;
            statusCursor.hit = s_smoothedPreviewCursor;
            statusCursor.rayName = "controllerXZ";
            statusCursorPtr = &statusCursor;
        }
        DrawNativePreflightStatus(
            BoundaryPreviewPathWithCursor(
                s_capture.vertices(),
                statusCursorPtr),
            s_captureFloorY,
            s_captureUsesStandingSpace,
            s_capturePreviewTransformValid ? s_capturePreviewTransform : BoundaryTargetToStandingTransform());
        DrawBoundaryPreviewStatus();
        ImGui::Spacing();
        const bool canApply = enoughPoints;
        if (!canApply) ImGui::BeginDisabled();
        if (ImGui::Button("Apply boundary##bnd_apply_live")) {
            ApplyCapturedBoundary();
        }
        if (!canApply) ImGui::EndDisabled();
        ImGui::SameLine();
        if (ImGui::Button("Cancel##bnd_cancel")) {
            s_capture.Cancel();
            StopBoundaryCapturePreview();
        }
    } else { // Finished
        const auto& verts = s_capture.vertices();
        const auto pts = wkopenvr::boundary::ProjectXZ(verts);
        const double area = wkopenvr::boundary::AbsoluteAreaXZ(pts);

        ImGui::Text("%d vertices, %.2f m^2", (int)verts.size(), area);
        ImGui::TextDisabled("Cleaned preview. Apply or cancel.");

        DrawPolygonPreview(verts);
        TickBoundaryPreviewForCapture();

        ImGui::Spacing();
        if (ImGui::Button("Apply boundary##bnd_apply")) {
            ApplyCapturedBoundary();
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel##bnd_discard_fin")) {
            s_capture.Cancel();
            StopBoundaryCapturePreview();
        }
    }

    if (!s_boundaryError.empty()) {
        ImGui::Spacing();
        openvr_pair::overlay::ui::DrawErrorBanner("Boundary error", s_boundaryError.c_str());
    }
    if (!s_boundaryNativeStatus.empty()) {
        ImGui::Spacing();
        ImGui::TextColored(pal.statusWarn, "%s", s_boundaryNativeStatus.c_str());
    }

    }
}

void DrawQuestAppPointerSection(ImVec2 panelSize) {
    openvr_pair::overlay::ui::PanelScope panel("Step 3: Quest App", panelSize);
    openvr_pair::overlay::ui::DrawTextWrapped(
        "Quest Boundary and headset app setup now live in the Quest App module.");
    ImGui::Spacing();
    ImGui::TextDisabled(
        "Enable Quest App from the Modules tab for Developer Mode setup, "
        "the Physical Space Features guide, and companion app install.");
}

struct BoundaryInputStats {
    int trackedControllers = 0;
    int matchingControllers = 0;
    int skippedTrackingSystem = 0;
    int controllerStateFailed = 0;
    int poseOk = 0;
    int poseInvalid = 0;
    bool chosenRawPoseValid = false;
    int lastDeviceId = -1;
    int chosenDeviceId = -1;
    double chosenY = 0.0;
    std::string lastTrackingSystem;
    std::string chosenTrackingSystem;
};

struct ControllerCandidate {
    vr::TrackedDeviceIndex_t deviceId = vr::k_unTrackedDeviceIndexInvalid;
    std::string trackingSystem;
    Eigen::Affine3d rawPose = Eigen::Affine3d::Identity();
    Eigen::Affine3d standingPose = Eigen::Affine3d::Identity();
    bool rawPoseValid = false;
};

std::string TrackingSystemName(vr::IVRSystem* vrs, vr::TrackedDeviceIndex_t deviceId) {
    char buffer[vr::k_unMaxPropertyStringSize] = {};
    vr::ETrackedPropertyError err = vr::TrackedProp_Success;
    vrs->GetStringTrackedDeviceProperty(
        deviceId,
        vr::Prop_TrackingSystemName_String,
        buffer,
        vr::k_unMaxPropertyStringSize,
        &err);
    if (err != vr::TrackedProp_Success) {
        return {};
    }
    return std::string(buffer);
}

Eigen::Affine3d DriverPoseToWorldAffine(const vr::DriverPose_t& dp) {
    Eigen::Quaterniond wfd(
        dp.qWorldFromDriverRotation.w,
        dp.qWorldFromDriverRotation.x,
        dp.qWorldFromDriverRotation.y,
        dp.qWorldFromDriverRotation.z);
    Eigen::Vector3d wfd_t(
        dp.vecWorldFromDriverTranslation[0],
        dp.vecWorldFromDriverTranslation[1],
        dp.vecWorldFromDriverTranslation[2]);
    Eigen::Quaterniond rot(dp.qRotation.w, dp.qRotation.x, dp.qRotation.y, dp.qRotation.z);
    if (wfd.norm() > 1e-12) wfd.normalize();
    if (rot.norm() > 1e-12) rot.normalize();

    const Eigen::Vector3d pos(dp.vecPosition[0], dp.vecPosition[1], dp.vecPosition[2]);
    Eigen::Affine3d pose = Eigen::Affine3d::Identity();
    pose.translate(wfd_t + wfd * pos);
    pose.rotate(wfd * rot);
    return pose;
}

Eigen::Affine3d HmdMatrix34ToAffine(const vr::HmdMatrix34_t& m)
{
    Eigen::Affine3d affine = Eigen::Affine3d::Identity();
    affine.linear() << m.m[0][0], m.m[0][1], m.m[0][2],
                       m.m[1][0], m.m[1][1], m.m[1][2],
                       m.m[2][0], m.m[2][1], m.m[2][2];
    affine.translation() = Eigen::Vector3d(m.m[0][3], m.m[1][3], m.m[2][3]);
    return affine;
}

void WriteBoundaryInputSummary(const BoundaryInputStats& stats, uint64_t sessionId, size_t rawCount) {
    char lbuf[760];
    snprintf(lbuf, sizeof lbuf,
        "[boundary-capture] waiting: session=%llu raw=%zu target='%s' capture_space=%s controllers=%d matching=%d skipped_system=%d state_failed=%d pose_ok=%d pose_invalid=%d last_device=%d last_system='%s' chosen_device=%d chosen_system='%s' chosen_y=%.3f chosen_raw_pose=%d",
        static_cast<unsigned long long>(sessionId),
        rawCount,
        CalCtx.targetTrackingSystem.c_str(),
        s_captureUsesStandingSpace ? "standing" : "target",
        stats.trackedControllers,
        stats.matchingControllers,
        stats.skippedTrackingSystem,
        stats.controllerStateFailed,
        stats.poseOk,
        stats.poseInvalid,
        stats.lastDeviceId,
        stats.lastTrackingSystem.c_str(),
        stats.chosenDeviceId,
        stats.chosenTrackingSystem.c_str(),
        stats.chosenY,
        stats.chosenRawPoseValid ? 1 : 0);
    Metrics::WriteLogAnnotation(lbuf);
    openvr_pair::common::DiagnosticLog("boundary-capture", "%s", lbuf);
}

// Once-on-change [boundary] status line so a live session can tell whether the
// applied filled overlay actually rendered, instead of guessing from a faint fill.
void LogPersistentBoundaryStatusOnChange(size_t vertexCount) {
    const auto status = wkopenvr::boundary::GetBoundaryPreviewStatus();
    static bool s_logged = false;
    static int s_sig = 0;
    const int sig =
        (status.visible ? 1 : 0)
        | (status.textureReady ? 2 : 0)
        | (status.uploadsDisabled ? 4 : 0)
        | (status.plane.valid ? 8 : 0)
        | (static_cast<int>(vertexCount & 0xFFFFu) << 8);
    if (s_logged && sig == s_sig) return;
    s_logged = true;
    s_sig = sig;

    char lbuf[256];
    snprintf(lbuf, sizeof lbuf,
        "[boundary] persistent overlay: vertices=%zu plane_valid=%d visible=%d texture_ready=%d uploads_disabled=%d file_markers=%zu",
        vertexCount,
        status.plane.valid ? 1 : 0,
        status.visible ? 1 : 0,
        status.textureReady ? 1 : 0,
        status.uploadsDisabled ? 1 : 0,
        status.fileMarkerCount);
    Metrics::WriteLogAnnotation(lbuf);
    openvr_pair::common::DiagnosticLog("boundary", "%s", lbuf);
}

// Always-on applied boundary. The headset runtime owns the chaperone and ignores
// our perimeter push, so when a boundary is enabled and we're not actively
// capturing we draw it ourselves as a single translucent filled floor with a thin
// outline (no per-vertex icons) as the safety reference.
void TickPersistentBoundaryOverlay() {
    if (!CalCtx.boundary.enabled || CalCtx.boundary.vertices.size() < 3) {
        HideBoundaryPreviewOverlay();
        return;
    }

    std::vector<BoundaryVertex> standingVerts;
    double standingFloor = CalCtx.boundary.floorY;
    if (CalCtx.boundary.standingSpace) {
        standingVerts = CalCtx.boundary.vertices;
    } else if (BoundaryTransformReady()) {
        const auto xf = BoundaryTargetToStandingTransform();
        standingVerts = BoundaryVerticesToStanding(CalCtx.boundary.vertices, xf);
        standingFloor = BoundaryHeightToStanding(CalCtx.boundary.vertices, CalCtx.boundary.floorY);
    } else {
        HideBoundaryPreviewOverlay();
        return;
    }

    // Skip the per-tick raster rebuild when the applied boundary hasn't moved.
    // SteamVR keeps showing the last uploaded texture until we hide it, so
    // re-rasterizing + re-uploading 512x512 every tick is pure waste once the
    // overlay is on screen. Rebuild only when the geometry/floor changes, or
    // when the overlay is not actually shown yet (first show, or retrying after
    // an upload failure -- status.visible is false until a clean upload).
    {
        static uint64_t s_shownSig = ~0ull;
        uint64_t sig = 1469598103934665603ull;
        auto mix = [&sig](double d) {
            sig ^= static_cast<uint64_t>(std::llround(d * 1000.0));
            sig *= 1099511628211ull;
        };
        for (const auto& v : standingVerts) { mix(v.x); mix(v.y); mix(v.z); }
        mix(standingFloor);
        const auto st = wkopenvr::boundary::GetBoundaryPreviewStatus();
        if (sig == s_shownSig && st.visible && st.textureReady) {
            return;
        }
        s_shownSig = sig;
    }

    const auto commands = wkopenvr::boundary::BuildPersistentBoundaryCommands(
        standingVerts, standingFloor);
    wkopenvr::boundary::TickBoundaryPreview(
        true, commands, standingFloor, "persistent", /*showFileMarkers=*/false);
    LogPersistentBoundaryStatusOnChange(standingVerts.size());
}

} // namespace

void CCal_InvalidateBoundaryFloorSourceCache()
{
    s_cachedFloorSourceInitialized = false;
}

// ---------------------------------------------------------------------------
// Capture tick -- called each CalibrationTick so capture runs regardless of
// which tab is visible.
// ---------------------------------------------------------------------------

void CCal_TickBoundaryCapture() {
    const bool captureActive = s_capture.state() == wkopenvr::boundary::CaptureState::Active;
    if (!captureActive && !s_floorCapture.active()) {
        TickPersistentBoundaryOverlay();
        return;
    }

    auto tickPreview = []() {
        TickBoundaryPreviewForCapture();
    };

    auto* vrs = vr::VRSystem();
    const uint64_t sessionId = s_floorCapture.active() ? s_floorSessionId : s_capture.sessionId();
    static uint64_t s_lastSessionId = 0;
    static uint32_t s_waitTicks = 0;
    static uint64_t s_noVrLoggedSession = 0;
    if (sessionId != s_lastSessionId) {
        s_lastSessionId = sessionId;
        s_waitTicks = 0;
    }

    if (!vrs) {
        if (s_noVrLoggedSession != sessionId) {
            s_noVrLoggedSession = sessionId;
            Metrics::WriteLogAnnotation("[boundary-capture] waiting: no VRSystem available");
        }
        tickPreview();
        return;
    }

    BoundaryInputStats stats;
    std::vector<ControllerCandidate> candidates;
    vr::TrackedDevicePose_t standingPoses[vr::k_unMaxTrackedDeviceCount]{};
    vrs->GetDeviceToAbsoluteTrackingPose(
        vr::TrackingUniverseStanding,
        0.0f,
        standingPoses,
        vr::k_unMaxTrackedDeviceCount);
    for (vr::TrackedDeviceIndex_t deviceId = 0; deviceId < vr::k_unMaxTrackedDeviceCount; ++deviceId) {
        if (vrs->GetTrackedDeviceClass(deviceId) != vr::TrackedDeviceClass_Controller) {
            continue;
        }
        ++stats.trackedControllers;
        stats.lastDeviceId = static_cast<int>(deviceId);
        stats.lastTrackingSystem = TrackingSystemName(vrs, deviceId);

        if (TrackingSystemMatchesBoundaryTarget(stats.lastTrackingSystem)) {
            ++stats.matchingControllers;
        } else {
            ++stats.skippedTrackingSystem;
        }

        const auto& standingTrackedPose = standingPoses[deviceId];
        if (!standingTrackedPose.bDeviceIsConnected ||
            !standingTrackedPose.bPoseIsValid ||
            standingTrackedPose.eTrackingResult != vr::ETrackingResult::TrackingResult_Running_OK)
        {
            ++stats.poseInvalid;
            continue;
        }
        ++stats.poseOk;

        ControllerCandidate candidate;
        candidate.deviceId = deviceId;
        candidate.trackingSystem = stats.lastTrackingSystem;
        candidate.standingPose = HmdMatrix34ToAffine(
            standingTrackedPose.mDeviceToAbsoluteTracking);
        const auto& dp = CalCtx.devicePoses[deviceId];
        if (dp.poseIsValid && dp.result == vr::ETrackingResult::TrackingResult_Running_OK) {
            candidate.rawPose = DriverPoseToWorldAffine(dp);
            candidate.rawPoseValid = true;
        } else {
            candidate.rawPose = candidate.standingPose;
        }

        candidates.push_back(candidate);
    }

    if (candidates.empty()) {
        tickPreview();
        ++s_waitTicks;
        if (s_waitTicks == 1 || (s_waitTicks % 120) == 0) {
            WriteBoundaryInputSummary(stats, sessionId, s_capture.rawVertexCount());
        }
        return;
    }

    const auto chosenIt = std::find_if(
        candidates.begin(),
        candidates.end(),
        [](const ControllerCandidate& c) {
            return static_cast<int32_t>(c.deviceId) == s_boundaryControllerDeviceId;
        });
    if (chosenIt == candidates.end()) {
        tickPreview();
        ++s_waitTicks;
        if (s_waitTicks == 1 || (s_waitTicks % 120) == 0) {
            WriteBoundaryInputSummary(stats, sessionId, s_capture.rawVertexCount());
        }
        return;
    }
    const ControllerCandidate& chosen = *chosenIt;
    stats.chosenDeviceId = static_cast<int>(chosen.deviceId);
    stats.chosenTrackingSystem = chosen.trackingSystem;
    stats.chosenY = chosen.standingPose.translation().y();
    stats.chosenRawPoseValid = chosen.rawPoseValid;

    if (s_floorCapture.active()) {
        const std::string controllerType = DeviceStringProperty(
            vrs,
            chosen.deviceId,
            vr::Prop_ControllerType_String);
        const double controllerOriginY = chosen.standingPose.translation().y();
        const double contactOffset =
            wkopenvr::boundary::ControllerFloorContactOffsetMeters(
                controllerType,
                chosen.standingPose);
        Eigen::Affine3d contactPose = chosen.standingPose;
        contactPose.translation().y() =
            wkopenvr::boundary::AdjustControllerFloorYForContact(
                controllerOriginY,
                controllerType,
                chosen.standingPose);
        const bool updated = s_floorCapture.Observe(
            contactPose,
            static_cast<int>(chosen.deviceId),
            chosen.trackingSystem);
        const auto& floor = s_floorCapture.candidate();
        if (floor.valid) {
            if (updated || !s_floorPreviewVisible) {
                const auto marker = wkopenvr::boundary::BuildFloorMarkerVertices(
                    floor.pose,
                    floor.floorY);
                TickBoundaryPreviewForStandingVerticesAtFloor(
                    true,
                    marker,
                    floor.floorY,
                    true);
                s_floorPreviewVisible = true;
            }
            if (updated) {
                char fbuf[320];
                snprintf(fbuf, sizeof fbuf,
                    "[boundary-floor] preview: samples=%zu ready=%d stable=%d jitter_mm=%.1f device=%d system='%s' type='%s' origin_y=%.3f contact_offset=%.4f floor_y=%.3f controller=(%.3f,%.3f,%.3f)",
                    floor.sampleCount,
                    floor.ready ? 1 : 0,
                    floor.stable ? 1 : 0,
                    floor.jitterMeters * 1000.0,
                    floor.deviceId,
                    floor.trackingSystem.c_str(),
                    controllerType.c_str(),
                    controllerOriginY,
                    contactOffset,
                    floor.floorY,
                    floor.pose.translation().x(),
                    floor.pose.translation().y(),
                    floor.pose.translation().z());
                Metrics::WriteLogAnnotation(fbuf);
                openvr_pair::common::DiagnosticLog("boundary-floor", "%s", fbuf);
            }
        } else {
            HideBoundaryPreviewOverlay();
        }
        return;
    }

    if (!s_captureUsesStandingSpace && !chosen.rawPoseValid) {
        tickPreview();
        ++s_waitTicks;
        if (s_waitTicks == 1 || (s_waitTicks % 120) == 0) {
            char wbuf[256];
            snprintf(wbuf, sizeof wbuf,
                "[boundary-capture] waiting: target raw pose unavailable session=%llu device=%d system='%s'",
                static_cast<unsigned long long>(sessionId),
                static_cast<int>(chosen.deviceId),
                chosen.trackingSystem.c_str());
            Metrics::WriteLogAnnotation(wbuf);
            openvr_pair::common::DiagnosticLog("boundary-capture", "%s", wbuf);
        }
        return;
    }

    const Eigen::Affine3d capturePose =
        s_captureUsesStandingSpace ? chosen.standingPose : chosen.rawPose;
    const double captureFloorY = s_captureFloorY;

    s_captureLastTriggerReadable = false;
    s_captureLastTriggerHeld = !s_captureRequireTrigger;
    s_captureLastTriggerReading = {};
    if (s_captureRequireTrigger) {
        vr::VRControllerState_t state{};
        s_captureLastTriggerReadable =
            vrs->GetControllerState(
                chosen.deviceId,
                &state,
                sizeof(state));
        if (s_captureLastTriggerReadable) {
            s_captureLastTriggerHeld =
                wkopenvr::controller_input::IsTriggerHeld(
                    vrs,
                    chosen.deviceId,
                    state,
                    0.25f,
                    &s_captureLastTriggerReading);
        }
    }

    wkopenvr::boundary::FloorHitPreview previewCursor;
    previewCursor.valid = true;
    const BoundaryVertex measuredCursor{
        capturePose.translation().x(),
        captureFloorY,
        capturePose.translation().z()
    };
    if (!s_smoothedPreviewCursorValid ||
        s_smoothedPreviewCursorSession != sessionId)
    {
        s_smoothedPreviewCursor = measuredCursor;
        s_smoothedPreviewCursorSession = sessionId;
        s_smoothedPreviewCursorValid = true;
    } else {
        constexpr double kCursorFollow = 0.45;
        constexpr double kJumpResetMeters = 0.35;
        const double dx = measuredCursor.x - s_smoothedPreviewCursor.x;
        const double dz = measuredCursor.z - s_smoothedPreviewCursor.z;
        if ((dx * dx + dz * dz) > (kJumpResetMeters * kJumpResetMeters)) {
            s_smoothedPreviewCursor = measuredCursor;
        } else {
            s_smoothedPreviewCursor.x += dx * kCursorFollow;
            s_smoothedPreviewCursor.y = captureFloorY;
            s_smoothedPreviewCursor.z += dz * kCursorFollow;
        }
    }
    previewCursor.hit = s_smoothedPreviewCursor;
    previewCursor.rayName = "controllerXZ";

    const bool accepted = s_capture.TickProjectedPosition(
        capturePose,
        s_captureLastTriggerHeld,
        captureFloorY);
    if (accepted) {
        s_waitTicks = 0;
        char cbuf[760];
        snprintf(cbuf, sizeof cbuf,
            "[boundary-capture] accepted controller_xz: session=%llu device=%d system='%s' raw=%zu floor_y=%.3f space=%s trigger=%d raw_pos=(%.3f,%.3f,%.3f) standing_pos=(%.3f,%.3f,%.3f) capture_pos=(%.3f,%.3f,%.3f)",
            static_cast<unsigned long long>(sessionId),
            static_cast<int>(chosen.deviceId),
            chosen.trackingSystem.c_str(),
            s_capture.rawVertexCount(),
            captureFloorY,
            s_captureUsesStandingSpace ? "standing" : "target",
            s_captureLastTriggerHeld ? 1 : 0,
            chosen.rawPose.translation().x(),
            chosen.rawPose.translation().y(),
            chosen.rawPose.translation().z(),
            chosen.standingPose.translation().x(),
            chosen.standingPose.translation().y(),
            chosen.standingPose.translation().z(),
            capturePose.translation().x(),
            capturePose.translation().y(),
            capturePose.translation().z());
        Metrics::WriteLogAnnotation(cbuf);
        openvr_pair::common::DiagnosticLog("boundary-capture", "%s", cbuf);
    } else if (s_captureRequireTrigger) {
        ++s_waitTicks;
        if (s_waitTicks == 1 || (s_waitTicks % 120) == 0) {
            char wbuf[384];
            snprintf(wbuf, sizeof wbuf,
                "[boundary-capture] waiting: trigger session=%llu raw=%zu readable=%d held=%d button=%d axis=%d value=%.3f trigger_axes=%d axis_prop_errors=%d",
                static_cast<unsigned long long>(sessionId),
                s_capture.rawVertexCount(),
                s_captureLastTriggerReadable ? 1 : 0,
                s_captureLastTriggerHeld ? 1 : 0,
                s_captureLastTriggerReading.buttonPressed ? 1 : 0,
                s_captureLastTriggerReading.analogAxis,
                static_cast<double>(s_captureLastTriggerReading.analogValue),
                s_captureLastTriggerReading.triggerAxisCount,
                s_captureLastTriggerReading.propertyErrors);
            Metrics::WriteLogAnnotation(wbuf);
            openvr_pair::common::DiagnosticLog("boundary-capture", "%s", wbuf);
        }
    }
    TickBoundaryPreviewForCapture(&previewCursor);
}

// ---------------------------------------------------------------------------
// Public entry point
// ---------------------------------------------------------------------------

void CCal_DrawBoundaryTab() {
    ImVec2 panelSize{
        ImGui::GetWindowContentRegionMax().x - ImGui::GetWindowContentRegionMin().x,
        0.0f
    };

    // Top-level framing: this tab is one continuous story, not three
    // independent feature islands. Each step depends on the previous one,
    // and the disabled-state copy nudges the user forward.
    ImGui::TextDisabled(
        "Continuous-mode headset tracker, lighthouse boundary, and Quest App handoff.");
    ImGui::Spacing();

    CCal_DrawHeadMountSection(panelSize);
    ImGui::Spacing();
    DrawBoundarySection(panelSize);
    ImGui::Spacing();
    DrawQuestAppPointerSection(panelSize);
}
