#include "Calibration.h"
#include "CalibrationMetrics.h"
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
std::string s_boundaryError;
uint64_t s_floorSessionId = 0;
bool s_steamVrWorkingPreviewVisible = false;
bool s_floorPreviewVisible = false;
Eigen::AffineCompact3d s_capturePreviewTransform = Eigen::AffineCompact3d::Identity();
bool s_capturePreviewTransformValid = false;
bool s_captureUsesStandingSpace = false;
double s_captureFloorY = 0.0;
bool s_captureRequireTrigger = true;
bool s_moveSteamVrFloorOnApply = false;
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

std::string TrackingSystemName(vr::IVRSystem* vrs, vr::TrackedDeviceIndex_t deviceId);
bool ShouldUseTargetSpaceForBoundaryController(const BoundaryControllerChoice* choice);

void HideBoundaryPreviewOverlay() {
    wkopenvr::boundary::TickBoundaryPreview(false, {}, CalCtx.boundary.floorY, false);
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
        closeLoop);

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
        closeLoop);
}

bool PushTargetBoundaryToChaperoneWithTransform(
    std::string& error,
    const Eigen::AffineCompact3d& targetToStanding,
    const char* source)
{
    if (!CalCtx.boundary.standingSpace && !BoundaryTransformReady()) {
        error = "Run calibration first so the boundary can be mapped into SteamVR space.";
        return false;
    }
    if (CalCtx.boundary.vertices.size() < 3) {
        error = "Boundary needs at least three floor points.";
        return false;
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

    const bool ok = wkopenvr::boundary::PushToChaperone(
        standingVertices,
        standingFloor,
        standingCeiling);
    if (!ok) {
        error = "PushToChaperone failed. Is SteamVR running?";
    } else {
        NoteBoundaryPushedForTransform(targetToStanding);
    }
    return ok;
}

bool PushTargetBoundaryToChaperone(std::string& error)
{
    return PushTargetBoundaryToChaperoneWithTransform(
        error,
        BoundaryTargetToStandingTransform(),
        "current");
}

void RepushActiveBoundaryAfterEdit()
{
    if (!CalCtx.boundary.enabled) return;

    std::string pushError;
    if (!PushTargetBoundaryToChaperone(pushError)) {
        CalCtx.boundary.enabled = false;
        s_boundaryError = pushError;
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
    s_capture.Start();

    char lbuf[256];
    snprintf(lbuf, sizeof lbuf,
        "[boundary-capture] mode: space=%s floor_y=%.3f require_trigger=%d controller=%d system='%s'",
        s_captureUsesStandingSpace ? "standing" : "target",
        s_captureFloorY,
        s_captureRequireTrigger ? 1 : 0,
        selectedController ? selectedController->deviceId : -1,
        selectedController ? selectedController->trackingSystem.c_str() : "");
    Metrics::WriteLogAnnotation(lbuf);
    openvr_pair::common::DiagnosticLog("boundary-capture", "%s", lbuf);
}

void StopBoundaryCapturePreview()
{
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
    return choice
        && choice->matchesTarget
        && BoundaryTransformReady();
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

void TickBoundaryPreviewForCapture(
    const wkopenvr::boundary::FloorHitPreview* cursor = nullptr)
{
    const auto preview = BoundaryPreviewPathWithCursor(s_capture.vertices(), cursor);
    if (s_captureUsesStandingSpace) {
        TickBoundaryPreviewForStandingVerticesAtFloor(
            true,
            preview,
            s_captureFloorY,
            BoundaryLoopNearlyClosed(preview));
        return;
    }
    TickBoundaryPreviewForTargetVerticesAtFloor(
        true,
        preview,
        s_captureFloorY,
        BoundaryLoopNearlyClosed(preview));
}

bool ApplyFloorFromControllerPose(
    const Eigen::Affine3d& rawPose,
    vr::TrackedDeviceIndex_t deviceId,
    const std::string& trackingSystem)
{
    const double floorY = rawPose.translation().y();
    if (!std::isfinite(floorY) || floorY < -5.0 || floorY > 5.0) {
        s_boundaryError = "Controller floor sample was outside the expected tracking range.";
        return false;
    }
    const double measuredStandingY = rawPose.translation().y();
    if (!std::isfinite(measuredStandingY) || measuredStandingY < -5.0 || measuredStandingY > 5.0) {
        s_boundaryError = "Mapped controller floor sample was outside the expected SteamVR range.";
        char fbuf[256];
        snprintf(fbuf, sizeof fbuf,
            "[boundary-floor] apply failed: standing_y_out_of_range device=%d system='%s' target_y=%.4f standing_y=%.4f",
            static_cast<int>(deviceId),
            trackingSystem.c_str(),
            floorY,
            measuredStandingY);
        Metrics::WriteLogAnnotation(fbuf);
        openvr_pair::common::DiagnosticLog("boundary-floor", "%s", fbuf);
        return false;
    }

    char floorError[192] = {};
    bool steamFloorApplied = false;
    if (s_moveSteamVrFloorOnApply) {
        steamFloorApplied =
            wkopenvr::boundary::ApplySteamVrFloorOffset(
                measuredStandingY,
                floorError,
                sizeof floorError);
    }
    {
        char lbuf[384];
        snprintf(lbuf, sizeof lbuf,
            "[boundary-floor] apply requested: device=%d system='%s' standing_y=%.4f steamvr_apply=%d boundary_only=%d",
            static_cast<int>(deviceId),
            trackingSystem.c_str(),
            measuredStandingY,
            steamFloorApplied ? 1 : 0,
            s_moveSteamVrFloorOnApply ? 0 : 1);
        Metrics::WriteLogAnnotation(lbuf);
        openvr_pair::common::DiagnosticLog("boundary-floor", "%s", lbuf);
    }
    if (s_moveSteamVrFloorOnApply && !steamFloorApplied) {
        s_boundaryError = floorError[0] ? floorError : "SteamVR floor apply failed.";
        return false;
    }
    if (s_moveSteamVrFloorOnApply) {
        auto* vrs = vr::VRSystem();
        if (vrs) {
            vr::TrackedDevicePose_t standingPoses[vr::k_unMaxTrackedDeviceCount]{};
            vrs->GetDeviceToAbsoluteTrackingPose(
                vr::TrackingUniverseStanding,
                0.0f,
                standingPoses,
                vr::k_unMaxTrackedDeviceCount);
            if (deviceId < vr::k_unMaxTrackedDeviceCount) {
                const auto& pose = standingPoses[deviceId];
                char pbuf[256];
                snprintf(pbuf, sizeof pbuf,
                    "[boundary-floor] post-apply device pose: device=%d connected=%d valid=%d result=%d standing_y=%.4f",
                    static_cast<int>(deviceId),
                    pose.bDeviceIsConnected ? 1 : 0,
                    pose.bPoseIsValid ? 1 : 0,
                    static_cast<int>(pose.eTrackingResult),
                    static_cast<double>(pose.mDeviceToAbsoluteTracking.m[1][3]));
                Metrics::WriteLogAnnotation(pbuf);
                openvr_pair::common::DiagnosticLog("boundary-floor", "%s", pbuf);
            }
        }
    }

    double boundaryFloorY =
        wkopenvr::boundary::BoundaryFloorYAfterApply(
            measuredStandingY,
            s_moveSteamVrFloorOnApply);
    if (!CalCtx.boundary.standingSpace && BoundaryTransformReady()) {
        boundaryFloorY =
            wkopenvr::boundary::TargetFloorYForStandingFloor(
                CalCtx.boundary.vertices,
                BoundaryTargetToStandingTransform(),
                boundaryFloorY);
    }
    ApplyBoundaryFloorY(boundaryFloorY);
    s_boundaryError.clear();

    if (CalCtx.boundary.enabled) {
        std::string pushError;
        if (!PushTargetBoundaryToChaperone(pushError)) {
            CalCtx.boundary.enabled = false;
            s_boundaryError = pushError;
        }
    }

    SaveProfile(CalCtx);

    char lbuf[224];
    snprintf(lbuf, sizeof lbuf,
        "[boundary-floor] set from controller: device=%d system='%s' boundary_floor_y=%.3f measured_standing_y=%.3f move_steamvr_floor=%d vertices=%zu",
        static_cast<int>(deviceId),
        trackingSystem.c_str(),
        CalCtx.boundary.floorY,
        measuredStandingY,
        s_moveSteamVrFloorOnApply ? 1 : 0,
        CalCtx.boundary.vertices.size());
    Metrics::WriteLogAnnotation(lbuf);
    openvr_pair::common::DiagnosticLog("boundary-floor", "%s", lbuf);
    return true;
}

bool ApplyCapturedBoundary()
{
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

    auto previousBoundary = CalCtx.boundary;
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
    if (!PushTargetBoundaryToChaperoneWithTransform(
            s_boundaryError,
            applyTransform,
            "capture_apply"))
    {
        CalCtx.boundary = std::move(previousBoundary);
        return false;
    }

    SaveProfile(CalCtx);
    s_capture.Cancel();
    StopBoundaryCapturePreview();
    s_boundaryError.clear();
    return true;
}

void DrawPolygonPreview(const std::vector<BoundaryVertex>& verts,
                        bool closeLoop = true,
                        bool activePath = false) {
    if (verts.empty()) return;

    double xMin = verts[0].x, xMax = verts[0].x;
    double zMin = verts[0].z, zMax = verts[0].z;
    for (const auto& v : verts) {
        if (v.x < xMin) xMin = v.x;
        if (v.x > xMax) xMax = v.x;
        if (v.z < zMin) zMin = v.z;
        if (v.z > zMax) zMax = v.z;
    }
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

    {
        const float barPxLen = (float)(1.0 / drawSpan * canvasW);
        const ImVec2 barA(p0.x + 8.0f, p1.y - 12.0f);
        const ImVec2 barB(p0.x + 8.0f + barPxLen, p1.y - 12.0f);
        dl->AddLine(barA, barB, IM_COL32(200, 200, 200, 200), 2.0f);
        dl->AddText(ImVec2(barA.x, barA.y - 14.0f), IM_COL32(200, 200, 200, 200), "1 m");
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

    ImGui::TextWrapped(
        "Set the floor from the selected controller, then walk it around the play-space edge.");
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip(
            "Target-system controllers draw in calibrated tracker space. Other controllers draw\n"
            "in SteamVR standing space. If a controller is virtual or unstable, choose a tracked controller.");
    }
    if (!transformReady) {
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
            }
            if (!floor.valid || !floor.stable) ImGui::BeginDisabled();
            if (ImGui::Button("Apply floor")) {
                Eigen::Affine3d floorPose = floor.pose;
                floorPose.translation().y() = floor.floorY;
                const bool applied = ApplyFloorFromControllerPose(
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
                HideBoundaryPreviewOverlay();
            }
        } else {
            if (!controllerReady) ImGui::BeginDisabled();
            if (ImGui::Button("Set floor from controller")) {
                s_floorCapture.Begin(CalCtx.boundary.floorY, CalCtx.boundary.ceilingY);
                ++s_floorSessionId;
                s_boundaryError.clear();
                HideBoundaryPreviewOverlay();
                Metrics::WriteLogAnnotation("[boundary-floor] preview started");
            }
            if (!controllerReady) ImGui::EndDisabled();
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("No numeric floor entry. Touch the controller to the floor, then apply the live preview.");
            }
            ImGui::Checkbox("Move SteamVR floor when applying", &s_moveSteamVrFloorOnApply);
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip(
                    "Off keeps the floor change inside this boundary. On writes SteamVR's standing origin and can move the whole play space.");
            }
        }

        ImGui::Spacing();

        ImGui::TextDisabled(
            "Floor %.2f m. Boundary wall height is automatic.",
            CalCtx.boundary.floorY);
        if (transformReady) {
            ImGui::TextDisabled("Preview maps floor to SteamVR Y %.2f m",
                BoundaryHeightToStanding(CalCtx.boundary.vertices, CalCtx.boundary.floorY));
        }

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
                HideBoundaryPreviewOverlay();
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
                HideBoundaryPreviewOverlay();
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

            DrawPolygonPreview(CalCtx.boundary.vertices);
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
    char lbuf[640];
    snprintf(lbuf, sizeof lbuf,
        "[boundary-capture] waiting: session=%llu raw=%zu target='%s' controllers=%d matching=%d skipped_system=%d state_failed=%d pose_ok=%d pose_invalid=%d last_device=%d last_system='%s' chosen_device=%d chosen_system='%s' chosen_y=%.3f",
        static_cast<unsigned long long>(sessionId),
        rawCount,
        CalCtx.targetTrackingSystem.c_str(),
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
        stats.chosenY);
    Metrics::WriteLogAnnotation(lbuf);
    openvr_pair::common::DiagnosticLog("boundary-capture", "%s", lbuf);
}

} // namespace

// ---------------------------------------------------------------------------
// Capture tick -- called each CalibrationTick so capture runs regardless of
// which tab is visible.
// ---------------------------------------------------------------------------

void CCal_TickBoundaryCapture() {
    const bool captureActive = s_capture.state() == wkopenvr::boundary::CaptureState::Active;
    if (!captureActive && !s_floorCapture.active())
        return;

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

        ++stats.matchingControllers;

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
