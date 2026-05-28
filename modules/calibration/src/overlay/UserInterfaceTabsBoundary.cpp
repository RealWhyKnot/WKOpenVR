#include "Calibration.h"
#include "CalibrationMetrics.h"
#include "Configuration.h"
#include "DiagnosticsLog.h"
#include "Boundary.h"
#include "BoundaryCapture.h"
#include "BoundaryFloorCapture.h"
#include "BoundaryPreview.h"
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

void HideBoundaryPreviewOverlay() {
    wkopenvr::boundary::TickBoundaryPreview(false, {}, CalCtx.boundary.floorY, false);
    if (s_steamVrWorkingPreviewVisible) {
        wkopenvr::boundary::HideWorkingChaperonePreview();
        s_steamVrWorkingPreviewVisible = false;
    }
}

bool BoundaryTransformReady()
{
    return CalCtx.validProfile;
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
    return wkopenvr::boundary::TransformHeightToStandingUniverse(
        targetVertices,
        targetY,
        BoundaryTargetToStandingTransform());
}

std::vector<BoundaryVertex> BoundaryVerticesToStanding(
    const std::vector<BoundaryVertex>& targetVertices)
{
    return wkopenvr::boundary::TransformToStandingUniverse(
        targetVertices,
        BoundaryTargetToStandingTransform());
}

void TickBoundaryPreviewForTargetVertices(
    bool wantVisible,
    const std::vector<BoundaryVertex>& targetVertices,
    bool closeLoop)
{
    if (!wantVisible || targetVertices.empty() || !BoundaryTransformReady()) {
        HideBoundaryPreviewOverlay();
        return;
    }

    wkopenvr::boundary::TickBoundaryPreview(
        true,
        BoundaryVerticesToStanding(targetVertices),
        BoundaryHeightToStanding(targetVertices, CalCtx.boundary.floorY),
        closeLoop);

    if (s_steamVrWorkingPreviewVisible) {
        wkopenvr::boundary::HideWorkingChaperonePreview();
        s_steamVrWorkingPreviewVisible = false;
    }
}

bool PushTargetBoundaryToChaperone(std::string& error)
{
    if (!BoundaryTransformReady()) {
        error = "Run calibration first so the boundary can be mapped into SteamVR space.";
        return false;
    }
    if (CalCtx.boundary.vertices.size() < 3) {
        error = "Boundary needs at least three floor points.";
        return false;
    }

    const bool ok = wkopenvr::boundary::PushToChaperone(
        BoundaryVerticesToStanding(CalCtx.boundary.vertices),
        BoundaryHeightToStanding(CalCtx.boundary.vertices, CalCtx.boundary.floorY),
        BoundaryHeightToStanding(CalCtx.boundary.vertices, CalCtx.boundary.ceilingY));
    if (!ok) {
        error = "PushToChaperone failed. Is SteamVR running?";
    }
    return ok;
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

double BoundaryRoomHeightMeters()
{
    return std::max(0.5, CalCtx.boundary.ceilingY - CalCtx.boundary.floorY);
}

void ApplyBoundaryFloorY(double floorY)
{
    if (!std::isfinite(floorY)) return;

    const double height = BoundaryRoomHeightMeters();
    CalCtx.boundary.floorY = floorY;
    CalCtx.boundary.ceilingY = floorY + height;
    for (auto& v : CalCtx.boundary.vertices) {
        v.y = floorY;
    }
}

void ApplyBoundaryCeilingY(double ceilingY)
{
    if (!std::isfinite(ceilingY)) return;

    const double minCeiling = CalCtx.boundary.floorY + 0.5;
    CalCtx.boundary.ceilingY = std::max(ceilingY, minCeiling);
}

void RestoreFloorCaptureOriginal()
{
    if (!s_floorCapture.active()) return;
    CalCtx.boundary.floorY = s_floorCapture.originalFloorY();
    CalCtx.boundary.ceilingY = s_floorCapture.originalCeilingY();
    for (auto& v : CalCtx.boundary.vertices) {
        v.y = CalCtx.boundary.floorY;
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

    ApplyBoundaryFloorY(floorY);
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
        "[boundary-floor] set from controller: device=%d system='%s' floor_y=%.3f vertices=%zu",
        static_cast<int>(deviceId),
        trackingSystem.c_str(),
        CalCtx.boundary.floorY,
        CalCtx.boundary.vertices.size());
    Metrics::WriteLogAnnotation(lbuf);
    openvr_pair::common::DiagnosticLog("boundary-floor", "%s", lbuf);
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
    const auto& pal = openvr_pair::overlay::ui::GetPalette();
    const auto state = s_capture.state();
    const bool transformReady = BoundaryTransformReady();

    ImGui::TextWrapped(
        "Draw by moving one lighthouse controller around the play-space edge. The lowest matching controller is used.");
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip(
            "Stored in the target tracking system's coordinates. Unlike Quest Guardian, this\n"
            "polygon does not move when the headset re-localizes -- it's pinned to your\n"
            "physical lighthouses.");
    }
    if (!transformReady) {
        ImGui::TextColored(pal.statusWarn,
            "Waiting for a valid calibration profile before preview or SteamVR chaperone push.");
    }
    ImGui::Spacing();

    if (state == wkopenvr::boundary::CaptureState::Idle) {
        const bool hasVerts = !CalCtx.boundary.vertices.empty();
        if (s_floorCapture.active()) {
            const auto& floor = s_floorCapture.candidate();
            ImGui::TextColored(pal.statusWarn,
                floor.valid
                    ? "Floor preview is following the lowest controller sample."
                    : "Move a lighthouse controller down to the floor.");
            if (floor.valid) {
                ImGui::TextDisabled("Preview floor %.2f m from device %d (%s)",
                    floor.floorY,
                    floor.deviceId,
                    floor.trackingSystem.c_str());
            }
            if (floor.valid && ImGui::Button("Apply floor")) {
                Eigen::Affine3d floorPose = floor.pose;
                floorPose.translation().y() = floor.floorY;
                ApplyFloorFromControllerPose(
                    floorPose,
                    static_cast<vr::TrackedDeviceIndex_t>(floor.deviceId),
                    floor.trackingSystem);
                s_floorCapture.Reset();
                s_boundaryError.clear();
            }
            ImGui::SameLine();
            if (ImGui::SmallButton("Cancel floor set")) {
                RestoreFloorCaptureOriginal();
                RepushActiveBoundaryAfterEdit();
                s_floorCapture.Reset();
                s_boundaryError.clear();
                HideBoundaryPreviewOverlay();
            }
        } else {
            if (ImGui::Button("Set floor from controller")) {
                s_floorCapture.Begin(CalCtx.boundary.floorY, CalCtx.boundary.ceilingY);
                ++s_floorSessionId;
                s_boundaryError.clear();
                HideBoundaryPreviewOverlay();
                Metrics::WriteLogAnnotation("[boundary-floor] preview started");
            }
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("No trigger needed. The lowest tracked lighthouse controller sets the preview; click Apply floor when it looks right.");
            }
        }

        ImGui::Spacing();

        float floorY = (float)CalCtx.boundary.floorY;
        float ceilingY = (float)CalCtx.boundary.ceilingY;
        ImGui::PushItemWidth(110.0f);
        if (ImGui::DragFloat("Floor (m)##bnd_floor", &floorY, 0.01f, -3.0f, 3.0f, "%.2f")) {
            ApplyBoundaryFloorY((double)floorY);
            RepushActiveBoundaryAfterEdit();
            SaveProfile(CalCtx);
        }
        ImGui::SameLine();
        if (ImGui::DragFloat("Ceiling (m)##bnd_ceil", &ceilingY, 0.01f, -2.5f, 5.0f, "%.2f")) {
            ApplyBoundaryCeilingY((double)ceilingY);
            RepushActiveBoundaryAfterEdit();
            SaveProfile(CalCtx);
        }
        ImGui::PopItemWidth();
        if (transformReady) {
            ImGui::SameLine();
            ImGui::TextDisabled("SteamVR floor %.2f m",
                BoundaryHeightToStanding(CalCtx.boundary.vertices, CalCtx.boundary.floorY));
        }

        ImGui::Spacing();

        if (!hasVerts) {
            if (!transformReady || s_floorCapture.active()) ImGui::BeginDisabled();
            if (ImGui::Button("Draw boundary")) {
                s_capture.Start();
                HideBoundaryPreviewOverlay();
                s_boundaryError.clear();
            }
            if (!transformReady || s_floorCapture.active()) ImGui::EndDisabled();
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Move the lowest lighthouse controller around the edge. Trigger input is not required.");
            }
        } else {
            if (!transformReady || s_floorCapture.active()) ImGui::BeginDisabled();
            if (ImGui::Button("Re-draw")) {
                s_capture.Start();
                HideBoundaryPreviewOverlay();
                s_boundaryError.clear();
            }
            if (!transformReady || s_floorCapture.active()) ImGui::EndDisabled();
            ImGui::SameLine();
            if (ImGui::Button("Discard")) {
                CalCtx.boundary.vertices.clear();
                CalCtx.boundary.enabled = false;
                SaveProfile(CalCtx);
                s_boundaryError.clear();
            }
            ImGui::SameLine();

            bool enabled = CalCtx.boundary.enabled;
            const bool disableActiveCheckbox = !transformReady && !enabled;
            if (disableActiveCheckbox) ImGui::BeginDisabled();
            if (ImGui::Checkbox("Active in-headset", &enabled)) {
                CalCtx.boundary.enabled = enabled;
                if (enabled) {
                    if (!CalCtx.boundary.priorChaperoneCaptured) {
                        CalCtx.boundary.priorChaperone = wkopenvr::boundary::SnapshotCurrentChaperone();
                        CalCtx.boundary.priorChaperoneCaptured = !CalCtx.boundary.priorChaperone.empty();
                    }
                    if (!PushTargetBoundaryToChaperone(s_boundaryError)) {
                        CalCtx.boundary.enabled = false;
                    }
                } else {
                    if (CalCtx.boundary.priorChaperoneCaptured) {
                        wkopenvr::boundary::RestoreChaperoneFromSnapshot(CalCtx.boundary.priorChaperone);
                    }
                }
                SaveProfile(CalCtx);
            }
            if (disableActiveCheckbox) ImGui::EndDisabled();
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Push this boundary to SteamVR chaperone so it appears in-headset.");
            }

            ImGui::Spacing();

            // Summary.
            const auto pts = wkopenvr::boundary::ProjectXZ(CalCtx.boundary.vertices);
            const double area = wkopenvr::boundary::AbsoluteAreaXZ(pts);
            ImGui::TextDisabled("%d pts, %.2f m^2",
                (int)CalCtx.boundary.vertices.size(), area);

            DrawPolygonPreview(CalCtx.boundary.vertices);

            if (CalCtx.boundary.priorChaperoneCaptured) {
                ImGui::Spacing();
                if (ImGui::SmallButton("Restore SteamVR's original chaperone")) {
                    if (!wkopenvr::boundary::RestoreChaperoneFromSnapshot(CalCtx.boundary.priorChaperone)) {
                        s_boundaryError = "Restore failed: snapshot may be corrupted.";
                    } else {
                        CalCtx.boundary.enabled = false;
                        s_boundaryError.clear();
                    }
                    SaveProfile(CalCtx);
                }
                if (ImGui::IsItemHovered()) {
                    ImGui::SetTooltip("Reverts SteamVR chaperone to what it was before WKOpenVR first pushed a boundary.");
                }
            }
        }
    } else if (state == wkopenvr::boundary::CaptureState::Active) {
        const bool closePreview = BoundaryLoopNearlyClosed(s_capture.vertices());
        ImGui::TextColored(pal.statusWarn,
            "Recording. Move the lowest controller around the edge, then click Done.");
        ImGui::TextDisabled("Raw vertices: %d", (int)s_capture.rawVertexCount());
        DrawPolygonPreview(s_capture.vertices(), closePreview, true);
        TickBoundaryPreviewForTargetVertices(
            true,
            s_capture.vertices(),
            closePreview);
        ImGui::Spacing();
        if (ImGui::Button("Done##bnd_done")) {
            s_capture.Finish();
            TickBoundaryPreviewForTargetVertices(
                true,
                s_capture.vertices(),
                true);
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel##bnd_cancel")) {
            s_capture.Cancel();
            HideBoundaryPreviewOverlay();
        }
    } else { // Finished
        const auto& verts = s_capture.vertices();
        const auto pts = wkopenvr::boundary::ProjectXZ(verts);
        const double area = wkopenvr::boundary::AbsoluteAreaXZ(pts);

        ImGui::Text("%d vertices, %.2f m^2", (int)verts.size(), area);
        ImGui::TextDisabled("Cleaned to edge vertices.");

        DrawPolygonPreview(verts);
        TickBoundaryPreviewForTargetVertices(
            true,
            verts,
            true);

        ImGui::Spacing();
        if (ImGui::Button("Keep this boundary##bnd_apply")) {
            if (verts.size() < 3 || area < 0.25) {
                s_boundaryError = "Boundary needs at least three floor points and a usable area.";
            } else {
                if (!CalCtx.boundary.priorChaperoneCaptured) {
                    auto snapshot = wkopenvr::boundary::SnapshotCurrentChaperone();
                    if (!snapshot.empty()) {
                        CalCtx.boundary.priorChaperone = std::move(snapshot);
                        CalCtx.boundary.priorChaperoneCaptured = true;
                    }
                }
                CalCtx.boundary.vertices = verts;
                bool appliedOk = true;
                if (CalCtx.boundary.enabled) {
                    if (!PushTargetBoundaryToChaperone(s_boundaryError)) {
                        appliedOk = false;
                    }
                }
                SaveProfile(CalCtx);
                if (appliedOk) {
                    s_capture.Cancel();
                    HideBoundaryPreviewOverlay();
                    s_boundaryError.clear();
                }
            }
        }
        ImGui::SameLine();
        if (ImGui::Button("Discard##bnd_discard_fin")) {
            s_capture.Cancel();
            HideBoundaryPreviewOverlay();
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

void WriteBoundaryInputSummary(const BoundaryInputStats& stats, uint64_t sessionId, size_t rawCount) {
    char lbuf[640];
    snprintf(lbuf, sizeof lbuf,
        "[boundary-capture] waiting: session=%llu raw=%zu target='%s' controllers=%d matching=%d skipped_system=%d pose_ok=%d pose_invalid=%d last_device=%d last_system='%s' chosen_device=%d chosen_system='%s' chosen_y=%.3f",
        static_cast<unsigned long long>(sessionId),
        rawCount,
        CalCtx.targetTrackingSystem.c_str(),
        stats.trackedControllers,
        stats.matchingControllers,
        stats.skippedTrackingSystem,
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
        TickBoundaryPreviewForTargetVertices(
            true,
            s_capture.vertices(),
            BoundaryLoopNearlyClosed(s_capture.vertices()));
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
    for (vr::TrackedDeviceIndex_t deviceId = 0; deviceId < vr::k_unMaxTrackedDeviceCount; ++deviceId) {
        if (vrs->GetTrackedDeviceClass(deviceId) != vr::TrackedDeviceClass_Controller) {
            continue;
        }
        ++stats.trackedControllers;
        stats.lastDeviceId = static_cast<int>(deviceId);
        stats.lastTrackingSystem = TrackingSystemName(vrs, deviceId);

        if (!CalCtx.targetTrackingSystem.empty()
            && stats.lastTrackingSystem != CalCtx.targetTrackingSystem)
        {
            ++stats.skippedTrackingSystem;
            continue;
        }
        ++stats.matchingControllers;

        const auto& dp = CalCtx.devicePoses[deviceId];
        if (!dp.poseIsValid || dp.result != vr::ETrackingResult::TrackingResult_Running_OK) {
            ++stats.poseInvalid;
            continue;
        }
        ++stats.poseOk;

        const Eigen::Affine3d rawPose = DriverPoseToWorldAffine(dp);
        candidates.push_back({ deviceId, stats.lastTrackingSystem, rawPose });
    }

    if (candidates.empty()) {
        tickPreview();
        ++s_waitTicks;
        if (s_waitTicks == 1 || (s_waitTicks % 120) == 0) {
            WriteBoundaryInputSummary(stats, sessionId, s_capture.rawVertexCount());
        }
        return;
    }

    const auto chosenIt = std::min_element(
        candidates.begin(),
        candidates.end(),
        [](const ControllerCandidate& a, const ControllerCandidate& b) {
            return a.rawPose.translation().y() < b.rawPose.translation().y();
        });
    const ControllerCandidate& chosen = *chosenIt;
    stats.chosenDeviceId = static_cast<int>(chosen.deviceId);
    stats.chosenTrackingSystem = chosen.trackingSystem;
    stats.chosenY = chosen.rawPose.translation().y();

    if (s_floorCapture.active()) {
        const bool updated = s_floorCapture.Observe(
            chosen.rawPose,
            static_cast<int>(chosen.deviceId),
            chosen.trackingSystem);
        const auto& floor = s_floorCapture.candidate();
        if (floor.valid) {
            ApplyBoundaryFloorY(floor.floorY);
            const auto marker = wkopenvr::boundary::BuildFloorMarkerVertices(
                floor.pose,
                floor.floorY);
            TickBoundaryPreviewForTargetVertices(true, marker, true);
            if (updated) {
                char fbuf[320];
                snprintf(fbuf, sizeof fbuf,
                    "[boundary-floor] preview: samples=%zu device=%d system='%s' floor_y=%.3f controller=(%.3f,%.3f,%.3f)",
                    floor.sampleCount,
                    floor.deviceId,
                    floor.trackingSystem.c_str(),
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

    const bool accepted = s_capture.TickProjectedPosition(
        chosen.rawPose,
        true,
        CalCtx.boundary.floorY);
    if (accepted) {
        Eigen::Affine3d standingPose = chosen.rawPose;
        if (CalCtx.validProfile) {
            standingPose = wkopenvr::boundary::TransformPoseToStandingUniverse(
                chosen.rawPose,
                BoundaryTargetToStandingTransform());
        }
        char cbuf[640];
        snprintf(cbuf, sizeof cbuf,
            "[boundary-capture] accepted controller position: session=%llu device=%d system='%s' raw=%zu floor_y=%.3f raw_pos=(%.3f,%.3f,%.3f) standing_pos=(%.3f,%.3f,%.3f)",
            static_cast<unsigned long long>(sessionId),
            static_cast<int>(chosen.deviceId),
            chosen.trackingSystem.c_str(),
            s_capture.rawVertexCount(),
            CalCtx.boundary.floorY,
            chosen.rawPose.translation().x(),
            chosen.rawPose.translation().y(),
            chosen.rawPose.translation().z(),
            standingPose.translation().x(),
            standingPose.translation().y(),
            standingPose.translation().z());
        Metrics::WriteLogAnnotation(cbuf);
        openvr_pair::common::DiagnosticLog("boundary-capture", "%s", cbuf);
    }
    tickPreview();
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
