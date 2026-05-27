#include "Calibration.h"
#include "CalibrationMetrics.h"
#include "Configuration.h"
#include "ControllerInput.h"
#include "DiagnosticsLog.h"
#include "Boundary.h"
#include "BoundaryCapture.h"
#include "BoundaryPreview.h"
#include "UserInterfaceHeadMount.h"
#include "UiHelpers.h"

#include <openvr.h>
#include <algorithm>
#include <string>
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
std::string s_boundaryError;

void HideBoundaryPreviewOverlay() {
    wkopenvr::boundary::TickBoundaryPreview(false, {}, CalCtx.boundary.floorY, false);
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
    for (size_t i = 0; i < segmentCount; ++i) {
        const auto& a = verts[i];
        const auto& b = verts[(i + 1) % verts.size()];
        dl->AddLine(toCanvas(a.x, a.z), toCanvas(b.x, b.z), lineColor, activePath ? 2.5f : 1.5f);
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

    ImGui::TextWrapped(
        "Point an Index controller at the floor, hold the trigger, and trace the play-space edge.");
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip(
            "Stored in the target tracking system's coordinates. Unlike Quest Guardian, this\n"
            "polygon does not move when the headset re-localizes -- it's pinned to your\n"
            "physical lighthouses.");
    }
    ImGui::Spacing();

    if (state == wkopenvr::boundary::CaptureState::Idle) {
        const bool hasVerts = !CalCtx.boundary.vertices.empty();
        if (!hasVerts) {
            if (ImGui::Button("Draw boundary")) {
                s_capture.Start();
                HideBoundaryPreviewOverlay();
                s_boundaryError.clear();
            }
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Aim the controller at the floor. The pointer ray paints floor points while the trigger is held.");
            }
        } else {
            if (ImGui::Button("Re-draw")) {
                s_capture.Start();
                HideBoundaryPreviewOverlay();
                s_boundaryError.clear();
            }
            ImGui::SameLine();
            if (ImGui::Button("Discard")) {
                CalCtx.boundary.vertices.clear();
                CalCtx.boundary.enabled = false;
                SaveProfile(CalCtx);
                s_boundaryError.clear();
            }
            ImGui::SameLine();

            bool enabled = CalCtx.boundary.enabled;
            if (ImGui::Checkbox("Active in-headset", &enabled)) {
                CalCtx.boundary.enabled = enabled;
                if (enabled) {
                    if (!wkopenvr::boundary::PushToChaperone(
                            CalCtx.boundary.vertices,
                            CalCtx.boundary.floorY,
                            CalCtx.boundary.ceilingY)) {
                        s_boundaryError = "PushToChaperone failed. Is SteamVR running?";
                        CalCtx.boundary.enabled = false;
                    }
                } else {
                    if (CalCtx.boundary.priorChaperoneCaptured) {
                        wkopenvr::boundary::RestoreChaperoneFromSnapshot(CalCtx.boundary.priorChaperone);
                    }
                }
                SaveProfile(CalCtx);
            }
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Push this boundary to SteamVR chaperone so it appears in-headset.");
            }

            ImGui::Spacing();

            // Floor / ceiling on one line so they don't dominate.
            float floorY    = (float)CalCtx.boundary.floorY;
            float ceilingY  = (float)CalCtx.boundary.ceilingY;
            ImGui::PushItemWidth(110.0f);
            if (ImGui::DragFloat("Floor (m)##bnd_floor", &floorY, 0.01f, -2.0f, 2.0f, "%.2f")) {
                CalCtx.boundary.floorY = (double)floorY;
                SaveProfile(CalCtx);
            }
            ImGui::SameLine();
            if (ImGui::DragFloat("Ceiling (m)##bnd_ceil", &ceilingY, 0.01f, 0.5f, 5.0f, "%.2f")) {
                CalCtx.boundary.ceilingY = (double)ceilingY;
                SaveProfile(CalCtx);
            }
            ImGui::PopItemWidth();

            // Summary.
            const auto pts = wkopenvr::boundary::ProjectXZ(CalCtx.boundary.vertices);
            const double area = wkopenvr::boundary::AbsoluteAreaXZ(pts);
            ImGui::SameLine();
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
        ImGui::TextColored(pal.statusWarn,
            "Recording. Point at the floor and trace the edge. Click Done when finished.");
        ImGui::TextDisabled("Raw vertices: %d", (int)s_capture.rawVertexCount());
        DrawPolygonPreview(s_capture.vertices(), false, true);
        wkopenvr::boundary::TickBoundaryPreview(
            true,
            s_capture.vertices(),
            CalCtx.boundary.floorY,
            false);
        ImGui::Spacing();
        if (ImGui::Button("Done##bnd_done")) {
            s_capture.Finish();
            wkopenvr::boundary::TickBoundaryPreview(
                true,
                s_capture.vertices(),
                CalCtx.boundary.floorY,
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
        wkopenvr::boundary::TickBoundaryPreview(
            true,
            verts,
            CalCtx.boundary.floorY,
            true);

        ImGui::Spacing();
        if (ImGui::Button("Keep this boundary##bnd_apply")) {
            if (verts.size() < 3 || area < 0.25) {
                s_boundaryError = "Boundary needs at least three floor points and a usable area.";
            } else {
                if (!CalCtx.boundary.priorChaperoneCaptured) {
                    CalCtx.boundary.priorChaperone = wkopenvr::boundary::SnapshotCurrentChaperone();
                    CalCtx.boundary.priorChaperoneCaptured = true;
                }
                CalCtx.boundary.vertices = verts;
                bool appliedOk = true;
                if (CalCtx.boundary.enabled) {
                    if (!wkopenvr::boundary::PushToChaperone(
                            CalCtx.boundary.vertices,
                            CalCtx.boundary.floorY,
                            CalCtx.boundary.ceilingY)) {
                        s_boundaryError = "PushToChaperone failed after Apply.";
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
    int fallbackAnySystem = 0;
    int skippedTrackingSystem = 0;
    int controllerStateFailed = 0;
    int stateOk = 0;
    int triggerHeld = 0;
    int buttonPressed = 0;
    int legacyFallbackUsed = 0;
    int poseOk = 0;
    int poseInvalid = 0;
    int axisPropertyErrors = 0;
    int triggerAxisCount = 0;
    int lastDeviceId = -1;
    int lastTriggerAxis = -1;
    float lastTriggerValue = 0.0f;
    std::string lastTrackingSystem;
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
        "[boundary-capture] waiting: session=%llu raw=%zu target='%s' controllers=%d matching=%d fallback_any=%d skipped_system=%d state_ok=%d state_failed=%d trigger=%d button=%d legacy=%d pose_ok=%d pose_invalid=%d trigger_axes=%d axis_prop_errors=%d last_device=%d last_system='%s' last_axis=%d last_value=%.3f",
        static_cast<unsigned long long>(sessionId),
        rawCount,
        CalCtx.targetTrackingSystem.c_str(),
        stats.trackedControllers,
        stats.matchingControllers,
        stats.fallbackAnySystem,
        stats.skippedTrackingSystem,
        stats.stateOk,
        stats.controllerStateFailed,
        stats.triggerHeld,
        stats.buttonPressed,
        stats.legacyFallbackUsed,
        stats.poseOk,
        stats.poseInvalid,
        stats.triggerAxisCount,
        stats.axisPropertyErrors,
        stats.lastDeviceId,
        stats.lastTrackingSystem.c_str(),
        stats.lastTriggerAxis,
        stats.lastTriggerValue);
    Metrics::WriteLogAnnotation(lbuf);
    openvr_pair::common::DiagnosticLog("boundary-capture", "%s", lbuf);
}

} // namespace

// ---------------------------------------------------------------------------
// Capture tick -- called each CalibrationTick so capture runs regardless of
// which tab is visible.
// ---------------------------------------------------------------------------

void CCal_TickBoundaryCapture() {
    if (s_capture.state() != wkopenvr::boundary::CaptureState::Active)
        return;

    auto tickPreview = []() {
        wkopenvr::boundary::TickBoundaryPreview(
            true,
            s_capture.vertices(),
            CalCtx.boundary.floorY,
            false);
    };

    auto* vrs = vr::VRSystem();
    const uint64_t sessionId = s_capture.sessionId();
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
    auto tryController = [&](vr::TrackedDeviceIndex_t deviceId, bool enforceTrackingSystem, bool countDevice) {
        if (vrs->GetTrackedDeviceClass(deviceId) != vr::TrackedDeviceClass_Controller) {
            return false;
        }
        if (countDevice) ++stats.trackedControllers;
        stats.lastDeviceId = static_cast<int>(deviceId);
        stats.lastTrackingSystem = TrackingSystemName(vrs, deviceId);

        if (enforceTrackingSystem && !CalCtx.targetTrackingSystem.empty()
            && stats.lastTrackingSystem != CalCtx.targetTrackingSystem)
        {
            if (countDevice) ++stats.skippedTrackingSystem;
            return false;
        }
        ++stats.matchingControllers;

        vr::VRControllerState_t st = {};
        if (!vrs->GetControllerState(deviceId, &st, sizeof(st))) {
            ++stats.controllerStateFailed;
            return false;
        }
        ++stats.stateOk;

        wkopenvr::controller_input::TriggerReading trigger;
        if (!wkopenvr::controller_input::IsTriggerHeld(vrs, deviceId, st, 0.75f, &trigger)) {
            stats.axisPropertyErrors += trigger.propertyErrors;
            stats.triggerAxisCount += trigger.triggerAxisCount;
            if (trigger.buttonPressed) ++stats.buttonPressed;
            if (trigger.legacyFallbackUsed) ++stats.legacyFallbackUsed;
            if (trigger.analogAxis >= 0) {
                stats.lastTriggerAxis = trigger.analogAxis;
                stats.lastTriggerValue = trigger.analogValue;
            }
            return false;
        }
        ++stats.triggerHeld;
        if (trigger.buttonPressed) ++stats.buttonPressed;
        if (trigger.legacyFallbackUsed) ++stats.legacyFallbackUsed;
        stats.axisPropertyErrors += trigger.propertyErrors;
        stats.triggerAxisCount += trigger.triggerAxisCount;
        if (trigger.analogAxis >= 0) {
            stats.lastTriggerAxis = trigger.analogAxis;
            stats.lastTriggerValue = trigger.analogValue;
        }

        const auto& dp = CalCtx.devicePoses[deviceId];
        if (!dp.poseIsValid || dp.result != vr::ETrackingResult::TrackingResult_Running_OK) {
            ++stats.poseInvalid;
            return false;
        }
        ++stats.poseOk;

        const Eigen::Affine3d rawPose = DriverPoseToWorldAffine(dp);
        const bool applyCalibrationToController =
            CalCtx.enabled
            && CalCtx.validProfile
            && !CalCtx.targetTrackingSystem.empty()
            && stats.lastTrackingSystem == CalCtx.targetTrackingSystem;
        Eigen::Affine3d pose = rawPose;
        if (applyCalibrationToController) {
            pose = wkopenvr::boundary::TransformPoseToStandingUniverse(
                rawPose,
                wkopenvr::boundary::ProfileTransformFromCalibration(
                    CalCtx.calibratedRotation,
                    CalCtx.calibratedTranslation));
        }
        const bool accepted = s_capture.Tick(pose, true, CalCtx.boundary.floorY);
        if (accepted) {
            char cbuf[520];
            snprintf(cbuf, sizeof cbuf,
                "[boundary-capture] accepted controller input: session=%llu device=%d system='%s' raw=%zu button=%d legacy=%d axis=%d value=%.3f fallback_any=%d applied_cal=%d raw_pos=(%.3f,%.3f,%.3f) paint_pos=(%.3f,%.3f,%.3f)",
                static_cast<unsigned long long>(sessionId),
                static_cast<int>(deviceId),
                stats.lastTrackingSystem.c_str(),
                s_capture.rawVertexCount(),
                trigger.buttonPressed ? 1 : 0,
                trigger.legacyFallbackUsed ? 1 : 0,
                trigger.analogAxis,
                trigger.analogValue,
                stats.fallbackAnySystem,
                applyCalibrationToController ? 1 : 0,
                rawPose.translation().x(),
                rawPose.translation().y(),
                rawPose.translation().z(),
                pose.translation().x(),
                pose.translation().y(),
                pose.translation().z());
            Metrics::WriteLogAnnotation(cbuf);
            openvr_pair::common::DiagnosticLog("boundary-capture", "%s", cbuf);
        }
        return accepted;
    };

    bool captured = false;
    for (vr::TrackedDeviceIndex_t deviceId = 0; deviceId < vr::k_unMaxTrackedDeviceCount; ++deviceId) {
        if (tryController(deviceId, true, true)) {
            captured = true;
            break;
        }
    }

    if (!captured
        && stats.trackedControllers > 0
        && stats.matchingControllers == 0
        && !CalCtx.targetTrackingSystem.empty())
    {
        stats.fallbackAnySystem = 1;
        for (vr::TrackedDeviceIndex_t deviceId = 0; deviceId < vr::k_unMaxTrackedDeviceCount; ++deviceId) {
            if (tryController(deviceId, false, false)) {
                captured = true;
                break;
            }
        }
    }

    if (captured) {
        tickPreview();
        return;
    }

    tickPreview();
    ++s_waitTicks;
    if (s_waitTicks == 1 || (s_waitTicks % 120) == 0) {
        WriteBoundaryInputSummary(stats, sessionId, s_capture.rawVertexCount());
    }
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
