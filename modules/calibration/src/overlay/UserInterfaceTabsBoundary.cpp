#include "Calibration.h"
#include "Configuration.h"
#include "CalibrationMetrics.h"
#include "Boundary.h"
#include "BoundaryCapture.h"
#include "GuardianAutoApply.h"
#include "HeadMountOffsetModal.h"
#include "HeadMountPreview.h"
#include "HeadFromTrackerSolve.h"
#include "HeadMountTargetBinding.h"
#include "IPCClient.h"
#include "Protocol.h"
#include "SpaceCalibratorUmbrellaRuntime.h"
#include "UiHelpers.h"

#include <AdbSetupWizard.h>

#include <openvr.h>
#include <string>
#include <cstdio>
#include <cstring>
#include <exception>
#include <imgui/imgui.h>
#include "imgui_extensions.h"

extern SCIPCClient Driver;
extern CalibrationContext CalCtx;

void SaveProfile(CalibrationContext& ctx);

// Forward declaration for the device-label helper defined in UserInterface.cpp.
// The Headset-tab UI labels the continuous target (a StandbyDevice), not a
// live VRDevice, so this matches the call site at the Step 1 status block.
std::string LabelString(const StandbyDevice& device);

// ---------------------------------------------------------------------------
// Section A: Head-mounted tracker
// ---------------------------------------------------------------------------
//
// User journey for this whole tab:
//   1. (Head-mount section)   Stabilize SLAM drift with the continuous target.
//   2. (Boundary section)     Draw your own play-space safety polygon.
//   3. (Quest Guardian section) Hide Quest's overlapping guardian.
//
// Each section is self-contained but the disabled-state copy nudges the user
// from one step to the next without long instructional paragraphs.

namespace {

// IPC: push the active head-mount config to the driver.
void SendHeadMountConfig() {
    const auto& hm = CalCtx.headMount;
    protocol::Request req(protocol::RequestSetHeadMountConfig);
    auto& p = req.setHeadMountConfig;
    p.mode             = static_cast<uint32_t>(hm.mode);
    p.deviceId         = hm.deviceID;
    p.hideTracker      = hm.hideTracker;
    p.offsetCalibrated = hm.offsetCalibrated;

    size_t len = hm.trackerSerial.size();
    if (len >= sizeof p.trackerSerial) len = sizeof p.trackerSerial - 1;
    memcpy(p.trackerSerial, hm.trackerSerial.data(), len);
    p.trackerSerial[len] = '\0';

    len = hm.trackerTrackingSystem.size();
    if (len >= sizeof p.trackerTrackingSystem) len = sizeof p.trackerTrackingSystem - 1;
    memcpy(p.trackerTrackingSystem, hm.trackerTrackingSystem.data(), len);
    p.trackerTrackingSystem[len] = '\0';

    Eigen::Quaterniond q(hm.headFromTracker.linear());
    q.normalize();
    const Eigen::Vector3d t = hm.headFromTracker.translation();
    p.headFromTrackerTrans[0] = t.x();
    p.headFromTrackerTrans[1] = t.y();
    p.headFromTrackerTrans[2] = t.z();
    p.headFromTrackerRot[0]   = q.x();
    p.headFromTrackerRot[1]   = q.y();
    p.headFromTrackerRot[2]   = q.z();
    p.headFromTrackerRot[3]   = q.w();

    try {
        Driver.SendBlocking(req);
    } catch (const std::exception& e) {
        char buf[240];
        std::snprintf(buf, sizeof buf,
            "[head-mount] config push failed: %s", e.what());
        Metrics::WriteLogAnnotation(buf);
    }
}

// Fine-tune sliders toggle. File-scope so the user's open/closed choice
// survives tab switches within a single session.
bool s_offsetSlidersOpen = false;

void DrawHeadMountSection(const ImVec2& panelSize) {
    const auto& pal = openvr_pair::overlay::ui::GetPalette();
    auto& hm = CalCtx.headMount;
    const bool bindingChanged = wkopenvr::headmount::BindHeadMountToContinuousTarget(CalCtx);
    if (bindingChanged) {
        SaveProfile(CalCtx);
        SendHeadMountConfig();
    }
    const bool hasContinuousTarget = wkopenvr::headmount::HasContinuousTargetIdentity(CalCtx);

    {
    openvr_pair::overlay::ui::PanelScope panel("Step 1: Head-mounted tracker", panelSize);

    // Short one-line framing. Detail is in the tooltip.
    ImGui::TextWrapped(
        "Uses the active continuous target as the lighthouse tracker attached to your headset.");
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip(
            "Start continuous calibration with the headset-mounted lighthouse tracker selected as the target.\n"
            "This tab binds to that target automatically, so you do not have to pick the same tracker twice.");
    }

    ImGui::Spacing();

    // --- Continuous target ----------------------------------------------
    ImGui::TextUnformatted("1. Continuous target");
    if (hasContinuousTarget) {
        const std::string label = LabelString(CalCtx.targetStandby);
        ImGui::TextWrapped("%s", label.c_str());
        if (CalCtx.targetID < 0) {
            ImGui::SameLine();
            ImGui::TextDisabled("(waiting for device)");
        }
    } else {
        ImGui::TextDisabled("Start continuous calibration with the headset-mounted tracker as the target.");
    }

    const bool hasTracker = hasContinuousTarget && !hm.trackerSerial.empty();
    const bool offsetOk   = hm.offsetCalibrated;

    // --- Offset calibration step -----------------------------------------
    ImGui::Spacing();
    ImGui::TextUnformatted("2. Calibrate the tracker-to-headset offset");
    {
        openvr_pair::overlay::ui::DisabledSection ds(!hasTracker,
            "Start continuous calibration with the headset-mounted tracker as the target first.");
        const char* btnLabel = offsetOk
            ? "Re-calibrate offset"
            : "Calibrate offset";
        if (ImGui::Button(btnLabel)) {
            wkopenvr::headmount::OpenOffsetModal();
        }
        ds.AttachReasonTooltip();
    }
    if (ImGui::IsItemHovered() && hasTracker) {
        ImGui::SetTooltip(
            "Solves the rigid offset from the tracker to the headset.\n"
            "Move your head slowly through pitch, yaw, and roll for ~10 seconds.");
    }
    ImGui::SameLine();
    {
        openvr_pair::overlay::ui::DisabledSection ds(!hasTracker || !offsetOk,
            !hasTracker ? "Start continuous calibration with the headset-mounted tracker as the target first."
                        : "Calibrate offset first.");
        if (ImGui::Button(s_offsetSlidersOpen ? "Hide fine-tune" : "Fine-tune offset")) {
            s_offsetSlidersOpen = !s_offsetSlidersOpen;
        }
        ds.AttachReasonTooltip();
    }
    if (ImGui::IsItemHovered() && offsetOk) {
        ImGui::SetTooltip(
            "Nudge the solved offset by hand. Useful if the auto-solve\n"
            "got close but the in-headset preview marker doesn't sit\n"
            "exactly at your eye position.");
    }

    if (s_offsetSlidersOpen && offsetOk) {
        ImGui::Spacing();
        wkopenvr::headmount::DrawOffsetInlinePanel();
    }

    // --- Mode picker -----------------------------------------------------
    ImGui::Spacing();
    ImGui::TextUnformatted("3. Choose what the tracker does for continuous calibration");
    {
        openvr_pair::overlay::ui::DisabledSection ds(!hasTracker,
            "Start continuous calibration with the headset-mounted tracker as the target first.");

        struct ModeOpt {
            HeadMountMode value;
            const char*   label;
            const char*   tip;
            bool          requiresOffset;
        };
        const ModeOpt opts[] = {
            { HeadMountMode::Off,         "Off",
              "No head-mount features. Body trackers drift with SLAM as before.",
              false },
            { HeadMountMode::AutoPaired,  "Stabilize continuous calibration",
              "The continuous target becomes a constant paired observation from the headset-mounted tracker.\n"
              "Drift correction stays smooth across long sessions. Requires the offset to be calibrated.",
              true },
            { HeadMountMode::Corroborate, "Block SLAM re-localization jumps",
              "When Quest re-localizes (passthrough, room scan), the headset pose jumps.\n"
              "With this on, the head-tracker provides an independent witness so those\n"
              "jumps don't trigger recovery or feed bad samples into the solver.\n"
              "Requires the offset to be calibrated.",
              true },
#if WKOPENVR_BUILD_IS_DEV
            { HeadMountMode::DriverSynth, "EXPERIMENTAL: synthesize headset pose from tracker",
              "Replaces the rendered headset pose with one derived from the head-tracker.\n"
              "Known compositor and comfort risks. Dev builds only.",
              true },
#endif
        };

        for (const auto& opt : opts) {
            const bool needsOffset = opt.requiresOffset && !offsetOk;
            openvr_pair::overlay::ui::DisabledSection inner(needsOffset,
                "Calibrate the offset (step 2) before using this mode.");
            const bool selected = (hm.mode == opt.value);
            // ImGui RadioButton id pinned per-option so SameLine layout
            // doesn't merge ids.
            ImGui::PushID(static_cast<int>(opt.value));
            if (ImGui::RadioButton(opt.label, selected)) {
                if (hm.mode != opt.value) {
                    hm.mode = opt.value;
                    SaveProfile(CalCtx);
                    SendHeadMountConfig();
                }
            }
            ImGui::PopID();
            inner.AttachReasonTooltip();
            if (ImGui::IsItemHovered() && !needsOffset) {
                ImGui::SetTooltip("%s", opt.tip);
            }
        }
        ds.AttachReasonTooltip();
    }

    // --- Hide in OpenVR --------------------------------------------------
    ImGui::Spacing();
    {
        openvr_pair::overlay::ui::DisabledSection ds(!hasTracker,
            "Start continuous calibration with the headset-mounted tracker as the target first.");
        if (ImGui::Checkbox("Hide this tracker from games", &hm.hideTracker)) {
            CalCtx.quashTargetInContinuous = hm.hideTracker;
            SaveProfile(CalCtx);
            SendHeadMountConfig();
        }
        ds.AttachReasonTooltip();
    }
    if (ImGui::IsItemHovered() && hasTracker) {
        ImGui::SetTooltip(
            "Suppress the head-tracker's pose in OpenVR so it doesn't appear as a\n"
            "floating tracker in-headset. The continuous calibration math still uses its pose internally.");
    }

    // --- Status line -----------------------------------------------------
    ImGui::Spacing();
    {
        // Diagnostic: dump the head-mount resolution snapshot once per change
        // so the next session log reveals which precondition fails when the
        // status flips red. targetID + state are included so a "still broken"
        // report immediately points at the right gap (AssignTargets rescan
        // dead vs binding dead vs driver pose pipeline).
        {
            static int32_t s_lastDeviceID = -2;
            static std::string s_lastSerial, s_lastModel, s_lastSystem;
            static int s_lastBits = -1;
            const int32_t curDev = hm.deviceID;
            const bool inRange = curDev >= 0
                && (uint32_t)curDev < vr::k_unMaxTrackedDeviceCount;
            const bool poseValid = inRange && CalCtx.devicePoses[curDev].poseIsValid;
            const int curBits = (inRange ? 1 : 0) | (poseValid ? 2 : 0);
            if (curDev != s_lastDeviceID
                || hm.trackerSerial != s_lastSerial
                || hm.trackerModel != s_lastModel
                || hm.trackerTrackingSystem != s_lastSystem
                || curBits != s_lastBits)
            {
                s_lastDeviceID = curDev;
                s_lastSerial = hm.trackerSerial;
                s_lastModel = hm.trackerModel;
                s_lastSystem = hm.trackerTrackingSystem;
                s_lastBits = curBits;
                char lbuf[320];
                std::snprintf(lbuf, sizeof lbuf,
                    "[head-mount-status] deviceID=%d inRange=%d poseIsValid=%d"
                    " serial='%s' model='%s' system='%s' targetID=%d state=%d",
                    (int)curDev, (int)inRange, (int)poseValid,
                    hm.trackerSerial.c_str(), hm.trackerModel.c_str(),
                    hm.trackerTrackingSystem.c_str(),
                    (int)CalCtx.targetID, (int)CalCtx.state);
                Metrics::WriteLogAnnotation(lbuf);
            }
        }

        const bool trackerValid =
            hm.deviceID >= 0
            && (uint32_t)hm.deviceID < vr::k_unMaxTrackedDeviceCount
            && CalCtx.devicePoses[hm.deviceID].poseIsValid;

        if (!hasTracker) {
            ImGui::TextDisabled("No tracker selected.");
        } else if (!trackerValid) {
            openvr_pair::overlay::ui::DrawStatusDot(pal.dotError);
            ImGui::TextColored(pal.statusError, "Tracker not reporting a valid pose.");
        } else if (!offsetOk) {
            openvr_pair::overlay::ui::DrawStatusDot(pal.dotPending);
            ImGui::TextColored(pal.statusWarn, "Offset uncalibrated.");
        } else if (hm.mode == HeadMountMode::Off) {
            openvr_pair::overlay::ui::DrawStatusDot(pal.dotPending);
            ImGui::TextColored(pal.statusWarn, "Ready. Pick a mode above to activate.");
        } else {
            const double residualMm = (Metrics::headMountResidualMm.size() > 0)
                ? Metrics::headMountResidualMm.last() : 0.0;
            char buf[128];
            if (residualMm > 0.0) {
                std::snprintf(buf, sizeof buf,
                    "Active. Offset residual %.2f mm.", residualMm);
            } else {
                std::snprintf(buf, sizeof buf, "Active.");
            }
            openvr_pair::overlay::ui::DrawStatusDot(pal.dotOk);
            ImGui::TextColored(pal.statusOk, "%s", buf);
        }
    }

    }
}

// ---------------------------------------------------------------------------
// Section B: Safety boundary
// ---------------------------------------------------------------------------

wkopenvr::boundary::CaptureSession s_capture;
std::string s_boundaryError;
std::string s_guardianError;

void DrawPolygonPreview(const std::vector<BoundaryVertex>& verts) {
    if (verts.size() < 2) return;

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
    const double drawSpan = range + pad * 2.0;

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

    for (size_t i = 0; i < verts.size(); ++i) {
        const auto& a = verts[i];
        const auto& b = verts[(i + 1) % verts.size()];
        dl->AddLine(toCanvas(a.x, a.z), toCanvas(b.x, b.z), IM_COL32(0, 200, 100, 255), 1.5f);
    }
    for (const auto& v : verts) {
        dl->AddCircleFilled(toCanvas(v.x, v.z), 3.0f, IM_COL32(0, 200, 100, 255));
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
                s_boundaryError.clear();
            }
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Aim the controller at the floor. The pointer ray paints floor points while the trigger is held.");
            }
        } else {
            if (ImGui::Button("Re-draw")) {
                s_capture.Start();
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
        ImGui::Spacing();
        if (ImGui::Button("Done##bnd_done")) {
            s_capture.Finish();
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel##bnd_cancel")) {
            s_capture.Cancel();
        }
    } else { // Finished
        const auto& verts = s_capture.vertices();
        const auto pts = wkopenvr::boundary::ProjectXZ(verts);
        const double area = wkopenvr::boundary::AbsoluteAreaXZ(pts);

        ImGui::Text("%d vertices, %.2f m^2", (int)verts.size(), area);
        ImGui::TextDisabled("Cleaned to edge vertices.");

        DrawPolygonPreview(verts);

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
                    s_boundaryError.clear();
                }
            }
        }
        ImGui::SameLine();
        if (ImGui::Button("Discard##bnd_discard_fin")) {
            s_capture.Cancel();
        }
    }

    if (!s_boundaryError.empty()) {
        ImGui::Spacing();
        openvr_pair::overlay::ui::DrawErrorBanner("Boundary error", s_boundaryError.c_str());
    }

    }
}

// ---------------------------------------------------------------------------
// Section C: Quest Guardian
// ---------------------------------------------------------------------------

// Setup wizard modal state.
bool s_showWizard = false;
bool s_awaitPolarityConfirm = false;
char s_wifiHostPort[64] = {};
char s_wifiCode[16] = {};

wkopenvr::adb::SetupWizard* WizardPtr() {
    static wkopenvr::adb::SetupWizard inst(CCal_GetAdb());
    return &inst;
}

// Wizard modal -- single-current-step layout. The full 8-step list is collapsed
// behind a small "Show all steps" disclosure so users don't get visually
// dog-piled by [x] indicators before they have done anything.
void DrawSetupWizardModal() {
    if (!s_showWizard) return;

    ImGui::SetNextWindowSize(ImVec2(760.0f, 560.0f), ImGuiCond_Appearing);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(20.0f, 18.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(12.0f, 8.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(10.0f, 10.0f));
    if (ImGui::BeginPopupModal("Connect to Quest##wiz", &s_showWizard, 0)) {
        ImGui::SetWindowFontScale(1.18f);

        auto* wiz = WizardPtr();
        const auto& pal = openvr_pair::overlay::ui::GetPalette();
        const float actionHeight = ImGui::GetTextLineHeightWithSpacing() * 1.45f;
        const ImVec2 actionButtonSize(260.0f, actionHeight);
        auto ActionButton = [&](const char* label) {
            return ImGui::Button(label, actionButtonSize);
        };

        struct StepInfo { wkopenvr::adb::WizardStep step; const char* label; const char* help; };
        static const StepInfo kSteps[] = {
            { wkopenvr::adb::WizardStep::CheckBinary,    "ADB binary",
              "Verify the bundled adb.exe can run." },
            { wkopenvr::adb::WizardStep::CheckDevAccount,"Meta developer account",
              "In the Meta Horizon app, open headset settings and turn Developer Mode on." },
            { wkopenvr::adb::WizardStep::CheckDevMode,   "USB authorization",
              "Connect a USB-C data cable. In-headset, open Settings > Developer, turn on MTP Notification, then accept 'Allow USB debugging?'." },
            { wkopenvr::adb::WizardStep::UsbPair,        "USB pairing",
              "Confirm the Quest is paired over USB." },
            { wkopenvr::adb::WizardStep::WifiTcpip,      "Enable Wi-Fi ADB",
              "Switch the Quest into Wi-Fi ADB mode (adb tcpip 5555)." },
            { wkopenvr::adb::WizardStep::WifiDiscover,   "Discover Quest IP",
              "Read the Quest's Wi-Fi IP via the USB connection." },
            { wkopenvr::adb::WizardStep::WifiPair,       "Wi-Fi pair",
              "Unplug USB. On the Quest, open Settings > System > Developer > Wireless ADB. Enter the host:port and 6-digit code shown on-screen." },
            { wkopenvr::adb::WizardStep::WifiVerify,     "Verify Wi-Fi",
              "Connect over Wi-Fi and probe the Guardian property." },
        };

        // Find current step's index for the progress line.
        const wkopenvr::adb::WizardStep cur = wiz->currentStep();
        int curIdx = 0;
        for (int i = 0; i < (int)(sizeof kSteps / sizeof kSteps[0]); ++i) {
            if (kSteps[i].step == cur) { curIdx = i; break; }
        }

        if (!wiz->IsDone()) {
            ImGui::Text("Step %d of %d", curIdx + 1, (int)(sizeof kSteps / sizeof kSteps[0]));
            ImGui::SameLine();
            ImGui::TextDisabled("--");
            ImGui::SameLine();
            ImGui::TextUnformatted(kSteps[curIdx].label);
            ImGui::Separator();
            ImGui::Spacing();
            ImGui::TextWrapped("%s", kSteps[curIdx].help);
            ImGui::Spacing();

            // Per-step input + run button.
            switch (cur) {
            case wkopenvr::adb::WizardStep::Start:
            case wkopenvr::adb::WizardStep::CheckBinary:
                if (ActionButton("Check##binary")) wiz->RunCheckBinary();
                break;
            case wkopenvr::adb::WizardStep::CheckDevAccount:
                if (ActionButton("Check##devacct")) wiz->RunCheckDevAccount();
                break;
            case wkopenvr::adb::WizardStep::CheckDevMode:
                if (ActionButton("Check##devmode")) wiz->RunCheckDevMode();
                break;
            case wkopenvr::adb::WizardStep::UsbPair:
                if (ActionButton("Confirm##usbpair")) wiz->RunUsbPair();
                break;
            case wkopenvr::adb::WizardStep::WifiTcpip:
                if (ActionButton("Enable Wi-Fi ADB##tcpip")) wiz->RunWifiTcpip();
                break;
            case wkopenvr::adb::WizardStep::WifiDiscover:
                if (ActionButton("Discover##disc")) wiz->RunWifiDiscover();
                break;
            case wkopenvr::adb::WizardStep::WifiPair:
                ImGui::SetNextItemWidth(420.0f);
                ImGui::InputText("Host:port##wiz_hp", s_wifiHostPort, sizeof(s_wifiHostPort));
                ImGui::SetNextItemWidth(180.0f);
                ImGui::InputText("Code##wiz_code",   s_wifiCode,     sizeof(s_wifiCode));
                if (ActionButton("Pair##pair")) {
                    wiz->RunWifiPair(s_wifiHostPort, s_wifiCode);
                }
                break;
            case wkopenvr::adb::WizardStep::WifiVerify:
                if (ActionButton("Verify##verify")) {
                    auto res = wiz->RunWifiVerify();
                    if (res.status == wkopenvr::adb::StepStatus::Passed) {
                        wkopenvr::adb::ProbeGuardianPolarity(CCal_GetAdb());
                        const std::string ep = wiz->DiscoveredEndpoint();
                        if (!ep.empty()) {
                            CalCtx.adb.savedEndpoint = ep;
                            SaveProfile(CalCtx);
                        }
                        s_awaitPolarityConfirm = true;
                    }
                }
                break;
            default:
                break;
            }

            // Show current step's failure detail right where the action button is.
            const auto res = wiz->stepResult(cur);
            if (res.status == wkopenvr::adb::StepStatus::Failed && !res.detail.empty()) {
                ImGui::Spacing();
                ImGui::PushStyleColor(ImGuiCol_Text, pal.statusError);
                ImGui::TextWrapped("%s", res.detail.c_str());
                ImGui::PopStyleColor();
            }
        } else if (s_awaitPolarityConfirm) {
            // Polarity confirmation. The setprop value semantic flipped at
            // some point in Meta's runtime; the only reliable signal is the
            // user telling us whether the boundary actually disappeared.
            ImGui::TextWrapped("Did Quest Guardian visibly disappear in-headset just now?");
            ImGui::Spacing();
            if (ImGui::Button("Yes, Guardian disappeared", actionButtonSize)) {
                CalCtx.adb.setupCompleted = true;
                SaveProfile(CalCtx);
                s_awaitPolarityConfirm = false;
                s_showWizard = false;
            }
            ImGui::SameLine();
            if (ImGui::Button("No, flip the value", actionButtonSize)) {
                wkopenvr::adb::SetGuardianPauseValueOverride(CCal_GetAdb(),
                    CalCtx.adb.guardianPauseValue == 1 ? 0 : 1);
                CalCtx.adb.setupCompleted = true;
                SaveProfile(CalCtx);
                s_awaitPolarityConfirm = false;
                s_showWizard = false;
            }
        } else {
            ImGui::TextColored(pal.statusOk, "Setup complete.");
            if (ImGui::Button("Close", actionButtonSize)) { s_showWizard = false; }
        }

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        // Collapsible "Show all steps" view -- legacy checklist for users
        // who want to see where they are in the overall sequence.
        if (ImGui::TreeNode("Show all steps")) {
            for (const auto& si : kSteps) {
                const auto res = wiz->stepResult(si.step);
                switch (res.status) {
                case wkopenvr::adb::StepStatus::Passed:
                    ImGui::TextColored(pal.statusOk, "[+] %s", si.label);
                    break;
                case wkopenvr::adb::StepStatus::Failed:
                    ImGui::TextColored(pal.statusError, "[x] %s", si.label);
                    break;
                case wkopenvr::adb::StepStatus::InProgress:
                    ImGui::TextColored(pal.statusWarn, "[~] %s", si.label);
                    break;
                default:
                    ImGui::TextDisabled("[ ] %s", si.label);
                    break;
                }
                if (!res.detail.empty()) {
                    ImGui::Indent();
                    ImGui::PushStyleColor(ImGuiCol_Text,
                        ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
                    ImGui::TextWrapped("%s", res.detail.c_str());
                    ImGui::PopStyleColor();
                    ImGui::Unindent();
                }
            }
            ImGui::TreePop();
        }

        if (!wiz->IsDone()) {
            ImGui::Spacing();
            if (ImGui::Button("Reset", ImVec2(120.0f, actionHeight * 0.85f))) wiz->Reset();
        }

        ImGui::EndPopup();
    }
    ImGui::PopStyleVar(3);
}

void DrawGuardianSection(ImVec2 panelSize) {
    {
    openvr_pair::overlay::ui::PanelScope panel("Step 3: Quest Guardian", panelSize);
    const auto& pal = openvr_pair::overlay::ui::GetPalette();

    ImGui::TextWrapped(
        "Pause Quest's own Guardian so it doesn't fight your drawn boundary in-headset.");
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip(
            "Requires ADB connectivity to the Quest. Guardian is paused via a Meta\n"
            "runtime property; nothing is installed on the headset. Re-enable any time.");
    }
    ImGui::Spacing();

    const bool adbConn = Metrics::adbConnected.last();
    const bool guardianPaused = Metrics::guardianPaused.last();
    const bool hasBoundary = CalCtx.boundary.enabled && CalCtx.boundary.vertices.size() >= 3;

    // Single-row status: ADB dot + Guardian dot side by side.
    {
        openvr_pair::overlay::ui::DrawStatusDot(adbConn ? pal.dotOk : pal.dotError);
        ImGui::TextUnformatted(adbConn ? "ADB connected" : "ADB not connected");
        if (!adbConn && !CalCtx.adb.savedEndpoint.empty()) {
            ImGui::SameLine();
            ImGui::TextDisabled("(saved endpoint: %s)", CalCtx.adb.savedEndpoint.c_str());
        }
        ImGui::SameLine();
        ImGui::TextDisabled("  |  ");
        ImGui::SameLine();
        if (!adbConn) {
            openvr_pair::overlay::ui::DrawStatusDot(pal.dotPending);
            ImGui::TextDisabled("Guardian: unknown");
        } else if (guardianPaused) {
            openvr_pair::overlay::ui::DrawStatusDot(pal.dotOk);
            ImGui::TextUnformatted("Guardian: paused");
        } else {
            openvr_pair::overlay::ui::DrawStatusDot(pal.dotPending);
            ImGui::TextUnformatted("Guardian: active");
        }
    }

    ImGui::Spacing();

    // Progressive disclosure: show only the action that's currently meaningful.
    if (!CalCtx.adb.setupCompleted || !adbConn) {
        // Path 1: ADB setup not done. Big primary button.
        if (ImGui::Button("Connect to Quest via ADB...")) {
            WizardPtr()->Reset();
            s_awaitPolarityConfirm = false;
            s_showWizard = true;
            ImGui::OpenPopup("Connect to Quest##wiz");
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Walks through Meta developer-mode + USB + Wi-Fi pairing.");
        }
        if (CalCtx.adb.setupCompleted && !adbConn) {
            ImGui::SameLine();
            ImGui::TextDisabled("(setup ran previously; reconnect to retry)");
        }
    } else if (!hasBoundary) {
        // Path 2: ADB ready, but no boundary -- nudge user upward.
        ImGui::TextDisabled("Draw and enable a boundary above before pausing Guardian.");
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip(
                "Pausing Quest's Guardian without a replacement boundary is a safety\n"
                "regression. WKOpenVR refuses to do this until the Safety boundary\n"
                "above is active.");
        }
    } else if (!guardianPaused) {
        // Path 3: Ready to pause.
        if (ImGui::Button("Pause Quest Guardian")) {
            if (!wkopenvr::adb::ApplyGuardianPauseSetting(CCal_GetAdb(), true)) {
                s_guardianError = "Failed to pause Guardian. Check ADB connection.";
            } else {
                CalCtx.adb.guardianPauseEnabled = true;
                SaveProfile(CalCtx);
                s_guardianError.clear();
            }
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Sets debug.oculus.guardian_pause on the Quest so Guardian disappears in-headset.");
        }
    } else {
        // Path 4: Currently paused -- single resume button.
        if (ImGui::Button("Resume Quest Guardian")) {
            if (!wkopenvr::adb::ApplyGuardianPauseSetting(CCal_GetAdb(), false)) {
                s_guardianError = "Failed to resume Guardian.";
            } else {
                CalCtx.adb.guardianPauseEnabled = false;
                SaveProfile(CalCtx);
                s_guardianError.clear();
            }
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Clears the pause property so Guardian becomes visible in-headset again.");
        }
    }

    // Always offer a "Re-run setup" link once a setup completed (e.g. user
    // moved house and Wi-Fi changed). Small button, secondary.
    if (CalCtx.adb.setupCompleted) {
        ImGui::SameLine();
        if (ImGui::SmallButton("Re-run setup##rerun")) {
            WizardPtr()->Reset();
            s_awaitPolarityConfirm = false;
            s_showWizard = true;
            ImGui::OpenPopup("Connect to Quest##wiz");
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Walk through the ADB pairing again, e.g. after a Wi-Fi change.");
        }
    }

    if (!s_guardianError.empty()) {
        ImGui::Spacing();
        openvr_pair::overlay::ui::DrawErrorBanner("Guardian error", s_guardianError.c_str());
    }

    }
}

} // namespace

// ---------------------------------------------------------------------------
// Capture tick -- called each CalibrationTick so capture runs regardless of
// which tab is visible.
// ---------------------------------------------------------------------------

void CCal_TickBoundaryCapture() {
    if (s_capture.state() != wkopenvr::boundary::CaptureState::Active)
        return;

    auto* vrs = vr::VRSystem();
    if (!vrs) return;

    for (int i = 0; i < (int)CalCtx.MAX_CONTROLLERS; ++i) {
        const int ctrlId = CalCtx.controllerIDs[i];
        if (ctrlId < 0) {
            continue;
        }

        vr::VRControllerState_t st = {};
        if (!vrs->GetControllerState(ctrlId, &st, sizeof(st))) {
            continue;
        }
        const bool triggerHeld =
            (st.ulButtonPressed & vr::ButtonMaskFromId(vr::k_EButton_SteamVR_Trigger)) != 0 ||
            st.rAxis[vr::k_eControllerAxis_Trigger].x > 0.75f ||
            st.rAxis[vr::k_eControllerAxis_TrackPad].x > 0.75f;
        if (!triggerHeld) {
            continue;
        }

        const auto& dp = CalCtx.devicePoses[ctrlId];
        if (!dp.poseIsValid || dp.result != vr::ETrackingResult::TrackingResult_Running_OK) {
            continue;
        }

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
        Eigen::Vector3d pos(dp.vecPosition[0], dp.vecPosition[1], dp.vecPosition[2]);
        Eigen::Affine3d pose = Eigen::Affine3d::Identity();
        pose.translate(wfd_t + wfd * pos);
        pose.rotate(wfd * rot);

        s_capture.Tick(pose, true, CalCtx.boundary.floorY);
        return;
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
        "Continuous-mode headset tracker, lighthouse boundary, and Quest Guardian pause.");
    ImGui::Spacing();

    DrawHeadMountSection(panelSize);
    ImGui::Spacing();
    DrawBoundarySection(panelSize);
    ImGui::Spacing();
    DrawGuardianSection(panelSize);

    // Wizard modal: rendered in the same stack frame as the OpenPopup call.
    DrawSetupWizardModal();
}
