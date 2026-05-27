#include "Calibration.h"
#include "CalibrationMetrics.h"
#include "Configuration.h"
#include "ControllerInput.h"
#include "DiagnosticsLog.h"
#include "Boundary.h"
#include "BoundaryCapture.h"
#include "GuardianAutoApply.h"
#include "SpaceCalibratorUmbrellaRuntime.h"
#include "UserInterfaceHeadMount.h"
#include "UiHelpers.h"

#include <AdbSetupWizard.h>

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
//   3. Hide Quest's overlapping guardian.

namespace {

// ---------------------------------------------------------------------------
// Section B: Safety boundary
// ---------------------------------------------------------------------------

wkopenvr::boundary::CaptureSession s_capture;
std::string s_boundaryError;
std::string s_guardianError;
std::string s_adbCleanupStatus;
bool s_adbCleanupHadWarning = false;

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
bool s_showAdbCleanupConfirm = false;
char s_wifiHostPort[64] = {};
char s_wifiCode[16] = {};
char s_wifiConnectEndpoint[64] = {};

wkopenvr::adb::SetupWizard* WizardPtr() {
    static wkopenvr::adb::SetupWizard inst(CCal_GetAdb());
    return &inst;
}

void RemoveAdbSetupFromQuest() {
    AdbController& adb = CCal_GetAdb();
    const std::string endpoint = CalCtx.adb.savedEndpoint;
    const bool hadEndpoint = !endpoint.empty();

    bool connectOk = !hadEndpoint;
    if (hadEndpoint) {
        connectOk = adb.Connect(endpoint);
    }

    bool guardianCleared = false;
    if (connectOk) {
        guardianCleared = adb.SetGuardianPaused(false, 0);
    }

    const bool wirelessDisabled = adb.DisableWirelessAdb(endpoint);
    const bool disconnected = adb.Disconnect(endpoint);

    CalCtx.adb.setupCompleted = false;
    CalCtx.adb.savedEndpoint.clear();
    CalCtx.adb.guardianPauseEnabled = false;
    SaveProfile(CalCtx);

    const bool remoteConfirmed = (!hadEndpoint || connectOk) && guardianCleared && wirelessDisabled;
    s_adbCleanupHadWarning = !remoteConfirmed;
    if (remoteConfirmed) {
        s_adbCleanupStatus =
            "ADB setup removed. Quest Guardian was resumed, Wi-Fi ADB was switched back to USB mode, and the saved endpoint was cleared.";
    } else {
        s_adbCleanupStatus =
            "Local ADB setup was cleared, but the headset did not confirm every cleanup step. If you do not want ADB available, disable Developer Mode in the Meta Horizon app or headset settings.";
    }
    s_guardianError.clear();
    Metrics::adbConnected.Push(false);
    Metrics::guardianPaused.Push(false);

    fprintf(stderr,
        "[adb-ui] remove setup: endpoint='%s' connect=%d guardian_clear=%d usb=%d disconnect=%d remote_confirmed=%d\n",
        endpoint.c_str(), connectOk ? 1 : 0, guardianCleared ? 1 : 0,
        wirelessDisabled ? 1 : 0, disconnected ? 1 : 0, remoteConfirmed ? 1 : 0);
}

// Wizard modal -- single-current-step layout. The full 8-step list is collapsed
// behind a small "Show all steps" disclosure so users don't get visually
// dog-piled by [x] indicators before they have done anything.
void DrawSetupWizardModal() {
    if (!s_showWizard) return;

    ImGui::SetNextWindowSize(ImVec2(980.0f, 740.0f), ImGuiCond_Appearing);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(20.0f, 18.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(12.0f, 8.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(10.0f, 10.0f));
    if (ImGui::BeginPopupModal("Connect to Quest##wiz", &s_showWizard, 0)) {
        ImGui::SetWindowFontScale(1.22f);

        auto* wiz = WizardPtr();
        const auto& pal = openvr_pair::overlay::ui::GetPalette();
        const float actionHeight = ImGui::GetTextLineHeightWithSpacing() * 1.45f;
        const ImVec2 actionButtonSize(360.0f, actionHeight);
        auto ActionButton = [&](const char* label) {
            return ImGui::Button(label, actionButtonSize);
        };
        auto FullWidthInputText = [](const char* visibleLabel, const char* id, char* buffer, size_t bufferSize) {
            ImGui::TextWrapped("%s", visibleLabel);
            ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x);
            return ImGui::InputText(id, buffer, bufferSize);
        };

        ImGui::BeginChild("adb_setup_scroll_region", ImVec2(0.0f, 0.0f), false,
            ImGuiWindowFlags_AlwaysVerticalScrollbar);

        openvr_pair::overlay::ui::DrawBanner(
            "ADB warning",
            "ADB authorization gives this PC debugging access to the Quest while it is trusted. Continue only with your own unlocked headset, and remove ADB setup when you are done if you do not want Wi-Fi debugging left available.",
            pal.bannerWarnBg, pal.bannerWarnTitle, pal.bannerWarnDetail);
        ImGui::Spacing();

        struct StepInfo { wkopenvr::adb::WizardStep step; const char* label; const char* help; };
        static const StepInfo kSteps[] = {
            { wkopenvr::adb::WizardStep::CheckBinary,    "ADB binary",
              "Verify the bundled adb.exe can run." },
            { wkopenvr::adb::WizardStep::CheckDevAccount,"Meta developer account",
              "In the Meta Horizon app, open headset settings and turn Developer Mode on." },
            { wkopenvr::adb::WizardStep::CheckDevMode,   "USB authorization",
              "Connect a USB-C data cable, unlock the headset, then accept 'Allow USB debugging?'. MTP Notification is only the file-transfer notification; it does not authorize ADB." },
            { wkopenvr::adb::WizardStep::UsbPair,        "USB pairing",
              "Confirm the Quest is paired over USB." },
            { wkopenvr::adb::WizardStep::WifiTcpip,      "Enable Wi-Fi ADB",
              "Switch the Quest into Wi-Fi ADB mode (adb tcpip 5555)." },
            { wkopenvr::adb::WizardStep::WifiDiscover,   "Discover Quest IP",
              "Read the Quest's Wi-Fi IP via the USB connection. This normal Quest path does not need a pairing code." },
            { wkopenvr::adb::WizardStep::WifiVerify,     "Verify Wi-Fi",
              "Connect over Wi-Fi and probe the Guardian property." },
        };
        static const StepInfo kManualWirelessSteps[] = {
            { wkopenvr::adb::WizardStep::WifiPair,       "Manual wireless pair",
              "Only use this if the Quest shows an Android Wireless debugging pairing-code screen. Many Quest builds do not expose it; the USB-authorized path above is the expected flow." },
            { wkopenvr::adb::WizardStep::WifiVerify,     "Verify Wi-Fi",
              "Connect over Wi-Fi and probe the Guardian property." },
        };

        // Find current step's index for the progress line.
        const wkopenvr::adb::WizardStep cur = wiz->currentStep();
        const bool manualWirelessFlow =
            cur == wkopenvr::adb::WizardStep::WifiPair ||
            (cur == wkopenvr::adb::WizardStep::WifiVerify && wiz->DiscoveredEndpoint().empty());
        const StepInfo* steps = manualWirelessFlow ? kManualWirelessSteps : kSteps;
        const int stepCount = manualWirelessFlow
            ? (int)(sizeof kManualWirelessSteps / sizeof kManualWirelessSteps[0])
            : (int)(sizeof kSteps / sizeof kSteps[0]);
        int curIdx = 0;
        for (int i = 0; i < stepCount; ++i) {
            if (steps[i].step == cur) { curIdx = i; break; }
        }

        if (!wiz->IsDone()) {
            ImGui::TextWrapped("Step %d of %d - %s", curIdx + 1, stepCount, steps[curIdx].label);
            ImGui::Separator();
            ImGui::Spacing();
            ImGui::TextWrapped("%s", steps[curIdx].help);
            ImGui::Spacing();

            if (cur == wkopenvr::adb::WizardStep::WifiTcpip ||
                cur == wkopenvr::adb::WizardStep::WifiPair ||
                cur == wkopenvr::adb::WizardStep::WifiVerify) {
                openvr_pair::overlay::ui::DrawBanner(
                    "Wireless ADB",
                    "Wireless ADB opens a debugging endpoint on the headset's network. Use Remove ADB setup from the Guardian panel when finished, or disable Developer Mode in Meta settings.",
                    pal.bannerWarnBg, pal.bannerWarnTitle, pal.bannerWarnDetail);
                ImGui::Spacing();
            }

            // Per-step input + run button.
            switch (cur) {
            case wkopenvr::adb::WizardStep::Start:
            case wkopenvr::adb::WizardStep::CheckBinary:
                if (ActionButton("Check##binary")) wiz->RunCheckBinary();
                if (Metrics::adbConnected.last()) {
                    ImGui::Spacing();
                    ImGui::TextDisabled("ADB is already connected.");
                    if (ActionButton("Use current ADB connection##use_current_adb")) {
                        wiz->UseCurrentAdbConnection();
                    }
                }
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
                FullWidthInputText("Pairing host:port", "##wiz_hp",
                    s_wifiHostPort, sizeof(s_wifiHostPort));
                ImGui::TextWrapped("Pairing code");
                ImGui::SetNextItemWidth(220.0f);
                ImGui::InputText("##wiz_code", s_wifiCode, sizeof(s_wifiCode));
                if (wiz->DiscoveredEndpoint().empty()) {
                    FullWidthInputText("Connect endpoint", "##wiz_ep",
                        s_wifiConnectEndpoint, sizeof(s_wifiConnectEndpoint));
                    if (ImGui::IsItemHovered()) {
                        ImGui::SetTooltip("The endpoint from the Wireless ADB screen, usually IP:port. This is not always the same port as the pairing host:port.");
                    }
                } else {
                    ImGui::PushStyleColor(ImGuiCol_Text,
                        ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
                    ImGui::TextWrapped("Connect endpoint: %s", wiz->DiscoveredEndpoint().c_str());
                    ImGui::PopStyleColor();
                }
                if (ActionButton("Pair##pair")) {
                    wiz->RunWifiPair(s_wifiHostPort, s_wifiCode);
                }
                break;
            case wkopenvr::adb::WizardStep::WifiVerify:
                if (wiz->DiscoveredEndpoint().empty()) {
                    FullWidthInputText("Connect endpoint", "##wiz_verify_ep",
                        s_wifiConnectEndpoint, sizeof(s_wifiConnectEndpoint));
                } else {
                    ImGui::PushStyleColor(ImGuiCol_Text,
                        ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
                    ImGui::TextWrapped("Connect endpoint: %s", wiz->DiscoveredEndpoint().c_str());
                    ImGui::PopStyleColor();
                }
                if (ActionButton("Verify##verify")) {
                    const std::string manualEndpoint =
                        wiz->DiscoveredEndpoint().empty()
                            ? std::string(s_wifiConnectEndpoint)
                            : std::string();
                    auto res = wiz->RunWifiVerify(manualEndpoint);
                    if (res.status == wkopenvr::adb::StepStatus::Passed) {
                        const auto polarity =
                            wkopenvr::adb::ProbeGuardianPolarity(CCal_GetAdb());
                        {
                            char pbuf[160];
                            snprintf(pbuf, sizeof pbuf,
                                "[adb-wizard-ui] guardian probe complete: wrote=%d read=%d match=%d",
                                polarity.writtenValue,
                                polarity.readBackValue,
                                polarity.readMatchesWrite ? 1 : 0);
                            Metrics::WriteLogAnnotation(pbuf);
                            openvr_pair::common::DiagnosticLog("adb-wizard-ui", "%s", pbuf);
                        }
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
            if ((cur == wkopenvr::adb::WizardStep::CheckDevMode ||
                 cur == wkopenvr::adb::WizardStep::UsbPair) &&
                res.status == wkopenvr::adb::StepStatus::Failed) {
                ImGui::Spacing();
                ImGui::TextDisabled("No USB prompt?");
                ImGui::TextWrapped("Retry restarts the ADB server and checks the headset state. The manual wireless fallback only applies if the headset shows Android Wireless debugging pairing details.");
                if (ImGui::Button("Try to show USB prompt##retry_usb", actionButtonSize)) {
                    wiz->RunRetryUsbPrompt();
                }
                if (ImGui::GetContentRegionAvail().x >= actionButtonSize.x + ImGui::GetStyle().ItemSpacing.x) {
                    ImGui::SameLine();
                }
                if (ImGui::Button("Manual pairing-code fallback##wireless_fallback", actionButtonSize)) {
                    wiz->UseWirelessFallback();
                }
            }
        } else if (s_awaitPolarityConfirm) {
            // Polarity confirmation. The setprop value semantic flipped at
            // some point in Meta's runtime; the only reliable signal is the
            // user telling us whether the boundary actually disappeared.
            ImGui::TextWrapped("Did Quest Guardian visibly disappear in-headset just now?");
            ImGui::Spacing();
            if (ImGui::Button("Yes, Guardian disappeared", actionButtonSize)) {
                Metrics::WriteLogAnnotation("[adb-wizard-ui] guardian confirmation: manual_confirmed_disappeared");
                openvr_pair::common::DiagnosticLog(
                    "adb-wizard-ui", "guardian confirmation manual_confirmed_disappeared value=%d",
                    CalCtx.adb.guardianPauseValue);
                wkopenvr::adb::RecordGuardianPausedConfirmation("wizard_probe");
                CalCtx.adb.setupCompleted = true;
                SaveProfile(CalCtx);
                s_awaitPolarityConfirm = false;
                s_showWizard = false;
            }
            if (ImGui::GetContentRegionAvail().x >= actionButtonSize.x + ImGui::GetStyle().ItemSpacing.x) {
                ImGui::SameLine();
            }
            if (ImGui::Button("No, flip the value", actionButtonSize)) {
                Metrics::WriteLogAnnotation("[adb-wizard-ui] guardian confirmation: manual_requested_flip");
                openvr_pair::common::DiagnosticLog(
                    "adb-wizard-ui", "guardian confirmation manual_requested_flip old_value=%d new_value=%d",
                    CalCtx.adb.guardianPauseValue,
                    CalCtx.adb.guardianPauseValue == 1 ? 0 : 1);
                const bool confirmed = wkopenvr::adb::SetGuardianPauseValueOverride(CCal_GetAdb(),
                    CalCtx.adb.guardianPauseValue == 1 ? 0 : 1);
                if (confirmed) {
                    wkopenvr::adb::RecordGuardianPausedConfirmation("wizard_flip");
                    CalCtx.adb.setupCompleted = true;
                    SaveProfile(CalCtx);
                    s_awaitPolarityConfirm = false;
                    s_showWizard = false;
                } else {
                    s_guardianError = "Guardian did not confirm after flipping the pause value.";
                }
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
            if (ImGui::Button("Reset", ImVec2(120.0f, actionHeight * 0.85f))) {
                wiz->Reset();
                s_wifiHostPort[0] = '\0';
                s_wifiCode[0] = '\0';
                s_wifiConnectEndpoint[0] = '\0';
            }
        }

        ImGui::EndChild();
        ImGui::EndPopup();
    }
    ImGui::PopStyleVar(3);
}

void DrawAdbCleanupConfirmModal() {
    if (!s_showAdbCleanupConfirm) return;

    ImGui::SetNextWindowSize(ImVec2(680.0f, 300.0f), ImGuiCond_Appearing);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(18.0f, 16.0f));
    if (ImGui::BeginPopupModal("Remove ADB setup##adb_cleanup",
                               &s_showAdbCleanupConfirm, 0)) {
        const auto& pal = openvr_pair::overlay::ui::GetPalette();
        const float actionHeight = ImGui::GetTextLineHeightWithSpacing() * 1.35f;
        const ImVec2 actionButtonSize(250.0f, actionHeight);

        openvr_pair::overlay::ui::DrawBanner(
            "Remove ADB setup",
            "This resumes Quest Guardian, asks the headset to leave Wi-Fi ADB mode, disconnects the saved endpoint, and clears WKOpenVR's saved ADB setup. No app is installed by this feature.",
            pal.bannerWarnBg, pal.bannerWarnTitle, pal.bannerWarnDetail);
        ImGui::Spacing();

        if (ImGui::Button("Remove setup##confirm_adb_cleanup", actionButtonSize)) {
            RemoveAdbSetupFromQuest();
            s_showAdbCleanupConfirm = false;
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel##cancel_adb_cleanup", actionButtonSize)) {
            s_showAdbCleanupConfirm = false;
            ImGui::CloseCurrentPopup();
        }

        ImGui::EndPopup();
    }
    ImGui::PopStyleVar();
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

    if (CalCtx.adb.setupCompleted || !CalCtx.adb.savedEndpoint.empty() ||
        CalCtx.adb.guardianPauseEnabled) {
        ImGui::SameLine();
        if (ImGui::SmallButton("Remove ADB setup##remove_adb_setup")) {
            s_showAdbCleanupConfirm = true;
            ImGui::OpenPopup("Remove ADB setup##adb_cleanup");
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip(
                "Resume Guardian, turn off Wi-Fi ADB when reachable, disconnect the saved endpoint, and clear the saved setup.");
        }
    }

    if (!s_adbCleanupStatus.empty()) {
        ImGui::Spacing();
        if (s_adbCleanupHadWarning) {
            openvr_pair::overlay::ui::DrawBanner(
                "ADB cleanup", s_adbCleanupStatus.c_str(),
                pal.bannerWarnBg, pal.bannerWarnTitle, pal.bannerWarnDetail);
        } else {
            openvr_pair::overlay::ui::DrawInfoBanner("ADB cleanup", s_adbCleanupStatus.c_str());
        }
    }

    if (!s_guardianError.empty()) {
        ImGui::Spacing();
        openvr_pair::overlay::ui::DrawErrorBanner("Guardian error", s_guardianError.c_str());
    }

    DrawAdbCleanupConfirmModal();

    }
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

        const Eigen::Affine3d pose = DriverPoseToWorldAffine(dp);
        const bool accepted = s_capture.Tick(pose, true, CalCtx.boundary.floorY);
        if (accepted) {
            char cbuf[256];
            snprintf(cbuf, sizeof cbuf,
                "[boundary-capture] accepted controller input: session=%llu device=%d system='%s' raw=%zu button=%d legacy=%d axis=%d value=%.3f fallback_any=%d",
                static_cast<unsigned long long>(sessionId),
                static_cast<int>(deviceId),
                stats.lastTrackingSystem.c_str(),
                s_capture.rawVertexCount(),
                trigger.buttonPressed ? 1 : 0,
                trigger.legacyFallbackUsed ? 1 : 0,
                trigger.analogAxis,
                trigger.analogValue,
                stats.fallbackAnySystem);
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
        return;
    }

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
        "Continuous-mode headset tracker, lighthouse boundary, and Quest Guardian pause.");
    ImGui::Spacing();

    CCal_DrawHeadMountSection(panelSize);
    ImGui::Spacing();
    DrawBoundarySection(panelSize);
    ImGui::Spacing();
    DrawGuardianSection(panelSize);

    // Wizard modal: rendered in the same stack frame as the OpenPopup call.
    DrawSetupWizardModal();
}
