#include "Calibration.h"
#include "Configuration.h"
#include "VRState.h"
#include "UiHelpers.h"

#include <string>
#include <openvr.h>
#include <imgui/imgui.h>
#include "imgui_extensions.h"

void SaveProfile(CalibrationContext& ctx);

static inline const char* GetPrettyTrackingSystemName(const std::string& value)
{
	// To comply with SteamVR branding guidelines (page 29), we rename devices under lighthouse tracking to SteamVR
	// Tracking.
	if (value == "lighthouse" || value == "aapvr") {
		return "SteamVR Tracking";
	}
	return value.c_str();
}

// Mirror of the "Reference HMD not detected" banner from BuildMenu, rendered
// inside the continuous-cal Status tab so the user sees it even when they jump
// straight into continuous mode. Returns true if the banner was drawn.
static bool DrawProfileMismatchBanner()
{
	if (!CalCtx.validProfile || CalCtx.enabled) return false;
	const char* refSystem = GetPrettyTrackingSystemName(CalCtx.referenceTrackingSystem);
	// The "actual" tracking system is whatever the current HMD reports -- fall
	// back to a plain "current HMD" wording when we can't read it cleanly.
	std::string actualSystem;
	if (auto vrSystem = vr::VRSystem()) {
		char buffer[vr::k_unMaxPropertyStringSize] = {0};
		vr::ETrackedPropertyError err = vr::TrackedProp_Success;
		vrSystem->GetStringTrackedDeviceProperty(vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_TrackingSystemName_String,
		                                         buffer, sizeof buffer, &err);
		if (err == vr::TrackedProp_Success && buffer[0] != 0) {
			actualSystem = GetPrettyTrackingSystemName(buffer);
		}
	}
	const auto& pal = openvr_pair::overlay::ui::GetPalette();
	ImGui::PushStyleColor(ImGuiCol_Text, pal.statusWarn);
	if (!actualSystem.empty()) {
		ImGui::TextWrapped("Profile expects %s HMD but current HMD is on %s. Calibration not applied.", refSystem,
		                   actualSystem.c_str());
	}
	else {
		ImGui::TextWrapped("Profile expects %s HMD but current HMD is unavailable or on a different tracking system. "
		                   "Calibration not applied.",
		                   refSystem);
	}
	ImGui::PopStyleColor();
	if (ImGui::Button("Clear profile")) {
		CalCtx.Clear();
		SaveProfile(CalCtx);
	}
	ImGui::SameLine();
	if (ImGui::Button("Recalibrate")) {
		// Same trigger BuildMenu uses for "Start Calibration" -- kicks the state
		// machine into Begin via StartCalibration. The popup isn't relevant here
		// since the user is already inside the continuous-cal window.
		StartCalibration("ui_inprogress_recal");
	}
	ImGui::Separator();
	return true;
}

void CCal_BasicInfo()
{
	ImVec2 panelSize{ImGui::GetWindowContentRegionMax().x - ImGui::GetWindowContentRegionMin().x, 0};

	// (The tip lives in the global footer, and the mismatch banner sits
	// below the Actions panel -- both stay visible without pushing the
	// device readout off-screen on small windows.)

	// --- Devices panel -----------------------------------------------------
	// The same reference + target table as before, but wrapped in a group
	// panel so it visually matches the panels in Advanced. The panel widget
	// owns its own width so we don't pass a non-zero panelSize here -- letting
	// the panel auto-fit its content prevents a stray right-edge gap when the
	// table is narrower than the window.
	openvr_pair::overlay::ui::DrawPanel(
	    "Devices",
	    [&] {
		    namespace ui = openvr_pair::overlay::ui;

		    const char* refTrackingSystem = GetPrettyTrackingSystemName(CalCtx.referenceStandby.trackingSystem);
		    const char* targetTrackingSystem = GetPrettyTrackingSystemName(CalCtx.targetStandby.trackingSystem);

		    const char* refStatus;
		    ui::StatusTone refTone;
		    if (CalCtx.referenceID < 0) {
			    refStatus = "NOT FOUND";
			    refTone = ui::StatusTone::Error;
		    }
		    else if (!CalCtx.ReferencePoseIsValidSimple()) {
			    refStatus = "NOT TRACKING";
			    refTone = ui::StatusTone::Warn;
		    }
		    else {
			    refStatus = "OK";
			    refTone = ui::StatusTone::Ok;
		    }

		    const char* targetStatus;
		    ui::StatusTone targetTone;
		    if (CalCtx.targetID < 0) {
			    targetStatus = "NOT FOUND";
			    targetTone = ui::StatusTone::Error;
		    }
		    else if (!CalCtx.TargetPoseIsValidSimple()) {
			    targetStatus = "NOT TRACKING";
			    targetTone = ui::StatusTone::Warn;
		    }
		    else {
			    targetStatus = "OK";
			    targetTone = ui::StatusTone::Ok;
		    }

		    // One device per cell. Two columns side by side on a wide window,
		    // collapsing to a single stacked column once the panel narrows so the
		    // device strings stay readable.
		    auto drawDevice = [](const char* role, const char* system, const std::string& model,
		                         const std::string& serial, const char* status, ui::StatusTone tone) {
			    ui::SetCellToneBg(tone);
			    ImGui::TextDisabled("%s", role);
			    ImGui::TextWrapped("%s / %s / %s", system, model.c_str(), serial.c_str());
			    ImGui::TextUnformatted("Status:");
			    ImGui::SameLine();
			    ui::DrawStatusText(status, tone);
		    };

		    ui::ResponsiveColumnsScope devices("DeviceInfo", 2, 300.0f);
		    if (devices) {
			    drawDevice("Reference device", refTrackingSystem, CalCtx.referenceStandby.model,
			               CalCtx.referenceStandby.serial, refStatus, refTone);
			    devices.Next();
			    drawDevice("Target device", targetTrackingSystem, CalCtx.targetStandby.model,
			               CalCtx.targetStandby.serial, targetStatus, targetTone);
		    }
	    },
	    panelSize);

	// --- Actions panel -----------------------------------------------------
	openvr_pair::overlay::ui::DrawPanel(
	    "Actions",
	    [&] {
		    float width = ImGui::GetWindowContentRegionWidth();

		    // (Removed 2026-05-04: "Recalibrate from scratch" button. The wedge case
		    // it served as escape-hatch for is now handled silently by the load-time
		    // guard in Configuration.cpp::ParseProfile and the runtime detector in
		    // Calibration.cpp::CalibrationTick. Per
		    // feedback_no_button_to_recover_broken_tracking.md (memory): a recovery
		    // flow whose precondition is broken tracking can't require interaction
		    // from the user via that same broken tracking -- auto-detect + auto-fix
		    // is the only correct shape.)

		    if (ImGui::BeginTable("##CCal_Actions", 2, 0, ImVec2(width, ImGui::GetTextLineHeight() * 2))) {
			    ImGui::TableNextRow();
			    ImGui::TableSetColumnIndex(0);
			    // User-facing rename of the old "Debug: Force break calibration"
			    // button. Same underlying call -- pushing a random offset forces the
			    // solver to re-search from samples instead of trusting the current
			    // estimate, which is what "restart sampling" means in practice.
			    if (ImGui::Button("Restart sampling", ImVec2(-FLT_MIN, 0.0f))) {
				    DebugApplyRandomOffset();
			    }
			    if (ImGui::IsItemHovered()) {
				    ImGui::SetTooltip("Discards the current incremental estimate and forces continuous calibration to "
				                      "recollect samples from scratch.\n"
				                      "Use this if the calibration looks off and you want a fresh search instead of "
				                      "nudging from the current estimate.");
			    }

			    ImGui::TableSetColumnIndex(1);
			    // Toggle button: while paused, the calibration tick is expected to
			    // skip ComputeIncremental so the live offset stays put. The flag
			    // itself lives on CalibrationContext (see Calibration.h); the gate
			    // is on the math/tick side.
			    const bool paused = CalCtx.calibrationPaused;
			    if (paused) {
				    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.65f, 0.45f, 0.15f, 1.0f));
				    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.75f, 0.55f, 0.20f, 1.0f));
				    ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.55f, 0.40f, 0.10f, 1.0f));
			    }
			    if (ImGui::Button(paused ? "Resume updates" : "Pause updates", ImVec2(-FLT_MIN, 0.0f))) {
				    CalCtx.calibrationPaused = !CalCtx.calibrationPaused;
			    }
			    if (paused) {
				    ImGui::PopStyleColor(3);
			    }
			    if (ImGui::IsItemHovered()) {
				    ImGui::SetTooltip("Freeze the live calibration offset without ending continuous mode.\n"
				                      "Useful when something looks momentarily wrong and you want to investigate "
				                      "before the solver self-corrects.");
			    }

			    ImGui::EndTable();
		    }

		    // Profile-mismatch banner lives inside the Actions panel. The banner
		    // only renders when the active HMD's tracking system disagrees with
		    // the saved profile, so users without a mismatch see no extra rows.
		    // Inside-the-panel placement keeps the recovery actions (Clear
		    // profile / Recalibrate) visually grouped with the rest of the
		    // session-control buttons above.
		    DrawProfileMismatchBanner();
	    },
	    panelSize);

	// Head-mount tracker, drawable boundary, and Quest Guardian pause all
	// live on the Play Space tab; they form one setup flow that doesn't
	// belong inside the continuous-cal status surface.

	// === Status messages ===================================================
	// Whatever the calibration state machine has logged this session, plus
	// a tiny scrollback so a user can see the most recent "Applying updated
	// transformation..." line without enabling the debug log.
	ImGui::Spacing();
	ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(0, 0, 0, 1));
	for (const auto& msg : CalCtx.messages) {
		if (msg.type == CalibrationContext::Message::String) {
			ImGui::TextWrapped("> %s", msg.str.c_str());
		}
	}
	ImGui::PopStyleColor();
}
