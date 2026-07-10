#include "SettingsTab.h"

#include "FacetrackingPlugin.h"
#include "UiHelpers.h"

#include <imgui/imgui.h>

#include <cstring>

namespace facetracking::ui {

using namespace openvr_pair::overlay::ui;

void DrawSettingsTab(FacetrackingPlugin& plugin)
{
	FacetrackingProfile& p = plugin.profile_.current;

	// ---- Auto Calibration ----
	DrawSectionHeading("Auto Calibration");

	if (CheckboxWithTooltip("Auto calibration", &p.continuous_calib_enabled,
	                        "Learns your personal expression ranges during normal play and\n"
	                        "remaps them toward full output, so weak smiles and half blinks\n"
	                        "reach the avatar without hand-tuning every shape.\n"
	                        "Amplification is capped per shape and idle noise is gated, so a\n"
	                        "resting face stays at rest. Learning takes effect gradually over\n"
	                        "the first seconds of a session.")) {
		plugin.PushConfigToDriver();
	}

	{
		const auto& t = plugin.driver_telemetry_.Snapshot();
		if (!p.continuous_calib_enabled) {
			ImGui::TextDisabled("Off");
		}
		else if (!t.valid || t.stale || !t.calib_enabled) {
			ImGui::TextDisabled("Waiting for driver...");
		}
		else if (!t.calib_loaded) {
			ImGui::TextDisabled("Waiting for a tracking module...");
		}
		else {
			ImGui::Text("Learning: %d%% confident (weakest shape %d%%)", (int)(t.calib_avg_conf * 100.f),
			            (int)(t.calib_min_conf * 100.f));
		}
	}

	if (!p.continuous_calib_enabled) ImGui::BeginDisabled();
	if (ImGui::Button("Reset all##calib")) {
		plugin.SendCalibrationCommand(protocol::FaceCalibResetAll);
	}
	TooltipForLastItem("Forget every learned range and start over.");
	ImGui::SameLine();
	if (ImGui::Button("Reset eyes##calib")) {
		plugin.SendCalibrationCommand(protocol::FaceCalibResetEye);
	}
	TooltipForLastItem("Forget the learned eye openness range only.");
	ImGui::SameLine();
	if (ImGui::Button("Reset expressions##calib")) {
		plugin.SendCalibrationCommand(protocol::FaceCalibResetExpr);
	}
	TooltipForLastItem("Forget the learned face expression ranges only.");
	if (!p.continuous_calib_enabled) ImGui::EndDisabled();

	// ---- Eyelid Sync ----
	DrawSectionHeading("Eyelid Sync");

	if (CheckboxWithTooltip("Eyelid Sync", &p.eyelid_sync_enabled,
	                        "Blends both eye openness values toward the selected eyelid target\n"
	                        "to reduce asymmetric flicker from tracking noise.\n"
	                        "Strength 0 = no effect even when enabled.")) {
		plugin.PushConfigToDriver();
	}

	if (!p.eyelid_sync_enabled) ImGui::BeginDisabled();

	if (SliderIntWithTooltip("Sync Strength##eyelid", &p.eyelid_sync_strength, 0, 100, "%d%%",
	                         "How aggressively both eyes are pulled toward the selected target.\n"
	                         "100 = fully forced equal; 0 = no correction applied.\n"
	                         "70 is a safe default -- covers sensor noise without\n"
	                         "flattening deliberate asymmetry.")) {
		plugin.PushConfigToDriver();
	}

	if (RadioButtonWithTooltip("Most closed##eyelid_sync_mode",
	                           p.eyelid_sync_mode == protocol::FACETRACKING_EYELID_SYNC_MOST_CLOSED,
	                           "The more-closed eye drives both eyelids.")) {
		p.eyelid_sync_mode = protocol::FACETRACKING_EYELID_SYNC_MOST_CLOSED;
		plugin.PushConfigToDriver();
	}
	ImGui::SameLine();
	if (RadioButtonWithTooltip("Most open##eyelid_sync_mode",
	                           p.eyelid_sync_mode == protocol::FACETRACKING_EYELID_SYNC_MOST_OPEN,
	                           "The more-open eye drives both eyelids.")) {
		p.eyelid_sync_mode = protocol::FACETRACKING_EYELID_SYNC_MOST_OPEN;
		plugin.PushConfigToDriver();
	}

	if (CheckboxWithTooltip("Preserve intentional winks", &p.eyelid_sync_preserve_winks,
	                        "Detects a deliberate one-eye wink (asymmetry > 45%% sustained\n"
	                        "for 120 ms with good confidence) and bypasses sync during\n"
	                        "it so winks survive even at high strength settings.")) {
		plugin.PushConfigToDriver();
	}

	if (!p.eyelid_sync_enabled) ImGui::EndDisabled();

	// ---- Eye Closure Assistance ----
	DrawSectionHeading("Eye Closure Assistance");

	if (CheckboxWithTooltip("Easier eye close", &p.eye_close_assist_enabled,
	                        "Helps your eyes reach fully closed on avatars where they stay\n"
	                        "slightly open even when your real eyes are shut.\n"
	                        "Strength 0 = no effect even when enabled.")) {
		plugin.PushConfigToDriver();
	}

	if (!p.eye_close_assist_enabled) ImGui::BeginDisabled();

	if (SliderIntWithTooltip("Close strength##eye_close_assist", &p.eye_close_assist_strength, 0, 100, "%d%%",
	                         "How easily your eyes snap shut. Higher counts more of your\n"
	                         "near-closed range as fully closed, so stubborn avatars blink.\n"
	                         "Too high can make relaxed eyes look half-closed -- back off if so.")) {
		plugin.PushConfigToDriver();
	}

	if (!p.eye_close_assist_enabled) ImGui::EndDisabled();

	// ---- Expression Corrections ----
	DrawSectionHeading("Expression Corrections");

	if (CheckboxWithTooltip("Mouth-close compensation", &p.mouth_close_compensation_enabled,
	                        "Reduces JawOpen when MouthClose is also active.\n"
	                        "Use this when the tracker leaves the avatar's mouth slightly open\n"
	                        "even though the lips are meant to be closed.")) {
		plugin.PushConfigToDriver();
	}

	if (CheckboxWithTooltip("Smile opens mouth", &p.smile_mouth_open_assist_enabled,
	                        "Adds a small JawOpen floor as smiles get stronger.\n"
	                        "Use this on avatars where a broad tracked smile looks too stiff\n"
	                        "with the mouth fully closed.")) {
		plugin.PushConfigToDriver();
	}

	if (!p.smile_mouth_open_assist_enabled) ImGui::BeginDisabled();

	if (SliderIntWithTooltip("Smile open strength##smile_mouth_open", &p.smile_mouth_open_strength, 0, 100, "%d%%",
	                         "Maximum mouth-open assist for strong smiles.\n"
	                         "Lower values keep closed-mouth smiles mostly intact; higher\n"
	                         "values force a clearer open-mouth grin.")) {
		plugin.PushConfigToDriver();
	}

	if (!p.smile_mouth_open_assist_enabled) ImGui::EndDisabled();

	if (CheckboxWithTooltip("Auto-close idle mouth", &p.idle_mouth_auto_close_enabled,
	                        "Closes a small, steady JawOpen value after it persists for a\n"
	                        "short time with little other mouth expression.\n"
	                        "Use this for headset noise that appears when reclining or lying down.")) {
		plugin.PushConfigToDriver();
	}

	if (CheckboxWithTooltip("Sync brows with closed eyes", &p.eyelid_brow_sync_enabled,
	                        "Softly reduces brow-up shapes and adds a small brow-lowerer\n"
	                        "hint when eyelids are closed.\n"
	                        "Use this when blinks or reclined poses make brows and lids look\n"
	                        "out of sync on a specific avatar.")) {
		plugin.PushConfigToDriver();
	}

	if (!p.eyelid_brow_sync_enabled) ImGui::BeginDisabled();

	if (SliderIntWithTooltip("Brow sync strength##eyelid_brow", &p.eyelid_brow_sync_strength, 0, 100, "%d%%",
	                         "How strongly closed eyelids influence brow shapes.\n"
	                         "Lower values preserve expressive brows; higher values make\n"
	                         "blinks and closed-eye poses more tightly coordinated.")) {
		plugin.PushConfigToDriver();
	}

	if (!p.eyelid_brow_sync_enabled) ImGui::EndDisabled();

	// ---- Vergence Lock ----
	DrawSectionHeading("Vergence Lock");

	if (CheckboxWithTooltip("Vergence Lock", &p.vergence_lock_enabled,
	                        "Reconstructs a shared focus point from both gaze rays each\n"
	                        "frame (skew-line midpoint) and nudges both eyes toward it.\n"
	                        "Reduces the jitter and crossed-eye artefacts that appear\n"
	                        "when each eye's tracker drifts independently.")) {
		plugin.PushConfigToDriver();
	}

	if (!p.vergence_lock_enabled) ImGui::BeginDisabled();

	if (SliderIntWithTooltip("Lock Strength##vergence", &p.vergence_lock_strength, 0, 100, "%d%%",
	                         "How much each eye's gaze is pulled toward the computed\n"
	                         "focus point. Start at 50 and increase if you still see\n"
	                         "crossed-eye artefacts; lower if the focus point feels\n"
	                         "sluggish on fast saccades.")) {
		plugin.PushConfigToDriver();
	}

	if (!p.vergence_lock_enabled) ImGui::EndDisabled();

	// ---- Output ----
	DrawSectionHeading("Output");

	if (CheckboxWithTooltip("OSC (VRChat)", &p.output_osc_enabled,
	                        "Sends /avatar/parameters/* to the OSC router, which forwards\n"
	                        "them to VRChat. Target host and port are configured on the\n"
	                        "OSC Router tab. Uncheck to stop all FT OSC output.")) {
		plugin.PushConfigToDriver();
	}
}

} // namespace facetracking::ui
