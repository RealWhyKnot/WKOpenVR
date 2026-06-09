#include "SmoothingPlugin.h"

#include "Protocol.h"
#include "UiHelpers.h"

#include <cfloat>

#include <imgui.h>

namespace {

constexpr const char* kFingerLabels[5] = {"Thumb", "Index", "Middle", "Ring", "Pinky"};
constexpr const char* kHandLabels[2] = {"Left", "Right"};

} // namespace

void SmoothingPlugin::DrawFingersTab()
{
	bool dirty = false;

	openvr_pair::overlay::ui::DrawTextWrapped(
	    "Index Knuckles only. The driver slerps every bone toward the incoming pose so "
	    "what reaches VRChat (and any other consumer of /input/skeleton) is the smoothed "
	    "signal.");

	bool dashboardPassthrough = cfg_.dashboard_finger_passthrough;
	if (ImGui::Checkbox("Dashboard finger passthrough", &dashboardPassthrough)) {
		cfg_.dashboard_finger_passthrough = dashboardPassthrough;
		dashboardStateDirty_ = true;
		dirty = true;
	}
	openvr_pair::overlay::ui::TooltipForLastItem(
	    "Keeps dashboard windows on the live incoming skeleton stream when SteamVR is showing the dashboard.");

	openvr_pair::overlay::ui::DrawSectionHeading("Strength");
	int smoothness = cfg_.smoothness;
	ImGui::SetNextItemWidth(260.0f);
	if (openvr_pair::overlay::ui::SliderIntWithTooltip(
	        "Strength##fingers", &smoothness, 0, 100, "%d%%",
	        "0   = no smoothing (each frame snaps to incoming bones).\n"
	        "50  = moderate (good starting point).\n"
	        "100 = heavy lag (slerp factor 0.05 per frame). Never fully freezes.\n"
	        "Applied to every enabled finger below.\n"
	        "Drag above 0% to enable per-finger overrides below.")) {
		if (smoothness < 0) smoothness = 0;
		if (smoothness > 100) smoothness = 100;
		cfg_.smoothness = smoothness;
		dirty = true;
	}

	openvr_pair::overlay::ui::DisabledSection smoothingEnabled(
	    cfg_.smoothness == 0, "Raise Strength above 0% to enable per-finger controls.");

	openvr_pair::overlay::ui::DrawSectionHeading("Per-finger");
	openvr_pair::overlay::ui::DrawTextWrapped(
	    "Uncheck a finger to pass it through raw. Useful for isolating one finger whose "
	    "smoothing produces an artifact without giving up the feature on the other nine.");
	ImGui::Spacing();

	{
		openvr_pair::overlay::ui::TableScope fingerTable("fingers_grid", 6,
		                                                 ImGuiTableFlags_BordersInnerH | ImGuiTableFlags_BordersInnerV);
		if (fingerTable) {
			ImGui::TableSetupColumn("Hand");
			for (int f = 0; f < 5; ++f)
				ImGui::TableSetupColumn(kFingerLabels[f]);
			ImGui::TableHeadersRow();
			for (int hand = 0; hand < 2; ++hand) {
				ImGui::TableNextRow();
				ImGui::TableNextColumn();
				ImGui::TextUnformatted(kHandLabels[hand]);
				for (int finger = 0; finger < 5; ++finger) {
					const int bit = hand * 5 + finger;
					ImGui::TableNextColumn();
					bool enabled = ((cfg_.finger_mask >> bit) & 1) != 0;
					ImGui::PushID(bit);
					if (ImGui::Checkbox("##fingerbit", &enabled)) {
						if (enabled)
							cfg_.finger_mask |= (uint16_t)(1u << bit);
						else
							cfg_.finger_mask &= (uint16_t)~(1u << bit);
						dirty = true;
					}
					if (ImGui::IsItemHovered()) {
						ImGui::SetTooltip("%s %s", kHandLabels[hand], kFingerLabels[finger]);
					}
					smoothingEnabled.AttachReasonTooltip();
					ImGui::PopID();
				}
			}
		}
	}

	ImGui::Spacing();
	if (ImGui::Button("Enable all fingers")) {
		if (cfg_.finger_mask != protocol::kAllFingersMask) {
			cfg_.finger_mask = protocol::kAllFingersMask;
			dirty = true;
		}
	}
	openvr_pair::overlay::ui::TooltipForLastItem(
	    "Sets every finger bit. Use after Disable all when you want to start over.");
	ImGui::SameLine();
	if (ImGui::Button("Disable all fingers")) {
		if (cfg_.finger_mask != 0) {
			cfg_.finger_mask = 0;
			dirty = true;
		}
	}
	openvr_pair::overlay::ui::TooltipForLastItem(
	    "Clears every finger bit. Strength has no effect with all fingers disabled.");

	openvr_pair::overlay::ui::DrawSectionHeading("Per-finger strength");
	openvr_pair::overlay::ui::DrawTextWrapped(
	    "Override the global Strength above on a per-finger basis. 0 = use the global "
	    "value. Useful when one finger needs more or less smoothing than the rest.");
	ImGui::Spacing();

	{
		openvr_pair::overlay::ui::TableScope strengthTable(
		    "fingers_strength_grid", 6, ImGuiTableFlags_BordersInnerH | ImGuiTableFlags_BordersInnerV);
		if (strengthTable) {
			ImGui::TableSetupColumn("Hand");
			for (int f = 0; f < 5; ++f)
				ImGui::TableSetupColumn(kFingerLabels[f]);
			ImGui::TableHeadersRow();
			for (int hand = 0; hand < 2; ++hand) {
				ImGui::TableNextRow();
				ImGui::TableNextColumn();
				ImGui::TextUnformatted(kHandLabels[hand]);
				for (int finger = 0; finger < 5; ++finger) {
					const int idx = hand * 5 + finger;
					const bool fingerEnabled = ((cfg_.finger_mask >> idx) & 1) != 0;
					ImGui::TableNextColumn();
					ImGui::PushID(idx);
					ImGui::BeginDisabled(!fingerEnabled);
					int v = cfg_.per_finger_smoothness[idx];
					ImGui::SetNextItemWidth(-FLT_MIN);
					if (ImGui::SliderInt("##perfingerstrength", &v, 0, 100, "%d")) {
						if (v < 0) v = 0;
						if (v > 100) v = 100;
						if (cfg_.per_finger_smoothness[idx] != v) {
							cfg_.per_finger_smoothness[idx] = v;
							dirty = true;
						}
					}
					if (ImGui::IsItemHovered()) {
						ImGui::SetTooltip("%s %s\n0 = use global Strength (%d).", kHandLabels[hand],
						                  kFingerLabels[finger], cfg_.smoothness);
					}
					ImGui::EndDisabled();
					smoothingEnabled.AttachReasonTooltip();
					ImGui::PopID();
				}
			}
		}
	}

	if (dirty) {
		SaveConfig(cfg_);
		SendConfig();
	}
}
