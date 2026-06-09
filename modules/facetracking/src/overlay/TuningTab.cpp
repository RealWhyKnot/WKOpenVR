#include "TuningTab.h"

#include "FacetrackingPlugin.h"
#include "UiHelpers.h"
#include "facetracking/ExpressionNames.h"

#include <imgui/imgui.h>

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

namespace facetracking::ui {

using namespace openvr_pair::overlay::ui;

namespace {

enum class ShapeGroup
{
	All = 0,
	Eyes,
	Brows,
	CheeksNose,
	Jaw,
	MouthLips,
	Tongue,
};

constexpr const char* kGroupLabels[] = {
    "All", "Eyes", "Brows", "Cheeks/Nose", "Jaw", "Mouth/Lips", "Tongue",
};

struct TuningTabState
{
	char filter[64] = {};
	char rename[128] = {};
	std::string rename_key;
	int group = 0;
};

static TuningTabState g_state;

bool StartsWith(const char* value, const char* prefix)
{
	return value && prefix && std::strncmp(value, prefix, std::strlen(prefix)) == 0;
}

bool MatchesGroup(const char* name, ShapeGroup group)
{
	switch (group) {
		case ShapeGroup::All:
			return true;
		case ShapeGroup::Eyes:
			return StartsWith(name, "Eye");
		case ShapeGroup::Brows:
			return StartsWith(name, "Brow");
		case ShapeGroup::CheeksNose:
			return StartsWith(name, "Cheek") || StartsWith(name, "Nose");
		case ShapeGroup::Jaw:
			return StartsWith(name, "Jaw");
		case ShapeGroup::MouthLips:
			return StartsWith(name, "Mouth") || StartsWith(name, "Lip");
		case ShapeGroup::Tongue:
			return StartsWith(name, "Tongue");
	}
	return true;
}

bool ContainsNoCase(const char* haystack, const char* needle)
{
	if (!needle || needle[0] == '\0') return true;
	if (!haystack) return false;

	const size_t needleLen = std::strlen(needle);
	const size_t hayLen = std::strlen(haystack);
	if (needleLen > hayLen) return false;

	for (size_t start = 0; start + needleLen <= hayLen; ++start) {
		bool match = true;
		for (size_t i = 0; i < needleLen; ++i) {
			const auto a = static_cast<unsigned char>(haystack[start + i]);
			const auto b = static_cast<unsigned char>(needle[i]);
			if (std::tolower(a) != std::tolower(b)) {
				match = false;
				break;
			}
		}
		if (match) return true;
	}
	return false;
}

void SyncRenameBuffer(FacetrackingPlugin& plugin, const std::string& avatarKey)
{
	const std::string normalized = NormalizeAvatarShapeTuningKey(avatarKey);
	if (g_state.rename_key == normalized) return;

	g_state.rename_key = normalized;
	const AvatarShapeTuningMetadata* metadata = FindMetadataForAvatar(plugin.Profile().current, normalized);
	const std::string custom = metadata ? metadata->custom_name : std::string{};
	std::snprintf(g_state.rename, sizeof(g_state.rename), "%s", custom.c_str());
}

void DrawAvatarProfiles(FacetrackingPlugin& plugin)
{
	const std::vector<std::string> keys = plugin.AvatarTuningKeys();
	const std::string activeKey = plugin.CurrentAvatarTuningKey();
	std::string selectedKey = plugin.SelectedAvatarTuningKey();

	ImGuiTableFlags flags =
	    ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_Resizable | ImGuiTableFlags_SizingStretchProp;
	if (!ImGui::BeginTable("ft_avatar_tuning_profiles", 4, flags,
	                       ImVec2(0.0f, ImGui::GetTextLineHeightWithSpacing() * 5.0f))) {
		return;
	}

	ImGui::TableSetupColumn("Now", ImGuiTableColumnFlags_WidthFixed, 44.0f);
	ImGui::TableSetupColumn("Avatar");
	ImGui::TableSetupColumn("Last used", ImGuiTableColumnFlags_WidthFixed, 150.0f);
	ImGui::TableSetupColumn("Overrides", ImGuiTableColumnFlags_WidthFixed, 80.0f);
	ImGui::TableHeadersRow();

	for (const std::string& key : keys) {
		const bool isActive = (key == activeKey);
		const bool isSelected = (key == selectedKey);
		const std::string label = plugin.AvatarDisplayLabel(key);
		const std::string lastUsed = plugin.AvatarLastUsedLabel(key);
		const uint32_t overrides = plugin.AvatarOverrideCount(key);

		ImGui::TableNextRow();
		ImGui::PushID(key.c_str());

		ImGui::TableSetColumnIndex(0);
		if (isActive) {
			DrawStatusText("On", StatusTone::Ok);
		}
		else {
			ImGui::TextDisabled("-");
		}

		ImGui::TableSetColumnIndex(1);
		if (ImGui::Selectable(label.c_str(), isSelected, ImGuiSelectableFlags_SpanAllColumns)) {
			plugin.SelectAvatarTuningKey(key);
			selectedKey = key;
			SyncRenameBuffer(plugin, key);
		}
		TooltipForLastItem(key.c_str());

		ImGui::TableSetColumnIndex(2);
		ImGui::TextDisabled("%s", lastUsed.c_str());

		ImGui::TableSetColumnIndex(3);
		ImGui::Text("%u", (unsigned)overrides);

		ImGui::PopID();
	}

	ImGui::EndTable();
}

void DrawSelectedAvatarDetails(FacetrackingPlugin& plugin, const std::string& selectedKey)
{
	SyncRenameBuffer(plugin, selectedKey);

	const std::string activeKey = plugin.CurrentAvatarTuningKey();
	const std::string selectedLabel = plugin.AvatarDisplayLabel(selectedKey);
	const AvatarShapeTuningMetadata* metadata = FindMetadataForAvatar(plugin.Profile().current, selectedKey);

	ImGui::Spacing();
	ImGui::Text("Selected: %s", selectedLabel.c_str());
	if (selectedKey == activeKey) {
		ImGui::SameLine();
		DrawStatusText("Active", StatusTone::Ok);
	}

	ImGui::SetNextItemWidth(320.0f);
	ImGui::InputText("Alias##ft_avatar_alias", g_state.rename, sizeof(g_state.rename));
	if (ImGui::IsItemDeactivatedAfterEdit()) {
		plugin.RenameAvatarTuningKey(selectedKey, g_state.rename);
		plugin.Profile().Save();
		g_state.rename_key.clear();
		SyncRenameBuffer(plugin, selectedKey);
		metadata = FindMetadataForAvatar(plugin.Profile().current, selectedKey);
	}
	TooltipForLastItem("Optional display name for this avatar's tuning profile.");

	if (metadata && !metadata->auto_name.empty()) {
		ImGui::TextDisabled("OSC name: %s", metadata->auto_name.c_str());
		if (!metadata->custom_name.empty()) {
			ImGui::SameLine();
			if (ImGui::SmallButton("Use OSC name##ft_avatar_use_osc_name")) {
				g_state.rename[0] = '\0';
				plugin.RenameAvatarTuningKey(selectedKey, "");
				plugin.Profile().Save();
				g_state.rename_key.clear();
				SyncRenameBuffer(plugin, selectedKey);
				metadata = FindMetadataForAvatar(plugin.Profile().current, selectedKey);
			}
			TooltipForLastItem("Clear the alias and show the OSC config name.");
		}
	}

	ImGui::TextDisabled("Last used: %s", plugin.AvatarLastUsedLabel(selectedKey).c_str());
	ImGui::TextDisabled("ID:");
	ImGui::SameLine();
	DrawFilePath(selectedKey.c_str());

	if (metadata && !metadata->config_path.empty()) {
		ImGui::TextDisabled("OSC config:");
		ImGui::SameLine();
		DrawFilePath(metadata->config_path.c_str());
	}
}

} // namespace

void DrawTuningTab(FacetrackingPlugin& plugin)
{
	DrawSectionHeading("Avatar tuning");

	const std::string activeKey = plugin.CurrentAvatarTuningKey();
	ImGui::Text("Active: %s", plugin.CurrentAvatarLabel().c_str());
	ImGui::SameLine();
	ImGui::TextDisabled("%s", plugin.AvatarLastUsedLabel(activeKey).c_str());

	DrawAvatarProfiles(plugin);

	const std::string selectedKey = plugin.SelectedAvatarTuningKey();
	DrawSelectedAvatarDetails(plugin, selectedKey);

	const FaceShapeScaleArray* stored = FindShapeTuningForAvatar(plugin.Profile().current, selectedKey);
	FaceShapeScaleArray values = stored ? *stored : DefaultFaceShapeScales();

	const uint32_t overrideCount = plugin.AvatarOverrideCount(selectedKey);
	ImGui::TextDisabled("Overrides: %u", (unsigned)overrideCount);
	ImGui::SameLine();
	ImGui::BeginDisabled(overrideCount == 0);
	if (ImGui::Button("Reset selected avatar##ft_shape_reset_avatar")) {
		plugin.ResetAvatarShapeTuning(selectedKey);
		plugin.Profile().Save();
		values = DefaultFaceShapeScales();
	}
	ImGui::EndDisabled();
	TooltipForLastItem("Reset every expression scale for the selected avatar.");

	ImGui::Spacing();
	ImGui::InputText("Search##ft_shape_search", g_state.filter, sizeof(g_state.filter));
	ImGui::SameLine();
	const int groupCount = static_cast<int>(sizeof(kGroupLabels) / sizeof(kGroupLabels[0]));
	g_state.group = std::clamp(g_state.group, 0, groupCount - 1);
	if (ImGui::BeginCombo("Group##ft_shape_group", kGroupLabels[g_state.group])) {
		for (int i = 0; i < groupCount; ++i) {
			const bool selected = (g_state.group == i);
			if (ImGui::Selectable(kGroupLabels[i], selected)) g_state.group = i;
			if (selected) ImGui::SetItemDefaultFocus();
		}
		ImGui::EndCombo();
	}

	ImGuiTableFlags flags =
	    ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_Resizable | ImGuiTableFlags_SizingStretchProp;
	if (!ImGui::BeginTable("ft_shape_tuning_table", 3, flags)) return;
	ImGui::TableSetupColumn("Expression");
	ImGui::TableSetupColumn("Scale", ImGuiTableColumnFlags_WidthStretch, 1.4f);
	ImGui::TableSetupColumn("Action", ImGuiTableColumnFlags_WidthFixed, 72.0f);
	ImGui::TableHeadersRow();

	for (uint32_t i = 0; i < protocol::FACETRACKING_EXPRESSION_COUNT; ++i) {
		const char* name = facetracking::ExpressionName(i);
		if (!MatchesGroup(name, static_cast<ShapeGroup>(g_state.group))) continue;
		if (!ContainsNoCase(name, g_state.filter)) continue;

		ImGui::TableNextRow();
		ImGui::PushID(static_cast<int>(i));

		ImGui::TableSetColumnIndex(0);
		ImGui::TextUnformatted(name);

		ImGui::TableSetColumnIndex(1);
		int value = std::clamp(values[i], 0, static_cast<int>(protocol::FACETRACKING_SHAPE_TUNING_MAX_PERCENT));
		ImGui::SetNextItemWidth(-1.0f);
		if (ImGui::SliderInt("##scale", &value, 0, protocol::FACETRACKING_SHAPE_TUNING_MAX_PERCENT, "%d%%")) {
			values[i] = value;
			plugin.SetAvatarShapeScale(selectedKey, i, value);
		}
		if (ImGui::IsItemDeactivatedAfterEdit()) {
			plugin.Profile().Save();
		}

		ImGui::TableSetColumnIndex(2);
		if (values[i] != protocol::FACETRACKING_SHAPE_TUNING_DEFAULT_PERCENT) {
			if (ImGui::SmallButton("Reset")) {
				values[i] = protocol::FACETRACKING_SHAPE_TUNING_DEFAULT_PERCENT;
				plugin.SetAvatarShapeScale(selectedKey, i, protocol::FACETRACKING_SHAPE_TUNING_DEFAULT_PERCENT);
				plugin.Profile().Save();
			}
		}
		else {
			ImGui::TextDisabled("-");
		}

		ImGui::PopID();
	}

	ImGui::EndTable();
}

} // namespace facetracking::ui
