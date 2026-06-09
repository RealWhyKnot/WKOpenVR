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
	char avatar_filter[64] = {};
	char filter[64] = {};
	char rename[128] = {};
	std::string rename_key;
	int group = 0;
	bool modified_only = false;
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

const char* ShapeScaleStateLabel(int value)
{
	if (value < protocol::FACETRACKING_SHAPE_TUNING_DEFAULT_PERCENT) return "Under";
	if (value > protocol::FACETRACKING_SHAPE_TUNING_DEFAULT_PERCENT) return "Over";
	return "Default";
}

StatusTone ShapeScaleStateTone(int value)
{
	if (value < protocol::FACETRACKING_SHAPE_TUNING_DEFAULT_PERCENT) return StatusTone::Warn;
	if (value > protocol::FACETRACKING_SHAPE_TUNING_DEFAULT_PERCENT) return StatusTone::Info;
	return StatusTone::Idle;
}

StatusTone AvatarSourceTone(const std::string& source)
{
	if (source == "Alias") return StatusTone::Info;
	if (source == "OSC") return StatusTone::Ok;
	return StatusTone::Idle;
}

bool AvatarMatchesFilter(FacetrackingPlugin& plugin, const std::string& key)
{
	if (g_state.avatar_filter[0] == '\0') return true;

	const AvatarShapeTuningMetadata* metadata = FindMetadataForAvatar(plugin.Profile().current, key);
	if (ContainsNoCase(plugin.AvatarDisplayLabel(key).c_str(), g_state.avatar_filter)) return true;
	if (ContainsNoCase(key.c_str(), g_state.avatar_filter)) return true;
	if (metadata) {
		if (ContainsNoCase(metadata->custom_name.c_str(), g_state.avatar_filter)) return true;
		if (ContainsNoCase(metadata->auto_name.c_str(), g_state.avatar_filter)) return true;
		if (ContainsNoCase(metadata->config_path.c_str(), g_state.avatar_filter)) return true;
	}
	return false;
}

std::vector<uint32_t> VisibleShapeIndices(const FaceShapeScaleArray& values)
{
	std::vector<uint32_t> indices;
	for (uint32_t i = 0; i < protocol::FACETRACKING_EXPRESSION_COUNT; ++i) {
		const char* name = facetracking::ExpressionName(i);
		if (!MatchesGroup(name, static_cast<ShapeGroup>(g_state.group))) continue;
		if (!ContainsNoCase(name, g_state.filter)) continue;
		if (g_state.modified_only && values[i] == protocol::FACETRACKING_SHAPE_TUNING_DEFAULT_PERCENT) continue;
		indices.push_back(i);
	}
	return indices;
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

void CopyTextButton(const char* id, const std::string& value)
{
	if (value.empty()) return;
	ImGui::PushID(id);
	if (ImGui::SmallButton("Copy")) {
		ImGui::SetClipboardText(value.c_str());
	}
	ImGui::PopID();
	TooltipForLastItem("Copy to clipboard.");
}

void DrawAvatarProfiles(FacetrackingPlugin& plugin)
{
	const std::vector<std::string> keys = plugin.AvatarTuningKeys();
	const std::string activeKey = plugin.CurrentAvatarTuningKey();
	std::string selectedKey = plugin.SelectedAvatarTuningKey();

	ImGui::TextDisabled("Avatar profiles");
	ImGui::SameLine();
	ImGui::SetNextItemWidth(220.0f);
	ImGui::InputTextWithHint("##ft_avatar_filter", "Filter avatars", g_state.avatar_filter,
	                         sizeof(g_state.avatar_filter));
	if (g_state.avatar_filter[0] != '\0') {
		ImGui::SameLine();
		if (ImGui::SmallButton("Clear##ft_avatar_filter_clear")) {
			g_state.avatar_filter[0] = '\0';
		}
	}

	std::vector<std::string> visibleKeys;
	visibleKeys.reserve(keys.size());
	for (const std::string& key : keys) {
		if (AvatarMatchesFilter(plugin, key)) visibleKeys.push_back(key);
	}

	const float rowHeight = ImGui::GetTextLineHeightWithSpacing();
	const float tableHeight = rowHeight * static_cast<float>(std::clamp<size_t>(visibleKeys.size() + 1, 4, 8));
	ImGuiTableFlags flags = ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_Resizable |
	                        ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_ScrollY;
	if (!ImGui::BeginTable("ft_avatar_tuning_profiles", 5, flags, ImVec2(0.0f, tableHeight))) {
		return;
	}

	ImGui::TableSetupColumn("Now", ImGuiTableColumnFlags_WidthFixed, 44.0f);
	ImGui::TableSetupColumn("Avatar");
	ImGui::TableSetupColumn("Source", ImGuiTableColumnFlags_WidthFixed, 72.0f);
	ImGui::TableSetupColumn("Last used", ImGuiTableColumnFlags_WidthFixed, 150.0f);
	ImGui::TableSetupColumn("Overrides", ImGuiTableColumnFlags_WidthFixed, 80.0f);
	ImGui::TableHeadersRow();

	for (const std::string& key : visibleKeys) {
		const bool isActive = (key == activeKey);
		const bool isSelected = (key == selectedKey);
		const std::string label = plugin.AvatarDisplayLabel(key);
		const AvatarShapeTuningMetadata* metadata = FindMetadataForAvatar(plugin.Profile().current, key);
		const std::string source = AvatarDisplaySourceLabel(key, metadata);
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
		DrawStatusText(source.c_str(), AvatarSourceTone(source));
		TooltipForLastItem(source == "Alias"     ? "This row uses a saved alias."
		                   : source == "OSC"     ? "This row uses the name from the VRChat OSC config."
		                   : source == "ID"      ? "No alias or OSC name is available yet."
		                   : source == "Default" ? "Fallback profile used before an avatar is detected."
		                                         : nullptr);

		ImGui::TableSetColumnIndex(3);
		ImGui::TextDisabled("%s", lastUsed.c_str());

		ImGui::TableSetColumnIndex(4);
		ImGui::Text("%u", (unsigned)overrides);

		ImGui::PopID();
	}

	ImGui::EndTable();

	if (visibleKeys.empty()) {
		DrawEmptyState("No avatar profiles match the filter.");
	}
}

void DrawSelectedAvatarDetails(FacetrackingPlugin& plugin, const std::string& selectedKey)
{
	SyncRenameBuffer(plugin, selectedKey);

	const std::string activeKey = plugin.CurrentAvatarTuningKey();
	const std::string selectedLabel = plugin.AvatarDisplayLabel(selectedKey);
	const AvatarShapeTuningMetadata* metadata = FindMetadataForAvatar(plugin.Profile().current, selectedKey);
	std::string source = AvatarDisplaySourceLabel(selectedKey, metadata);
	const uint32_t overrideCount = plugin.AvatarOverrideCount(selectedKey);
	const bool isActive = selectedKey == activeKey;

	ImGui::Spacing();
	ImGui::SeparatorText("Selected profile");

	ImGui::TextUnformatted(selectedLabel.c_str());
	ImGui::SameLine();
	DrawStatusText(isActive ? "Active" : "Saved", isActive ? StatusTone::Ok : StatusTone::Warn);
	TooltipForLastItem(isActive ? "Edits are pushed to the driver immediately."
	                            : "Edits are saved and apply when this avatar becomes active.");
	if (!isActive) {
		ImGui::SameLine();
		if (ImGui::SmallButton("Use active avatar##ft_avatar_select_active")) {
			plugin.SelectAvatarTuningKey(activeKey);
			g_state.rename_key.clear();
			SyncRenameBuffer(plugin, activeKey);
			return;
		}
	}

	ImGui::SetNextItemWidth(360.0f);
	ImGui::InputTextWithHint("Alias##ft_avatar_alias", "Custom display name", g_state.rename, sizeof(g_state.rename));
	if (ImGui::IsItemDeactivatedAfterEdit()) {
		plugin.RenameAvatarTuningKey(selectedKey, g_state.rename);
		plugin.Profile().Save();
		g_state.rename_key.clear();
		SyncRenameBuffer(plugin, selectedKey);
		metadata = FindMetadataForAvatar(plugin.Profile().current, selectedKey);
		source = AvatarDisplaySourceLabel(selectedKey, metadata);
	}
	TooltipForLastItem("Optional display name for this avatar's tuning profile.");

	bool hasCustomName = metadata && !metadata->custom_name.empty();
	bool hasAutoName = metadata && !metadata->auto_name.empty();
	ImGui::BeginDisabled(!hasCustomName);
	if (ImGui::SmallButton("Clear alias##ft_avatar_clear_alias")) {
		g_state.rename[0] = '\0';
		plugin.RenameAvatarTuningKey(selectedKey, "");
		plugin.Profile().Save();
		g_state.rename_key.clear();
		SyncRenameBuffer(plugin, selectedKey);
		metadata = FindMetadataForAvatar(plugin.Profile().current, selectedKey);
		source = AvatarDisplaySourceLabel(selectedKey, metadata);
		hasCustomName = metadata && !metadata->custom_name.empty();
		hasAutoName = metadata && !metadata->auto_name.empty();
	}
	ImGui::EndDisabled();
	TooltipForLastItem("Remove the saved alias.");

	ImGui::SameLine();
	ImGui::BeginDisabled(!hasAutoName || !hasCustomName);
	if (ImGui::SmallButton("Use OSC name##ft_avatar_use_osc_name")) {
		g_state.rename[0] = '\0';
		plugin.RenameAvatarTuningKey(selectedKey, "");
		plugin.Profile().Save();
		g_state.rename_key.clear();
		SyncRenameBuffer(plugin, selectedKey);
		metadata = FindMetadataForAvatar(plugin.Profile().current, selectedKey);
		source = AvatarDisplaySourceLabel(selectedKey, metadata);
		hasCustomName = metadata && !metadata->custom_name.empty();
		hasAutoName = metadata && !metadata->auto_name.empty();
	}
	ImGui::EndDisabled();
	TooltipForLastItem("Clear the alias and show the OSC config name.");

	if (ImGui::BeginTable("ft_avatar_profile_details", 3, ImGuiTableFlags_SizingStretchProp)) {
		ImGui::TableSetupColumn("Label", ImGuiTableColumnFlags_WidthFixed, 86.0f);
		ImGui::TableSetupColumn("Value");
		ImGui::TableSetupColumn("Action", ImGuiTableColumnFlags_WidthFixed, 48.0f);

		auto detailRow = [](const char* label, const std::string& value, const char* copyId) {
			ImGui::TableNextRow();
			ImGui::TableSetColumnIndex(0);
			ImGui::TextDisabled("%s", label);
			ImGui::TableSetColumnIndex(1);
			if (value.empty()) {
				ImGui::TextDisabled("Unavailable");
			}
			else {
				DrawFilePath(value.c_str());
			}
			ImGui::TableSetColumnIndex(2);
			CopyTextButton(copyId, value);
		};

		ImGui::TableNextRow();
		ImGui::TableSetColumnIndex(0);
		ImGui::TextDisabled("Source");
		ImGui::TableSetColumnIndex(1);
		DrawStatusText(source.c_str(), AvatarSourceTone(source));
		ImGui::SameLine();
		ImGui::TextDisabled("Overrides: %u", (unsigned)overrideCount);
		ImGui::TableSetColumnIndex(2);
		ImGui::TextDisabled("-");

		detailRow("Last used", plugin.AvatarLastUsedLabel(selectedKey), "ft_copy_avatar_last_used");
		detailRow("OSC name", hasAutoName ? metadata->auto_name : std::string{}, "ft_copy_avatar_osc_name");
		detailRow("Avatar ID", selectedKey, "ft_copy_avatar_id");
		if (metadata && !metadata->config_path.empty()) {
			detailRow("OSC config", metadata->config_path, "ft_copy_avatar_config");
		}
		ImGui::EndTable();
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

	std::string selectedKey = plugin.SelectedAvatarTuningKey();
	DrawSelectedAvatarDetails(plugin, selectedKey);
	selectedKey = plugin.SelectedAvatarTuningKey();

	const FaceShapeScaleArray* stored = FindShapeTuningForAvatar(plugin.Profile().current, selectedKey);
	FaceShapeScaleArray values = stored ? *stored : DefaultFaceShapeScales();

	ImGui::SeparatorText("Expression scales");

	const uint32_t overrideCount = plugin.AvatarOverrideCount(selectedKey);
	const int groupCount = static_cast<int>(sizeof(kGroupLabels) / sizeof(kGroupLabels[0]));
	g_state.group = std::clamp(g_state.group, 0, groupCount - 1);
	std::vector<uint32_t> visibleIndices = VisibleShapeIndices(values);

	ImGui::TextDisabled("Overrides: %u", (unsigned)overrideCount);
	ImGui::SameLine();
	ImGui::TextDisabled("Visible: %u", (unsigned)visibleIndices.size());
	ImGui::SameLine();
	ImGui::BeginDisabled(overrideCount == 0);
	if (ImGui::SmallButton("Reset avatar##ft_shape_reset_avatar")) {
		plugin.ResetAvatarShapeTuning(selectedKey);
		plugin.Profile().Save();
		values = DefaultFaceShapeScales();
		visibleIndices = VisibleShapeIndices(values);
	}
	ImGui::EndDisabled();
	TooltipForLastItem("Reset every expression scale for the selected avatar.");

	ImGui::SameLine();
	ImGui::BeginDisabled(visibleIndices.empty());
	if (ImGui::SmallButton("Reset visible##ft_shape_reset_visible")) {
		for (uint32_t index : visibleIndices) {
			values[index] = protocol::FACETRACKING_SHAPE_TUNING_DEFAULT_PERCENT;
			plugin.SetAvatarShapeScale(selectedKey, index, protocol::FACETRACKING_SHAPE_TUNING_DEFAULT_PERCENT);
		}
		plugin.Profile().Save();
		visibleIndices = VisibleShapeIndices(values);
	}
	ImGui::EndDisabled();
	TooltipForLastItem("Reset the expressions currently shown by the filters.");

	ImGui::Spacing();
	ImGui::SetNextItemWidth(220.0f);
	ImGui::InputTextWithHint("##ft_shape_search", "Filter expressions", g_state.filter, sizeof(g_state.filter));
	if (g_state.filter[0] != '\0') {
		ImGui::SameLine();
		if (ImGui::SmallButton("Clear##ft_shape_search_clear")) {
			g_state.filter[0] = '\0';
			visibleIndices = VisibleShapeIndices(values);
		}
	}
	ImGui::SameLine();
	ImGui::SetNextItemWidth(150.0f);
	if (ImGui::BeginCombo("Group##ft_shape_group", kGroupLabels[g_state.group])) {
		for (int i = 0; i < groupCount; ++i) {
			const bool selected = (g_state.group == i);
			if (ImGui::Selectable(kGroupLabels[i], selected)) {
				g_state.group = i;
				visibleIndices = VisibleShapeIndices(values);
			}
			if (selected) ImGui::SetItemDefaultFocus();
		}
		ImGui::EndCombo();
	}
	ImGui::SameLine();
	if (ImGui::Checkbox("Modified only##ft_shape_modified_only", &g_state.modified_only)) {
		visibleIndices = VisibleShapeIndices(values);
	}
	TooltipForLastItem("Show only expressions that differ from 100%.");

	ImGuiTableFlags flags =
	    ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_Resizable | ImGuiTableFlags_SizingStretchProp;
	if (!ImGui::BeginTable("ft_shape_tuning_table", 4, flags)) return;
	ImGui::TableSetupColumn("Expression");
	ImGui::TableSetupColumn("Scale", ImGuiTableColumnFlags_WidthStretch, 1.4f);
	ImGui::TableSetupColumn("State", ImGuiTableColumnFlags_WidthFixed, 78.0f);
	ImGui::TableSetupColumn("Action", ImGuiTableColumnFlags_WidthFixed, 72.0f);
	ImGui::TableHeadersRow();

	for (uint32_t i : visibleIndices) {
		const char* name = facetracking::ExpressionName(i);

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
		DrawStatusText(ShapeScaleStateLabel(values[i]), ShapeScaleStateTone(values[i]));

		ImGui::TableSetColumnIndex(3);
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

	if (visibleIndices.empty()) {
		DrawEmptyState("No expressions match the current filters.");
	}
}

} // namespace facetracking::ui
