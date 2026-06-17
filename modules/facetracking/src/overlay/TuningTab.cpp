#include "TuningTab.h"

#include "FacetrackingPlugin.h"
#include "UiHelpers.h"
#include "facetracking/ExpressionNames.h"

#include <imgui/imgui.h>

#include <algorithm>
#include <cctype>
#include <cmath>
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
	bool edit_global = false;
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

bool IsShapeModified(const FaceShapeTuningValue& value)
{
	return !IsDefaultFaceShapeTuningValue(value);
}

const char* ShapeScaleStateLabel(const FaceShapeTuningValue& value)
{
	const FaceShapeTuningValue clamped = ClampFaceShapeTuningValue(value);
	if (clamped.min_percent != protocol::FACETRACKING_SHAPE_TUNING_DEFAULT_MIN_PERCENT ||
	    clamped.max_percent != protocol::FACETRACKING_SHAPE_TUNING_DEFAULT_MAX_PERCENT) {
		return "Limited";
	}
	if (clamped.scale_percent < protocol::FACETRACKING_SHAPE_TUNING_DEFAULT_PERCENT) return "Under";
	if (clamped.scale_percent > protocol::FACETRACKING_SHAPE_TUNING_DEFAULT_PERCENT) return "Over";
	return "Default";
}

StatusTone ShapeScaleStateTone(const FaceShapeTuningValue& value)
{
	const FaceShapeTuningValue clamped = ClampFaceShapeTuningValue(value);
	if (clamped.min_percent != protocol::FACETRACKING_SHAPE_TUNING_DEFAULT_MIN_PERCENT ||
	    clamped.max_percent != protocol::FACETRACKING_SHAPE_TUNING_DEFAULT_MAX_PERCENT) {
		return StatusTone::Info;
	}
	if (clamped.scale_percent < protocol::FACETRACKING_SHAPE_TUNING_DEFAULT_PERCENT) return StatusTone::Warn;
	if (clamped.scale_percent > protocol::FACETRACKING_SHAPE_TUNING_DEFAULT_PERCENT) return StatusTone::Info;
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

bool ActiveAvatarOverridesShape(FacetrackingPlugin& plugin, uint32_t index)
{
	if (index >= protocol::FACETRACKING_EXPRESSION_COUNT) return false;
	const FaceShapeScaleArray* values =
	    FindShapeTuningForAvatar(plugin.Profile().current, plugin.CurrentAvatarTuningKey());
	return values && !IsDefaultFaceShapeTuningValue((*values)[index]);
}

bool GlobalOverridesShape(FacetrackingPlugin& plugin, uint32_t index)
{
	if (index >= protocol::FACETRACKING_EXPRESSION_COUNT) return false;
	return !IsDefaultFaceShapeTuningValue(plugin.Profile().current.global_shape_tuning[index]);
}

std::vector<uint32_t> VisibleShapeIndices(const FaceShapeScaleArray& values)
{
	std::vector<uint32_t> indices;
	for (uint32_t i = 0; i < protocol::FACETRACKING_EXPRESSION_COUNT; ++i) {
		const char* name = facetracking::ExpressionName(i);
		if (!MatchesGroup(name, static_cast<ShapeGroup>(g_state.group))) continue;
		if (!ContainsNoCase(name, g_state.filter)) continue;
		if (g_state.modified_only && !IsShapeModified(values[i])) continue;
		indices.push_back(i);
	}
	return indices;
}

void DrawShapeValueMeter(float preTuning, float postTuning, bool live)
{
	const float width = ImGui::GetContentRegionAvail().x;
	const float height = ImGui::GetFrameHeight();
	if (!live || width <= 4.0f) {
		ImGui::ProgressBar(0.0f, ImVec2(-1.0f, height), "no live data");
		TooltipForLastItem("Waiting for current driver expression values.");
		return;
	}

	preTuning = std::isfinite(preTuning) ? std::clamp(preTuning, 0.0f, 2.0f) : 0.0f;
	postTuning = std::isfinite(postTuning) ? std::clamp(postTuning, 0.0f, 2.0f) : 0.0f;

	char overlay[96];
	std::snprintf(overlay, sizeof(overlay), "%.0f%% -> %.0f%%  avatar %.0f%%", preTuning * 100.0f, postTuning * 100.0f,
	              std::min(postTuning, 1.0f) * 100.0f);
	ImGui::ProgressBar(postTuning / 2.0f, ImVec2(-1.0f, height), overlay);

	ImDrawList* drawList = ImGui::GetWindowDrawList();
	const ImVec2 min = ImGui::GetItemRectMin();
	const ImVec2 max = ImGui::GetItemRectMax();
	const float markerX = min.x + (max.x - min.x) * 0.5f;
	const ImU32 markerColor = ImGui::GetColorU32(ImGuiCol_TextDisabled);
	drawList->AddLine(ImVec2(markerX, min.y + 2.0f), ImVec2(markerX, max.y - 2.0f), markerColor, 1.0f);

	const float rawX = min.x + (max.x - min.x) * (preTuning / 2.0f);
	const ImU32 rawColor = ImGui::GetColorU32(ImGuiCol_Text);
	drawList->AddCircleFilled(ImVec2(rawX, (min.y + max.y) * 0.5f), 3.0f, rawColor);
	TooltipForLastItem("Bar scale is 0..200%. The vertical marker is the usual 100% avatar amplitude limit.");
}

bool DrawPercentSlider(const char* label, int& value, int minValue, int maxValue, const char* tooltip,
                       bool& deactivatedAfterEdit)
{
	ImGui::SetNextItemWidth(-1.0f);
	const bool changed = ImGui::SliderInt(label, &value, minValue, maxValue, "%d%%");
	if (ImGui::IsItemDeactivatedAfterEdit()) deactivatedAfterEdit = true;
	TooltipForLastItem(tooltip);
	return changed;
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
	if (ImGui::Button("Copy", ImVec2(-1.0f, 0.0f))) {
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
	{
		TableScope profilesTable("ft_avatar_tuning_profiles", 5, flags, ImVec2(0.0f, tableHeight));
		if (!profilesTable) return;

		SetupFixedColumn("Now", 44.0f);
		SetupStretchColumn("Avatar");
		SetupFixedColumn("Source", 72.0f);
		SetupFixedColumn("Last used", 150.0f);
		SetupFixedColumn("Overrides", 80.0f);
		DrawTableHeader();

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
				g_state.edit_global = false;
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
	}

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

	TableScope detailsTable("ft_avatar_profile_details", 3, ImGuiTableFlags_SizingStretchProp);
	if (detailsTable) {
		SetupFixedColumn("Label", 86.0f);
		SetupStretchColumn("Value");
		SetupFixedColumn("Action", 68.0f);

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
	}
}

void DrawTuningScopeSelector()
{
	ImGui::TextDisabled("Edit scope");
	ImGui::SameLine();
	if (ImGui::RadioButton("Global (all avatars)##ft_tuning_scope_global", g_state.edit_global)) {
		g_state.edit_global = true;
	}
	TooltipForLastItem("Tune the baseline used by every avatar.");
	ImGui::SameLine();
	if (ImGui::RadioButton("Avatar profile##ft_tuning_scope_avatar", !g_state.edit_global)) {
		g_state.edit_global = false;
	}
	TooltipForLastItem("Tune the selected avatar; these values override Global for this avatar.");
}

void DrawGlobalTuningDetails(FacetrackingPlugin& plugin)
{
	ImGui::Spacing();
	ImGui::SeparatorText("Selected profile");

	ImGui::TextUnformatted("Global (all avatars)");
	ImGui::SameLine();
	DrawStatusText("Global", StatusTone::Info);
	TooltipForLastItem("Global values apply to every avatar unless that avatar overrides the same expression.");

	TableScope detailsTable("ft_global_profile_details", 3, ImGuiTableFlags_SizingStretchProp);
	if (!detailsTable) return;

	SetupFixedColumn("Label", 86.0f);
	SetupStretchColumn("Value");
	SetupFixedColumn("Action", 68.0f);

	ImGui::TableNextRow();
	ImGui::TableSetColumnIndex(0);
	ImGui::TextDisabled("Scope");
	ImGui::TableSetColumnIndex(1);
	DrawStatusText("All avatars", StatusTone::Info);
	ImGui::TableSetColumnIndex(2);
	ImGui::TextDisabled("-");

	ImGui::TableNextRow();
	ImGui::TableSetColumnIndex(0);
	ImGui::TextDisabled("Active");
	ImGui::TableSetColumnIndex(1);
	ImGui::TextUnformatted(plugin.CurrentAvatarLabel().c_str());
	ImGui::TableSetColumnIndex(2);
	ImGui::TextDisabled("-");

	ImGui::TableNextRow();
	ImGui::TableSetColumnIndex(0);
	ImGui::TextDisabled("Overrides");
	ImGui::TableSetColumnIndex(1);
	ImGui::Text("%u", (unsigned)plugin.GlobalOverrideCount());
	ImGui::TableSetColumnIndex(2);
	ImGui::TextDisabled("-");
}

} // namespace

void DrawTuningTab(FacetrackingPlugin& plugin)
{
	DrawSectionHeading("Expression tuning");

	const std::string activeKey = plugin.CurrentAvatarTuningKey();
	ImGui::Text("Active: %s", plugin.CurrentAvatarLabel().c_str());
	ImGui::SameLine();
	ImGui::TextDisabled("%s", plugin.AvatarLastUsedLabel(activeKey).c_str());

	DrawTuningScopeSelector();
	DrawAvatarProfiles(plugin);

	std::string selectedKey = plugin.SelectedAvatarTuningKey();
	if (g_state.edit_global) {
		DrawGlobalTuningDetails(plugin);
	}
	else {
		DrawSelectedAvatarDetails(plugin, selectedKey);
	}
	selectedKey = plugin.SelectedAvatarTuningKey();

	const bool editGlobal = g_state.edit_global;
	const FaceShapeScaleArray* stored = FindShapeTuningForAvatar(plugin.Profile().current, selectedKey);
	FaceShapeScaleArray values =
	    editGlobal ? plugin.Profile().current.global_shape_tuning : (stored ? *stored : DefaultFaceShapeScales());

	ImGui::SeparatorText(editGlobal ? "Global expression scales" : "Avatar expression scales");

	const uint32_t overrideCount = editGlobal ? plugin.GlobalOverrideCount() : plugin.AvatarOverrideCount(selectedKey);
	const int groupCount = static_cast<int>(sizeof(kGroupLabels) / sizeof(kGroupLabels[0]));
	g_state.group = std::clamp(g_state.group, 0, groupCount - 1);
	std::vector<uint32_t> visibleIndices = VisibleShapeIndices(values);

	ImGui::TextDisabled("Overrides: %u", (unsigned)overrideCount);
	ImGui::SameLine();
	ImGui::TextDisabled("Visible: %u", (unsigned)visibleIndices.size());
	ImGui::SameLine();
	ImGui::BeginDisabled(overrideCount == 0);
	if (ImGui::SmallButton(editGlobal ? "Reset global##ft_shape_reset_global"
	                                  : "Reset avatar##ft_shape_reset_avatar")) {
		if (editGlobal) {
			plugin.ResetGlobalShapeTuning();
		}
		else {
			plugin.ResetAvatarShapeTuning(selectedKey);
		}
		plugin.Profile().Save();
		values = DefaultFaceShapeScales();
		visibleIndices = VisibleShapeIndices(values);
	}
	ImGui::EndDisabled();
	TooltipForLastItem(editGlobal ? "Reset every global expression scale."
	                              : "Reset every expression scale for the selected avatar.");

	ImGui::SameLine();
	ImGui::BeginDisabled(visibleIndices.empty());
	if (ImGui::SmallButton("Reset visible##ft_shape_reset_visible")) {
		for (uint32_t index : visibleIndices) {
			values[index] = DefaultFaceShapeTuningValue();
			if (editGlobal) {
				plugin.SetGlobalShapeTuning(index, values[index]);
			}
			else {
				plugin.SetAvatarShapeTuning(selectedKey, index, values[index]);
			}
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

	const auto& telemetry = plugin.driver_telemetry_.Snapshot();
	const bool liveValuesAvailable =
	    (editGlobal || selectedKey == activeKey) && telemetry.valid && !telemetry.stale && telemetry.shape_values_valid;

	ImGuiTableFlags flags =
	    ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_Resizable | ImGuiTableFlags_SizingStretchProp;
	{
		TableScope shapeTable("ft_shape_tuning_table", 4, flags);
		if (!shapeTable) return;
		SetupStretchColumn("Expression", 1.0f);
		SetupStretchColumn("Live", 1.35f);
		SetupStretchColumn("Tuning", 1.7f);
		SetupFixedColumn("Action", 74.0f);
		DrawTableHeader();

		for (uint32_t i : visibleIndices) {
			const char* name = facetracking::ExpressionName(i);
			FaceShapeTuningValue tuning = ClampFaceShapeTuningValue(values[i]);

			ImGui::TableNextRow();
			ImGui::PushID(static_cast<int>(i));

			ImGui::TableSetColumnIndex(0);
			ImGui::TextUnformatted(name);
			DrawStatusText(ShapeScaleStateLabel(tuning), ShapeScaleStateTone(tuning));
			if (editGlobal && ActiveAvatarOverridesShape(plugin, i)) {
				DrawStatusText("Avatar override", StatusTone::Warn);
				TooltipForLastItem("The active avatar overrides this global value.");
			}
			else if (!editGlobal && IsDefaultFaceShapeTuningValue(tuning) && GlobalOverridesShape(plugin, i)) {
				DrawStatusText("Global", StatusTone::Info);
				TooltipForLastItem("This avatar inherits the global value for this expression.");
			}

			ImGui::TableSetColumnIndex(1);
			DrawShapeValueMeter(telemetry.pre_tuning_expressions[i], telemetry.post_tuning_expressions[i],
			                    liveValuesAvailable);

			ImGui::TableSetColumnIndex(2);
			bool changed = false;
			bool saveAfterEdit = false;
			int scale = tuning.scale_percent;
			int minValue = tuning.min_percent;
			int maxValue = tuning.max_percent;
			changed |= DrawPercentSlider("Scale##scale", scale, 0, protocol::FACETRACKING_SHAPE_TUNING_MAX_PERCENT,
			                             "Multiplier applied before output limits.", saveAfterEdit);
			changed |= DrawPercentSlider("Min##min", minValue, 0, protocol::FACETRACKING_SHAPE_TUNING_MAX_PERCENT,
			                             "Lowest value this expression can output after scaling.", saveAfterEdit);
			changed |= DrawPercentSlider("Max##max", maxValue, 0, protocol::FACETRACKING_SHAPE_TUNING_MAX_PERCENT,
			                             "Highest value this expression can output after scaling.", saveAfterEdit);
			if (changed) {
				tuning.scale_percent = scale;
				tuning.min_percent = minValue;
				tuning.max_percent = maxValue;
				tuning = ClampFaceShapeTuningValue(tuning);
				values[i] = tuning;
				if (editGlobal) {
					plugin.SetGlobalShapeTuning(i, tuning);
				}
				else {
					plugin.SetAvatarShapeTuning(selectedKey, i, tuning);
				}
			}
			if (saveAfterEdit) plugin.Profile().Save();

			ImGui::TableSetColumnIndex(3);
			if (IsShapeModified(tuning)) {
				if (ImGui::SmallButton("Reset")) {
					values[i] = DefaultFaceShapeTuningValue();
					if (editGlobal) {
						plugin.SetGlobalShapeTuning(i, values[i]);
					}
					else {
						plugin.SetAvatarShapeTuning(selectedKey, i, values[i]);
					}
					plugin.Profile().Save();
				}
			}
			else {
				ImGui::TextDisabled("-");
			}

			ImGui::PopID();
		}
	}

	if (visibleIndices.empty()) {
		DrawEmptyState("No expressions match the current filters.");
	}
}

} // namespace facetracking::ui
