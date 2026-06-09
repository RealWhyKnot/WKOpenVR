#include "TuningTab.h"

#include "FacetrackingPlugin.h"
#include "UiHelpers.h"
#include "facetracking/ExpressionNames.h"

#include <imgui/imgui.h>

#include <algorithm>
#include <cctype>
#include <cstring>
#include <string>

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

uint32_t CountOverrides(const FaceShapeScaleArray& values)
{
	uint32_t count = 0;
	for (int value : values) {
		if (value != protocol::FACETRACKING_SHAPE_TUNING_DEFAULT_PERCENT) ++count;
	}
	return count;
}

} // namespace

void DrawTuningTab(FacetrackingPlugin& plugin)
{
	DrawSectionHeading("Avatar tuning");

	const std::string avatarLabel = plugin.CurrentAvatarLabel();
	ImGui::Text("Avatar: %s", avatarLabel.c_str());
	const FaceShapeScaleArray* stored =
	    FindShapeTuningForAvatar(plugin.profile_.current, plugin.CurrentAvatarTuningKey());
	FaceShapeScaleArray values = stored ? *stored : DefaultFaceShapeScales();

	const uint32_t overrideCount = CountOverrides(values);
	ImGui::TextDisabled("Overrides: %u", (unsigned)overrideCount);
	ImGui::SameLine();
	if (ImGui::Button("Reset avatar##ft_shape_reset_avatar")) {
		plugin.ResetCurrentAvatarShapeTuning();
		plugin.profile_.Save();
		values = DefaultFaceShapeScales();
	}
	TooltipForLastItem("Reset every expression scale for the current avatar.");

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
			plugin.SetCurrentAvatarShapeScale(i, value);
		}
		if (ImGui::IsItemDeactivatedAfterEdit()) {
			plugin.profile_.Save();
		}

		ImGui::TableSetColumnIndex(2);
		if (values[i] != protocol::FACETRACKING_SHAPE_TUNING_DEFAULT_PERCENT) {
			if (ImGui::SmallButton("Reset")) {
				values[i] = protocol::FACETRACKING_SHAPE_TUNING_DEFAULT_PERCENT;
				plugin.SetCurrentAvatarShapeScale(i, protocol::FACETRACKING_SHAPE_TUNING_DEFAULT_PERCENT);
				plugin.profile_.Save();
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
