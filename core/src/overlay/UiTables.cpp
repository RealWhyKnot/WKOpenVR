#include "UiTables.h"

namespace openvr_pair::overlay::ui {

TableScope::TableScope(const char* id, int columns, ImGuiTableFlags flags, const ImVec2& outerSize, float innerWidth)
{
	open = ImGui::BeginTable(id, columns, flags, outerSize, innerWidth);
}

TableScope::~TableScope()
{
	if (open) ImGui::EndTable();
}

SettingTableScope::SettingTableScope(const char* id, float labelWidth)
    : table(id, 2, ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_NoBordersInBody)
{
	if (!table.open) return;
	ImGui::TableSetupColumn("##label", ImGuiTableColumnFlags_WidthFixed, labelWidth);
	ImGui::TableSetupColumn("##control", ImGuiTableColumnFlags_WidthStretch);
}

void SettingTableScope::Row(const char* label)
{
	ImGui::TableNextRow();
	ImGui::TableSetColumnIndex(0);
	ImGui::AlignTextToFramePadding();
	ImGui::TextUnformatted(label ? label : "");
	ImGui::TableSetColumnIndex(1);
}

void SetupStretchColumn(const char* label, float weight)
{
	ImGui::TableSetupColumn(label, ImGuiTableColumnFlags_WidthStretch, weight);
}

void SetupFixedColumn(const char* label, float width)
{
	ImGui::TableSetupColumn(label, ImGuiTableColumnFlags_WidthFixed, width);
}

void DrawTableHeader()
{
	ImGui::TableHeadersRow();
}

void NextRow()
{
	ImGui::TableNextRow();
}

void NextColumn()
{
	ImGui::TableNextColumn();
}

void SetColumn(int column)
{
	ImGui::TableSetColumnIndex(column);
}

void DrawKeyValueRow(const char* label, const char* value)
{
	ImGui::TableNextRow();
	ImGui::TableSetColumnIndex(0);
	ImGui::TextUnformatted(label ? label : "");
	ImGui::TableSetColumnIndex(1);
	ImGui::TextUnformatted(value ? value : "");
}

void DrawStatusCell(const char* text, StatusTone tone, bool rightAlign)
{
	if (rightAlign) {
		RightAlignText(text, StatusColor(tone), tone != StatusTone::Idle);
		return;
	}
	if (tone == StatusTone::Idle) {
		DrawEmptyState(text);
	}
	else {
		DrawStatusText(text, tone);
	}
}

} // namespace openvr_pair::overlay::ui
