#include "UiResponsive.h"

namespace openvr_pair::overlay::ui {

ResponsiveColumnsScope::ResponsiveColumnsScope(const char* id, int requestedColumns, float minColumnWidth)
{
	const float avail = ImGui::GetContentRegionAvail().x;
	cols_ = ComputeResponsiveColumnCount(avail, requestedColumns, minColumnWidth);
	// No TableSetupColumn calls: with SizingStretchSame and no explicit setup,
	// ImGui creates equal-width stretch columns, which is exactly the grid we
	// want and avoids duplicate column-label IDs.
	open_ = ImGui::BeginTable(id, cols_, ImGuiTableFlags_SizingStretchSame);
	if (open_) {
		ImGui::TableNextRow();
		ImGui::TableSetColumnIndex(0);
	}
}

ResponsiveColumnsScope::~ResponsiveColumnsScope()
{
	if (open_) ImGui::EndTable();
}

void ResponsiveColumnsScope::Next()
{
	if (!open_) return;
	++cellIndex_;
	const int column = cellIndex_ % cols_;
	if (column == 0) ImGui::TableNextRow();
	ImGui::TableSetColumnIndex(column);
}

FlowRowScope::FlowRowScope(float spacing)
{
	spacing_ = (spacing < 0.0f) ? ImGui::GetStyle().ItemSpacing.x : spacing;
}

void FlowRowScope::Item()
{
	if (first_) {
		const ImVec2 cursor = ImGui::GetCursorScreenPos();
		lineStartX_ = cursor.x;
		rightEdge_ = cursor.x + ImGui::GetContentRegionAvail().x;
		first_ = false;
		return;
	}

	// The previous item was just drawn. Estimate the upcoming item's width from
	// it (flow rows hold similarly-sized chips/badges/buttons). Same line when
	// it still fits; otherwise let ImGui drop to the next line on its own.
	const float prevRight = ImGui::GetItemRectMax().x;
	const float prevWidth = ImGui::GetItemRectMax().x - ImGui::GetItemRectMin().x;
	if (!FlowShouldWrap(prevRight, prevWidth, rightEdge_, spacing_)) {
		ImGui::SameLine(0.0f, spacing_);
	}
}

} // namespace openvr_pair::overlay::ui
