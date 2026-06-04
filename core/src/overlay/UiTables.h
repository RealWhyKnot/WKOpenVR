#pragma once

#include "UiControls.h"

#include <imgui.h>

#include <utility>

namespace openvr_pair::overlay::ui {

struct TableScope
{
	bool open = false;
	TableScope(const char* id, int columns, ImGuiTableFlags flags = ImGuiTableFlags_None,
	           const ImVec2& outerSize = ImVec2(0.0f, 0.0f), float innerWidth = 0.0f);
	~TableScope();
	explicit operator bool() const { return open; }
	TableScope(const TableScope&) = delete;
	TableScope& operator=(const TableScope&) = delete;
};

struct SettingTableScope
{
	TableScope table;
	SettingTableScope(const char* id, float labelWidth);
	explicit operator bool() const { return table.open; }
	void Row(const char* label);
	SettingTableScope(const SettingTableScope&) = delete;
	SettingTableScope& operator=(const SettingTableScope&) = delete;
};

void SetupStretchColumn(const char* label, float weight = 1.0f);
void SetupFixedColumn(const char* label, float width);
void DrawTableHeader();
void NextRow();
void NextColumn();
void SetColumn(int column);
void DrawKeyValueRow(const char* label, const char* value);
void DrawStatusCell(const char* text, StatusTone tone, bool rightAlign = false);

template <typename Body> void DrawSettingTable(const char* id, float labelWidth, Body&& body)
{
	SettingTableScope table(id, labelWidth);
	if (!table) return;
	std::forward<Body>(body)(table);
}

template <typename Body> void SettingRow(SettingTableScope& table, const char* label, Body&& body)
{
	table.Row(label);
	std::forward<Body>(body)();
}

} // namespace openvr_pair::overlay::ui
