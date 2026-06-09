#pragma once

#include "UiControls.h"

#include <imgui.h>

#include <utility>

namespace openvr_pair::overlay::ui {

void ApplyOverlayStyle();

void BeginGroupPanel(const char* name, const ImVec2& size = ImVec2(0.0f, 0.0f));
void EndGroupPanel();

struct PanelScope
{
	explicit PanelScope(const char* name, const ImVec2& size = ImVec2(0.0f, 0.0f));
	~PanelScope();
	PanelScope(const PanelScope&) = delete;
	PanelScope& operator=(const PanelScope&) = delete;
};

struct ChildScope
{
	bool open = false;
	ChildScope(const char* id, const ImVec2& size, ImGuiChildFlags childFlags = ImGuiChildFlags_None,
	           ImGuiWindowFlags windowFlags = ImGuiWindowFlags_None);
	~ChildScope();
	explicit operator bool() const { return open; }
	ChildScope(const ChildScope&) = delete;
	ChildScope& operator=(const ChildScope&) = delete;
};

struct TabBarScope
{
	bool open = false;
	TabBarScope(const char* id, ImGuiTabBarFlags flags = ImGuiTabBarFlags_None);
	~TabBarScope();
	explicit operator bool() const { return open; }
	TabBarScope(const TabBarScope&) = delete;
	TabBarScope& operator=(const TabBarScope&) = delete;
};

struct TabItemScope
{
	bool open = false;
	TabItemScope(const char* label, bool* visible = nullptr, ImGuiTabItemFlags flags = ImGuiTabItemFlags_None);
	~TabItemScope();
	explicit operator bool() const { return open; }
	TabItemScope(const TabItemScope&) = delete;
	TabItemScope& operator=(const TabItemScope&) = delete;
};

template <typename Body> void DrawPanel(const char* name, Body&& body, const ImVec2& size = ImVec2(0.0f, 0.0f))
{
	PanelScope panel(name, size);
	std::forward<Body>(body)();
}

// A group panel whose title is tinted by a status tone -- a self-describing
// section header for state-bearing groups (e.g. "Connection", "Calibration").
// StatusTone::Idle renders an ordinary, untinted panel. The accent colors only
// the title; the body keeps the normal text color. The border follows the
// theme like every other panel.
template <typename Body>
void DrawCard(const char* title, StatusTone accent, Body&& body, const ImVec2& size = ImVec2(0.0f, 0.0f))
{
	const bool tinted = accent != StatusTone::Idle;
	if (tinted) ImGui::PushStyleColor(ImGuiCol_Text, StatusColor(accent));
	BeginGroupPanel(title, size); // draws the title synchronously, so the pop below is safe
	if (tinted) ImGui::PopStyleColor();
	std::forward<Body>(body)();
	EndGroupPanel();
}

template <typename Body>
bool DrawTabItem(const char* label, Body&& body, ImGuiTabItemFlags flags = ImGuiTabItemFlags_None)
{
	TabItemScope tab(label, nullptr, flags);
	if (!tab) return false;
	std::forward<Body>(body)();
	return true;
}

template <typename Body>
bool DrawScrollableTabItem(const char* label, Body&& body, ImGuiTabItemFlags flags = ImGuiTabItemFlags_None)
{
	return DrawTabItem(
	    label,
	    [&] {
		    ChildScope child("##tab_body", ImVec2(0.0f, 0.0f));
		    std::forward<Body>(body)();
	    },
	    flags);
}

} // namespace openvr_pair::overlay::ui
