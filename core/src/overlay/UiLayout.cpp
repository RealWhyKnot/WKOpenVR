#include "UiLayout.h"

#include <imgui_internal.h>

namespace openvr_pair::overlay::ui {

namespace {

ImVector<ImRect> g_groupPanelLabelStack;

} // namespace

void ApplyOverlayStyle()
{
	ImGuiStyle& style = ImGui::GetStyle();

	style.WindowPadding = ImVec2(12.0f, 10.0f);
	style.FramePadding = ImVec2(8.0f, 4.0f);
	style.CellPadding = ImVec2(8.0f, 4.0f);
	style.ItemSpacing = ImVec2(10.0f, 6.0f);
	style.ItemInnerSpacing = ImVec2(8.0f, 4.0f);
	style.IndentSpacing = 20.0f;
	style.ScrollbarSize = 14.0f;

	// Rounding + a hairline window border give every theme a softer, more
	// cohesive frame. These are geometry only, so they survive a theme switch
	// (SetTheme rewrites colors, never the style metrics).
	style.WindowRounding = 6.0f;
	style.ChildRounding = 6.0f;
	style.FrameRounding = 4.0f;
	style.PopupRounding = 4.0f;
	style.ScrollbarRounding = 6.0f;
	style.TabRounding = 4.0f;
	style.GrabRounding = 4.0f;
	style.WindowBorderSize = 1.0f;
}

void BeginGroupPanel(const char* name, const ImVec2& size)
{
	ImGui::BeginGroup();

	const ImGuiStyle& style = ImGui::GetStyle();
	const ImVec2 itemSpacing = style.ItemSpacing;
	ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(0.0f, 0.0f));
	ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(0.0f, 0.0f));

	const float frameHeight = ImGui::GetFrameHeight();
	ImGui::BeginGroup();

	ImVec2 effectiveSize = size;
	if (size.x < 0.0f) {
		effectiveSize.x = ImGui::GetContentRegionAvail().x;
	}
	else {
		effectiveSize.x = size.x;
	}
	ImGui::Dummy(ImVec2(effectiveSize.x, 0.0f));

	ImGui::Dummy(ImVec2(frameHeight * 0.5f, 0.0f));
	ImGui::SameLine(0.0f, 0.0f);
	ImGui::BeginGroup();
	ImGui::Dummy(ImVec2(frameHeight * 0.5f, 0.0f));
	ImGui::SameLine(0.0f, 0.0f);
	ImGui::TextUnformatted(name ? name : "");
	const ImVec2 labelMin = ImGui::GetItemRectMin();
	const ImVec2 labelMax = ImGui::GetItemRectMax();
	ImGui::SameLine(0.0f, 0.0f);
	ImGui::Dummy(ImVec2(0.0f, frameHeight + itemSpacing.y));
	ImGui::BeginGroup();

	ImGui::PopStyleVar(2);

#if IMGUI_VERSION_NUM >= 17301
	ImGui::GetCurrentWindow()->ContentRegionRect.Max.x -= frameHeight * 0.5f;
	ImGui::GetCurrentWindow()->WorkRect.Max.x -= frameHeight * 0.5f;
	ImGui::GetCurrentWindow()->InnerRect.Max.x -= frameHeight * 0.5f;
#else
	ImGui::GetCurrentWindow()->ContentsRegionRect.Max.x -= frameHeight * 0.5f;
#endif
	ImGui::GetCurrentWindow()->Size.x -= frameHeight;

	const float itemWidth = ImGui::CalcItemWidth();
	ImGui::PushItemWidth(ImMax(0.0f, itemWidth - frameHeight));

	g_groupPanelLabelStack.push_back(ImRect(labelMin, labelMax));
}

void EndGroupPanel()
{
	ImGui::PopItemWidth();

	const ImGuiStyle& style = ImGui::GetStyle();
	const ImVec2 itemSpacing = style.ItemSpacing;

	ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(0.0f, 0.0f));
	ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(0.0f, 0.0f));

	const float frameHeight = ImGui::GetFrameHeight();

	ImGui::EndGroup();
	ImGui::EndGroup();

	ImGui::SameLine(0.0f, 0.0f);
	ImGui::Dummy(ImVec2(frameHeight * 0.5f, 0.0f));
	ImGui::Dummy(ImVec2(0.0f, frameHeight - frameHeight * 0.5f - itemSpacing.y));

	ImGui::EndGroup();

	const ImVec2 itemMin = ImGui::GetItemRectMin();
	const ImVec2 itemMax = ImGui::GetItemRectMax();
	const ImRect labelRectOriginal = g_groupPanelLabelStack.back();
	g_groupPanelLabelStack.pop_back();

	const ImVec2 halfFrame = ImVec2(frameHeight * 0.25f, frameHeight) * 0.5f;
	const ImRect frameRect(itemMin + halfFrame, itemMax - ImVec2(halfFrame.x, 0.0f));
	ImRect labelRect = labelRectOriginal;
	labelRect.Min.x -= itemSpacing.x;
	labelRect.Max.x += itemSpacing.x;
	for (int i = 0; i < 4; ++i) {
		switch (i) {
			case 0:
				ImGui::PushClipRect(ImVec2(-FLT_MAX, -FLT_MAX), ImVec2(labelRect.Min.x, FLT_MAX), true);
				break;
			case 1:
				ImGui::PushClipRect(ImVec2(labelRect.Max.x, -FLT_MAX), ImVec2(FLT_MAX, FLT_MAX), true);
				break;
			case 2:
				ImGui::PushClipRect(ImVec2(labelRect.Min.x, -FLT_MAX), ImVec2(labelRect.Max.x, labelRect.Min.y), true);
				break;
			case 3:
				ImGui::PushClipRect(ImVec2(labelRect.Min.x, labelRect.Max.y), ImVec2(labelRect.Max.x, FLT_MAX), true);
				break;
		}

		ImGui::GetWindowDrawList()->AddRect(frameRect.Min, frameRect.Max,
		                                    ImColor(ImGui::GetStyleColorVec4(ImGuiCol_Border)), halfFrame.x);

		ImGui::PopClipRect();
	}

	ImGui::PopStyleVar(2);

#if IMGUI_VERSION_NUM >= 17301
	ImGui::GetCurrentWindow()->ContentRegionRect.Max.x += frameHeight * 0.5f;
	ImGui::GetCurrentWindow()->WorkRect.Max.x += frameHeight * 0.5f;
	ImGui::GetCurrentWindow()->InnerRect.Max.x += frameHeight * 0.5f;
#else
	ImGui::GetCurrentWindow()->ContentsRegionRect.Max.x += frameHeight * 0.5f;
#endif
	ImGui::GetCurrentWindow()->Size.x += frameHeight;

	ImGui::Dummy(ImVec2(0.0f, 0.0f));
	ImGui::EndGroup();
}

PanelScope::PanelScope(const char* name, const ImVec2& size)
{
	BeginGroupPanel(name, size);
}

PanelScope::~PanelScope()
{
	EndGroupPanel();
}

ChildScope::ChildScope(const char* id, const ImVec2& size, ImGuiChildFlags childFlags, ImGuiWindowFlags windowFlags)
{
	open = ImGui::BeginChild(id, size, childFlags, windowFlags);
}

ChildScope::~ChildScope()
{
	ImGui::EndChild();
}

TabBarScope::TabBarScope(const char* id, ImGuiTabBarFlags flags)
{
	open = ImGui::BeginTabBar(id, flags);
}

TabBarScope::~TabBarScope()
{
	if (open) ImGui::EndTabBar();
}

TabItemScope::TabItemScope(const char* label, bool* visible, ImGuiTabItemFlags flags)
{
	open = ImGui::BeginTabItem(label, visible, flags);
}

TabItemScope::~TabItemScope()
{
	if (open) ImGui::EndTabItem();
}

} // namespace openvr_pair::overlay::ui
