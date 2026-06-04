#include "imgui_extensions.h"

#include "UiLayout.h"

void ImGui::BeginGroupPanel(const char* name, const ImVec2& size)
{
	openvr_pair::overlay::ui::BeginGroupPanel(name, size);
}

void ImGui::EndGroupPanel()
{
	openvr_pair::overlay::ui::EndGroupPanel();
}
