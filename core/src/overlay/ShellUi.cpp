#include "ShellUi.h"

#include "BugReportUi.h"
#include "DebugLogging.h"
#include "FeaturePlugin.h"
#include "ModuleToggleUi.h"
#include "ShellContext.h"
#include "Theme.h"
#include "UiCore.h"

#include <imgui.h>

#include <cstring>
#include <string>
#include <vector>

namespace openvr_pair::overlay {

namespace {

void DrawTransientStatus(ShellContext &context)
{
	if (context.status.empty()) return;
	ImGui::Separator();
	ui::DrawTextWrapped(context.status.c_str());
}

} // namespace

void DrawFeatureTab(ShellContext &context, FeaturePlugin &plugin, ImGuiTabItemFlags flags = ImGuiTabItemFlags_None)
{
	ui::DrawScrollableTabItem(plugin.Name(), [&] {
		plugin.DrawTab(context);
	}, flags);
}

void DrawLogsTab(ShellContext &context, std::vector<std::unique_ptr<FeaturePlugin>> &plugins)
{
	ui::DrawTextWrapped(
		"Per-module logs. All overlay-side logs land in "
		"%LocalAppDataLow%\\WKOpenVR\\Logs\\; driver-side logs land in "
		"%LocalAppDataLow%\\WKOpenVR\\Logs\\.");
	ImGui::Spacing();

	ui::DrawSectionHeading("Debug logging");
	const bool forced = common::IsDebugLoggingForcedOn();
	bool debugLogging = common::IsDebugLoggingEnabled();
	if (forced) debugLogging = true;
	{
		ui::DisabledSection locked(forced, "Dev builds keep debug logging enabled.");
		if (ui::CheckboxWithTooltip(
				"Enable debug logging", &debugLogging,
				"Release builds stay quiet until this is enabled.\n"
				"Dev builds keep it on so repro sessions leave a diagnostic trail.\n"
				"State is shared by the overlay, driver, and host sidecars.")) {
			common::SetDebugLoggingEnabled(debugLogging);
		}
		locked.AttachReasonTooltip();
	}
	ImGui::SameLine();
	ImGui::TextDisabled(forced ? "(dev build: always on)" : (debugLogging ? "(on)" : "(off)"));

	ImGui::Spacing();
	ui::DrawSectionHeading("Bug reports");
	DrawBugReportButton(context);

	const bool effectiveDebugLogging = common::IsDebugLoggingEnabled();
	for (auto &plugin : plugins) {
		plugin->OnDebugLoggingChanged(effectiveDebugLogging);
	}

	ImGui::Spacing();
	ImGui::Separator();
	ImGui::Spacing();

	bool anyDrawn = false;
	for (auto &plugin : plugins) {
		if (plugin->Channel() == FeaturePluginChannel::DevTools) continue;
		if (!plugin->IsInstalled(context)) continue;
		ImGui::PushID(plugin->Name());
		ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
		if (ImGui::CollapsingHeader(plugin->Name())) {
			ImGui::Indent();
			plugin->DrawLogsSection(context);
			ImGui::Unindent();
		}
		ImGui::PopID();
		anyDrawn = true;
	}
	if (!anyDrawn) {
		ui::DrawEmptyState("No installed feature plugins.");
	}
}

void DrawModulesTab(ShellContext &context, std::vector<std::unique_ptr<FeaturePlugin>> &plugins)
{
	ImGui::TextUnformatted("Modules");
	ui::DrawTextWrapped(
		"Toggle features on or off. Each change pops a UAC prompt. "
		"Changes take effect the next time SteamVR loads the driver.");
	ImGui::Spacing();

	std::vector<FeaturePlugin *> releaseModules;
	for (auto &plugin : plugins) {
		if (ShouldShowInModulesTab(*plugin)) {
			releaseModules.push_back(plugin.get());
		}
	}
	DrawModuleToggleTable(context, releaseModules, "modules", "No release modules were compiled into this build.");
}

void DrawThemesTab(ShellContext &)
{
	ui::DrawSectionHeading("Color theme");
	ui::DrawTextWrapped("Choose a color theme. Changes apply immediately and persist across launches.");
	ImGui::Spacing();

	const ui::ThemeId current = ui::GetCurrentThemeId();
	for (int i = 0; i < (int)ui::ThemeId::Count_; ++i) {
		const ui::ThemeId id = (ui::ThemeId)i;
		const bool selected = (id == current);
		ImGui::PushID(i);
		if (ui::RadioButtonWithTooltip(ui::ThemeName(id), selected, ui::ThemeCaption(id))) {
			ui::SetTheme(id);
		}
		ImGui::SameLine();
		ImGui::TextDisabled("%s", ui::ThemeCaption(id));
		ImGui::PopID();
	}
}

void DrawShellWindow(ShellContext &context, std::vector<std::unique_ptr<FeaturePlugin>> &plugins)
{
	static bool desktopDefaultTabApplied = false;

	const ImGuiViewport *vp = ImGui::GetMainViewport();
	ImGui::SetNextWindowPos(vp->WorkPos);
	ImGui::SetNextWindowSize(vp->WorkSize);
	const ImGuiWindowFlags flags =
		ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
		ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse |
		ImGuiWindowFlags_NoBringToFrontOnFocus |
		ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse;

	ImGui::Begin("WKOpenVR", nullptr, flags);

	const bool hasStatus = !context.status.empty();
	const float statusReserve = hasStatus
		? ImGui::GetTextLineHeightWithSpacing() * 3.0f + ImGui::GetStyle().ItemSpacing.y
		: 0.0f;

	if (ImGui::BeginChild("##shell_content", ImVec2(0.0f, hasStatus ? -statusReserve : 0.0f),
			false, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse)) {
		ui::TabBarScope tabs("tabs");
		if (tabs) {
			for (auto &plugin : plugins) {
				if (!plugin->IsInstalled(context)) continue;
				ImGuiTabItemFlags tabFlags = ImGuiTabItemFlags_None;
				if (!desktopDefaultTabApplied &&
					!context.dashboardVisible &&
					std::strcmp(plugin->FlagFileName(), "enable_questapp.flag") == 0) {
					tabFlags |= ImGuiTabItemFlags_SetSelected;
					desktopDefaultTabApplied = true;
				}
				DrawFeatureTab(context, *plugin, tabFlags);
			}
			ui::DrawScrollableTabItem("Logs", [&] {
				DrawLogsTab(context, plugins);
			});
			ui::DrawScrollableTabItem("Modules", [&] {
				DrawModulesTab(context, plugins);
			});
			ui::DrawScrollableTabItem("Themes", [&] {
				DrawThemesTab(context);
			});
		}
	}
	ImGui::EndChild();

	DrawTransientStatus(context);
	ImGui::End();
}

} // namespace openvr_pair::overlay
