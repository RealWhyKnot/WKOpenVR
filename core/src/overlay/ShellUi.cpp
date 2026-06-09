#include "ShellUi.h"

#include "BugReportUi.h"
#include "DebugLogging.h"
#include "FeaturePlugin.h"
#include "ModuleToggleUi.h"
#include "ShellContext.h"
#include "ShellUiLogic.h"
#include "Theme.h"
#include "UiCore.h"

#include <imgui.h>

#include <cstring>
#include <string>
#include <vector>

namespace openvr_pair::overlay {

namespace {

void DrawTransientStatus(ShellContext& context)
{
	if (context.status.empty()) return;
	ImGui::Separator();
	if (ImGui::SmallButton("x##dismiss_shell_status")) {
		context.ClearStatus();
		return;
	}
	if (ImGui::IsItemHovered()) {
		ImGui::SetTooltip("Dismiss status");
	}
	ImGui::SameLine();
	ui::DrawTextWrapped(context.status.c_str());
}

FeaturePlugin* FindDefaultLogsPanelPlugin(std::vector<std::unique_ptr<FeaturePlugin>>& plugins)
{
	for (auto& plugin : plugins) {
		if (plugin && IsDefaultLogsPanelPlugin(plugin->FlagFileName())) {
			return plugin.get();
		}
	}
	return nullptr;
}

FeaturePlugin* FindFeatureByFlag(const std::vector<FeaturePlugin*>& plugins, std::string_view flag)
{
	for (FeaturePlugin* plugin : plugins) {
		if (plugin && flag == plugin->FlagFileName()) return plugin;
	}
	return nullptr;
}

void DrawFallbackLogsPanel(ShellContext& context)
{
	ui::DrawSectionHeading("Debug logging");
	const bool forced = common::IsDebugLoggingForcedOn();
	bool debugLogging = common::IsDebugLoggingEnabled();
	if (forced) debugLogging = true;
	{
		ui::DisabledSection locked(forced, "Dev builds keep debug logging enabled.");
		if (ui::CheckboxWithTooltip("Enable debug logging", &debugLogging,
		                            "Write WKOpenVR diagnostics to %LocalAppDataLow%\\WKOpenVR\\Logs\\.")) {
			common::SetDebugLoggingEnabled(debugLogging);
		}
		locked.AttachReasonTooltip();
	}
	ImGui::SameLine();
	ImGui::TextDisabled(forced ? "(dev build: always on)" : (debugLogging ? "(on)" : "(off)"));

	ImGui::Spacing();
	ui::DrawSectionHeading("Bug reports");
	DrawBugReportButton(context);
}

void NotifyDebugLoggingChanged(std::vector<std::unique_ptr<FeaturePlugin>>& plugins)
{
	const bool effectiveDebugLogging = common::IsDebugLoggingEnabled();
	for (auto& plugin : plugins) {
		if (plugin) {
			plugin->OnDebugLoggingChanged(effectiveDebugLogging);
		}
	}
}

void DrawFeaturePicker(ShellContext& context, const std::vector<FeaturePlugin*>& installedPlugins,
                       std::string& selectedFeatureFlag)
{
	if (installedPlugins.empty()) {
		ui::DrawSectionHeading("Feature");
		ui::DrawTextWrapped("Enable a module from Modules.");
		return;
	}

	FeaturePlugin* selected = FindFeatureByFlag(installedPlugins, selectedFeatureFlag);
	if (!selected) {
		selected = installedPlugins.front();
		selectedFeatureFlag = selected->FlagFileName();
	}

	ImGui::AlignTextToFramePadding();
	ImGui::TextUnformatted("Module");
	ImGui::SameLine();
	ImGui::SetNextItemWidth(300.0f);
	if (ImGui::BeginCombo("##feature_module_picker", selected->Name())) {
		for (FeaturePlugin* plugin : installedPlugins) {
			const bool isSelected = (plugin == selected);
			if (ImGui::Selectable(plugin->Name(), isSelected)) {
				selected = plugin;
				selectedFeatureFlag = plugin->FlagFileName();
			}
			if (isSelected) ImGui::SetItemDefaultFocus();
		}
		ImGui::EndCombo();
	}

	ImGui::Spacing();
	ImGui::Separator();
	ImGui::Spacing();
	selected->DrawTab(context);
}

} // namespace

void DrawFeatureTab(ShellContext& context, FeaturePlugin& plugin, ImGuiTabItemFlags flags = ImGuiTabItemFlags_None)
{
	ui::DrawScrollableTabItem(plugin.Name(), [&] { plugin.DrawTab(context); }, flags);
}

void DrawLogsTab(ShellContext& context, std::vector<std::unique_ptr<FeaturePlugin>>& plugins)
{
	if (FeaturePlugin* logsPlugin = FindDefaultLogsPanelPlugin(plugins)) {
		logsPlugin->DrawLogsSection(context);
		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();
		ui::DrawSectionHeading("Bug reports");
		DrawBugReportButton(context);
	}
	else {
		DrawFallbackLogsPanel(context);
	}
	NotifyDebugLoggingChanged(plugins);
}

void DrawModulesTab(ShellContext& context, std::vector<std::unique_ptr<FeaturePlugin>>& plugins)
{
	ImGui::TextUnformatted("Modules");
	ui::DrawTextWrapped("Toggle features on or off. Each change pops a UAC prompt. "
	                    "Changes take effect the next time SteamVR loads the driver.");
	ImGui::Spacing();

	std::vector<FeaturePlugin*> modules;
	for (auto& plugin : plugins) {
		if (ShouldShowInModulesTab(*plugin)) {
			modules.push_back(plugin.get());
		}
	}
	ModuleToggleTableOptions options;
	options.markDevelopmentModules = true;
	DrawModuleToggleTable(context, modules, "modules", "No modules were compiled into this build.", options);
}

void DrawThemesTab(ShellContext&)
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

void DrawShellWindow(ShellContext& context, std::vector<std::unique_ptr<FeaturePlugin>>& plugins)
{
	static std::string desktopDefaultTabAppliedFor;
	static std::string selectedFeatureFlag;
	context.TickStatus();

	const ImGuiViewport* vp = ImGui::GetMainViewport();
	ImGui::SetNextWindowPos(vp->WorkPos);
	ImGui::SetNextWindowSize(vp->WorkSize);
	const ImGuiWindowFlags flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
	                               ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoBringToFrontOnFocus |
	                               ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse;

	ImGui::Begin("WKOpenVR", nullptr, flags);

	const bool hasStatus = !context.status.empty();
	const float statusReserve =
	    hasStatus ? ImGui::GetTextLineHeightWithSpacing() * 3.0f + ImGui::GetStyle().ItemSpacing.y : 0.0f;

	if (ImGui::BeginChild("##shell_content", ImVec2(0.0f, hasStatus ? -statusReserve : 0.0f), false,
	                      ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse)) {
		ui::TabBarScope tabs("tabs");
		if (tabs) {
			std::vector<FeaturePlugin*> installedPlugins;
			std::vector<std::string_view> installedFlags;
			for (auto& plugin : plugins) {
				if (!plugin || !plugin->IsInstalled(context)) continue;
				installedPlugins.push_back(plugin.get());
				installedFlags.push_back(plugin->FlagFileName());
			}

			const std::string desktopDefaultFlag = context.DesktopDefaultModuleFlagFileName();
			const FeaturePickerSelection selection =
			    ResolveFeaturePickerSelection(context.vrConnected, selectedFeatureFlag, desktopDefaultFlag,
			                                  desktopDefaultTabAppliedFor, installedFlags);
			ImGuiTabItemFlags featureTabFlags = ImGuiTabItemFlags_None;
			if (!selection.flag.empty()) {
				selectedFeatureFlag.assign(selection.flag.data(), selection.flag.size());
				if (selection.applyDesktopDefault) {
					featureTabFlags |= ImGuiTabItemFlags_SetSelected;
					desktopDefaultTabAppliedFor = desktopDefaultFlag;
				}
			}

			ui::DrawScrollableTabItem(
			    "Feature", [&] { DrawFeaturePicker(context, installedPlugins, selectedFeatureFlag); }, featureTabFlags);
			ui::DrawScrollableTabItem("Logs", [&] { DrawLogsTab(context, plugins); });
			ui::DrawScrollableTabItem("Modules", [&] { DrawModulesTab(context, plugins); });
			ui::DrawScrollableTabItem("Themes", [&] { DrawThemesTab(context); });
		}
	}
	ImGui::EndChild();

	DrawTransientStatus(context);
	ImGui::End();
}

} // namespace openvr_pair::overlay
