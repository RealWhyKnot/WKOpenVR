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

#include <algorithm>
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

enum class ShellBuiltInTab
{
	None,
	Logs,
	Modules,
	Themes,
};

struct ShellTabEntry
{
	std::string key;
	const char* label = nullptr;
	FeaturePlugin* plugin = nullptr;
	ShellBuiltInTab builtIn = ShellBuiltInTab::None;
};

constexpr std::string_view kModuleTabPrefix = "module:";
constexpr const char* kLogsTabKey = "shell:logs";
constexpr const char* kModulesTabKey = "shell:modules";
constexpr const char* kThemesTabKey = "shell:themes";

std::string ModuleTabKey(std::string_view flag)
{
	return std::string(kModuleTabPrefix) + std::string(flag);
}

std::string FeatureTabKey(const FeaturePlugin& plugin)
{
	const char* flag = plugin.FlagFileName();
	if (flag && IsValidModuleFlagForShellOrder(flag)) return ModuleTabKey(flag);
	return std::string("plugin:") + (plugin.Name() ? plugin.Name() : "");
}

std::vector<std::string_view> ModuleFlagsFromPlugins(const std::vector<FeaturePlugin*>& plugins)
{
	std::vector<std::string_view> flags;
	for (FeaturePlugin* plugin : plugins) {
		if (!plugin) continue;
		const char* flag = plugin->FlagFileName();
		if (flag && IsValidModuleFlagForShellOrder(flag)) {
			flags.push_back(flag);
		}
	}
	return flags;
}

std::vector<FeaturePlugin*> OrderPluginsByModuleTabOrder(const std::vector<FeaturePlugin*>& plugins,
                                                         const std::vector<std::string>& preferredOrder)
{
	std::vector<FeaturePlugin*> ordered;
	const std::vector<std::string> resolvedOrder =
	    ResolveModuleTabOrder(preferredOrder, ModuleFlagsFromPlugins(plugins));
	for (const std::string& flag : resolvedOrder) {
		if (FeaturePlugin* plugin = FindFeatureByFlag(plugins, flag)) {
			ordered.push_back(plugin);
		}
	}
	for (FeaturePlugin* plugin : plugins) {
		if (!plugin) continue;
		const char* flag = plugin->FlagFileName();
		if (flag && IsValidModuleFlagForShellOrder(flag)) continue;
		ordered.push_back(plugin);
	}
	return ordered;
}

bool ContainsShellTabKey(const std::vector<ShellTabEntry>& entries, std::string_view key)
{
	for (const ShellTabEntry& entry : entries) {
		if (entry.key == key) return true;
	}
	return false;
}

int ShellTabIndex(const std::vector<ShellTabEntry>& entries, std::string_view key)
{
	for (size_t i = 0; i < entries.size(); ++i) {
		if (entries[i].key == key) return static_cast<int>(i);
	}
	return -1;
}

void MoveShellTabSelection(const std::vector<ShellTabEntry>& entries, std::string& selectedKey, int offset)
{
	if (entries.empty() || offset == 0) return;
	int index = ShellTabIndex(entries, selectedKey);
	if (index < 0) index = 0;
	const int target = std::clamp(index + offset, 0, static_cast<int>(entries.size()) - 1);
	selectedKey = entries[static_cast<size_t>(target)].key;
}

std::vector<ShellTabEntry> BuildShellTabEntries(const std::vector<FeaturePlugin*>& installedPlugins)
{
	std::vector<ShellTabEntry> entries;
	entries.reserve(installedPlugins.size() + 3);
	for (FeaturePlugin* plugin : installedPlugins) {
		if (!plugin) continue;
		entries.push_back({FeatureTabKey(*plugin), plugin->Name(), plugin, ShellBuiltInTab::None});
	}
	entries.push_back({kLogsTabKey, "Logs", nullptr, ShellBuiltInTab::Logs});
	entries.push_back({kModulesTabKey, "Modules", nullptr, ShellBuiltInTab::Modules});
	entries.push_back({kThemesTabKey, "Themes", nullptr, ShellBuiltInTab::Themes});
	return entries;
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

void ResolveShellTabSelection(ShellContext& context, const std::vector<ShellTabEntry>& entries,
                              const std::vector<std::string_view>& installedFlags, std::string& selectedShellTabKey,
                              std::string& desktopDefaultTabAppliedFor)
{
	const std::string desktopDefaultFlag = context.DesktopDefaultModuleFlagFileName();
	if (!context.vrConnected && !desktopDefaultFlag.empty() && desktopDefaultTabAppliedFor != desktopDefaultFlag &&
	    ContainsFeatureFlag(installedFlags, desktopDefaultFlag)) {
		selectedShellTabKey = ModuleTabKey(desktopDefaultFlag);
		desktopDefaultTabAppliedFor = desktopDefaultFlag;
	}
	if (!ContainsShellTabKey(entries, selectedShellTabKey)) {
		selectedShellTabKey = installedFlags.empty() && ContainsShellTabKey(entries, kModulesTabKey)
		                          ? std::string(kModulesTabKey)
		                          : (entries.empty() ? std::string() : entries.front().key);
	}
}

const ShellTabEntry* FindShellTabEntry(const std::vector<ShellTabEntry>& entries, std::string_view key)
{
	for (const ShellTabEntry& entry : entries) {
		if (entry.key == key) return &entry;
	}
	return nullptr;
}

void DrawShellTabStrip(const std::vector<ShellTabEntry>& entries, std::string& selectedShellTabKey)
{
	if (entries.empty()) return;

	const int selectedIndex = ShellTabIndex(entries, selectedShellTabKey);
	const bool canMoveLeft = selectedIndex > 0;
	const ImGuiStyle& style = ImGui::GetStyle();
	const float buttonSize = ImGui::GetFrameHeight();

	{
		ui::DisabledSection disabled(!canMoveLeft, canMoveLeft ? nullptr : "This is the first tab.");
		if (ImGui::ArrowButton("##shell_tab_left", ImGuiDir_Left)) {
			MoveShellTabSelection(entries, selectedShellTabKey, -1);
		}
		disabled.AttachReasonTooltip();
	}
	if (ImGui::IsItemHovered()) {
		ImGui::SetTooltip("Previous tab");
	}

	ImGui::SameLine();
	const float stripWidth = std::max(120.0f, ImGui::GetContentRegionAvail().x - buttonSize - style.ItemSpacing.x);
	ui::ChildScope strip("##shell_tab_strip",
	                     ImVec2(stripWidth, ImGui::GetFrameHeightWithSpacing() + style.ItemSpacing.y),
	                     ImGuiChildFlags_None, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
	if (strip) {
		ui::TabBarScope tabs("tabs", ImGuiTabBarFlags_FittingPolicyScroll | ImGuiTabBarFlags_NoTabListScrollingButtons);
		if (tabs) {
			for (const ShellTabEntry& entry : entries) {
				ImGuiTabItemFlags flags = ImGuiTabItemFlags_None;
				if (entry.key == selectedShellTabKey) {
					flags |= ImGuiTabItemFlags_SetSelected;
				}
				ui::TabItemScope tab(entry.label, nullptr, flags);
				if (tab) {
					selectedShellTabKey = entry.key;
				}
			}
		}
	}

	ImGui::SameLine();
	const int rightSelectedIndex = ShellTabIndex(entries, selectedShellTabKey);
	const bool canMoveRight = rightSelectedIndex >= 0 && rightSelectedIndex + 1 < static_cast<int>(entries.size());
	{
		ui::DisabledSection disabled(!canMoveRight, canMoveRight ? nullptr : "This is the last tab.");
		if (ImGui::ArrowButton("##shell_tab_right", ImGuiDir_Right)) {
			MoveShellTabSelection(entries, selectedShellTabKey, 1);
		}
		disabled.AttachReasonTooltip();
	}
	if (ImGui::IsItemHovered()) {
		ImGui::SetTooltip("Next tab");
	}
}

} // namespace

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
	modules = OrderPluginsByModuleTabOrder(modules, context.ModuleTabOrder());
	ModuleToggleTableOptions options;
	options.markDevelopmentModules = true;
	options.allowTabReorder = true;
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
	static std::string selectedShellTabKey;
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
		std::vector<FeaturePlugin*> installedPlugins;
		for (auto& plugin : plugins) {
			if (!plugin || !plugin->IsInstalled(context)) continue;
			installedPlugins.push_back(plugin.get());
		}
		installedPlugins = OrderPluginsByModuleTabOrder(installedPlugins, context.ModuleTabOrder());
		const std::vector<std::string_view> installedFlags = ModuleFlagsFromPlugins(installedPlugins);
		const std::vector<ShellTabEntry> entries = BuildShellTabEntries(installedPlugins);
		ResolveShellTabSelection(context, entries, installedFlags, selectedShellTabKey, desktopDefaultTabAppliedFor);
		DrawShellTabStrip(entries, selectedShellTabKey);

		ImGui::Spacing();
		ui::ChildScope body("##shell_tab_body", ImVec2(0.0f, 0.0f));
		if (body) {
			const ShellTabEntry* active = FindShellTabEntry(entries, selectedShellTabKey);
			if (active && active->plugin) {
				active->plugin->DrawTab(context);
			}
			else if (active && active->builtIn == ShellBuiltInTab::Logs) {
				DrawLogsTab(context, plugins);
			}
			else if (active && active->builtIn == ShellBuiltInTab::Modules) {
				DrawModulesTab(context, plugins);
			}
			else if (active && active->builtIn == ShellBuiltInTab::Themes) {
				DrawThemesTab(context);
			}
		}
	}
	ImGui::EndChild();

	DrawTransientStatus(context);
	ImGui::End();
}

} // namespace openvr_pair::overlay
