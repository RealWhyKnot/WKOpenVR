#include "ShellUi.h"

#include "BugReportUi.h"
#include "DebugLogging.h"
#include "FeaturePlugin.h"
#include "ModuleToggleUi.h"
#include "ShellContext.h"
#include "ShellUiLogic.h"
#include "Theme.h"
#include "UiCore.h"
#include "UpdateNotice.h"

#include <imgui.h>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <shellapi.h>
#endif

#include <algorithm>
#include <cstdio>
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

std::string FormatUpdateDownloadProgress(const UpdateInstallState& install)
{
	if (install.totalBytes <= 0) return {};
	const double pct =
	    install.bytesDownloaded <= 0
	        ? 0.0
	        : (100.0 * static_cast<double>(install.bytesDownloaded) / static_cast<double>(install.totalBytes));
	char buf[160] = {};
	snprintf(buf, sizeof(buf), "%s of %s (%.0f%%)", ui::FormatByteCount(install.bytesDownloaded).c_str(),
	         ui::FormatByteCount(install.totalBytes).c_str(), pct);
	return buf;
}

void DrawUpdatePrompt(ShellContext& context, const std::vector<std::string_view>& installedFlags)
{
	const UpdateNoticeState notice = GetUpdateNoticeState();
	const UpdateInstallState& install = notice.install;
	const bool showPrompt =
	    notice.available || install.queuedForSteamVrExit || install.phase == UpdateInstallPhase::Failed;
	if (!showPrompt) return;

	ui::StatusTone tone = ui::StatusTone::Info;
	if (install.phase == UpdateInstallPhase::Failed) tone = ui::StatusTone::Error;
	if (install.phase == UpdateInstallPhase::Downloading) tone = ui::StatusTone::Warn;
	if (install.phase == UpdateInstallPhase::Ready || install.phase == UpdateInstallPhase::Launching) {
		tone = ui::StatusTone::Ok;
	}

	ui::DrawCard("Update pending", tone, [&] {
		const std::string version = !notice.latestVersion.empty() ? notice.latestVersion : install.targetVersion;
		if (!version.empty()) {
			ImGui::TextUnformatted(("Version v" + version).c_str());
		}

		if (install.phase == UpdateInstallPhase::Downloading) {
			std::string progress = FormatUpdateDownloadProgress(install);
			ui::DrawTextWrapped(progress.empty() ? "Downloading and checking installer..."
			                                     : ("Downloading and checking installer: " + progress).c_str());
		}
		else if (install.phase == UpdateInstallPhase::Ready) {
			ui::DrawTextWrapped("Installer verified. It will start after SteamVR closes.");
		}
		else if (install.phase == UpdateInstallPhase::Launching) {
			ui::DrawTextWrapped("Installer will open after WKOpenVR exits.");
		}
		else if (install.phase == UpdateInstallPhase::Failed) {
			ui::DrawTextWrapped(install.errorMessage.empty() ? "Update download failed."
			                                                 : install.errorMessage.c_str());
		}
		else {
			ui::DrawTextWrapped("Queue the installer now. It will run after SteamVR closes.");
		}

		ImGui::Spacing();
		if (install.phase == UpdateInstallPhase::Idle || install.phase == UpdateInstallPhase::Failed) {
			if (ImGui::Button("Queue update")) {
				std::string error;
				if (!QueueUpdateForSteamVrClose(installedFlags, &error)) {
					context.SetStatus(error.empty() ? "Update was not queued." : error);
				}
				else {
					context.SetStatus("Update queued for the next SteamVR close.");
				}
			}
			if (ImGui::IsItemHovered()) {
				ImGui::SetTooltip("Download, verify, and queue the installer.");
			}
		}
		else {
			if (ImGui::Button("Cancel update")) {
				CancelQueuedUpdate();
				context.SetStatus("Update queue cancelled.");
			}
			if (ImGui::IsItemHovered()) {
				ImGui::SetTooltip("Cancel the queued installer.");
			}
		}

		if (!notice.releaseUrl.empty()) {
			ImGui::SameLine();
			if (ImGui::Button("Release notes")) {
#ifdef _WIN32
				ShellExecuteA(nullptr, "open", notice.releaseUrl.c_str(), nullptr, nullptr, SW_SHOWNORMAL);
#endif
			}
			if (ImGui::IsItemHovered()) {
				ImGui::SetTooltip("%s", notice.releaseUrl.c_str());
			}
		}
	});
	ImGui::Spacing();
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
                              std::string& desktopDefaultTabAppliedFor, std::string& pendingTabJumpKey)
{
	const std::string desktopDefaultFlag = context.DesktopDefaultModuleFlagFileName();
	if (!context.vrConnected && !desktopDefaultFlag.empty() && desktopDefaultTabAppliedFor != desktopDefaultFlag &&
	    ContainsFeatureFlag(installedFlags, desktopDefaultFlag)) {
		selectedShellTabKey = ModuleTabKey(desktopDefaultFlag);
		pendingTabJumpKey = selectedShellTabKey;
		desktopDefaultTabAppliedFor = desktopDefaultFlag;
	}
	if (!ContainsShellTabKey(entries, selectedShellTabKey)) {
		selectedShellTabKey = installedFlags.empty() && ContainsShellTabKey(entries, kModulesTabKey)
		                          ? std::string(kModulesTabKey)
		                          : (entries.empty() ? std::string() : entries.front().key);
		pendingTabJumpKey = selectedShellTabKey;
	}
}

const ShellTabEntry* FindShellTabEntry(const std::vector<ShellTabEntry>& entries, std::string_view key)
{
	for (const ShellTabEntry& entry : entries) {
		if (entry.key == key) return &entry;
	}
	return nullptr;
}

void DrawShellTabStrip(const std::vector<ShellTabEntry>& entries, std::string& selectedShellTabKey,
                       std::string& pendingTabJumpKey)
{
	if (entries.empty()) {
		pendingTabJumpKey.clear();
		return;
	}

	const int selectedIndex = ShellTabIndex(entries, selectedShellTabKey);
	const bool canMoveLeft = selectedIndex > 0;
	const ImGuiStyle& style = ImGui::GetStyle();
	const float buttonSize = ImGui::GetFrameHeight();

	// Previous-tab arrow. Replaced by a same-size spacer on the first tab so the
	// control only shows when it can do something and the strip never shifts.
	if (canMoveLeft) {
		if (ImGui::ArrowButton("##shell_tab_left", ImGuiDir_Left)) {
			std::string target = selectedShellTabKey;
			MoveShellTabSelection(entries, target, -1);
			pendingTabJumpKey = target;
		}
		if (ImGui::IsItemHovered()) {
			ImGui::SetTooltip("Previous tab");
		}
	}
	else {
		ImGui::Dummy(ImVec2(buttonSize, buttonSize));
	}

	ImGui::SameLine();
	// Reserve the next-tab button plus a spacing of margin so it never lands on
	// the content clip edge (which would clip it out of view entirely).
	const float stripWidth =
	    std::max(120.0f, ImGui::GetContentRegionAvail().x - buttonSize - style.ItemSpacing.x * 2.0f);
	{
		ui::ChildScope strip("##shell_tab_strip",
		                     ImVec2(stripWidth, ImGui::GetFrameHeightWithSpacing() + style.ItemSpacing.y),
		                     ImGuiChildFlags_None, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
		if (strip) {
			ui::TabBarScope tabs("tabs",
			                     ImGuiTabBarFlags_FittingPolicyScroll | ImGuiTabBarFlags_NoTabListScrollingButtons);
			if (tabs) {
				for (const ShellTabEntry& entry : entries) {
					// SetSelected is applied only on the single frame a jump was
					// requested (arrow button or desktop default). Forcing it every
					// frame would override the user's click and snap the selection
					// back to the tracked tab; ImGui owns the selection otherwise.
					ImGuiTabItemFlags flags = ImGuiTabItemFlags_None;
					if (!pendingTabJumpKey.empty() && entry.key == pendingTabJumpKey) {
						flags |= ImGuiTabItemFlags_SetSelected;
					}
					ui::TabItemScope tab(entry.label, nullptr, flags);
					if (tab) {
						selectedShellTabKey = entry.key;
					}
				}
			}
		}
	}
	// A jump request lives for exactly one strip draw. Clearing it here lets the
	// next-tab arrow below schedule a fresh jump for the following frame.
	pendingTabJumpKey.clear();

	ImGui::SameLine();
	const int rightSelectedIndex = ShellTabIndex(entries, selectedShellTabKey);
	const bool canMoveRight = rightSelectedIndex >= 0 && rightSelectedIndex + 1 < static_cast<int>(entries.size());
	if (canMoveRight) {
		if (ImGui::ArrowButton("##shell_tab_right", ImGuiDir_Right)) {
			std::string target = selectedShellTabKey;
			MoveShellTabSelection(entries, target, 1);
			pendingTabJumpKey = target;
		}
		if (ImGui::IsItemHovered()) {
			ImGui::SetTooltip("Next tab");
		}
	}
	else {
		ImGui::Dummy(ImVec2(buttonSize, buttonSize));
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
	static std::string pendingTabJumpKey;
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
		ResolveShellTabSelection(context, entries, installedFlags, selectedShellTabKey, desktopDefaultTabAppliedFor,
		                         pendingTabJumpKey);
		DrawUpdatePrompt(context, installedFlags);
		DrawShellTabStrip(entries, selectedShellTabKey, pendingTabJumpKey);

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
