#include "ShellUi.h"

#include "BugReportUi.h"
#include "DebugLogging.h"
#include "FeaturePlugin.h"
#include "ShellContext.h"
#include "Theme.h"
#include "UiCore.h"

#include <imgui.h>

#include <cstring>
#include <map>
#include <string>

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
	static std::map<std::string, bool> wanted;

	ImGui::TextUnformatted("Modules");
	ui::DrawTextWrapped(
		"Toggle features on or off. Each change pops a UAC prompt. "
		"Changes take effect the next time SteamVR loads the driver.");
	ImGui::Spacing();

	ui::TableScope table("modules", 3,
		ImGuiTableFlags_BordersInnerH | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp);
	if (!table) return;

	ui::SetupStretchColumn("Module", 1.0f);
	ui::SetupFixedColumn("Status", 340.0f);
	ui::SetupFixedColumn("Enabled", 100.0f);
	ui::DrawTableHeader();

	for (auto &plugin : plugins) {
		const bool installed = plugin->IsInstalled(context);
		const std::string key = plugin->FlagFileName();
		const bool isPending = context.IsTogglePending(key.c_str());

		auto it = wanted.find(key);
		if (!isPending && it != wanted.end()) {
			wanted.erase(it);
			it = wanted.end();
		}
		const bool displayState = (it != wanted.end()) ? it->second : installed;

		ui::NextRow();

		ui::NextColumn();
		ImGui::AlignTextToFramePadding();
		ImGui::TextUnformatted(plugin->Name());

		ui::NextColumn();
		ImGui::AlignTextToFramePadding();
		const char *statusText = nullptr;
		ui::StatusTone statusTone = ui::StatusTone::Idle;
		if (isPending) {
			statusText = (it != wanted.end() && it->second)
				? "Enabling -- takes effect on next SteamVR launch"
				: "Disabling -- takes effect on next SteamVR launch";
			statusTone = ui::StatusTone::Pending;
		} else if (installed) {
			statusText = "Enabled";
			statusTone = ui::StatusTone::Ok;
		} else {
			statusText = "Disabled";
		}
		ui::DrawStatusCell(statusText, statusTone, true);

		ui::NextColumn();
		ImGui::PushID(key.c_str());
		const std::string pendingReason =
			"Waiting for the elevated helper to finish. Reopens after SteamVR picks up the change.";
		const bool isRouterRow = (key == "enable_oscrouter.flag");
		const bool routerDependentOn = isRouterRow && displayState &&
			(context.IsFlagPresent("enable_facetracking.flag") ||
			 context.IsFlagPresent("enable_captions.flag"));

		const char *blockReason = nullptr;
		bool blocked = isPending;
		if (isPending) {
			blockReason = pendingReason.c_str();
		} else if (routerDependentOn) {
			blocked = true;
			blockReason =
				"Face Tracking and Captions publish through the OSC Router. "
				"Disable those modules first if you really want to turn the router off.";
		}

		ui::DisabledSection disabled(blocked, blockReason);
		bool checkbox = displayState;
		const std::string tooltip = std::string("Enable or disable ") + plugin->Name() +
			" for this profile. Takes effect next SteamVR launch.";
		if (ui::CheckboxWithTooltip("##enabled", &checkbox, tooltip.c_str())) {
			wanted[key] = checkbox;
			context.SetFlagPresent(plugin->FlagFileName(), checkbox);
		}
		disabled.AttachReasonTooltip();
		ImGui::PopID();
	}
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
