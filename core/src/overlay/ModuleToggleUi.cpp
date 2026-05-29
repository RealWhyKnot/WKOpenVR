#include "ModuleToggleUi.h"

#include "UiCore.h"

#include <imgui.h>

#include <map>
#include <string>

namespace openvr_pair::overlay {
namespace {

std::map<std::string, bool> g_wantedModuleStates;

} // namespace

void DrawModuleToggleTable(
	ShellContext &context,
	const std::vector<FeaturePlugin *> &plugins,
	const char *tableId,
	const char *emptyMessage)
{
	if (plugins.empty()) {
		ui::DrawEmptyState(emptyMessage ? emptyMessage : "No modules.");
		return;
	}

	ui::TableScope table(tableId, 3,
		ImGuiTableFlags_BordersInnerH | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp);
	if (!table) return;

	ui::SetupStretchColumn("Module", 1.0f);
	ui::SetupFixedColumn("Status", 340.0f);
	ui::SetupFixedColumn("Enabled", 100.0f);
	ui::DrawTableHeader();

	for (FeaturePlugin *plugin : plugins) {
		if (!plugin) continue;
		const bool installed = plugin->IsInstalled(context);
		const std::string key = plugin->FlagFileName();
		const bool isPending = context.IsTogglePending(key.c_str());

		auto it = g_wantedModuleStates.find(key);
		if (!isPending && it != g_wantedModuleStates.end()) {
			g_wantedModuleStates.erase(it);
			it = g_wantedModuleStates.end();
		}
		const bool displayState = (it != g_wantedModuleStates.end()) ? it->second : installed;

		ui::NextRow();

		ui::NextColumn();
		ImGui::AlignTextToFramePadding();
		ImGui::TextUnformatted(plugin->Name());

		ui::NextColumn();
		ImGui::AlignTextToFramePadding();
		const char *statusText = nullptr;
		ui::StatusTone statusTone = ui::StatusTone::Idle;
		if (isPending) {
			statusText = (it != g_wantedModuleStates.end() && it->second)
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
			g_wantedModuleStates[key] = checkbox;
			context.SetFlagPresent(plugin->FlagFileName(), checkbox);
		}
		disabled.AttachReasonTooltip();
		ImGui::PopID();
	}
}

} // namespace openvr_pair::overlay
