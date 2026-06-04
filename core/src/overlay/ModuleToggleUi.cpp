#include "ModuleToggleUi.h"

#include "ModuleRegistry.h"
#include "UiCore.h"

#include <imgui.h>

#include <map>
#include <string>

namespace openvr_pair::overlay {
namespace {

namespace module_registry = openvr_pair::common::modules;

std::map<std::string, bool> g_wantedModuleStates;

bool IsEffectiveModuleEnabled(
	ShellContext &context,
	const module_registry::ModuleInfo &module)
{
	return context.IsFlagPresent(module.flag_file) &&
		!context.IsModuleAutoDisabled(module.flag_file);
}

} // namespace

void DrawModuleToggleTable(
	ShellContext &context,
	const std::vector<FeaturePlugin *> &plugins,
	const char *tableId,
	const char *emptyMessage,
	ModuleToggleTableOptions options)
{
	if (plugins.empty()) {
		ui::DrawEmptyState(emptyMessage ? emptyMessage : "No modules.");
		return;
	}

	ui::TableScope table(tableId, 4,
		ImGuiTableFlags_BordersInnerH | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp);
	if (!table) return;

	ui::SetupStretchColumn("Module", 1.0f);
	ui::SetupFixedColumn("Status", 300.0f);
	ui::SetupFixedColumn("Default", 90.0f);
	ui::SetupFixedColumn("Enabled", 100.0f);
	ui::DrawTableHeader();

	for (FeaturePlugin *plugin : plugins) {
		if (!plugin) continue;
		const bool installed = plugin->IsInstalled(context);
		const std::string key = plugin->FlagFileName();
		const module_registry::ModuleInfo *module = module_registry::FindByFlagFileName(key);
		const bool isPending = context.IsTogglePending(key.c_str());
		const bool autoDisabled = context.IsModuleAutoDisabled(key.c_str());
		const bool blockedByRouterSafety =
			module && module->requires_osc_router &&
			context.IsModuleAutoDisabled(module_registry::FlagFileName(module_registry::ModuleId::OscRouter));

		auto it = g_wantedModuleStates.find(key);
		if (!isPending && it != g_wantedModuleStates.end()) {
			g_wantedModuleStates.erase(it);
			it = g_wantedModuleStates.end();
		}
		const bool effectiveInstalled = installed && !autoDisabled && !blockedByRouterSafety;
		const bool displayState = (it != g_wantedModuleStates.end()) ? it->second : effectiveInstalled;

		ui::NextRow();

		ui::NextColumn();
		ImGui::AlignTextToFramePadding();
		const bool markDevelopment =
			options.markDevelopmentModules
			&& plugin->Channel() == FeaturePluginChannel::Development;
		if (markDevelopment) {
			const char *marker = (options.developmentMarker && options.developmentMarker[0])
				? options.developmentMarker
				: "(dev only)";
			const char *tooltip = (options.developmentTooltip && options.developmentTooltip[0])
				? options.developmentTooltip
				: "This module is compiled into dev builds and omitted from release builds.";
			ui::ScopedStyleColor textColor(ImGuiCol_Text, ui::StatusColor(ui::StatusTone::Info));
			ImGui::TextUnformatted(plugin->Name());
			bool hovered = ImGui::IsItemHovered();
			ImGui::SameLine();
			ImGui::TextUnformatted(marker);
			hovered = hovered || ImGui::IsItemHovered();
			if (hovered) {
				ImGui::SetTooltip("%s", tooltip);
			}
		} else {
			ImGui::TextUnformatted(plugin->Name());
		}

		ui::NextColumn();
		ImGui::AlignTextToFramePadding();
		const bool isRouterRow = module && module->id == module_registry::ModuleId::OscRouter;
		const bool faceTrackingEffective = IsEffectiveModuleEnabled(
			context,
			module_registry::Get(module_registry::ModuleId::FaceTracking));
		const bool captionsEffective = IsEffectiveModuleEnabled(
			context,
			module_registry::Get(module_registry::ModuleId::Captions));
		const bool routerRequired = isRouterRow &&
			(faceTrackingEffective || captionsEffective);
		std::string statusStorage;
		const char *statusText = nullptr;
		ui::StatusTone statusTone = ui::StatusTone::Idle;
		if (isPending) {
			statusText = (it != g_wantedModuleStates.end() && it->second)
				? "Enabling -- takes effect on next SteamVR launch"
				: "Disabling -- takes effect on next SteamVR launch";
			statusTone = ui::StatusTone::Pending;
		} else if (autoDisabled) {
			statusStorage = std::string("Auto-disabled: ") +
				context.ModuleAutoDisabledReason(key.c_str());
			statusText = statusStorage.c_str();
			statusTone = ui::StatusTone::Warn;
		} else if (blockedByRouterSafety) {
			statusText = "Blocked by OSC Router";
			statusTone = ui::StatusTone::Warn;
		} else if (routerRequired &&
			!context.IsFlagPresent(module_registry::FlagFileName(module_registry::ModuleId::OscRouter))) {
			statusText = "Enabled by OSC module";
			statusTone = ui::StatusTone::Ok;
		} else if (effectiveInstalled) {
			statusText = "Enabled";
			statusTone = ui::StatusTone::Ok;
		} else {
			statusText = "Disabled";
		}
		ui::DrawStatusCell(statusText, statusTone, true);

		ui::NextColumn();
		ImGui::PushID(key.c_str());
		const std::string defaultTooltip =
			std::string("Use ") + plugin->Name() + " as the desktop startup tab.";
		const bool defaultSelected = (context.DesktopDefaultModuleFlagFileName() == key);
		const char *defaultBlockReason = nullptr;
		bool defaultBlocked = isPending || !displayState;
		if (isPending) {
			defaultBlockReason = "Wait for the module change to finish before setting it as default.";
		} else if (!displayState) {
			defaultBlockReason = "Enable the module before setting it as the desktop default.";
		}
		{
			ui::DisabledSection disabled(defaultBlocked, defaultBlockReason);
			if (ui::RadioButtonWithTooltip("##desktop_default", defaultSelected, defaultTooltip.c_str())) {
				context.SetDesktopDefaultModuleFlagFileName(plugin->FlagFileName());
			}
			disabled.AttachReasonTooltip();
		}

		ui::NextColumn();
		const std::string pendingReason =
			"Waiting for the elevated helper to finish. Reopens after SteamVR picks up the change.";
		const bool routerDependentOn = routerRequired && displayState;

		const char *blockReason = nullptr;
		bool blocked = isPending;
		if (isPending) {
			blockReason = pendingReason.c_str();
		} else if (routerDependentOn) {
			blocked = true;
			blockReason =
				"An enabled module publishes through the OSC Router. "
				"Disable that module first if you want to turn the router off.";
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
