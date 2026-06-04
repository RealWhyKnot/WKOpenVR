#include "DevPlugin.h"

#include "ModuleToggleUi.h"
#include "ShellContext.h"
#include "UiHelpers.h"

#include <imgui.h>

#include <memory>
#include <utility>

using openvr_pair::overlay::FeaturePlugin;
using openvr_pair::overlay::FeaturePluginChannel;

DevPlugin::DevPlugin(std::vector<FeaturePlugin*> plugins) : plugins_(std::move(plugins)) {}

FeaturePluginChannel DevPlugin::Channel() const
{
	return FeaturePluginChannel::DevTools;
}

void DevPlugin::DrawTab(openvr_pair::overlay::ShellContext& context)
{
	namespace ui = openvr_pair::overlay::ui;

	ui::DrawInfoBanner(
	    "Dev build", "Development-only modules and tools are collected here. Release builds do not compile this tab.");

	ImGui::Spacing();
	ui::DrawSectionHeading("Development modules");
	ui::DrawTextWrapped("These modules are compiled in dev builds and excluded from release builds while they are "
	                    "still in development.");
	ImGui::Spacing();

	std::vector<FeaturePlugin*> developmentModules;
	for (FeaturePlugin* plugin : plugins_) {
		if (plugin && openvr_pair::overlay::ShouldShowInDevModuleList(*plugin)) {
			developmentModules.push_back(plugin);
		}
	}
	openvr_pair::overlay::DrawModuleToggleTable(context, developmentModules, "dev_modules",
	                                            "No development modules were compiled into this build.", {true});

	ImGui::Spacing();
	ImGui::Separator();
	ImGui::Spacing();
	ui::DrawSectionHeading("Dev tools");

	bool anyTools = false;
	for (FeaturePlugin* plugin : plugins_) {
		if (!plugin || !plugin->HasDevTools() || !plugin->IsInstalled(context)) continue;

		ImGui::PushID(plugin->Name());
		ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
		if (ImGui::CollapsingHeader(plugin->Name())) {
			ImGui::Indent();
			plugin->DrawDevTools(context);
			ImGui::Unindent();
		}
		ImGui::PopID();
		anyTools = true;
	}

	if (!anyTools) {
		ui::DrawEmptyState("No installed modules currently expose dev tools.");
	}
}

namespace openvr_pair::overlay {

std::unique_ptr<FeaturePlugin> CreateDevPlugin(std::vector<FeaturePlugin*> plugins)
{
	return std::make_unique<DevPlugin>(std::move(plugins));
}

} // namespace openvr_pair::overlay
