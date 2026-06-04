#include "SettingsTab.h"

#include "InputHealthPlugin.h"
#include "UiHelpers.h"

#include <imgui/imgui.h>

namespace inputhealth::ui {

void DrawSettingsTab(InputHealthPlugin& ui)
{
	openvr_pair::overlay::ui::DrawSectionHeading("Compensation families");

	openvr_pair::overlay::ui::DrawSettingTable(
	    "inputhealth_compensation_settings", 190.0f, [&](openvr_pair::overlay::ui::SettingTableScope& table) {
		    openvr_pair::overlay::ui::SettingRow(table, "Rest re-center (sticks)", [&] {
			    if (openvr_pair::overlay::ui::CheckboxWithTooltip(
			            "##rest_recenter", &ui.pending_config_.enable_rest_recenter,
			            "Enables learned stick rest-offset and deadzone correction for paths\n"
			            "that have enough clean samples. Off = observe without applying this\n"
			            "compensation family.")) {
				    ui.PushConfigToDriver();
				    ui.SaveGlobalConfig();
			    }
		    });

		    openvr_pair::overlay::ui::SettingRow(table, "Trigger remap", [&] {
			    if (openvr_pair::overlay::ui::CheckboxWithTooltip(
			            "##trigger_remap", &ui.pending_config_.enable_trigger_remap,
			            "Enables learned trigger min/max remapping for paths that have enough\n"
			            "range data. Off = observe without applying this compensation family.")) {
				    ui.PushConfigToDriver();
				    ui.SaveGlobalConfig();
			    }
		    });
	    });
}

} // namespace inputhealth::ui
