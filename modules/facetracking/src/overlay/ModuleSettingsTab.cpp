#include "ModuleSettingsTab.h"

#include "Logging.h"
#include "ModuleSources.h"
#include "UiHelpers.h"
#include "UiLayout.h"

#include "picojson.h"

#include <imgui/imgui.h>

#include <cstdio>
#include <map>
#include <string>

namespace facetracking::ui {

using namespace openvr_pair::overlay::ui;

namespace {

// Schema and values behind one module's tab. Values are held here between frames so typing into a
// text field does not rewrite the file on every keystroke; a write happens on commit.
struct PanelState
{
	std::string manifest_path; // what the schema and values were loaded for; reload when the row differs
	ModuleSettingsSchema schema;
	picojson::object values;
	std::string path;
	std::string error;
};

std::map<std::string, PanelState> g_panels; // keyed by module uuid

void OpenPanel(PanelState& st, const InstalledModule& m)
{
	st = {};
	st.manifest_path = m.manifest_path;
	st.schema = LoadModuleSettingsSchema(m.manifest_path);
	if (!st.schema.loaded) return;

	st.path = ModuleSettingsValuesPath(st.schema.values_file);
	if (st.path.empty()) {
		st.error = "This module declares an unusable settings file name.";
		return;
	}
	if (!ReadModuleSettingsValues(st.path, st.values)) {
		st.error = "Could not read " + st.path + " -- showing declared defaults.";
	}
}

void PersistPanel(PanelState& st)
{
	if (WriteModuleSettingsValues(st.path, st.values)) {
		st.error.clear();
		FT_LOG_OVL("[modules] settings written: %s", st.path.c_str());
		return;
	}
	st.error = "Could not write " + st.path;
	FT_LOG_OVL("[modules] settings write FAILED: %s", st.path.c_str());
}

// Values the module has not overridden are absent from the file, so every read falls back to the
// declared default rather than to a zero-initialised value.
bool SettingBool(const PanelState& st, const ModuleSettingSpec& spec)
{
	auto it = st.values.find(spec.key);
	if (it != st.values.end() && it->second.is<bool>()) return it->second.get<bool>();
	return spec.default_bool;
}

double SettingNumber(const PanelState& st, const ModuleSettingSpec& spec)
{
	auto it = st.values.find(spec.key);
	if (it != st.values.end() && it->second.is<double>()) return it->second.get<double>();
	return spec.default_number;
}

std::string SettingString(const PanelState& st, const ModuleSettingSpec& spec)
{
	auto it = st.values.find(spec.key);
	if (it != st.values.end() && it->second.is<std::string>()) return it->second.get<std::string>();
	return spec.default_string;
}

void DrawPanel(PanelState& st, const InstalledModule& m)
{
	if (st.manifest_path != m.manifest_path) OpenPanel(st, m);

	const auto& pal = GetPalette();
	if (!st.schema.loaded) {
		ImGui::TextColored(pal.statusError, "Could not read this module's settings_descriptor.json.");
		return;
	}

	if (!st.error.empty()) {
		ImGui::TextColored(pal.statusError, "%s", st.error.c_str());
	}

	ImGui::TextDisabled("Saved to %s", st.path.c_str());
	TooltipForLastItem("The module watches this file and picks changes up while it runs.\n"
	                   "Settings can be edited with the module stopped; they apply when it next starts.");

	bool dirty = false;
	for (const ModuleSettingSpec& spec : st.schema.settings) {
		ImGui::PushID(spec.key.c_str());

		if (spec.type == "bool") {
			bool v = SettingBool(st, spec);
			if (ImGui::Checkbox(spec.label.c_str(), &v)) {
				st.values[spec.key] = picojson::value(v);
				dirty = true;
			}
		}
		else if (spec.type == "int") {
			int v = static_cast<int>(SettingNumber(st, spec));
			const int lo = spec.has_range ? static_cast<int>(spec.min) : -1000000;
			const int hi = spec.has_range ? static_cast<int>(spec.max) : 1000000;
			if (ImGui::SliderInt(spec.label.c_str(), &v, lo, hi)) {
				st.values[spec.key] = picojson::value(static_cast<double>(v));
				dirty = true;
			}
		}
		else if (spec.type == "float") {
			float v = static_cast<float>(SettingNumber(st, spec));
			const float lo = spec.has_range ? static_cast<float>(spec.min) : 0.0f;
			const float hi = spec.has_range ? static_cast<float>(spec.max) : 1.0f;
			if (ImGui::SliderFloat(spec.label.c_str(), &v, lo, hi, "%.2f")) {
				st.values[spec.key] = picojson::value(static_cast<double>(v));
				dirty = true;
			}
		}
		else if (spec.type == "enum" && !spec.options.empty()) {
			const std::string current = SettingString(st, spec);
			int index = 0;
			for (size_t i = 0; i < spec.options.size(); ++i)
				if (spec.options[i] == current) index = static_cast<int>(i);

			if (ImGui::BeginCombo(spec.label.c_str(), spec.options[static_cast<size_t>(index)].c_str())) {
				for (size_t i = 0; i < spec.options.size(); ++i) {
					const bool selected = static_cast<int>(i) == index;
					if (ImGui::Selectable(spec.options[i].c_str(), selected)) {
						st.values[spec.key] = picojson::value(spec.options[i]);
						dirty = true;
					}
					if (selected) ImGui::SetItemDefaultFocus();
				}
				ImGui::EndCombo();
			}
		}
		else if (spec.type == "string") {
			char buf[256] = {};
			const std::string current = SettingString(st, spec);
			std::snprintf(buf, sizeof(buf), "%s", current.c_str());
			if (ImGui::InputText(spec.label.c_str(), buf, sizeof(buf),
			                     ImGuiInputTextFlags_EnterReturnsTrue | ImGuiInputTextFlags_AutoSelectAll)) {
				st.values[spec.key] = picojson::value(std::string(buf));
				dirty = true;
			}
			TooltipForLastItem("Press Enter to apply.");
		}
		else {
			ImGui::TextDisabled("%s (unsupported type '%s')", spec.label.c_str(), spec.type.c_str());
		}

		ImGui::PopID();
	}

	if (dirty) PersistPanel(st);
}

} // namespace

void DrawModuleSettingsTabs()
{
	for (const InstalledModule& m : InstalledModulesCached()) {
		if (!m.has_settings) continue;
		const std::string label = ModuleTabLabel(m.name) + "##" + m.uuid;
		DrawTabItem(label.c_str(), [&] { DrawPanel(g_panels[m.uuid], m); });
	}
}

} // namespace facetracking::ui
