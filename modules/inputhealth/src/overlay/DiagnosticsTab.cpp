#include "DiagnosticsTab.h"

#include "LearningEngine.h"
#include "Profiles.h"
#include "SnapshotReader.h"
#include "InputHealthPlugin.h"

#include "inputhealth/SnapshotDiagnostics.h"

#include "UiHelpers.h"

#include <imgui/imgui.h>

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

namespace {

namespace diag = inputhealth::diagnostics;

std::string SerialHex(uint64_t serial_hash)
{
	char buf[20];
	snprintf(buf, sizeof(buf), "%016llx", (unsigned long long)serial_hash);
	return buf;
}

std::string SerialShort(uint64_t serial_hash)
{
	return SerialHex(serial_hash).substr(0, 8);
}

std::string PathFromBody(const protocol::InputHealthSnapshotBody& b)
{
	const size_t path_len = diag::BoundedPathLength(b.path);
	return std::string(b.path, b.path + path_len);
}

// Detects whether a snapshot body's path looks like it comes from a real
// hand controller (Index / Touch / Reverb / WMR / Vive). Used to filter out
// HMD-class devices and SteamVR's synthetic input sources (proximity sensor,
// "remote", "tap" pseudo-buttons) from the InputHealth per-device list -- the
// snapshot wire format does not carry TrackedDeviceClass, so we infer from
// the component path. A device with at least one controller-like path keeps
// all its rows; a device with none is hidden entirely so the list shows the
// user's actual controllers and trackers, not the noise.
bool PathLooksLikeControllerInput(const protocol::InputHealthSnapshotBody& b)
{
	const std::string path = diag::LowerPath(b);
	static const char* const kControllerNeedles[] = {
	    "/trigger", "/grip", "/trackpad", "/joystick", "/thumbstick", "/skeleton",
	    "/squeeze", // OpenXR-style grip
	};
	for (const char* needle : kControllerNeedles) {
		if (path.find(needle) != std::string::npos) return true;
	}
	return false;
}

void DrawRangeCell(const protocol::InputHealthSnapshotBody& b)
{
	if (!b.is_scalar || !b.scalar_range_initialized) {
		ImGui::TextDisabled("-");
		return;
	}
	ImGui::Text("%.3f..%.3f", b.observed_min, b.observed_max);
}

void DrawHintCell(const protocol::InputHealthSnapshotBody& b)
{
	const auto& pal = openvr_pair::overlay::ui::GetPalette();

	if (b.is_boolean) {
		if (b.press_count == 0)
			ImGui::TextDisabled("no presses");
		else
			ImGui::Text("seen");
		return;
	}

	if (!b.is_scalar || !b.scalar_range_initialized) {
		ImGui::TextDisabled("waiting");
		return;
	}

	if (b.ph_triggered) {
		ImGui::TextColored(pal.statusWarn, "drift %s", b.ph_triggered_positive ? "+" : "-");
		return;
	}

	if (diag::LooksLikeTriggerValue(b)) {
		if (b.welford_count >= 20 && b.observed_min > 0.08f) {
			ImGui::TextColored(pal.statusWarn, "rest high?");
		}
		else if (b.welford_count >= 20 && b.observed_max < 0.85f) {
			ImGui::TextColored(pal.statusWarn, "max low?");
		}
		else {
			ImGui::TextDisabled("range");
		}
		return;
	}

	if (b.axis_role == 1 /* StickX */) {
		const diag::PolarSummary p = diag::SummarizePolar(b);
		if (!p.enough_coverage) {
			ImGui::TextDisabled("sweep %.0f%% H%.2f", p.coverage * 100.0, p.entropy);
		}
		else if (p.weak_bins > 0) {
			ImGui::TextColored(pal.statusWarn, "weak arc? %d", p.weak_bins);
		}
		else {
			ImGui::TextDisabled("coverage ok");
		}
		return;
	}

	ImGui::TextDisabled("range");
}

void DrawLearningCell(const LearningPathView& view)
{
	const auto& pal = openvr_pair::overlay::ui::GetPalette();
	if (!view.corrections_enabled) {
		ImGui::TextDisabled("off");
	}
	else if (view.drift_shift_pending) {
		ImGui::TextColored(pal.statusWarn, "drift-shift");
	}
	else if (view.ready) {
		ImGui::TextColored(pal.statusOk, "ready");
	}
	else {
		ImGui::Text("%llu/%llu", (unsigned long long)view.sample_count, (unsigned long long)view.threshold);
	}
}

void DrawCorrectionCell(LearningEngine& engine, const protocol::InputHealthSnapshotBody& b,
                        const LearningPathView& view)
{
	if (view.correction.empty() || view.correction == "-") {
		ImGui::TextDisabled("-");
	}
	else {
		ImGui::TextUnformatted(view.correction.c_str());
	}
	ImGui::SameLine();
	if (ImGui::SmallButton("Unlearn")) {
		engine.UnlearnPath(b.device_serial_hash, b.path);
	}
}

void DrawDeviceHeader(ProfileStore& profiles, LearningEngine& engine, uint64_t serial_hash)
{
	if (serial_hash == 0) {
		ImGui::Text("Device 00000000");
		ImGui::SameLine();
		ImGui::TextDisabled("Corrections unavailable until the driver reports a serial hash");
		return;
	}

	auto& profile = profiles.GetOrCreate(serial_hash);
	const std::string serial = SerialHex(serial_hash);

	ImGui::PushID(serial.c_str());
	ImGui::Text("Device %s", SerialShort(serial_hash).c_str());
	ImGui::SameLine();
	bool corrections_enabled = profile.corrections_enabled;
	if (ImGui::Checkbox("Corrections enabled", &corrections_enabled)) {
		engine.SetDeviceCorrectionsEnabled(serial_hash, corrections_enabled);
	}
	ImGui::SameLine();
	if (ImGui::Button("Unlearn all on this device")) {
		engine.UnlearnDevice(serial_hash);
	}
	ImGui::PopID();
}

void SetupComponentTableColumns()
{
	ImGui::TableSetupColumn("device");
	ImGui::TableSetupColumn("path");
	ImGui::TableSetupColumn("kind");
	ImGui::TableSetupColumn("role");
	ImGui::TableSetupColumn("n");
	ImGui::TableSetupColumn("range");
	ImGui::TableSetupColumn("mean");
	ImGui::TableSetupColumn("stddev");
	ImGui::TableSetupColumn("rest_min / press");
	ImGui::TableSetupColumn("hint");
	ImGui::TableSetupColumn("learning");
	ImGui::TableSetupColumn("correction");
	ImGui::TableHeadersRow();
}

void DrawComponentRow(LearningEngine& engine, const SnapshotReader::Entry& entry)
{
	const auto& b = entry.body;
	const std::string path = PathFromBody(b);
	const LearningPathView learning = engine.GetPathView(b.device_serial_hash, b.path);

	ImGui::PushID(SerialHex(b.device_serial_hash).c_str());
	ImGui::PushID(path.c_str());

	ImGui::TableNextRow();
	ImGui::TableSetColumnIndex(0);
	ImGui::TextUnformatted(SerialShort(b.device_serial_hash).c_str());
	ImGui::TableSetColumnIndex(1);
	ImGui::TextUnformatted(path.c_str());
	ImGui::TableSetColumnIndex(2);
	ImGui::TextUnformatted(diag::KindName(b));
	ImGui::TableSetColumnIndex(3);
	ImGui::TextUnformatted(diag::AxisRoleName(b.axis_role));

	ImGui::TableSetColumnIndex(4);
	ImGui::Text("%llu", (unsigned long long)b.welford_count);

	ImGui::TableSetColumnIndex(5);
	DrawRangeCell(b);

	ImGui::TableSetColumnIndex(6);
	if (b.welford_count > 0)
		ImGui::Text("%.4f", b.welford_mean);
	else
		ImGui::TextDisabled("-");

	ImGui::TableSetColumnIndex(7);
	if (b.welford_count > 1)
		ImGui::Text("%.4f", diag::SampleStdDev(b));
	else
		ImGui::TextDisabled("-");

	ImGui::TableSetColumnIndex(8);
	if (b.is_boolean) {
		ImGui::Text("%llu press / %s", (unsigned long long)b.press_count, b.last_boolean ? "down" : "up");
	}
	else if (b.rest_min_initialized) {
		ImGui::Text("%.4f", b.rest_min);
	}
	else {
		ImGui::TextDisabled("-");
	}

	ImGui::TableSetColumnIndex(9);
	DrawHintCell(b);

	ImGui::TableSetColumnIndex(10);
	DrawLearningCell(learning);

	ImGui::TableSetColumnIndex(11);
	DrawCorrectionCell(engine, b, learning);

	ImGui::PopID();
	ImGui::PopID();
}

} // namespace

namespace inputhealth::ui {

void DrawDiagnosticsTab(InputHealthPlugin& ui)
{
	const bool corrections_live = ui.pending_config_.master_enabled && !ui.pending_config_.diagnostics_only;
	if (corrections_live) {
		openvr_pair::overlay::ui::DrawTextWrapped("Input drift correction is active. Ready paths will rewrite values; "
		                                          "learning paths show their progress below.");
	}
	else if (!ui.pending_config_.master_enabled) {
		openvr_pair::overlay::ui::DrawTextWrapped("Driver is paused. Re-enable it on the Debug tab.");
	}
	else {
		openvr_pair::overlay::ui::DrawTextWrapped("Observe-only mode is on (Debug tab). Learning runs but the driver "
		                                          "will not rewrite inputs.");
	}

	ImGui::Separator();

	const auto& entries = ui.reader_.EntriesByHandle();
	if (entries.empty()) {
		openvr_pair::overlay::ui::DrawTextWrapped("No components seen yet. Move a controller to populate the table.");
		return;
	}

	std::vector<const SnapshotReader::Entry*> sorted;
	sorted.reserve(entries.size());
	for (const auto& kv : entries)
		sorted.push_back(&kv.second);
	std::sort(sorted.begin(), sorted.end(), [](const SnapshotReader::Entry* a, const SnapshotReader::Entry* b) {
		if (a->body.device_serial_hash != b->body.device_serial_hash) {
			return a->body.device_serial_hash < b->body.device_serial_hash;
		}
		return std::strncmp(a->body.path, b->body.path, protocol::INPUTHEALTH_PATH_LEN) < 0;
	});

	openvr_pair::overlay::ui::DrawSectionHeading("Per-device overrides");
	openvr_pair::overlay::ui::DrawTextWrapped("Each controller has its own Corrections-enabled checkbox and an Unlearn "
	                                          "button per input path. Use these to tweak a single tracker without "
	                                          "changing the global Settings.");
	ImGui::Spacing();

	for (size_t begin = 0; begin < sorted.size();) {
		const uint64_t serial_hash = sorted[begin]->body.device_serial_hash;
		size_t end = begin + 1;
		while (end < sorted.size() && sorted[end]->body.device_serial_hash == serial_hash)
			++end;

		// Skip devices that only expose non-controller components (HMD
		// proximity sensor, SteamVR's "remote"/"tap" synthetic devices).
		// They surfaced in the table as serial-hash rows the user had no
		// way to identify and added no diagnostic value.
		bool anyController = false;
		for (size_t i = begin; i < end; ++i) {
			if (PathLooksLikeControllerInput(sorted[i]->body)) {
				anyController = true;
				break;
			}
		}
		if (!anyController) {
			begin = end;
			continue;
		}

		DrawDeviceHeader(ui.profiles_, ui.engine_, serial_hash);

		const std::string table_id = "components##" + SerialHex(serial_hash);
		if (ImGui::BeginTable(table_id.c_str(), 12,
		                      ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_RowBg |
		                          ImGuiTableFlags_BordersInnerH)) {
			SetupComponentTableColumns();
			for (size_t i = begin; i < end; ++i) {
				DrawComponentRow(ui.engine_, *sorted[i]);
			}
			ImGui::EndTable();
		}

		begin = end;
		if (begin < sorted.size()) {
			ImGui::Spacing();
			ImGui::Separator();
			ImGui::Spacing();
		}
	}
}

} // namespace inputhealth::ui
