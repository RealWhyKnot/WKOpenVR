// openvr.h must precede anything that may pull in openvr_driver.h via the
// IPCClient -> Protocol include chain, matching the smoothing plugin's
// convention. Defining _OPENVR_API early makes Protocol.h skip the driver
// header so we don't redefine vr:: symbols.
#include <openvr.h>

#include "PhantomPlugin.h"

#include "DeviceFilters.h"
#include "DiagnosticsLog.h"
#include "PhantomUiLogic.h"
#include "Protocol.h"
#include "ShellContext.h"
#include "UiHelpers.h"

#include <imgui.h>

#include <algorithm>
#include <cfloat>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <exception>
#include <string>

namespace ui = openvr_pair::overlay::ui;

namespace {

const char* SolverModeLabel(uint8_t mode);
std::string SourceMaskLabel(uint16_t mask);

// Map the test-covered, ImGui-free PhantomTone onto the shared overlay
// StatusTone so diagnostics cells use the same semantic palette as the rest of
// the shell. The mapping itself lives in PhantomUiLogic.h (unit-tested); this is
// just the overlay-only glue.
ui::StatusTone ToStatusTone(phantom::ui::PhantomTone t)
{
	switch (t) {
		case phantom::ui::PhantomTone::Ok:
			return ui::StatusTone::Ok;
		case phantom::ui::PhantomTone::Pending:
			return ui::StatusTone::Pending;
		case phantom::ui::PhantomTone::Warn:
			return ui::StatusTone::Warn;
		case phantom::ui::PhantomTone::Error:
			return ui::StatusTone::Error;
		case phantom::ui::PhantomTone::Info:
			return ui::StatusTone::Info;
		case phantom::ui::PhantomTone::Idle:
			return ui::StatusTone::Idle;
	}
	return ui::StatusTone::Idle;
}

} // namespace

void PhantomPlugin::OnStart(openvr_pair::overlay::ShellContext&)
{
	connectError_.clear();
	connectFailureCount_ = 0;
	seededDriver_ = false;
}

void PhantomPlugin::Tick(openvr_pair::overlay::ShellContext& context)
{
	if (!phantom::ui::ShouldAttemptDriverConnection(context.vrConnected)) {
		if (ipc_.IsConnected()) ipc_.Close();
		if (stateShmemReady_) {
			stateShmem_.Close();
			stateShmemReady_ = false;
		}
		connectError_.clear();
		nextConnectAttempt_ = {};
		connectFailureCount_ = 0;
		seededDriver_ = false;
		return;
	}

	if (!ipc_.IsConnected() && ConnectIfNeeded()) {
		seededDriver_ = false;
	}

	if (ipc_.IsConnected() && !seededDriver_) {
		// Connected after startup or after a drop -- replay full state so the
		// driver is in sync with what the overlay believes.
		ReplayDriverState();
		seededDriver_ = true;
	}

	if (!stateShmemReady_) {
		if (stateShmem_.Open(OPENVR_PAIRDRIVER_PHANTOM_STATE_SHMEM_NAME)) {
			stateShmemReady_ = true;
		}
	}
}

bool PhantomPlugin::ConnectIfNeeded()
{
	if (ipc_.IsConnected()) return false;
	const auto now = std::chrono::steady_clock::now();
	if (nextConnectAttempt_.time_since_epoch().count() != 0 && now < nextConnectAttempt_) {
		return false;
	}

	try {
		ipc_.Connect();
		connectError_.clear();
		nextConnectAttempt_ = {};
		connectFailureCount_ = 0;
		return true;
	}
	catch (const std::exception& e) {
		connectError_ = e.what();
		++connectFailureCount_;
		const uint32_t retryDelayMs = phantom::ui::DriverConnectRetryDelayMs(connectFailureCount_);
		nextConnectAttempt_ = std::chrono::steady_clock::now() + std::chrono::milliseconds(retryDelayMs);
		openvr_pair::common::DiagnosticLog("phantom", "driver_connect_retry failures=%u retry_ms=%u error='%s'",
		                                   connectFailureCount_, retryDelayMs, connectError_.c_str());
	}
	return false;
}

void PhantomPlugin::SendConfig()
{
	if (!ipc_.IsConnected()) return;
	protocol::Request req(protocol::RequestSetPhantomConfig);
	auto& c = req.setPhantomConfig;
	c.master_enabled = cfg_.master_enabled ? 1u : 0u;
	c._pad0[0] = c._pad0[1] = c._pad0[2] = 0;
	c.blend_out_ms = cfg_.blend_out_ms;
	c.blend_in_ms = cfg_.blend_in_ms;
	c.reckon_hold_ms = cfg_.reckon_hold_ms;
	c.synth_hold_ms = cfg_.synth_hold_ms;
	c.lost_hold_ms = cfg_.lost_hold_ms;
	try {
		ipc_.SendBlocking(req);
	}
	catch (const std::exception& e) {
		connectError_ = e.what();
	}
}

void PhantomPlugin::SendSolverConfig()
{
	if (!ipc_.IsConnected()) return;
	protocol::Request req(protocol::RequestSetPhantomSolverConfig);
	auto& s = req.setPhantomSolverConfig;
	std::memset(&s, 0, sizeof(s));
	s.virtual_min_confidence = cfg_.solver.virtual_min_confidence;
	try {
		ipc_.SendBlocking(req);
	}
	catch (const std::exception& e) {
		connectError_ = e.what();
	}
}

void PhantomPlugin::SendDeviceOptIn(const std::string& serial, bool enabled)
{
	if (!ipc_.IsConnected()) return;
	// FNV-1a 64-bit of the serial string; matches the driver-side hash so
	// the slot lookup lands on the right device.
	uint64_t h = 0xcbf29ce484222325ull;
	for (char c : serial) {
		h ^= static_cast<unsigned char>(c);
		h *= 0x100000001b3ull;
	}
	protocol::Request req(protocol::RequestSetPhantomDeviceOptIn);
	req.setPhantomDeviceOptIn.device_serial_hash = h;
	req.setPhantomDeviceOptIn.dropout_enabled = enabled ? 1u : 0u;
	std::memset(req.setPhantomDeviceOptIn._reserved, 0, sizeof(req.setPhantomDeviceOptIn._reserved));
	try {
		ipc_.SendBlocking(req);
	}
	catch (const std::exception& e) {
		connectError_ = e.what();
	}
}

void PhantomPlugin::DrawTab(openvr_pair::overlay::ShellContext& context)
{
	if (!context.vrConnected) {
		openvr_pair::overlay::ui::DrawWaitingBanner(
		    "Waiting for SteamVR -- Phantom Trackers sync when the driver is live.");
		ImGui::Spacing();
	}

	// The shell already hosts this whole tab inside a scrolling child
	// (ShellUi.cpp), so the sub-tabs are plain tab items -- a nested scroll
	// child would fight the outer one.
	ui::TabBarScope tabs("PhantomTabs");
	if (tabs) {
		ui::DrawTabItem("Dropouts", [&] { DrawDropoutsTab(); });
		ui::DrawTabItem("Body", [&] { DrawBodyTab(); });
		ui::DrawTabItem("Absent", [&] { DrawAbsentTab(); });
		ui::DrawTabItem("Diagnostics", [&] { DrawDiagnosticsTab(); });
		ui::DrawTabItem("Advanced", [&] { DrawAdvancedTab(); });
	}

	if (phantom::ui::ShouldShowDriverError(context.vrConnected, !connectError_.empty()) && !ipc_.IsConnected()) {
		ImGui::Spacing();
		ImGui::TextColored(ui::GetPalette().statusError, "IPC: %s", connectError_.c_str());
	}
}

void PhantomPlugin::DrawDropoutsTab()
{
	ImGui::Spacing();

	ui::DrawPanel("Dropout bridging", [&] {
		if (ui::CheckboxWithTooltip("Bridge dropped trackers", &cfg_.master_enabled,
		                            "Master switch. With this on, the driver fills in plausible poses\n"
		                            "for any tracker you opted in below when its real pose goes silent.\n"
		                            "Past the synth-hold window the tracker is marked OutOfRange so\n"
		                            "VRChat / Resonite drop it from the IK chain cleanly.")) {
			SendConfig();
			SavePhantomConfig(cfg_);
		}
	});

	ui::DrawPanel("Per-tracker opt-in", [&] {
		auto* vrSystem = vr::VRSystem();
		if (!vrSystem) {
			ui::DrawEmptyState("VR system not available.");
			return;
		}

		ui::TableScope table("PhantomDropoutOptIn", 2,
		                     ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerH | ImGuiTableFlags_SizingStretchProp);
		if (!table) return;
		ui::SetupFixedColumn("Bridge", 64.0f);
		ui::SetupStretchColumn("Tracker", 1.0f);
		ImGui::TableHeadersRow();

		char buffer[vr::k_unMaxPropertyStringSize];
		bool anyShown = false;
		for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id) {
			const auto deviceClass = vrSystem->GetTrackedDeviceClass(id);
			if (deviceClass == vr::TrackedDeviceClass_Invalid) continue;

			vr::ETrackedPropertyError err = vr::TrackedProp_Success;
			vrSystem->GetStringTrackedDeviceProperty(id, vr::Prop_SerialNumber_String, buffer, sizeof(buffer), &err);
			if (err != vr::TrackedProp_Success || buffer[0] == 0) continue;
			const std::string serial = buffer;

			vrSystem->GetStringTrackedDeviceProperty(id, vr::Prop_RenderModelName_String, buffer, sizeof(buffer), &err);
			const std::string model = (err == vr::TrackedProp_Success) ? buffer : "";

			vrSystem->GetStringTrackedDeviceProperty(id, vr::Prop_TrackingSystemName_String, buffer, sizeof(buffer),
			                                         &err);
			const std::string trackingSystem = (err == vr::TrackedProp_Success) ? buffer : "";

			if (!openvr_pair::overlay::ShouldShowInSmoothingPredictionList(deviceClass, serial, model,
			                                                               trackingSystem)) {
				continue;
			}

			anyShown = true;
			bool enabled = cfg_.dropout_enabled.count(serial) ? cfg_.dropout_enabled[serial] : false;
			ImGui::PushID(("trk_" + serial).c_str());
			ImGui::TableNextRow();
			ImGui::TableSetColumnIndex(0);
			if (ImGui::Checkbox("##en", &enabled)) {
				cfg_.dropout_enabled[serial] = enabled;
				SendDeviceOptIn(serial, enabled);
				SavePhantomConfig(cfg_);
			}
			ImGui::TableSetColumnIndex(1);
			ImGui::TextUnformatted(model.empty() ? "(unknown model)" : model.c_str());
			ImGui::SameLine();
			ImGui::TextDisabled("[%s]", serial.c_str());
			ImGui::PopID();
		}
		if (!anyShown) {
			ImGui::TableNextRow();
			ImGui::TableSetColumnIndex(0);
			ui::DrawEmptyState("No bridgeable trackers detected. HMD, controllers, and generic body "
			                   "trackers appear here once SteamVR is running and the devices are on.");
		}
	});
}

void PhantomPlugin::DrawDiagnosticsTab()
{
	ImGui::Spacing();
	if (!stateShmemReady_ || !stateShmem_.layout()) {
		ui::DrawEmptyState("Driver state not yet available. Load the driver with the Phantom "
		                   "feature flag enabled to see live tracker status.");
		return;
	}
	const auto* layout = stateShmem_.layout();
	if (layout->magic != phantom::kPhantomStateShmemMagic) {
		ui::DrawErrorBanner("Driver mismatch",
		                    "Driver state has unexpected magic; reinstall so the driver and overlay match.");
		return;
	}
	if (layout->version != phantom::kPhantomStateShmemVersion) {
		ui::DrawErrorBanner("Driver mismatch",
		                    "Driver state has unexpected version; reinstall so the driver and overlay match.");
		return;
	}

	const ImGuiTableFlags diagFlags = ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersOuter |
	                                  ImGuiTableFlags_BordersInnerH | ImGuiTableFlags_SizingStretchProp;

	// Numeric cells sit right of their header in normal text color (the
	// muted-text path of RightAlignText is for placeholders, not live counts).
	auto rightCell = [](const char* text) {
		ui::RightAlignText(text, ImGui::GetStyleColorVec4(ImGuiCol_Text), true);
	};

	ui::DrawPanel("Live trackers", [&] {
		ui::TableScope table("PhantomDiag", 5, diagFlags);
		if (!table) return;
		ui::SetupStretchColumn("Serial", 2.0f);
		ui::SetupStretchColumn("State", 1.4f);
		ui::SetupFixedColumn("Drops", 60.0f);
		ui::SetupFixedColumn("Now (ms)", 80.0f);
		ui::SetupFixedColumn("Longest (ms)", 96.0f);
		ImGui::TableHeadersRow();

		int shown = 0;
		char buf[32];
		for (uint32_t i = 0; i < layout->device_count; ++i) {
			const auto& d = layout->devices[i];
			if (d.serial_len == 0) continue;
			++shown;
			const auto state = static_cast<phantom::TrackerState>(d.state);

			ImGui::TableNextRow();
			ImGui::TableSetColumnIndex(0);
			ImGui::Text("%.*s", (int)d.serial_len, d.serial);

			ImGui::TableSetColumnIndex(1);
			ui::SetCellToneBg(ToStatusTone(phantom::ui::TrackerStateTone(state)));
			ImGui::TextUnformatted(phantom::TrackerStateLabel(state));

			ImGui::TableSetColumnIndex(2);
			std::snprintf(buf, sizeof(buf), "%u", d.dropout_count);
			rightCell(buf);

			ImGui::TableSetColumnIndex(3);
			std::snprintf(buf, sizeof(buf), "%u", d.dropout_age_ms);
			rightCell(buf);

			ImGui::TableSetColumnIndex(4);
			std::snprintf(buf, sizeof(buf), "%u", d.longest_dropout_ms);
			rightCell(buf);
		}
		if (shown == 0) {
			ImGui::TableNextRow();
			ImGui::TableSetColumnIndex(0);
			ui::DrawEmptyState("No trackers observed yet.");
		}
	});

	ui::DrawPanel("Role completion", [&] {
		ui::TableScope roleTable("PhantomRoleDiag", 6, diagFlags);
		if (!roleTable) return;
		ui::SetupStretchColumn("Role", 1.5f);
		ui::SetupFixedColumn("Confidence", 84.0f);
		ui::SetupStretchColumn("Mode", 1.4f);
		ui::SetupStretchColumn("Source", 2.0f);
		ui::SetupFixedColumn("Age", 72.0f);
		ui::SetupFixedColumn("Position", 140.0f);
		ImGui::TableHeadersRow();

		int shown = 0;
		char buf[64];
		for (uint8_t i = 0; i < phantom::kBodyRoleCount; ++i) {
			const auto& r = layout->roles[i];
			if (!r.valid) continue;
			++shown;

			ImGui::TableNextRow();
			ImGui::TableSetColumnIndex(0);
			ImGui::TextUnformatted(phantom::BodyRoleLabel(static_cast<phantom::BodyRole>(r.role)));

			ImGui::TableSetColumnIndex(1);
			std::snprintf(buf, sizeof(buf), "%.2f", r.confidence);
			rightCell(buf);

			ImGui::TableSetColumnIndex(2);
			ui::SetCellToneBg(ToStatusTone(phantom::ui::SolverModeTone(r.solver_mode)));
			ImGui::TextUnformatted(SolverModeLabel(r.solver_mode));

			ImGui::TableSetColumnIndex(3);
			const std::string sources = SourceMaskLabel(r.source_mask);
			ImGui::TextUnformatted(sources.c_str());
			ui::TooltipOnHover(sources.c_str());

			ImGui::TableSetColumnIndex(4);
			std::snprintf(buf, sizeof(buf), "%u ms", r.age_ms);
			rightCell(buf);

			ImGui::TableSetColumnIndex(5);
			ImGui::Text("%.2f %.2f %.2f", r.position[0], r.position[1], r.position[2]);
		}
		if (shown == 0) {
			ImGui::TableNextRow();
			ImGui::TableSetColumnIndex(0);
			ui::DrawEmptyState("No completed body roles yet.");
		}
	});

	// State color key -- built from the same tone->color path the cells use, so
	// it can never drift from the row tints above.
	ImGui::Spacing();
	auto legendItem = [](const char* label, phantom::ui::PhantomTone tone) {
		ImGui::TextColored(ui::StatusColor(ToStatusTone(tone)), "%s", label);
	};
	ImGui::TextDisabled("State");
	ImGui::SameLine();
	legendItem("Real", phantom::ui::PhantomTone::Ok);
	ImGui::SameLine();
	legendItem("Blending", phantom::ui::PhantomTone::Pending);
	ImGui::SameLine();
	legendItem("Synthesizing", phantom::ui::PhantomTone::Warn);
	ImGui::SameLine();
	legendItem("Lost", phantom::ui::PhantomTone::Error);
}

void PhantomPlugin::DrawAdvancedTab()
{
	ImGui::Spacing();
	ui::DrawTextWrapped("Timing ladder. Lower the synth-hold to recover trackers faster after "
	                    "a stuck pose; raise it to bridge longer outages. Out-of-range happens "
	                    "at synth-hold; the device stops publishing at lost-hold.");
	ImGui::Spacing();

	auto currentTiming = [&] {
		return phantom::ui::DropoutTimingValues{
		    cfg_.blend_out_ms, cfg_.blend_in_ms, cfg_.reckon_hold_ms, cfg_.synth_hold_ms, cfg_.lost_hold_ms,
		};
	};
	auto applyTiming = [&](const phantom::ui::DropoutTimingValues& timing) {
		cfg_.blend_out_ms = timing.blend_out_ms;
		cfg_.blend_in_ms = timing.blend_in_ms;
		cfg_.reckon_hold_ms = timing.reckon_hold_ms;
		cfg_.synth_hold_ms = timing.synth_hold_ms;
		cfg_.lost_hold_ms = timing.lost_hold_ms;
		SendConfig();
		SavePhantomConfig(cfg_);
	};

	ui::DrawPanel("Timing ladder", [&] {
		const auto preset = phantom::ui::ClassifyDropoutTiming(currentTiming());
		if (ImGui::BeginCombo("Timing preset", phantom::ui::DropoutTimingPresetLabel(preset))) {
			const phantom::ui::DropoutTimingPreset presets[] = {
			    phantom::ui::DropoutTimingPreset::Conservative,
			    phantom::ui::DropoutTimingPreset::Balanced,
			    phantom::ui::DropoutTimingPreset::Extended,
			};
			for (const auto candidate : presets) {
				const bool selected = preset == candidate;
				if (ImGui::Selectable(phantom::ui::DropoutTimingPresetLabel(candidate), selected)) {
					applyTiming(phantom::ui::ValuesForDropoutTimingPreset(candidate));
				}
				if (ImGui::IsItemHovered()) {
					ImGui::SetTooltip("%s", phantom::ui::DropoutTimingPresetHelp(candidate));
				}
				if (selected) ImGui::SetItemDefaultFocus();
			}
			ImGui::EndCombo();
		}
		ui::TooltipForLastItem(phantom::ui::DropoutTimingPresetHelp(preset));

		ImGui::Spacing();

		auto sliderMs = [&](const char* label, uint32_t& v, uint32_t lo, uint32_t hi, const char* tip) {
			int tmp = static_cast<int>(v);
			if (ui::SliderIntWithTooltip(label, &tmp, (int)lo, (int)hi, "%d ms", tip)) {
				v = static_cast<uint32_t>(std::clamp(tmp, (int)lo, (int)hi));
				SendConfig();
				SavePhantomConfig(cfg_);
			}
		};

		sliderMs("Blend out", cfg_.blend_out_ms, 0, 500, "Real to synth fade duration on dropout start.");
		sliderMs("Blend in", cfg_.blend_in_ms, 0, 1000, "Synth to real fade duration when the real signal returns.");
		sliderMs("Reckon hold", cfg_.reckon_hold_ms, 0, 1000,
		         "How long dead reckoning is the primary synthesis source before the "
		         "ladder escalates (to IK / ML in later phases).");
		sliderMs("Synth hold", cfg_.synth_hold_ms, 100, 10000,
		         "Total time after dropout before ETrackingResult flips to OutOfRange.");
		sliderMs("Lost hold", cfg_.lost_hold_ms, 500, 60000,
		         "Total time after dropout before the device stops publishing entirely.");

		ImGui::Spacing();
		if (ImGui::Button("Reset to defaults")) {
			cfg_.blend_out_ms = phantom::DefaultTimings::kBlendOutMs;
			cfg_.blend_in_ms = phantom::DefaultTimings::kBlendInMs;
			cfg_.reckon_hold_ms = phantom::DefaultTimings::kReckonHoldMs;
			cfg_.synth_hold_ms = phantom::DefaultTimings::kSynthHoldMs;
			cfg_.lost_hold_ms = phantom::DefaultTimings::kLostHoldMs;
			SendConfig();
			SavePhantomConfig(cfg_);
		}
	});
}

namespace {

// FNV-1a 64-bit over a serial string. Matches the driver-side convention
// so per-serial-hash maps line up on both ends.
uint64_t Fnv1a64Local(const std::string& s)
{
	uint64_t h = 0xcbf29ce484222325ull;
	for (char c : s) {
		h ^= static_cast<unsigned char>(c);
		h *= 0x100000001b3ull;
	}
	return h;
}

const char* SolverModeLabel(uint8_t mode)
{
	switch (mode) {
		case 0:
			return "none";
		case 1:
			return "measured";
		case 2:
			return "hmd_root";
		case 3:
			return "controller_ik";
		case 4:
			return "floor_contact";
		case 5:
			return "held_contact";
		case 6:
			return "low_confidence";
	}
	return "unknown";
}

std::string SourceMaskLabel(uint16_t mask)
{
	std::string out;
	auto add = [&](uint16_t bit, const char* label) {
		if ((mask & bit) == 0) return;
		if (!out.empty()) out += ",";
		out += label;
	};
	add(1u << 0, "measured");
	add(1u << 1, "hmd");
	add(1u << 2, "controller");
	add(1u << 3, "floor");
	add(1u << 4, "contact");
	add(1u << 5, "predicted");
	add(1u << 6, "held");
	if (out.empty()) out = "none";
	return out;
}

} // namespace

void PhantomPlugin::SendDeviceRole(const std::string& serial, phantom::BodyRole role)
{
	if (!ipc_.IsConnected()) return;
	protocol::Request req(protocol::RequestSetPhantomDeviceRole);
	req.setPhantomDeviceRole.device_serial_hash = Fnv1a64Local(serial);
	req.setPhantomDeviceRole.body_role = static_cast<uint8_t>(role);
	std::memset(req.setPhantomDeviceRole._reserved, 0, sizeof(req.setPhantomDeviceRole._reserved));
	try {
		ipc_.SendBlocking(req);
	}
	catch (const std::exception& e) {
		connectError_ = e.what();
	}
}

void PhantomPlugin::SendVirtualEnabled(phantom::BodyRole role, bool enabled)
{
	if (!ipc_.IsConnected()) return;
	protocol::Request req(protocol::RequestSetPhantomVirtualEnabled);
	req.setPhantomVirtualEnabled.body_role = static_cast<uint8_t>(role);
	req.setPhantomVirtualEnabled.enabled = enabled ? 1u : 0u;
	std::memset(req.setPhantomVirtualEnabled._reserved, 0, sizeof(req.setPhantomVirtualEnabled._reserved));
	try {
		ipc_.SendBlocking(req);
	}
	catch (const std::exception& e) {
		connectError_ = e.what();
	}
}

void PhantomPlugin::ReplayDriverState()
{
	if (!ipc_.IsConnected()) return;
	SendConfig();
	for (const auto& kv : cfg_.dropout_enabled) {
		SendDeviceOptIn(kv.first, kv.second);
	}
	SendSolverConfig();
	for (const auto& kv : cfg_.device_role) {
		SendDeviceRole(kv.first, kv.second);
	}
	for (const auto& kv : cfg_.virtual_enabled) {
		if (kv.second) SendVirtualEnabled(kv.first, true);
	}
}

void PhantomPlugin::DrawAutoDetectPanel()
{
	ui::DrawPanel("Automatic role detection", [&] {
		ui::DrawTextWrapped("Move around normally for a few seconds. The driver watches each tracker's "
		                    "height and motion relative to your headset and works out which body point "
		                    "it sits on. Confident detections are applied live; with auto-save on they "
		                    "also persist so the mapping is remembered next session. The Assigned column "
		                    "badges whether a saved role was detected (Auto) or hand-picked below (Manual); "
		                    "Clear removes a saved role and Manual roles are never overwritten by detection.");
		ImGui::Spacing();

		if (ImGui::Checkbox("Save confident detections automatically", &cfg_.auto_accept_roles)) {
			SavePhantomConfig(cfg_);
		}
		ui::TooltipForLastItem("When on, a detection above ~70% confidence is written to the saved "
		                       "tracker mapping without pressing Accept.");
		ImGui::Spacing();

		if (!stateShmemReady_ || !stateShmem_.layout()) {
			ui::DrawEmptyState("Driver state not yet available. Start SteamVR with the Phantom feature on.");
			return;
		}
		const auto* layout = stateShmem_.layout();
		if (layout->magic != phantom::kPhantomStateShmemMagic ||
		    layout->version != phantom::kPhantomStateShmemVersion) {
			ui::DrawErrorBanner("Driver mismatch",
			                    "Driver state version differs; reinstall so the driver and overlay match.");
			return;
		}

		ui::TableScope table("PhantomAutoDetect", 5,
		                     ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerH | ImGuiTableFlags_SizingStretchProp);
		if (!table) return;
		ui::SetupStretchColumn("Tracker", 1.5f);
		ui::SetupStretchColumn("Assigned", 1.0f);
		ui::SetupStretchColumn("Detected role", 1.0f);
		ui::SetupStretchColumn("Confidence", 1.2f);
		ui::SetupFixedColumn("", 138.0f);
		ImGui::TableHeadersRow();

		constexpr float kAutoAcceptConfidence = 0.70f;
		int shown = 0;
		for (uint32_t i = 0; i < layout->device_count; ++i) {
			const auto& d = layout->devices[i];
			if (d.serial_len == 0) continue;

			// Epoch-stable read of the inference fields so an auto-save never
			// fires on a torn snapshot.
			const uint32_t epochBefore = d.epoch;
			MemoryBarrier();
			const auto inferredRole = static_cast<phantom::BodyRole>(d.inferred_role);
			const float conf = d.inferred_confidence;
			const bool applied = d.inferred_applied != 0;
			const std::string serial(d.serial, d.serial_len);
			MemoryBarrier();
			const bool stable = (epochBefore == d.epoch) && ((epochBefore & 1u) == 0u);

			const auto cfgIt = cfg_.device_role.find(serial);
			const phantom::BodyRole cfgRole =
			    (cfgIt != cfg_.device_role.end()) ? cfgIt->second : phantom::BodyRole::None;
			const auto manualIt = cfg_.role_manual.find(serial);
			const bool savedIsManual =
			    cfgRole != phantom::BodyRole::None && manualIt != cfg_.role_manual.end() && manualIt->second;

			// Only surface devices the inference cares about: either it has an
			// estimate, or the user already mapped this tracker.
			if (inferredRole == phantom::BodyRole::None && conf <= 0.0f && cfgRole == phantom::BodyRole::None) {
				continue;
			}
			++shown;

			ImGui::PushID(("auto_" + serial).c_str());
			ImGui::TableNextRow();
			ImGui::TableSetColumnIndex(0);
			ImGui::TextUnformatted(serial.c_str());

			// Assigned role + where it came from: a hand-picked role shows a
			// Manual badge, a detection-sourced one shows Auto.
			ImGui::TableSetColumnIndex(1);
			if (cfgRole == phantom::BodyRole::None) {
				ImGui::TextDisabled("-");
			}
			else {
				ImGui::TextUnformatted(phantom::BodyRoleLabel(cfgRole));
				ImGui::SameLine();
				ui::StatusBadge(savedIsManual ? "Manual" : "Auto",
				                savedIsManual ? ui::StatusTone::Info : ui::StatusTone::Ok);
			}

			ImGui::TableSetColumnIndex(2);
			ImGui::TextUnformatted(phantom::BodyRoleLabel(inferredRole));
			if (applied && cfgRole == phantom::BodyRole::None) {
				ImGui::SameLine();
				ImGui::TextDisabled("(live)");
			}

			ImGui::TableSetColumnIndex(3);
			char pct[16];
			std::snprintf(pct, sizeof(pct), "%d%%", static_cast<int>(std::clamp(conf, 0.0f, 1.0f) * 100.0f + 0.5f));
			ImGui::SetNextItemWidth(-FLT_MIN);
			ImGui::ProgressBar(std::clamp(conf, 0.0f, 1.0f), ImVec2(-FLT_MIN, 0.0f), pct);

			ImGui::TableSetColumnIndex(4);
			const bool acceptable = inferredRole != phantom::BodyRole::None && inferredRole != cfgRole;
			ImGui::BeginDisabled(!acceptable);
			if (ImGui::Button("Accept")) {
				cfg_.device_role[serial] = inferredRole;
				cfg_.role_manual[serial] = false; // sourced from detection
				if (cfg_.dropout_enabled.count(serial) == 0) {
					cfg_.dropout_enabled[serial] = true;
					SendDeviceOptIn(serial, true);
				}
				SendDeviceRole(serial, inferredRole);
				SavePhantomConfig(cfg_);
			}
			ImGui::EndDisabled();
			ui::TooltipForLastItem("Save the detected role for this tracker.");
			ImGui::SameLine();
			ImGui::BeginDisabled(cfgRole == phantom::BodyRole::None);
			if (ImGui::Button("Clear")) {
				cfg_.device_role.erase(serial);
				cfg_.role_manual.erase(serial);
				SendDeviceRole(serial, phantom::BodyRole::None);
				SavePhantomConfig(cfg_);
			}
			ImGui::EndDisabled();
			ui::TooltipForLastItem("Remove the saved role. Automatic detection may re-assign it.");
			ImGui::PopID();

			// Auto-save confident detections. The driver has already applied
			// them live; this only writes the persistent mapping once, and never
			// over a hand-picked (Manual) role.
			if (phantom::ui::ShouldAutoSaveDetectedRole(cfg_.auto_accept_roles, stable, inferredRole, cfgRole,
			                                            savedIsManual, conf, kAutoAcceptConfidence)) {
				cfg_.device_role[serial] = inferredRole;
				cfg_.role_manual[serial] = false; // sourced from detection
				if (cfg_.dropout_enabled.count(serial) == 0) {
					cfg_.dropout_enabled[serial] = true;
					SendDeviceOptIn(serial, true);
				}
				SendDeviceRole(serial, inferredRole);
				SavePhantomConfig(cfg_);
			}
		}

		if (shown == 0) {
			ImGui::TableNextRow();
			ImGui::TableSetColumnIndex(0);
			ui::DrawEmptyState("Detecting... move around so the driver can see each tracker.");
		}
	});
}

void PhantomPlugin::DrawBodyTab()
{
	ImGui::Spacing();
	DrawAutoDetectPanel();
	ImGui::Spacing();

	ui::DrawPanel("Body priors", [&] {
		ui::DrawTextWrapped("Phantom estimates body proportions automatically from the headset, controllers, "
		                    "and any assigned physical trackers. It falls back to conservative defaults while "
		                    "the estimate warms up.");
	});

	ui::DrawPanel("Estimation gate", [&] {
		float tmp = static_cast<float>(cfg_.solver.virtual_min_confidence);
		if (ImGui::SliderFloat("Minimum confidence", &tmp, 0.0f, 1.0f, "%.2f")) {
			cfg_.solver.virtual_min_confidence = static_cast<double>(std::clamp(tmp, 0.0f, 1.0f));
			SendSolverConfig();
			SavePhantomConfig(cfg_);
		}
		ui::TooltipForLastItem("Estimated trackers stop publishing below this confidence instead of guessing.");
	});

	auto* vrSystem = vr::VRSystem();
	if (!vrSystem) {
		ui::DrawPanel("Physical tracker roles", [&] { ui::DrawEmptyState("VR system not available."); });
		return;
	}

	ui::DrawPanel("Physical tracker roles", [&] {
		ui::TableScope table("PhantomRoles", 2,
		                     ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerH | ImGuiTableFlags_SizingStretchProp);
		if (!table) return;
		ui::SetupStretchColumn("Tracker", 1.6f);
		ui::SetupStretchColumn("Role", 1.0f);
		ImGui::TableHeadersRow();

		bool anyShown = false;
		char buffer[vr::k_unMaxPropertyStringSize];
		for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id) {
			const auto cls = vrSystem->GetTrackedDeviceClass(id);
			if (cls == vr::TrackedDeviceClass_Invalid) continue;

			vr::ETrackedPropertyError err = vr::TrackedProp_Success;
			vrSystem->GetStringTrackedDeviceProperty(id, vr::Prop_SerialNumber_String, buffer, sizeof(buffer), &err);
			if (err != vr::TrackedProp_Success || buffer[0] == 0) continue;
			const std::string serial = buffer;

			vrSystem->GetStringTrackedDeviceProperty(id, vr::Prop_RenderModelName_String, buffer, sizeof(buffer), &err);
			const std::string model = (err == vr::TrackedProp_Success) ? buffer : "";

			vrSystem->GetStringTrackedDeviceProperty(id, vr::Prop_TrackingSystemName_String, buffer, sizeof(buffer),
			                                         &err);
			const std::string trackingSystem = (err == vr::TrackedProp_Success) ? buffer : "";

			if (!openvr_pair::overlay::ShouldShowInSmoothingPredictionList(cls, serial, model, trackingSystem)) {
				continue;
			}

			anyShown = true;
			const auto roleIt = cfg_.device_role.find(serial);
			phantom::BodyRole cur = (roleIt != cfg_.device_role.end()) ? roleIt->second : phantom::BodyRole::None;

			ImGui::PushID(("body_" + serial).c_str());
			ImGui::TableNextRow();
			ImGui::TableSetColumnIndex(0);
			ImGui::TextUnformatted(model.empty() ? "(unknown model)" : model.c_str());
			ImGui::SameLine();
			ImGui::TextDisabled("[%s]", serial.c_str());

			ImGui::TableSetColumnIndex(1);
			ImGui::SetNextItemWidth(-FLT_MIN);
			if (ImGui::BeginCombo("##role", phantom::BodyRoleLabel(cur))) {
				for (uint8_t i = 0; i < phantom::kBodyRoleCount; ++i) {
					const auto r = static_cast<phantom::BodyRole>(i);
					// HMD / hand roles are not assignable on a body tracker;
					// skip them in the dropdown to avoid invalid combinations.
					if (r == phantom::BodyRole::Hmd || r == phantom::BodyRole::LeftHand ||
					    r == phantom::BodyRole::RightHand)
						continue;
					const bool selected = (r == cur);
					if (ImGui::Selectable(phantom::BodyRoleLabel(r), selected)) {
						if (r == phantom::BodyRole::None) {
							cfg_.device_role.erase(serial);
							cfg_.role_manual.erase(serial);
						}
						else {
							cfg_.device_role[serial] = r;
							cfg_.role_manual[serial] = true; // hand-picked
							if (cfg_.dropout_enabled.count(serial) == 0) {
								cfg_.dropout_enabled[serial] = true;
								SendDeviceOptIn(serial, true);
							}
						}
						SendDeviceRole(serial, r);
						SavePhantomConfig(cfg_);
					}
					if (selected) ImGui::SetItemDefaultFocus();
				}
				ImGui::EndCombo();
			}
			ImGui::PopID();
		}
		if (!anyShown) {
			ImGui::TableNextRow();
			ImGui::TableSetColumnIndex(0);
			ui::DrawEmptyState("No assignable trackers detected.");
		}
	});
}

void PhantomPlugin::DrawAbsentTab()
{
	ImGui::Spacing();
	ui::DrawInfoBanner(
	    "Estimated trackers",
	    "Enable only the roles you need. When confidence is low, Phantom stops publishing instead of guessing.");
	ImGui::Spacing();

	const phantom::BodyRole roles[] = {
	    phantom::BodyRole::Waist,     phantom::BodyRole::Chest,      phantom::BodyRole::LeftFoot,
	    phantom::BodyRole::RightFoot, phantom::BodyRole::LeftKnee,   phantom::BodyRole::RightKnee,
	    phantom::BodyRole::LeftElbow, phantom::BodyRole::RightElbow,
	};

	auto physicalRoleAssigned = [&](phantom::BodyRole role) {
		for (const auto& kv : cfg_.device_role) {
			if (kv.second == role) return true;
		}
		return false;
	};

	ui::DrawPanel("Per-role virtual trackers", [&] {
		ui::TableScope table("phantom_absent_roles", 4,
		                     ImGuiTableFlags_BordersOuter | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp);
		if (!table) return;
		ui::SetupFixedColumn("Enable", 70.0f);
		ui::SetupStretchColumn("Role", 1.5f);
		ui::SetupStretchColumn("Risk", 1.0f);
		ui::SetupStretchColumn("Status", 2.0f);
		ImGui::TableHeadersRow();

		for (auto role : roles) {
			bool enabled = cfg_.virtual_enabled.count(role) ? cfg_.virtual_enabled[role] : false;
			const auto tier = phantom::ui::GetVirtualRoleTier(role);
			const auto readiness = phantom::ui::EvaluateVirtualRoleReadiness(physicalRoleAssigned(role));
			const bool disableToggle = !enabled && !readiness.canEnable;

			ImGui::TableNextRow();
			ImGui::TableSetColumnIndex(0);
			ImGui::PushID(static_cast<int>(role));
			{
				ui::DisabledSection gate(disableToggle, readiness.reason);
				if (ImGui::Checkbox("##en", &enabled)) {
					cfg_.virtual_enabled[role] = enabled;
					SendVirtualEnabled(role, enabled);
					SavePhantomConfig(cfg_);
				}
				gate.AttachReasonTooltip();
			}

			ImGui::TableSetColumnIndex(1);
			ImGui::TextUnformatted(phantom::BodyRoleLabel(role));

			ImGui::TableSetColumnIndex(2);
			ImGui::TextUnformatted(phantom::ui::VirtualRoleTierLabel(tier));
			ui::TooltipOnHover(phantom::ui::VirtualRoleTierHelp(tier));

			ImGui::TableSetColumnIndex(3);
			if (!readiness.canEnable) {
				ImGui::TextDisabled("%s", readiness.reason);
			}
			else if (enabled) {
				ui::DrawStatusText("Publishing when confident", ui::StatusTone::Ok);
			}
			else {
				ImGui::TextDisabled("Ready");
			}
			ImGui::PopID();
		}
	});

	ImGui::Spacing();
	ImGui::TextDisabled(
	    "SteamVR keeps virtual devices until vrserver restarts; disabling a role stops pose publishing now.");
}

namespace openvr_pair::overlay {

std::unique_ptr<FeaturePlugin> CreatePhantomPlugin()
{
	return std::make_unique<PhantomPlugin>();
}

} // namespace openvr_pair::overlay
