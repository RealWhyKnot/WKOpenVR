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
#include <cmath>
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
	s.calibrated = cfg_.solver.calibrated ? 1u : 0u;
	s.floor_y_m = cfg_.solver.floor_y_m;
	s.height_m = cfg_.solver.height_m;
	s.forward_yaw_rad = cfg_.solver.forward_yaw_rad;
	s.stance_width_m = cfg_.solver.stance_width_m;
	s.shoulder_width_m = cfg_.solver.shoulder_width_m;
	s.pelvis_width_m = cfg_.solver.pelvis_width_m;
	s.upper_arm_m = cfg_.solver.upper_arm_m;
	s.lower_arm_m = cfg_.solver.lower_arm_m;
	s.upper_leg_m = cfg_.solver.upper_leg_m;
	s.lower_leg_m = cfg_.solver.lower_leg_m;
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
		ui::DrawTabItem("Calibration", [&] { DrawCalibrationTab(); });
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

// Quaternion conjugate (== inverse for a unit quaternion).
vr::HmdQuaternion_t QConj(const vr::HmdQuaternion_t& q)
{
	return {q.w, -q.x, -q.y, -q.z};
}

vr::HmdQuaternion_t QMul(const vr::HmdQuaternion_t& a, const vr::HmdQuaternion_t& b)
{
	vr::HmdQuaternion_t r;
	r.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
	r.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
	r.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
	r.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
	return r;
}

// HmdMatrix34_t (column-major 3x4 with column 3 = translation) -> position
// + quaternion. Matches the convention used by GetDeviceToAbsoluteTrackingPose.
void DecomposeMatrix34(const vr::HmdMatrix34_t& m, double pos[3], vr::HmdQuaternion_t& q)
{
	pos[0] = m.m[0][3];
	pos[1] = m.m[1][3];
	pos[2] = m.m[2][3];
	// Standard rotation-matrix to quaternion. The matrix's upper-left 3x3
	// is the rotation; we read it as row-major-3x3 directly.
	const double m00 = m.m[0][0], m01 = m.m[0][1], m02 = m.m[0][2];
	const double m10 = m.m[1][0], m11 = m.m[1][1], m12 = m.m[1][2];
	const double m20 = m.m[2][0], m21 = m.m[2][1], m22 = m.m[2][2];
	const double trace = m00 + m11 + m22;
	if (trace > 0.0) {
		const double s = std::sqrt(trace + 1.0) * 2.0;
		q.w = 0.25 * s;
		q.x = (m21 - m12) / s;
		q.y = (m02 - m20) / s;
		q.z = (m10 - m01) / s;
	}
	else if (m00 > m11 && m00 > m22) {
		const double s = std::sqrt(1.0 + m00 - m11 - m22) * 2.0;
		q.w = (m21 - m12) / s;
		q.x = 0.25 * s;
		q.y = (m01 + m10) / s;
		q.z = (m02 + m20) / s;
	}
	else if (m11 > m22) {
		const double s = std::sqrt(1.0 + m11 - m00 - m22) * 2.0;
		q.w = (m02 - m20) / s;
		q.x = (m01 + m10) / s;
		q.y = 0.25 * s;
		q.z = (m12 + m21) / s;
	}
	else {
		const double s = std::sqrt(1.0 + m22 - m00 - m11) * 2.0;
		q.w = (m10 - m01) / s;
		q.x = (m02 + m20) / s;
		q.y = (m12 + m21) / s;
		q.z = 0.25 * s;
	}
}

void QRotateInverse(const vr::HmdQuaternion_t& q, const double v[3], double out[3])
{
	// Rotate v by q^-1 (= conjugate for unit q). Standard formula via
	// q^-1 * v * q expressed without intermediate quaternions.
	const auto qi = QConj(q);
	const double ux = qi.x, uy = qi.y, uz = qi.z, s = qi.w;
	const double tx = 2.0 * (uy * v[2] - uz * v[1]);
	const double ty = 2.0 * (uz * v[0] - ux * v[2]);
	const double tz = 2.0 * (ux * v[1] - uy * v[0]);
	out[0] = v[0] + s * tx + (uy * tz - uz * ty);
	out[1] = v[1] + s * ty + (uz * tx - ux * tz);
	out[2] = v[2] + s * tz + (ux * ty - uy * tx);
}

double YawRadiansFromQuat(const vr::HmdQuaternion_t& q)
{
	const double siny = 2.0 * (q.w * q.y + q.x * q.z);
	const double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
	return std::atan2(siny, cosy);
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

void PhantomPlugin::SendTrackerOffset(phantom::BodyRole role, const PhantomRoleOffset& offset)
{
	if (!ipc_.IsConnected()) return;
	protocol::Request req(protocol::RequestSetPhantomTrackerOffset);
	auto& o = req.setPhantomTrackerOffset;
	o.body_role = static_cast<uint8_t>(role);
	o.calibrated = offset.calibrated ? 1u : 0u;
	std::memset(o._pad, 0, sizeof(o._pad));
	o.rel_position[0] = offset.rel_position_x;
	o.rel_position[1] = offset.rel_position_y;
	o.rel_position[2] = offset.rel_position_z;
	o.rel_rotation.w = offset.rel_rotation_w;
	o.rel_rotation.x = offset.rel_rotation_x;
	o.rel_rotation.y = offset.rel_rotation_y;
	o.rel_rotation.z = offset.rel_rotation_z;
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
		if (kv.second) SendDeviceOptIn(kv.first, true);
	}
	ReplayCalibration();
}

void PhantomPlugin::ReplayCalibration()
{
	if (!ipc_.IsConnected()) return;
	SendSolverConfig();
	for (const auto& kv : cfg_.device_role) {
		SendDeviceRole(kv.first, kv.second);
	}
	for (const auto& kv : cfg_.role_offset) {
		if (kv.second.calibrated) SendTrackerOffset(kv.first, kv.second);
	}
	for (const auto& kv : cfg_.virtual_enabled) {
		if (kv.second) SendVirtualEnabled(kv.first, true);
	}
}

void PhantomPlugin::CaptureNeutralStanding()
{
	auto* vrSystem = vr::VRSystem();
	if (!vrSystem) {
		lastSolverCalibrationSummary_ = "VR system not available; neutral pose not captured.";
		return;
	}

	vr::TrackedDevicePose_t poses[vr::k_unMaxTrackedDeviceCount]{};
	vrSystem->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseStanding,
	                                          /*predictedSecondsToPhotonsFromNow=*/0.0f, poses,
	                                          vr::k_unMaxTrackedDeviceCount);

	uint32_t hmdId = vr::k_unTrackedDeviceIndexInvalid;
	for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id) {
		if (vrSystem->GetTrackedDeviceClass(id) == vr::TrackedDeviceClass_HMD && poses[id].bPoseIsValid &&
		    poses[id].eTrackingResult == vr::TrackingResult_Running_OK) {
			hmdId = id;
			break;
		}
	}
	if (hmdId == vr::k_unTrackedDeviceIndexInvalid) {
		lastSolverCalibrationSummary_ = "HMD not tracked; neutral pose not captured.";
		return;
	}

	double hmdPos[3];
	vr::HmdQuaternion_t hmdRot;
	DecomposeMatrix34(poses[hmdId].mDeviceToAbsoluteTracking, hmdPos, hmdRot);

	cfg_.solver.calibrated = true;
	cfg_.solver.floor_y_m = 0.0;
	cfg_.solver.height_m = std::clamp(hmdPos[1] - cfg_.solver.floor_y_m, 1.0, 2.4);
	cfg_.solver.forward_yaw_rad = YawRadiansFromQuat(hmdRot);

	const double h = cfg_.solver.height_m;
	cfg_.solver.stance_width_m = std::clamp(h * 0.165, 0.10, 0.70);
	cfg_.solver.shoulder_width_m = std::clamp(h * 0.225, 0.20, 0.70);
	cfg_.solver.pelvis_width_m = std::clamp(h * 0.165, 0.15, 0.60);
	cfg_.solver.upper_arm_m = std::clamp(h * 0.176, 0.15, 0.55);
	cfg_.solver.lower_arm_m = std::clamp(h * 0.159, 0.15, 0.55);
	cfg_.solver.upper_leg_m = std::clamp(h * 0.265, 0.20, 0.70);
	cfg_.solver.lower_leg_m = std::clamp(h * 0.265, 0.20, 0.70);

	SendSolverConfig();
	SavePhantomConfig(cfg_);

	char tmp[128];
	std::snprintf(tmp, sizeof(tmp), "Neutral standing captured: height %.2f m, forward %.2f rad.", cfg_.solver.height_m,
	              cfg_.solver.forward_yaw_rad);
	lastSolverCalibrationSummary_ = tmp;
}

void PhantomPlugin::CaptureTPose()
{
	auto* vrSystem = vr::VRSystem();
	if (!vrSystem) {
		lastCalibrationSummary_ = "VR system not available; tracking poses not captured.";
		return;
	}

	vr::TrackedDevicePose_t poses[vr::k_unMaxTrackedDeviceCount]{};
	vrSystem->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseStanding,
	                                          /*predictedSecondsToPhotonsFromNow=*/0.0f, poses,
	                                          vr::k_unMaxTrackedDeviceCount);

	// Locate the HMD (Class_HMD). Without it we have no reference frame for
	// the IK fallback's rigid offsets.
	uint32_t hmdId = vr::k_unTrackedDeviceIndexInvalid;
	for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id) {
		if (vrSystem->GetTrackedDeviceClass(id) == vr::TrackedDeviceClass_HMD && poses[id].bPoseIsValid &&
		    poses[id].eTrackingResult == vr::TrackingResult_Running_OK) {
			hmdId = id;
			break;
		}
	}
	if (hmdId == vr::k_unTrackedDeviceIndexInvalid) {
		lastCalibrationSummary_ = "HMD not tracked; calibration aborted.";
		return;
	}

	double hmdPos[3];
	vr::HmdQuaternion_t hmdRot;
	DecomposeMatrix34(poses[hmdId].mDeviceToAbsoluteTracking, hmdPos, hmdRot);

	int captured = 0;
	int skipped = 0;
	char buffer[vr::k_unMaxPropertyStringSize];
	for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id) {
		const auto cls = vrSystem->GetTrackedDeviceClass(id);
		if (cls == vr::TrackedDeviceClass_Invalid || cls == vr::TrackedDeviceClass_HMD) {
			continue;
		}
		if (!poses[id].bPoseIsValid || poses[id].eTrackingResult != vr::TrackingResult_Running_OK) {
			continue;
		}
		vr::ETrackedPropertyError err = vr::TrackedProp_Success;
		vrSystem->GetStringTrackedDeviceProperty(id, vr::Prop_SerialNumber_String, buffer, sizeof(buffer), &err);
		if (err != vr::TrackedProp_Success || buffer[0] == 0) continue;
		const std::string serial = buffer;

		const auto roleIt = cfg_.device_role.find(serial);
		if (roleIt == cfg_.device_role.end() || roleIt->second == phantom::BodyRole::None) {
			++skipped;
			continue;
		}

		double trkPos[3];
		vr::HmdQuaternion_t trkRot;
		DecomposeMatrix34(poses[id].mDeviceToAbsoluteTracking, trkPos, trkRot);

		const double delta[3] = {
		    trkPos[0] - hmdPos[0],
		    trkPos[1] - hmdPos[1],
		    trkPos[2] - hmdPos[2],
		};
		double rel[3];
		QRotateInverse(hmdRot, delta, rel);
		const vr::HmdQuaternion_t relRot = QMul(QConj(hmdRot), trkRot);

		PhantomRoleOffset off{};
		off.calibrated = true;
		off.rel_position_x = rel[0];
		off.rel_position_y = rel[1];
		off.rel_position_z = rel[2];
		off.rel_rotation_w = relRot.w;
		off.rel_rotation_x = relRot.x;
		off.rel_rotation_y = relRot.y;
		off.rel_rotation_z = relRot.z;

		cfg_.role_offset[roleIt->second] = off;
		SendTrackerOffset(roleIt->second, off);
		++captured;
	}

	SavePhantomConfig(cfg_);
	char tmp[128];
	std::snprintf(tmp, sizeof(tmp), "Captured %d role%s; skipped %d unassigned tracker%s.", captured,
	              captured == 1 ? "" : "s", skipped, skipped == 1 ? "" : "s");
	lastCalibrationSummary_ = tmp;
}

void PhantomPlugin::DrawAutoDetectPanel()
{
	ui::DrawPanel("Automatic role detection", [&] {
		ui::DrawTextWrapped("Move around normally for a few seconds. The driver watches each tracker's "
		                    "height and motion relative to your headset and works out which body point "
		                    "it sits on. Confident detections are applied live; with auto-save on they "
		                    "also persist so the mapping is remembered next session.");
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

		ui::TableScope table("PhantomAutoDetect", 4,
		                     ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerH | ImGuiTableFlags_SizingStretchProp);
		if (!table) return;
		ui::SetupStretchColumn("Tracker", 1.6f);
		ui::SetupStretchColumn("Detected role", 1.1f);
		ui::SetupStretchColumn("Confidence", 1.3f);
		ui::SetupFixedColumn("", 84.0f);
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
			if (cfgRole != phantom::BodyRole::None) {
				ImGui::SameLine();
				ImGui::TextDisabled("(saved: %s)", phantom::BodyRoleLabel(cfgRole));
			}

			ImGui::TableSetColumnIndex(1);
			ImGui::TextUnformatted(phantom::BodyRoleLabel(inferredRole));
			if (applied) {
				ImGui::SameLine();
				ImGui::TextDisabled("(live)");
			}

			ImGui::TableSetColumnIndex(2);
			char pct[16];
			std::snprintf(pct, sizeof(pct), "%d%%", static_cast<int>(std::clamp(conf, 0.0f, 1.0f) * 100.0f + 0.5f));
			ImGui::SetNextItemWidth(-FLT_MIN);
			ImGui::ProgressBar(std::clamp(conf, 0.0f, 1.0f), ImVec2(-FLT_MIN, 0.0f), pct);

			ImGui::TableSetColumnIndex(3);
			const bool acceptable = inferredRole != phantom::BodyRole::None && inferredRole != cfgRole;
			ImGui::BeginDisabled(!acceptable);
			if (ImGui::Button("Accept")) {
				cfg_.device_role[serial] = inferredRole;
				SendDeviceRole(serial, inferredRole);
				SavePhantomConfig(cfg_);
			}
			ImGui::EndDisabled();
			ImGui::PopID();

			// Auto-save confident detections. The driver has already applied
			// them live; this only writes the persistent mapping once.
			if (phantom::ui::ShouldAutoSaveDetectedRole(cfg_.auto_accept_roles, stable, inferredRole, cfgRole, conf,
			                                            kAutoAcceptConfidence)) {
				cfg_.device_role[serial] = inferredRole;
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

void PhantomPlugin::DrawCalibrationTab()
{
	ImGui::Spacing();
	DrawAutoDetectPanel();
	ImGui::Spacing();
	ui::DrawTextWrapped("Automatic detection above is the main path. The controls below are optional: "
	                    "capture neutral standing for headset/controller body completion, or assign a "
	                    "tracker by hand to override or seed a detection.");
	ImGui::Spacing();

	ui::DrawPanel("Neutral standing", [&] {
		if (ImGui::Button("Capture neutral standing")) {
			CaptureNeutralStanding();
		}
		ui::TooltipForLastItem("Stand upright, face forward, and click. Uses the current HMD pose "
		                       "to set height, floor, forward direction, and starting proportions.");
		if (!lastSolverCalibrationSummary_.empty()) {
			ImGui::Spacing();
			ui::DrawStatusText(lastSolverCalibrationSummary_.c_str(), ui::StatusTone::Ok);
		}
	});

	ui::DrawPanel("Body proportions", [&] {
		ui::DrawSettingTable("PhantomBody", 150.0f, [&](ui::SettingTableScope& tbl) {
			auto solverRow = [&](const char* label, const char* id, double& v, float lo, float hi, const char* fmt) {
				ui::SettingRow(tbl, label, [&] {
					float tmp = static_cast<float>(v);
					ImGui::SetNextItemWidth(-FLT_MIN);
					if (ImGui::SliderFloat(id, &tmp, lo, hi, fmt)) {
						v = static_cast<double>(std::clamp(tmp, lo, hi));
						cfg_.solver.calibrated = true;
						SendSolverConfig();
						SavePhantomConfig(cfg_);
					}
				});
			};
			solverRow("Height", "##height", cfg_.solver.height_m, 1.0f, 2.4f, "%.2f m");
			solverRow("Floor Y", "##floory", cfg_.solver.floor_y_m, -1.0f, 1.0f, "%.2f m");
			solverRow("Stance width", "##stance", cfg_.solver.stance_width_m, 0.10f, 0.70f, "%.2f m");
			solverRow("Shoulder width", "##shoulder", cfg_.solver.shoulder_width_m, 0.20f, 0.70f, "%.2f m");
			solverRow("Pelvis width", "##pelvis", cfg_.solver.pelvis_width_m, 0.15f, 0.60f, "%.2f m");
			solverRow("Upper arm", "##uparm", cfg_.solver.upper_arm_m, 0.15f, 0.55f, "%.2f m");
			solverRow("Lower arm", "##loarm", cfg_.solver.lower_arm_m, 0.15f, 0.55f, "%.2f m");
			solverRow("Upper leg", "##upleg", cfg_.solver.upper_leg_m, 0.20f, 0.70f, "%.2f m");
			solverRow("Lower leg", "##loleg", cfg_.solver.lower_leg_m, 0.20f, 0.70f, "%.2f m");
		});
	});

	ui::DrawPanel("Estimation gate", [&] {
		float tmp = static_cast<float>(cfg_.solver.virtual_min_confidence);
		if (ImGui::SliderFloat("Minimum confidence", &tmp, 0.0f, 1.0f, "%.2f")) {
			cfg_.solver.virtual_min_confidence = static_cast<double>(std::clamp(tmp, 0.0f, 1.0f));
			cfg_.solver.calibrated = true;
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

			ImGui::PushID(("cal_" + serial).c_str());
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
						}
						else {
							cfg_.device_role[serial] = r;
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

	ui::DrawPanel("Tracker mounting offsets", [&] {
		if (ImGui::Button("Capture T-pose now")) {
			CaptureTPose();
		}
		ui::TooltipForLastItem("Stand in a T-pose (arms out, body upright, head level) and click.\n"
		                       "Captures the rigid mounting offset of every assigned physical\n"
		                       "tracker. Reassigning a role or moving a tracker means re-capturing.");
		if (!lastCalibrationSummary_.empty()) {
			ImGui::Spacing();
			ui::DrawStatusText(lastCalibrationSummary_.c_str(), ui::StatusTone::Ok);
		}
	});

	ui::DrawPanel("Calibration status", [&] {
		if (cfg_.role_offset.empty()) {
			ui::DrawEmptyState("No roles calibrated yet.");
			return;
		}
		ui::TableScope table("PhantomCalStatus", 2,
		                     ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerH | ImGuiTableFlags_SizingStretchProp);
		if (!table) return;
		ui::SetupStretchColumn("Role", 1.0f);
		ui::SetupStretchColumn("Status", 1.0f);
		ImGui::TableHeadersRow();
		for (const auto& kv : cfg_.role_offset) {
			ImGui::TableNextRow();
			ImGui::TableSetColumnIndex(0);
			ImGui::TextUnformatted(phantom::BodyRoleLabel(kv.first));
			ImGui::TableSetColumnIndex(1);
			ui::DrawStatusCell(kv.second.calibrated ? "Calibrated" : "Not captured",
			                   kv.second.calibrated ? ui::StatusTone::Ok : ui::StatusTone::Idle);
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
			const auto readiness =
			    phantom::ui::EvaluateVirtualRoleReadiness(cfg_.solver.calibrated, physicalRoleAssigned(role));
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
