#include "InputHealthPlugin.h"

#include "Config.h"
#include "DebugLogging.h"
#include "DebugTab.h"
#include "DiagnosticsTab.h"
#include "DiscordPresenceComposer.h"
#include "IPCClient.h"
#include "LearningEngine.h"
#include "Logging.h"
#include "SettingsTab.h"
#include "ShellFooter.h"
#include "SnapshotReader.h"
#include "UiHelpers.h"
#include "inputhealth/PresenceCounting.h"

#include <imgui/imgui.h>

#include <chrono>
#include <exception>
#include <memory>
#include <string>

using Clock = std::chrono::steady_clock;

InputHealthPlugin::InputHealthPlugin()
	: engine_(ipc_, profiles_)
{
	const InputHealthGlobalConfig saved = LoadInputHealthConfig();
	pending_config_.master_enabled       = saved.master_enabled;
	pending_config_.diagnostics_only     = saved.diagnostics_only;
	pending_config_.enable_rest_recenter = saved.enable_rest_recenter;
	pending_config_.enable_trigger_remap = saved.enable_trigger_remap;
	observed_ipc_generation_ = ipc_.ConnectionGeneration();
}

void InputHealthPlugin::OnStart(openvr_pair::overlay::ShellContext &)
{
	OpenLogFile();
	LOG("WKOpenVR-InputHealth plugin starting");
	profiles_.LoadAll();
	try {
		ipc_.Connect();
		LOG("[ui] IPC connected");
		engine_.PushReadyCompensations();
		PushConfigToDriver();
	} catch (const std::exception &e) {
		LOG("[ui] initial IPC connect failed: %s", e.what());
		last_error_ = std::string("InputHealth IPC: ") + e.what();
	}
	reader_.TryOpen();
	const auto now = Clock::now();
	last_refresh_ = now;
	last_connection_check_ = now;
}

void InputHealthPlugin::OnShutdown(openvr_pair::overlay::ShellContext &)
{
	engine_.Flush();
	reader_.Close();
	ipc_.Close();
	LOG("WKOpenVR-InputHealth plugin shutting down");
	LogFlush();
}

void InputHealthPlugin::Tick(openvr_pair::overlay::ShellContext &)
{
	const auto now = Clock::now();
	if (now - last_connection_check_ >= std::chrono::seconds(1)) {
		MaintainDriverConnection();
		last_connection_check_ = now;
	}
	if (now - last_refresh_ >= std::chrono::milliseconds(100)) {
		reader_.Refresh();
		engine_.Tick(reader_);
		last_refresh_ = now;
	}
}

void InputHealthPlugin::PushConfigToDriver()
{
	if (!ipc_.IsConnected()) {
		last_error_ = "Not connected to the InputHealth driver. Is SteamVR running?";
		return;
	}
	try {
		protocol::Request req(protocol::RequestSetInputHealthConfig);
		req.setInputHealthConfig = pending_config_;
		auto resp = ipc_.SendBlocking(req);
		if (resp.type != protocol::ResponseSuccess) {
			last_error_ = "Driver rejected SetInputHealthConfig (response type=" + std::to_string(resp.type) + ")";
			LOG("[ui] driver rejected config push: type=%d", (int)resp.type);
			return;
		}
		last_error_.clear();
		LOG("[ui] config pushed: master=%d diag_only=%d rest=%d trig=%d",
			(int)pending_config_.master_enabled,
			(int)pending_config_.diagnostics_only,
			(int)pending_config_.enable_rest_recenter,
			(int)pending_config_.enable_trigger_remap);
	} catch (const std::exception &e) {
		last_error_ = std::string("IPC error: ") + e.what();
		LOG("[ui] PushConfigToDriver failed: %s", e.what());
	}
}

void InputHealthPlugin::SaveGlobalConfig()
{
	InputHealthGlobalConfig cfg;
	cfg.master_enabled       = pending_config_.master_enabled;
	cfg.diagnostics_only     = pending_config_.diagnostics_only;
	cfg.enable_rest_recenter = pending_config_.enable_rest_recenter;
	cfg.enable_trigger_remap = pending_config_.enable_trigger_remap;
	SaveInputHealthConfig(cfg);
	LOG("[ui] global config saved: master=%d diag_only=%d rest=%d trig=%d",
		(int)cfg.master_enabled, (int)cfg.diagnostics_only,
		(int)cfg.enable_rest_recenter, (int)cfg.enable_trigger_remap);
}

void InputHealthPlugin::MaintainDriverConnection()
{
	try {
		if (!ipc_.IsConnected()) {
			ipc_.Connect();
			LOG("[ui] IPC connected from heartbeat");
		}

		const auto resp = ipc_.SendBlocking(protocol::Request(protocol::RequestHandshake));
		if (resp.type != protocol::ResponseHandshake || resp.protocol.version != protocol::Version) {
			last_error_ = "Driver protocol mismatch during heartbeat";
			LOG("[ui] heartbeat protocol mismatch: type=%d driverVersion=%u overlayVersion=%u",
				(int)resp.type, resp.protocol.version, protocol::Version);
			return;
		}

		const uint64_t gen = ipc_.ConnectionGeneration();
		if (gen != observed_ipc_generation_) {
			observed_ipc_generation_ = gen;
			LOG("[ui] IPC generation changed to %llu; re-sending InputHealth config and compensations",
				(unsigned long long)gen);
			PushConfigToDriver();
			engine_.PushReadyCompensations();
		}
		if (last_error_.find("Driver connection:") == 0 ||
			last_error_.find("InputHealth IPC") == 0 ||
			last_error_.find("Not connected") == 0) {
			last_error_.clear();
		}
	} catch (const std::exception &e) {
		last_error_ = std::string("Driver connection: ") + e.what();
		LOG("[ui] heartbeat failed: %s", e.what());
		ipc_.Close();
	}
}

void InputHealthPlugin::SendReset(uint64_t serial_hash, bool reset_passive, bool reset_active, bool reset_curves)
{
	if (!ipc_.IsConnected()) {
		last_error_ = "Not connected to the InputHealth driver. Is SteamVR running?";
		return;
	}
	try {
		protocol::Request req(protocol::RequestResetInputHealthStats);
		req.resetInputHealthStats.device_serial_hash = serial_hash;
		req.resetInputHealthStats.reset_passive      = reset_passive ? 1 : 0;
		req.resetInputHealthStats.reset_active       = reset_active  ? 1 : 0;
		req.resetInputHealthStats.reset_curves       = reset_curves  ? 1 : 0;
		auto resp = ipc_.SendBlocking(req);
		if (resp.type != protocol::ResponseSuccess) {
			last_error_ = "Driver rejected ResetInputHealthStats";
			return;
		}
		last_error_.clear();
		LOG("[ui] reset request sent: serial=0x%016llx passive=%d active=%d curves=%d",
			(unsigned long long)serial_hash, (int)reset_passive, (int)reset_active, (int)reset_curves);
	} catch (const std::exception &e) {
		last_error_ = std::string("IPC error: ") + e.what();
		LOG("[ui] SendReset failed: %s", e.what());
	}
}

void InputHealthPlugin::DrawStatusBanner()
{
	// Connection state lives in the shared footer; the live IPC / shmem /
	// publish_tick triple lives on the Logs tab. The top banner is reserved
	// for actual errors so a narrow window does not overflow with noise.
	if (!last_error_.empty()) {
		openvr_pair::overlay::ui::DrawErrorBanner(
			"InputHealth driver problem",
			last_error_.c_str());
	}
	// Shmem-not-open is normal during driver startup and can flicker briefly
	// on reconnects. Surface it as subtle disabled-grey text rather than a
	// hard red banner so it does not read as a tracking problem when it is
	// only a transient handshake state.
	if (!reader_.IsOpen() && !reader_.LastError().empty()) {
		ImGui::TextDisabled("Shmem: %s", reader_.LastError().c_str());
	}
}

void InputHealthPlugin::DrawDiagnosticsTab()
{
	inputhealth::ui::DrawDiagnosticsTab(*this);
}

void InputHealthPlugin::DrawSettingsTab()
{
	inputhealth::ui::DrawSettingsTab(*this);
}

void InputHealthPlugin::DrawAdvancedTab()
{
	inputhealth::ui::DrawAdvancedTab(*this);
}

void InputHealthPlugin::DrawLogsTab()
{
	inputhealth::ui::DrawLogsTab(*this);
}

void InputHealthPlugin::DrawTab(openvr_pair::overlay::ShellContext &)
{
	DrawStatusBanner();

	if (ImGui::BeginTabBar("inputhealth_tabs")) {
		if (ImGui::BeginTabItem("Diagnostics")) { DrawDiagnosticsTab(); ImGui::EndTabItem(); }
		if (ImGui::BeginTabItem("Settings"))    { DrawSettingsTab();    ImGui::EndTabItem(); }
		if (ImGui::BeginTabItem("Advanced"))    { DrawAdvancedTab();    ImGui::EndTabItem(); }
		// Logs moved to the umbrella's global Logs tab; the InputHealth-specific
		// list of files + IPC state surface there via DrawLogsSection().
		ImGui::EndTabBar();
	}

	openvr_pair::overlay::ShellFooterStatus footer;
	footer.driverConnected = ipc_.IsConnected();
	footer.driverLabel = "InputHealth driver";
	openvr_pair::overlay::DrawShellFooter(footer);
}

void InputHealthPlugin::DrawLogsSection(openvr_pair::overlay::ShellContext &)
{
	// Reuse the per-tab Logs implementation. The umbrella's global Logs tab
	// wraps each plugin in a collapsing header so the section reads with no
	// duplicate plugin-name heading.
	DrawLogsTab();
}

void InputHealthPlugin::ProvidePresence(WKOpenVR::PresenceComposer &composer)
{
	// Counting rules live in core/src/common/inputhealth/PresenceCounting.h
	// so the unit tests can exercise them with no SteamVR / IPC / UI deps.
	// The presence card had been reporting inflated "controller" counts
	// because the old loop inserted every device_serial_hash from every
	// slot in the shmem ring, including slots whose path was /proximity,
	// /eye/openness, etc., and slots whose serial hash was still 0.
	const inputhealth::PresenceCounts counts =
		inputhealth::CountInputHealthPresence(reader_.EntriesByHandle());

	WKOpenVR::PresenceUpdate u;
	if (counts.compensation_paths == 0) {
		u.priority = 0;
		u.details  = "Input Health";
		u.state    = "no input watched";
	} else if (counts.warnings > 0) {
		u.priority = 100;
		u.details  = "Input Health";
		u.state    = std::to_string(counts.devices) + " devices | " +
		             std::to_string(counts.compensation_paths) + " paths | " +
		             std::to_string(counts.warnings) + " warnings";
	} else {
		u.priority = 50;
		u.details  = "Watching controller inputs";
		u.state    = std::to_string(counts.devices) + " devices | " +
		             std::to_string(counts.compensation_paths) + " paths";
	}

	composer.Submit("Input Health", std::move(u));

	// Debug-mode count audit: one line when the count summary changes. The
	// fields zero_hash_slots and unsupported_slots are diagnostic flags --
	// non-zero on a steady-state session points at a driver-side publisher
	// bug, not a presence-side counting bug.
	if (openvr_pair::common::IsDebugLoggingEnabled()) {
		const auto &prev = last_presence_counts_;
		const bool changed =
			!last_presence_counts_logged_ ||
			prev.devices            != counts.devices ||
			prev.compensation_paths != counts.compensation_paths ||
			prev.diagnostic_paths   != counts.diagnostic_paths ||
			prev.warnings           != counts.warnings ||
			prev.zero_hash_slots    != counts.zero_hash_slots ||
			prev.unsupported_slots  != counts.unsupported_slots;
		if (changed) {
			LOG("[inputhealth-presence] devices=%d paths=%d diag=%d unsup=%d "
			    "warnings=%d zero_hash_slots=%d",
			    counts.devices, counts.compensation_paths,
			    counts.diagnostic_paths, counts.unsupported_slots,
			    counts.warnings, counts.zero_hash_slots);
			last_presence_counts_        = counts;
			last_presence_counts_logged_ = true;
		}
	}
}

namespace openvr_pair::overlay {

std::unique_ptr<FeaturePlugin> CreateInputHealthPlugin()
{
	return std::make_unique<InputHealthPlugin>();
}

} // namespace openvr_pair::overlay
