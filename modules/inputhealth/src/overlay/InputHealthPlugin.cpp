#include "InputHealthPlugin.h"

#include "Config.h"
#include "DebugLogging.h"
#include "DebugTab.h"
#include "DiagnosticsTab.h"
#include "IPCClient.h"
#include "InputHealthHealthSummary.h"
#include "InputHealthUiLogic.h"
#include "LearningEngine.h"
#include "Logging.h"
#include "SettingsTab.h"
#include "ShellContext.h"
#include "ShellFooter.h"
#include "SnapshotReader.h"
#include "UiHelpers.h"

#include <imgui/imgui.h>

#include <chrono>
#include <exception>
#include <memory>
#include <string>

using Clock = std::chrono::steady_clock;

namespace {

bool StartsWith(const std::string& value, const char* prefix)
{
	return prefix && value.rfind(prefix, 0) == 0;
}

bool IsDriverWaitError(const std::string& error)
{
	return StartsWith(error, "InputHealth IPC:") || StartsWith(error, "Driver connection:") ||
	       StartsWith(error, "Not connected");
}

} // namespace

InputHealthPlugin::InputHealthPlugin() : engine_(ipc_, profiles_)
{
	const InputHealthGlobalConfig saved = LoadInputHealthConfig();
	pending_config_.master_enabled = saved.master_enabled;
	pending_config_.diagnostics_only = saved.diagnostics_only;
	pending_config_.enable_rest_recenter = saved.enable_rest_recenter;
	pending_config_.enable_trigger_remap = saved.enable_trigger_remap;
	observed_ipc_generation_ = ipc_.ConnectionGeneration();
}

void InputHealthPlugin::OnStart(openvr_pair::overlay::ShellContext&)
{
	OpenLogFile();
	LOG("WKOpenVR-InputHealth plugin starting");
	profiles_.LoadAll();
	try {
		ipc_.Connect();
		LOG("[ui] IPC connected");
		engine_.PushReadyCompensations();
		PushConfigToDriver();
	}
	catch (const std::exception& e) {
		LOG("[ui] initial IPC connect failed: %s", e.what());
		last_error_ = std::string("InputHealth IPC: ") + e.what();
	}
	reader_.TryOpen();
	const auto now = Clock::now();
	last_refresh_ = now;
	last_connection_check_ = now;
	last_health_summary_ = now;
	WriteHealthSummary();
}

void InputHealthPlugin::OnShutdown(openvr_pair::overlay::ShellContext&)
{
	engine_.Flush();
	WriteHealthSummary();
	reader_.Close();
	ipc_.Close();
	LOG("WKOpenVR-InputHealth plugin shutting down");
	LogFlush();
}

void InputHealthPlugin::Tick(openvr_pair::overlay::ShellContext&)
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
	if (now - last_health_summary_ >= std::chrono::seconds(30)) {
		WriteHealthSummary();
		last_health_summary_ = now;
	}
}

void InputHealthPlugin::WriteHealthSummary()
{
	InputHealthHealthSummarySnapshot snapshot;
	snapshot.overlay_started = true;
	snapshot.ipc_connected = ipc_.IsConnected();
	snapshot.shmem_opened = reader_.IsOpen();
	snapshot.publish_tick = reader_.LastPublishTick();
	snapshot.live_components = reader_.EntriesByHandle().size();
	snapshot.profiles_loaded = profiles_.All().size();
	snapshot.config.master_enabled = pending_config_.master_enabled != 0;
	snapshot.config.diagnostics_only = pending_config_.diagnostics_only != 0;
	snapshot.config.enable_rest_recenter = pending_config_.enable_rest_recenter != 0;
	snapshot.config.enable_trigger_remap = pending_config_.enable_trigger_remap != 0;
	snapshot.path_families = CountInputHealthPathFamilies(reader_.EntriesByHandle());
	snapshot.learning = engine_.Stats();
	snapshot.profile_io = profiles_.Stats();
	WriteInputHealthHealthSummary(snapshot);
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
		LOG("[ui] config pushed: master=%d diag_only=%d rest=%d trig=%d", (int)pending_config_.master_enabled,
		    (int)pending_config_.diagnostics_only, (int)pending_config_.enable_rest_recenter,
		    (int)pending_config_.enable_trigger_remap);
	}
	catch (const std::exception& e) {
		last_error_ = std::string("IPC error: ") + e.what();
		LOG("[ui] PushConfigToDriver failed: %s", e.what());
	}
}

void InputHealthPlugin::SaveGlobalConfig()
{
	InputHealthGlobalConfig cfg;
	cfg.master_enabled = pending_config_.master_enabled;
	cfg.diagnostics_only = pending_config_.diagnostics_only;
	cfg.enable_rest_recenter = pending_config_.enable_rest_recenter;
	cfg.enable_trigger_remap = pending_config_.enable_trigger_remap;
	SaveInputHealthConfig(cfg);
	LOG("[ui] global config saved: master=%d diag_only=%d rest=%d trig=%d", (int)cfg.master_enabled,
	    (int)cfg.diagnostics_only, (int)cfg.enable_rest_recenter, (int)cfg.enable_trigger_remap);
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
			LOG("[ui] heartbeat protocol mismatch: type=%d driverVersion=%u overlayVersion=%u", (int)resp.type,
			    resp.protocol.version, protocol::Version);
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
		if (last_error_.find("Driver connection:") == 0 || last_error_.find("InputHealth IPC") == 0 ||
		    last_error_.find("Not connected") == 0) {
			last_error_.clear();
		}
	}
	catch (const std::exception& e) {
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
		req.resetInputHealthStats.reset_passive = reset_passive ? 1 : 0;
		req.resetInputHealthStats.reset_active = reset_active ? 1 : 0;
		req.resetInputHealthStats.reset_curves = reset_curves ? 1 : 0;
		auto resp = ipc_.SendBlocking(req);
		if (resp.type != protocol::ResponseSuccess) {
			last_error_ = "Driver rejected ResetInputHealthStats";
			return;
		}
		last_error_.clear();
		LOG("[ui] reset request sent: serial=0x%016llx passive=%d active=%d curves=%d", (unsigned long long)serial_hash,
		    (int)reset_passive, (int)reset_active, (int)reset_curves);
	}
	catch (const std::exception& e) {
		last_error_ = std::string("IPC error: ") + e.what();
		LOG("[ui] SendReset failed: %s", e.what());
	}
}

void InputHealthPlugin::DrawStatusBanner(const openvr_pair::overlay::ShellContext& context)
{
	// Connection state lives in the shared footer; the live IPC / shmem /
	// publish_tick triple lives on the Logs tab. The top banner is reserved
	// for actual errors so a narrow window does not overflow with noise.
	if (inputhealth::ui::ShouldShowDriverProblemBanner(!last_error_.empty(), IsDriverWaitError(last_error_))) {
		openvr_pair::overlay::ui::DrawErrorBanner("InputHealth driver problem", last_error_.c_str());
	}
	// Shmem-not-open is normal during driver startup and can flicker briefly
	// on reconnects. Surface it as subtle disabled-grey text rather than a
	// hard red banner so it does not read as a tracking problem when it is
	// only a transient handshake state.
	if (inputhealth::ui::ShouldShowShmemProblemText(context.vrConnected, reader_.IsOpen(), !reader_.LastError().empty(),
	                                                reader_.LastErrorIsVersionMismatch())) {
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

void InputHealthPlugin::DrawTab(openvr_pair::overlay::ShellContext& context)
{
	DrawStatusBanner(context);

	openvr_pair::overlay::ui::TabBarScope tabs("inputhealth_tabs");
	if (tabs) {
		openvr_pair::overlay::ui::DrawTabItem("Diagnostics", [&] { DrawDiagnosticsTab(); });
		openvr_pair::overlay::ui::DrawTabItem("Settings", [&] { DrawSettingsTab(); });
		openvr_pair::overlay::ui::DrawTabItem("Advanced", [&] { DrawAdvancedTab(); });
		// Logs moved to the umbrella's global Logs tab; the InputHealth-specific
		// list of files + IPC state surface there via DrawLogsSection().
	}

	openvr_pair::overlay::ShellFooterStatus footer;
	footer.driverConnected = ipc_.IsConnected();
	footer.vrConnected = context.vrConnected;
	footer.driverLabel = "InputHealth driver";
	openvr_pair::overlay::DrawShellFooter(footer);
}

void InputHealthPlugin::DrawLogsSection(openvr_pair::overlay::ShellContext&)
{
	// Reuse the per-tab Logs implementation. The umbrella's global Logs tab
	// wraps each plugin in a collapsing header so the section reads with no
	// duplicate plugin-name heading.
	DrawLogsTab();
}

namespace openvr_pair::overlay {

std::unique_ptr<FeaturePlugin> CreateInputHealthPlugin()
{
	return std::make_unique<InputHealthPlugin>();
}

} // namespace openvr_pair::overlay
