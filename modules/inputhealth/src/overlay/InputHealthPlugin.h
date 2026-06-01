#pragma once

#include "FeaturePlugin.h"
#include "IPCClient.h"
#include "LearningEngine.h"
#include "Profiles.h"
#include "Protocol.h"
#include "SnapshotReader.h"

#include <chrono>
#include <cstdint>
#include <string>

class InputHealthPlugin;

namespace inputhealth::ui {
void DrawDiagnosticsTab(InputHealthPlugin &ui);
void DrawSettingsTab(InputHealthPlugin &ui);
void DrawAdvancedTab(InputHealthPlugin &ui);
void DrawLogsTab(InputHealthPlugin &ui);
} // namespace inputhealth::ui

class InputHealthPlugin final : public openvr_pair::overlay::FeaturePlugin
{
public:
	InputHealthPlugin();

	const char *Name() const override { return "Input Health"; }
	const char *FlagFileName() const override { return "enable_inputhealth.flag"; }
	const char *PipeName() const override { return OPENVR_PAIRDRIVER_INPUTHEALTH_PIPE_NAME; }

	void OnStart(openvr_pair::overlay::ShellContext &context) override;
	void OnShutdown(openvr_pair::overlay::ShellContext &context) override;
	void Tick(openvr_pair::overlay::ShellContext &context) override;
	void DrawTab(openvr_pair::overlay::ShellContext &context) override;
	void DrawLogsSection(openvr_pair::overlay::ShellContext &context) override;

	// Push the in-memory config to the driver via IPC. Called after the
	// user toggles a switch in the Diagnostics or Settings tab. Quiet on
	// success; logs and surfaces a banner on failure.
	void PushConfigToDriver();

	// Persist the current global toggle state to disk so it survives restarts.
	void SaveGlobalConfig();

	// Send a per-device RequestResetInputHealthStats (or all-devices if
	// serial_hash == 0). Logs the outcome; surfaces a banner on failure.
	void SendReset(uint64_t serial_hash, bool reset_passive, bool reset_active, bool reset_curves);

private:
	friend void inputhealth::ui::DrawDiagnosticsTab(InputHealthPlugin &ui);
	friend void inputhealth::ui::DrawSettingsTab(InputHealthPlugin &ui);
	friend void inputhealth::ui::DrawAdvancedTab(InputHealthPlugin &ui);
	friend void inputhealth::ui::DrawLogsTab(InputHealthPlugin &ui);

	IPCClient      ipc_;
	SnapshotReader reader_;
	ProfileStore   profiles_;
	LearningEngine  engine_;

	// Pending config the user is editing. Pushed to the driver when the
	// user clicks Apply (or when a toggle they touched is committed).
	protocol::InputHealthConfig pending_config_{};

	// Most recent error string from a failed IPC call. Drawn at the top of
	// the window so the user does not have to dig through the log to see
	// that the driver has gone away.
	std::string last_error_;
	uint64_t observed_ipc_generation_ = 0;
	std::chrono::steady_clock::time_point last_refresh_{};
	std::chrono::steady_clock::time_point last_connection_check_{};

	void MaintainDriverConnection();
	void DrawStatusBanner(const openvr_pair::overlay::ShellContext &context);
	void DrawDiagnosticsTab();
	void DrawSettingsTab();
	void DrawAdvancedTab();
	void DrawLogsTab();
};
