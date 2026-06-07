#pragma once

#include "DriverTelemetryPoller.h"
#include "FeaturePlugin.h"
#include "HostStatusPoller.h"
#include "IPCClient.h"
#include "ModuleSources.h"
#include "Profiles.h"
#include "Protocol.h"

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

class FacetrackingPlugin;

namespace facetracking::ui {
void DrawSettingsTab(FacetrackingPlugin& plugin);
void DrawModulesTab(FacetrackingPlugin& plugin);
void DrawAdvancedTab(FacetrackingPlugin& plugin);
void DrawLogsSection(FacetrackingPlugin& plugin);
} // namespace facetracking::ui

class FacetrackingPlugin final : public openvr_pair::overlay::FeaturePlugin
{
public:
	FacetrackingPlugin();

	const char* Name() const override { return ModuleName(ModuleId::FaceTracking); }
	const char* FlagFileName() const override { return ModuleFlagFileName(ModuleId::FaceTracking); }
	const char* PipeName() const override { return ModulePipeName(ModuleId::FaceTracking); }
	openvr_pair::overlay::FeaturePluginChannel Channel() const override
	{
		return openvr_pair::overlay::FeaturePluginChannel::Development;
	}

	void OnStart(openvr_pair::overlay::ShellContext& ctx) override;
	void OnShutdown(openvr_pair::overlay::ShellContext& ctx) override;
	void Tick(openvr_pair::overlay::ShellContext& ctx) override;
	void DrawTab(openvr_pair::overlay::ShellContext& ctx) override;
	void DrawLogsSection(openvr_pair::overlay::ShellContext& ctx) override;
	void OnDebugLoggingChanged(bool enabled) override;

	// Build and push the current profile settings to the driver over IPC.
	// Quiet on success; sets last_error_ and logs on failure.
	void PushConfigToDriver();

	// Persist the user's enabled-modules set and push the active selection
	// to the driver / host. Backend currently runs a single module at a
	// time, so the first UUID in `uuids` is the one the host loads; the
	// remaining entries are stored in the profile against the day the
	// host learns to run multiple. Pass an empty list to clear (host
	// auto-selects).
	void SendEnabledModules(const std::vector<std::string>& uuids);

	// Called by Tick() and by the tab functions to keep the pipe alive
	// across SteamVR restarts. Not normally called directly from UI code.
	void MaintainDriverConnection();

	// Accessors used by Modules-tab helper functions (static helpers in
	// ModulesTab.cpp cannot be friends; these expose only what is needed).
	facetracking::HostStatusPoller& HostStatus() { return host_status_; }
	FacetrackingProfileStore& Profile() { return profile_; }
	const FacetrackingProfileStore& Profile() const { return profile_; }
	facetracking::ModuleSyncRunner& SyncRunner() { return sync_runner_; }
	std::optional<facetracking::SyncResult> ConsumeSyncResult();
	void ReconcileEnabledModulesWithInstalled(const std::string& preferred_uuid);

private:
	friend void facetracking::ui::DrawSettingsTab(FacetrackingPlugin& plugin);
	friend void facetracking::ui::DrawModulesTab(FacetrackingPlugin& plugin);
	friend void facetracking::ui::DrawAdvancedTab(FacetrackingPlugin& plugin);
	friend void facetracking::ui::DrawLogsSection(FacetrackingPlugin& plugin);

	FtIPCClient ipc_;
	FacetrackingProfileStore profile_;
	facetracking::HostStatusPoller host_status_;
	facetracking::DriverTelemetryPoller driver_telemetry_;
	facetracking::ModuleSyncRunner sync_runner_;
	facetracking::SourcesCatalogue sources_catalogue_;
	std::vector<facetracking::SyncResult> completed_sync_results_;

	std::string last_error_;
	uint64_t observed_ipc_generation_ = 0;

	std::chrono::steady_clock::time_point last_connection_check_{};
	std::chrono::steady_clock::time_point last_save_{};

	void DrawStatusBanner();
	void HandleSyncResult(const facetracking::SyncResult& result);
};
