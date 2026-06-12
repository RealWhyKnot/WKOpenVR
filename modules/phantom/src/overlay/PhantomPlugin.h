#pragma once

#include "Config.h"
#include "FeaturePlugin.h"
#include "IPCClient.h"
#include "PhantomStateShmem.h"
#include "RoleCatalog.h"

#include <chrono>
#include <string>
#include <unordered_map>

class PhantomPlugin final : public openvr_pair::overlay::FeaturePlugin
{
public:
	const char* Name() const override { return ModuleName(ModuleId::Phantom); }
	const char* FlagFileName() const override { return ModuleFlagFileName(ModuleId::Phantom); }
	const char* PipeName() const override { return ModulePipeName(ModuleId::Phantom); }
	openvr_pair::overlay::FeaturePluginChannel Channel() const override
	{
		return openvr_pair::overlay::FeaturePluginChannel::Development;
	}

	void OnStart(openvr_pair::overlay::ShellContext& context) override;
	void Tick(openvr_pair::overlay::ShellContext& context) override;
	void DrawTab(openvr_pair::overlay::ShellContext& context) override;

private:
	PhantomConfig cfg_ = LoadPhantomConfig();
	PhantomIPCClient ipc_;
	std::string connectError_;
	std::chrono::steady_clock::time_point nextConnectAttempt_{};
	uint32_t connectFailureCount_ = 0;

	// Read-side mapping of the per-device state shmem the driver publishes.
	// Opened on first Tick where the driver is up; closed lazily on shutdown.
	phantom::PhantomStateShmem stateShmem_;
	bool stateShmemReady_ = false;

	// Whether we have ever managed to send a config. Drives "reconnect"
	// replay logic to keep the driver consistent after IPC drops.
	bool seededDriver_ = false;

	// Most recent T-pose capture summary; populated by DrawCalibrationTab
	// and rendered as a one-line "captured N roles, M skipped" badge until
	// the user starts another capture.
	std::string lastCalibrationSummary_;
	std::string lastSolverCalibrationSummary_;

	void DrawDropoutsTab();
	void DrawDiagnosticsTab();
	void DrawAdvancedTab();
	void DrawCalibrationTab();
	void DrawAutoDetectPanel();
	void DrawAbsentTab();

	bool ConnectIfNeeded();
	void SendConfig();
	void SendSolverConfig();
	void SendDeviceOptIn(const std::string& serial, bool enabled);
	void SendDeviceRole(const std::string& serial, phantom::BodyRole role);
	void SendTrackerOffset(phantom::BodyRole role, const PhantomRoleOffset& offset);
	void SendVirtualEnabled(phantom::BodyRole role, bool enabled);
	void ReplayDriverState();
	void ReplayCalibration();
	void CaptureNeutralStanding();
	void CaptureTPose();
};
