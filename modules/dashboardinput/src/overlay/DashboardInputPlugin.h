#pragma once

#include "FeaturePlugin.h"
#include "IPCClient.h"

#include <chrono>
#include <cstdint>
#include <string>

class DashboardInputPlugin final : public openvr_pair::overlay::FeaturePlugin
{
public:
	const char* Name() const override { return ModuleName(ModuleId::DashboardInput); }
	const char* FlagFileName() const override { return ModuleFlagFileName(ModuleId::DashboardInput); }
	const char* PipeName() const override { return ModulePipeName(ModuleId::DashboardInput); }

	void OnStart(openvr_pair::overlay::ShellContext& context) override;
	void OnShutdown(openvr_pair::overlay::ShellContext& context) override;
	void Tick(openvr_pair::overlay::ShellContext& context) override;
	void DrawTab(openvr_pair::overlay::ShellContext& context) override;

private:
	DashboardInputIPCClient ipc_;
	std::string connectError_;
	std::chrono::steady_clock::time_point nextConnectAttempt_{};
	std::chrono::steady_clock::time_point lastStateSend_{};
	bool stateDirty_ = true;
	bool haveLastStateSent_ = false;
	bool lastEnabled_ = false;
	bool lastVisible_ = false;
	uint8_t lastPrimaryHand_ = 0;

	bool ConnectIfNeeded();
	void SendDashboardState(openvr_pair::overlay::ShellContext& context, bool enabled, bool forceLog);
	void TickDashboardState(openvr_pair::overlay::ShellContext& context);
};
