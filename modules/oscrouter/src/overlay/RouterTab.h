#pragma once

#include "OscRouterStatsReader.h"
#include "RouterIpcClient.h"
#include "ShellContext.h"

#include <chrono>

// Renders the OSC Router Modules-tab subpage. Displays send-target config,
// active route list with per-route message counts, and a test-publish row.
// No DockSpace or docking-branch ImGui APIs (pinned at master 82d0584e7).

class RouterTab
{
public:
	RouterTab() = default;

	void Tick(openvr_pair::overlay::ShellContext& ctx);
	void Draw(openvr_pair::overlay::ShellContext& ctx);

	const protocol::OscRouterStats& LastStats() const { return lastStats_; }

private:
	OscRouterStatsReader statsReader_;
	RouterIpcClient ipc_;
	std::chrono::steady_clock::time_point nextIpcConnectAttempt_{};
	std::chrono::steady_clock::time_point driverWaitStarted_{};

	// Test publish state.
	char testAddress_[64] = "/avatar/parameters/Test";
	char testValue_[32] = "0.5";
	char testStatus_[256] = "";

	// Send-port edit widget state. portEdit_ is hydrated once from the
	// on-disk profile via ReadProfileSendPort() on the first Draw() call;
	// subsequent edits flow through SendPortChanged() to persist + push.
	int portEdit_ = 9000;
	bool portLoaded_ = false;
	bool portPushedToDriver_ = false;

	// Last known global stats for display.
	protocol::OscRouterStats lastStats_ = {};

	void DrawConnectedModules(openvr_pair::overlay::ShellContext& ctx);
	void DrawRouteTable();
	void DrawTestPublish(openvr_pair::overlay::ShellContext& ctx);
	void TrySendTestPublish(openvr_pair::overlay::ShellContext& ctx);
	bool EnsureIpc(openvr_pair::overlay::ShellContext& ctx);
	bool PushLivePortConfig(int newPort);
	void SendPortChanged(openvr_pair::overlay::ShellContext& ctx, int newPort);
};
