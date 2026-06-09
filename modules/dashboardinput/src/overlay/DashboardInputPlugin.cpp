#include "DashboardInputPlugin.h"

#include "DiagnosticsLog.h"
#include "Protocol.h"
#include "ShellContext.h"
#include "ShellFooter.h"
#include "UiCore.h"

#include <imgui.h>

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

#include <algorithm>
#include <exception>
#include <memory>
#include <string>

namespace {

uint8_t NormalizeDashboardHand(int hand)
{
	if (hand == protocol::DashboardHandTrackingHandLeft || hand == protocol::DashboardHandTrackingHandRight) {
		return static_cast<uint8_t>(hand);
	}
	return protocol::DashboardHandTrackingHandUnknown;
}

const char* DashboardHandLabel(uint8_t hand)
{
	switch (hand) {
		case protocol::DashboardHandTrackingHandLeft:
			return "left";
		case protocol::DashboardHandTrackingHandRight:
			return "right";
		default:
			return "unknown";
	}
}

uint64_t MonotonicMilliseconds()
{
	return static_cast<uint64_t>(::GetTickCount64());
}

} // namespace

void DashboardInputPlugin::OnStart(openvr_pair::overlay::ShellContext& context)
{
	openvr_pair::common::DiagnosticLog("dashboardinput", "plugin_start protocol=%u", (unsigned)protocol::Version);
	ConnectIfNeeded();
	SendDashboardState(context, true, true);
}

void DashboardInputPlugin::OnShutdown(openvr_pair::overlay::ShellContext& context)
{
	SendDashboardState(context, false, true);
	ipc_.Close();
	openvr_pair::common::DiagnosticLog("dashboardinput", "plugin_shutdown");
}

void DashboardInputPlugin::Tick(openvr_pair::overlay::ShellContext& context)
{
	if (!ipc_.IsConnected() && ConnectIfNeeded()) {
		stateDirty_ = true;
	}
	TickDashboardState(context);
}

bool DashboardInputPlugin::ConnectIfNeeded()
{
	if (ipc_.IsConnected()) return false;
	const auto now = std::chrono::steady_clock::now();
	if (nextConnectAttempt_.time_since_epoch().count() != 0 && now < nextConnectAttempt_) {
		return false;
	}
	nextConnectAttempt_ = now + std::chrono::seconds(1);

	try {
		ipc_.Connect();
		if (!connectError_.empty()) {
			openvr_pair::common::DiagnosticLog("dashboardinput", "ipc_reconnect_ok previous='%s'",
			                                   connectError_.c_str());
		}
		connectError_.clear();
		nextConnectAttempt_ = {};
		return true;
	}
	catch (const std::exception& e) {
		if (connectError_ != e.what()) {
			openvr_pair::common::DiagnosticLog("dashboardinput", "ipc_connect_failed error='%s'", e.what());
		}
		connectError_ = e.what();
	}
	return false;
}

void DashboardInputPlugin::SendDashboardState(openvr_pair::overlay::ShellContext& context, bool enabled, bool forceLog)
{
	if (!ipc_.IsConnected()) {
		stateDirty_ = true;
		return;
	}

	const bool visible = context.anyDashboardVisible;
	const uint8_t primaryHand = NormalizeDashboardHand(context.primaryDashboardHand);

	protocol::Request req(protocol::RequestSetDashboardHandTrackingState);
	req.setDashboardHandTrackingState.enabled = enabled ? 1 : 0;
	req.setDashboardHandTrackingState.dashboard_visible = visible ? 1 : 0;
	req.setDashboardHandTrackingState.primary_hand = primaryHand;
	req.setDashboardHandTrackingState._reserved = 0;
	req.setDashboardHandTrackingState.update_mono_ms = MonotonicMilliseconds();

	try {
		ipc_.SendBlocking(req);
		connectError_.clear();
		stateDirty_ = false;
		haveLastStateSent_ = true;
		lastEnabled_ = enabled;
		lastVisible_ = visible;
		lastPrimaryHand_ = primaryHand;
		lastStateSend_ = std::chrono::steady_clock::now();
		if (forceLog) {
			openvr_pair::common::DiagnosticLog(
			    "dashboardinput",
			    "state_push enabled=%d any_dashboard_visible=%d active_dashboard_overlay=%d primary_device=%u "
			    "primary_hand=%s",
			    enabled ? 1 : 0, visible ? 1 : 0, context.activeDashboardOverlay ? 1 : 0,
			    context.primaryDashboardDevice, DashboardHandLabel(primaryHand));
		}
	}
	catch (const std::exception& e) {
		connectError_ = e.what();
		stateDirty_ = true;
	}
}

void DashboardInputPlugin::TickDashboardState(openvr_pair::overlay::ShellContext& context)
{
	const bool enabled = true;
	const bool visible = context.anyDashboardVisible;
	const uint8_t primaryHand = NormalizeDashboardHand(context.primaryDashboardHand);
	const bool changed =
	    !haveLastStateSent_ || enabled != lastEnabled_ || visible != lastVisible_ || primaryHand != lastPrimaryHand_;

	const auto now = std::chrono::steady_clock::now();
	const bool needsRefresh =
	    enabled && visible &&
	    (lastStateSend_.time_since_epoch().count() == 0 || now - lastStateSend_ >= std::chrono::milliseconds(250));
	if (stateDirty_ || changed || needsRefresh) {
		SendDashboardState(context, enabled, stateDirty_ || changed);
	}
}

void DashboardInputPlugin::DrawTab(openvr_pair::overlay::ShellContext& context)
{
	openvr_pair::overlay::ui::DrawSectionHeading("SteamVR dashboard");
	openvr_pair::overlay::ui::DrawSettingTable(
	    "dashboard_input_status", 170.0f, [&](openvr_pair::overlay::ui::SettingTableScope& table) {
		    openvr_pair::overlay::ui::SettingRow(table, "Finger passthrough", [&] {
			    openvr_pair::overlay::ui::DrawStatusCell("Enabled", openvr_pair::overlay::ui::StatusTone::Ok, false);
		    });
		    openvr_pair::overlay::ui::SettingRow(table, "Dashboard", [&] {
			    openvr_pair::overlay::ui::DrawStatusCell(context.anyDashboardVisible ? "Visible" : "Hidden",
			                                             context.anyDashboardVisible
			                                                 ? openvr_pair::overlay::ui::StatusTone::Ok
			                                                 : openvr_pair::overlay::ui::StatusTone::Idle,
			                                             false);
		    });
		    openvr_pair::overlay::ui::SettingRow(table, "Active tab", [&] {
			    openvr_pair::overlay::ui::DrawStatusCell(context.activeDashboardOverlay ? "WKOpenVR" : "Other",
			                                             context.activeDashboardOverlay
			                                                 ? openvr_pair::overlay::ui::StatusTone::Info
			                                                 : openvr_pair::overlay::ui::StatusTone::Idle,
			                                             false);
		    });
		    openvr_pair::overlay::ui::SettingRow(table, "Primary hand", [&] {
			    ImGui::TextUnformatted(DashboardHandLabel(NormalizeDashboardHand(context.primaryDashboardHand)));
		    });
	    });

	openvr_pair::overlay::ShellFooterStatus footer;
	footer.driverConnected = ipc_.IsConnected();
	footer.vrConnected = context.vrConnected;
	footer.driverLabel = "Dashboard Input driver";
	openvr_pair::overlay::DrawShellFooter(footer);
}

namespace openvr_pair::overlay {

std::unique_ptr<FeaturePlugin> CreateDashboardInputPlugin()
{
	return std::make_unique<DashboardInputPlugin>();
}

} // namespace openvr_pair::overlay
