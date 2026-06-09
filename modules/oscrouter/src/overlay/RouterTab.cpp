#include "RouterTab.h"
#include "OscRouterUiLogic.h"
#include "ShellContext.h"
#include "UiHelpers.h"
#include "JsonUtil.h"
#include "Win32Paths.h"
#include "Win32Text.h"
#include "DiagnosticsLog.h"

#include <imgui.h>
#include <cstring>
#include <cstdio>
#include <chrono>
#include <exception>
#include <fstream>
#include <sstream>

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

// ShellContext is forward-declared in FeaturePlugin.h; include the header
// so the definition is visible for field access.
#include "ShellContext.h"

// Read send_port from %LocalAppDataLow%\WKOpenVR\profiles\oscrouter.json.
// Returns 9000 (the default) if the file is absent or the key is missing.
static int ReadProfileSendPort()
{
	std::wstring profileDir = openvr_pair::common::WkOpenVrSubdirectoryPath(L"profiles", false);
	if (profileDir.empty()) {
		openvr_pair::common::DiagnosticLog("oscrouter", "profile read default: profile dir unavailable");
		return 9000;
	}
	std::wstring path = profileDir + L"\\oscrouter.json";

	std::ifstream in(path);
	if (!in) {
		openvr_pair::common::DiagnosticLog("oscrouter", "profile read default: oscrouter.json missing path='%s'",
		                                   openvr_pair::common::WideToUtf8(path).c_str());
		return 9000;
	}

	std::stringstream ss;
	ss << in.rdbuf();

	picojson::value root;
	if (!openvr_pair::common::json::ParseObject(root, ss.str())) {
		openvr_pair::common::DiagnosticLog("oscrouter", "profile read default: parse failed path='%s'",
		                                   openvr_pair::common::WideToUtf8(path).c_str());
		return 9000;
	}

	int port = openvr_pair::common::json::IntAt(root, "send_port", 9000);
	if (port <= 0 || port > 65535) {
		openvr_pair::common::DiagnosticLog("oscrouter", "profile read default: invalid send_port=%d path='%s'", port,
		                                   openvr_pair::common::WideToUtf8(path).c_str());
		return 9000;
	}
	openvr_pair::common::DiagnosticLog("oscrouter", "profile read send_port=%d", port);
	return port;
}

// Atomic write of send_port to %LocalAppDataLow%\WKOpenVR\profiles\
// oscrouter.json. Merges with any existing keys so a richer router profile
// (future fields) survives. Writes to .tmp then MoveFileExW so a crash
// mid-write doesn't corrupt the existing file.
static bool WriteProfileSendPort(int port)
{
	std::wstring profileDir = openvr_pair::common::WkOpenVrSubdirectoryPath(L"profiles", true);
	if (profileDir.empty()) {
		openvr_pair::common::DiagnosticLog("oscrouter", "profile write failed: profile dir unavailable");
		return false;
	}
	std::wstring path = profileDir + L"\\oscrouter.json";

	picojson::object obj;
	{
		std::ifstream in(path);
		if (in) {
			std::stringstream ss;
			ss << in.rdbuf();
			picojson::value root;
			if (openvr_pair::common::json::ParseObject(root, ss.str())) {
				obj = root.get<picojson::object>();
			}
			else {
				openvr_pair::common::DiagnosticLog(
				    "oscrouter", "profile write continuing with new object: existing parse failed path='%s'",
				    openvr_pair::common::WideToUtf8(path).c_str());
			}
		}
	}
	obj["send_port"] = picojson::value(static_cast<double>(port));
	std::string body = picojson::value(obj).serialize(true);

	std::wstring tmpPath = path + L".tmp";
	HANDLE h = CreateFileW(tmpPath.c_str(), GENERIC_WRITE, 0, nullptr, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
	if (h == INVALID_HANDLE_VALUE) {
		openvr_pair::common::DiagnosticLog("oscrouter", "profile write failed: CreateFile err=%lu path='%s'",
		                                   GetLastError(), openvr_pair::common::WideToUtf8(tmpPath).c_str());
		return false;
	}
	DWORD written = 0;
	BOOL ok = WriteFile(h, body.data(), (DWORD)body.size(), &written, nullptr);
	CloseHandle(h);
	if (!ok || written != (DWORD)body.size()) {
		openvr_pair::common::DiagnosticLog(
		    "oscrouter", "profile write failed: WriteFile ok=%d written=%lu expected=%zu path='%s'", ok ? 1 : 0,
		    written, body.size(), openvr_pair::common::WideToUtf8(tmpPath).c_str());
		DeleteFileW(tmpPath.c_str());
		return false;
	}
	if (!MoveFileExW(tmpPath.c_str(), path.c_str(), MOVEFILE_REPLACE_EXISTING)) {
		openvr_pair::common::DiagnosticLog("oscrouter", "profile write failed: MoveFileEx err=%lu path='%s'",
		                                   GetLastError(), openvr_pair::common::WideToUtf8(path).c_str());
		DeleteFileW(tmpPath.c_str());
		return false;
	}
	openvr_pair::common::DiagnosticLog("oscrouter", "profile write send_port=%d", port);
	return true;
}

bool RouterTab::EnsureIpc(openvr_pair::overlay::ShellContext& ctx)
{
	if (ipc_.IsConnected()) return true;
	if (!oscrouter::ui::ShouldAttemptLiveDriverIpc(ctx.vrConnected)) {
		nextIpcConnectAttempt_ = {};
		return false;
	}

	const auto now = std::chrono::steady_clock::now();
	const bool retryDue = nextIpcConnectAttempt_.time_since_epoch().count() == 0 || now >= nextIpcConnectAttempt_;
	if (!oscrouter::ui::ShouldRetryLiveDriverIpc(ctx.vrConnected, ipc_.IsConnected(), retryDue)) {
		return false;
	}
	nextIpcConnectAttempt_ = now + std::chrono::seconds(1);

	try {
		ipc_.Connect();
		nextIpcConnectAttempt_ = {};
		return true;
	}
	catch (...) {
		portPushedToDriver_ = false;
		return false;
	}
}

void RouterTab::Tick(openvr_pair::overlay::ShellContext& ctx)
{
	if (!ctx.vrConnected) {
		statsReader_.Close();
		if (ipc_.IsConnected()) ipc_.Close();
		lastStats_ = {};
		driverWaitStarted_ = {};
		nextIpcConnectAttempt_ = {};
		portPushedToDriver_ = false;
		return;
	}

	const bool statsOpen = statsReader_.TryOpen();
	if (!statsOpen) {
		if (driverWaitStarted_.time_since_epoch().count() == 0) {
			driverWaitStarted_ = std::chrono::steady_clock::now();
		}
	}
	else {
		driverWaitStarted_ = {};
	}

	if (statsReader_.IsOpen()) {
		statsReader_.ReadGlobal(lastStats_);
	}

	EnsureIpc(ctx);
	if (ipc_.IsConnected() && !portPushedToDriver_) {
		if (!portLoaded_) {
			portEdit_ = ReadProfileSendPort();
			portLoaded_ = true;
		}
		portPushedToDriver_ = PushLivePortConfig(portEdit_);
	}
}

void RouterTab::Draw(openvr_pair::overlay::ShellContext& ctx)
{
	// Send-port editor. Hydrated once from the profile on the first draw;
	// user edits commit on focus-out (IsItemDeactivatedAfterEdit) so dragging
	// through intermediate values doesn't fire a write per increment.
	if (!portLoaded_) {
		portEdit_ = ReadProfileSendPort();
		portLoaded_ = true;
	}
	ImGui::AlignTextToFramePadding();
	ImGui::Text("Send target: 127.0.0.1:");
	ImGui::SameLine();
	ImGui::SetNextItemWidth(80.0f);
	ImGui::InputInt("##oscrouter_send_port", &portEdit_, 0, 0);
	if (ImGui::IsItemDeactivatedAfterEdit()) {
		// Clamp to a valid UDP range; 0 and >65535 are nonsense.
		if (portEdit_ < 1) portEdit_ = 1;
		if (portEdit_ > 65535) portEdit_ = 65535;
		SendPortChanged(ctx, portEdit_);
	}
	openvr_pair::overlay::ui::TooltipOnHover("Outbound OSC target port. Edits write to profiles\\oscrouter.json\n"
	                                         "and push to the live driver immediately when SteamVR is running.");

	ImGui::Spacing();
	DrawConnectedModules(ctx);

	const bool driverWaitElapsed = driverWaitStarted_.time_since_epoch().count() != 0 &&
	                               (std::chrono::steady_clock::now() - driverWaitStarted_) >= std::chrono::seconds(5);
	const auto panelState =
	    oscrouter::ui::ResolveDriverPanelState(ctx.vrConnected, statsReader_.IsOpen(), driverWaitElapsed);

	if (panelState == oscrouter::ui::DriverPanelState::WaitingForSteamVr) {
		openvr_pair::overlay::ui::DrawWaitingBanner(
		    "Waiting for SteamVR -- OSC Router connects when the driver is live.");
		return;
	}
	if (panelState == oscrouter::ui::DriverPanelState::WaitingForDriver) {
		openvr_pair::overlay::ui::DrawWaitingBanner("Waiting for OSC Router driver to finish loading.");
		return;
	}
	if (panelState == oscrouter::ui::DriverPanelState::Problem) {
		openvr_pair::overlay::ui::DrawErrorBanner("OSC Router not active",
		                                          "SteamVR is running, but the OSC Router driver state is unavailable. "
		                                          "Restart SteamVR or reinstall WKOpenVR.");
		return;
	}

	if (!statsReader_.IsOpen()) {
		openvr_pair::overlay::ui::DrawErrorBanner(
		    "OSC Router not active", "Enable OSC Router or a module that publishes OSC, then restart SteamVR.");
		return;
	}

	ImGui::Separator();

	ImGui::Text("Packets sent: %llu  Bytes: %llu  Dropped: %llu  Routes: %u",
	            (unsigned long long)lastStats_.packets_sent, (unsigned long long)lastStats_.bytes_sent,
	            (unsigned long long)lastStats_.packets_dropped, (unsigned)lastStats_.active_routes);

	ImGui::Separator();
	DrawRouteTable();
	ImGui::Separator();
	DrawTestPublish(ctx);
}

void RouterTab::DrawConnectedModules(openvr_pair::overlay::ShellContext& ctx)
{
	struct Entry
	{
		const char* flag;
		const char* label;
		const char* summary;
	};
	static const Entry kEntries[] = {
	    {"enable_facetracking.flag", "Face Tracking", "legacy and v2 avatar parameters"},
	    {"enable_captions.flag", "Captions", "chatbox text and transcripts"},
	};

	ImGui::Spacing();
	ImGui::Text("Connected modules:");
	{
		openvr_pair::overlay::ui::TableScope table("oscrouter_connected", 3,
		                                           ImGuiTableFlags_BordersOuter | ImGuiTableFlags_RowBg |
		                                               ImGuiTableFlags_SizingStretchProp);
		if (table) {
			openvr_pair::overlay::ui::SetupStretchColumn("Module", 3.0f);
			openvr_pair::overlay::ui::SetupStretchColumn("Status", 1.5f);
			openvr_pair::overlay::ui::SetupStretchColumn("Sends", 4.0f);
			openvr_pair::overlay::ui::DrawTableHeader();

			for (const auto& e : kEntries) {
				const bool enabled = ctx.IsFlagPresent(e.flag);
				openvr_pair::overlay::ui::NextRow();
				openvr_pair::overlay::ui::SetColumn(0);
				ImGui::TextUnformatted(e.label);
				openvr_pair::overlay::ui::SetColumn(1);
				openvr_pair::overlay::ui::DrawStatusText(enabled ? "enabled" : "disabled",
				                                         enabled ? openvr_pair::overlay::ui::StatusTone::Ok
				                                                 : openvr_pair::overlay::ui::StatusTone::Idle);
				openvr_pair::overlay::ui::SetColumn(2);
				ImGui::TextUnformatted(e.summary);
			}
		}
	}
	ImGui::TextDisabled("These features send OSC through the router, which merges them into\n"
	                    "one stable connection to VRChat.");
}

void RouterTab::DrawRouteTable()
{
	ImGui::Text("Active routes:");
	openvr_pair::overlay::ui::TableScope table(
	    "routes", 3, ImGuiTableFlags_BordersOuter | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp);
	if (table) {
		openvr_pair::overlay::ui::SetupStretchColumn("Pattern", 5.0f);
		openvr_pair::overlay::ui::SetupStretchColumn("Matched", 1.5f);
		openvr_pair::overlay::ui::SetupStretchColumn("Dropped", 1.5f);
		openvr_pair::overlay::ui::DrawTableHeader();

		for (uint32_t i = 0; i < OscRouterStatsReader::RouteSlotCount(); ++i) {
			protocol::OscRouterRouteSlot slot;
			if (!statsReader_.ReadRoute(i, slot)) continue;
			if (!slot.active) continue;

			openvr_pair::overlay::ui::NextRow();
			openvr_pair::overlay::ui::SetColumn(0);
			ImGui::TextUnformatted(slot.address_pattern);
			openvr_pair::overlay::ui::SetColumn(1);
			ImGui::Text("%llu", (unsigned long long)slot.match_count.load(std::memory_order_relaxed));
			openvr_pair::overlay::ui::SetColumn(2);
			ImGui::Text("%llu", (unsigned long long)slot.drop_count.load(std::memory_order_relaxed));
		}
	}
}

void RouterTab::DrawTestPublish(openvr_pair::overlay::ShellContext& ctx)
{
	ImGui::Text("Test publish:");
	ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x * 0.6f);
	ImGui::InputText("Address##testaddr", testAddress_, sizeof(testAddress_));
	openvr_pair::overlay::ui::TooltipOnHover("OSC address to send (e.g. /avatar/parameters/JawOpen)");
	ImGui::SameLine();
	ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x * 0.4f);
	ImGui::InputText("Value##testval", testValue_, sizeof(testValue_));
	openvr_pair::overlay::ui::TooltipOnHover("Float value to send as a ,f argument");
	ImGui::SameLine();
	if (ImGui::Button("Send##testsend")) {
		TrySendTestPublish(ctx);
	}
	if (testStatus_[0] != '\0') {
		ImGui::TextUnformatted(testStatus_);
	}
}

bool RouterTab::PushLivePortConfig(int newPort)
{
	if (!ipc_.IsConnected()) return false;
	protocol::Request req(protocol::RequestSetOscRouterConfig);
	req.setOscRouterConfig.send_port = (uint16_t)newPort;
	for (int i = 0; i < 6; ++i)
		req.setOscRouterConfig._reserved[i] = 0;
	try {
		protocol::Response resp = ipc_.SendBlocking(req);
		if (resp.type != protocol::ResponseSuccess) {
			openvr_pair::common::DiagnosticLog("oscrouter", "live port push rejected send_port=%d response=%u", newPort,
			                                   (unsigned)resp.type);
			return false;
		}
		openvr_pair::common::DiagnosticLog("oscrouter", "live port push ok send_port=%d", newPort);
		return true;
	}
	catch (const std::exception& e) {
		openvr_pair::common::DiagnosticLog("oscrouter", "live port push failed send_port=%d error='%s'", newPort,
		                                   e.what());
		ipc_.Close();
		nextIpcConnectAttempt_ = {};
		portPushedToDriver_ = false;
		return false;
	}
	catch (...) {
		openvr_pair::common::DiagnosticLog("oscrouter", "live port push failed send_port=%d error='unknown'", newPort);
		ipc_.Close();
		nextIpcConnectAttempt_ = {};
		portPushedToDriver_ = false;
		return false;
	}
}

void RouterTab::SendPortChanged(openvr_pair::overlay::ShellContext& ctx, int newPort)
{
	// Persist first so the value survives a crash before the driver round-
	// trips. WriteProfileSendPort is best-effort -- a write failure leaves
	// the existing oscrouter.json intact and the UI keeps the new value in
	// memory so the next edit will retry the write.
	const bool profileOk = WriteProfileSendPort(newPort);
	if (!profileOk) {
		openvr_pair::common::DiagnosticLog("oscrouter", "send port profile write failed for port=%d", newPort);
	}

	// Push live if the IPC pipe is open. When the driver isn't running yet,
	// the on-disk write above is sufficient -- the driver reads send_port
	// from oscrouter.json at init time.
	EnsureIpc(ctx);
	if (!ipc_.IsConnected()) {
		portPushedToDriver_ = false;
		openvr_pair::common::DiagnosticLog("oscrouter", "send port live push deferred port=%d vr_connected=%d", newPort,
		                                   ctx.vrConnected ? 1 : 0);
		return;
	}
	portPushedToDriver_ = PushLivePortConfig(newPort);
}

void RouterTab::TrySendTestPublish(openvr_pair::overlay::ShellContext& ctx)
{
	testStatus_[0] = '\0';
	EnsureIpc(ctx);

	if (!ipc_.IsConnected()) {
		snprintf(testStatus_, sizeof(testStatus_),
		         ctx.vrConnected ? "Not connected to driver. Is OSC Router enabled?" : "Waiting for SteamVR.");
		return;
	}

	float fval = 0.0f;
	bool parsed = (sscanf_s(testValue_, "%f", &fval) == 1);
	if (!parsed) {
		snprintf(testStatus_, sizeof(testStatus_), "Could not parse value as float.");
		return;
	}

	// Encode as big-endian float.
	uint32_t bits;
	memcpy(&bits, &fval, 4);
	uint8_t argBytes[4];
	argBytes[0] = (uint8_t)(bits >> 24);
	argBytes[1] = (uint8_t)(bits >> 16);
	argBytes[2] = (uint8_t)(bits >> 8);
	argBytes[3] = (uint8_t)(bits);

	protocol::Request req;
	req.type = protocol::RequestOscPublish;
	memset(&req.oscPublish, 0, sizeof(req.oscPublish));
	{
		size_t n = 0;
		const char* src = testAddress_;
		for (; n < sizeof(req.oscPublish.address) - 1 && src[n]; ++n)
			req.oscPublish.address[n] = src[n];
		req.oscPublish.address[n] = '\0';
	}
	req.oscPublish.typetag[0] = ',';
	req.oscPublish.typetag[1] = 'f';
	req.oscPublish.typetag[2] = '\0';
	req.oscPublish.arg_len = 4;
	memcpy(req.oscPublish.arg_bytes, argBytes, 4);

	try {
		protocol::Response resp = ipc_.SendBlocking(req);
		if (resp.type == protocol::ResponseSuccess) {
			snprintf(testStatus_, sizeof(testStatus_), "Sent %s = %.4f", testAddress_, fval);
		}
		else {
			snprintf(testStatus_, sizeof(testStatus_), "Driver rejected publish (queue full?)");
		}
	}
	catch (const std::exception& e) {
		snprintf(testStatus_, sizeof(testStatus_), "IPC error: %s", e.what());
		ipc_.Close();
		nextIpcConnectAttempt_ = {};
		portPushedToDriver_ = false;
	}
}
