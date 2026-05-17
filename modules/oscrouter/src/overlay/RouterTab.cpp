#include "RouterTab.h"
#include "ShellContext.h"
#include "UiHelpers.h"
#include "JsonUtil.h"
#include "Win32Paths.h"
#include "Win32Text.h"

#include <imgui.h>
#include <cstring>
#include <cstdio>
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
    if (profileDir.empty()) return 9000;
    std::wstring path = profileDir + L"\\oscrouter.json";

    std::ifstream in(path);
    if (!in) return 9000;

    std::stringstream ss;
    ss << in.rdbuf();

    picojson::value root;
    if (!openvr_pair::common::json::ParseObject(root, ss.str())) return 9000;

    int port = openvr_pair::common::json::IntAt(root, "send_port", 9000);
    return (port > 0 && port <= 65535) ? port : 9000;
}

// Atomic write of send_port to %LocalAppDataLow%\WKOpenVR\profiles\
// oscrouter.json. Merges with any existing keys so a richer router profile
// (future fields) survives. Writes to .tmp then MoveFileExW so a crash
// mid-write doesn't corrupt the existing file.
static bool WriteProfileSendPort(int port)
{
    std::wstring profileDir = openvr_pair::common::WkOpenVrSubdirectoryPath(L"profiles", true);
    if (profileDir.empty()) return false;
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
        }
    }
    obj["send_port"] = picojson::value(static_cast<double>(port));
    std::string body = picojson::value(obj).serialize(true);

    std::wstring tmpPath = path + L".tmp";
    HANDLE h = CreateFileW(tmpPath.c_str(), GENERIC_WRITE, 0, nullptr,
        CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
    if (h == INVALID_HANDLE_VALUE) return false;
    DWORD written = 0;
    BOOL ok = WriteFile(h, body.data(), (DWORD)body.size(), &written, nullptr);
    CloseHandle(h);
    if (!ok || written != (DWORD)body.size()) {
        DeleteFileW(tmpPath.c_str());
        return false;
    }
    return MoveFileExW(tmpPath.c_str(), path.c_str(), MOVEFILE_REPLACE_EXISTING) != 0;
}

void RouterTab::EnsureIpc()
{
    if (!ipc_.IsConnected() && !ipcConnectAttempted_) {
        ipcConnectAttempted_ = true;
        try { ipc_.Connect(); } catch (...) {}
    }
}

void RouterTab::Tick(openvr_pair::overlay::ShellContext &)
{
    statsReader_.TryOpen();
    if (statsReader_.IsOpen()) {
        statsReader_.ReadGlobal(lastStats_);
    }
}

void RouterTab::Draw(openvr_pair::overlay::ShellContext &ctx)
{
    if (!statsReader_.IsOpen()) {
        openvr_pair::overlay::ui::DrawErrorBanner(
            "OSC Router not active",
            "The OSC Router feature is not enabled. "
            "Add enable_oscrouter.flag to the driver's resources folder "
            "and restart SteamVR.");
        return;
    }

    // Send-port editor. Hydrated once from the profile on the first draw;
    // user edits commit on focus-out (IsItemDeactivatedAfterEdit) so dragging
    // through intermediate values doesn't fire a write per increment.
    if (!portLoaded_) {
        portEdit_   = ReadProfileSendPort();
        portLoaded_ = true;
    }
    ImGui::AlignTextToFramePadding();
    ImGui::Text("Send target: 127.0.0.1:");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(80.0f);
    ImGui::InputInt("##oscrouter_send_port", &portEdit_, 0, 0);
    if (ImGui::IsItemDeactivatedAfterEdit()) {
        // Clamp to a valid UDP range; 0 and >65535 are nonsense.
        if (portEdit_ < 1)     portEdit_ = 1;
        if (portEdit_ > 65535) portEdit_ = 65535;
        SendPortChanged(portEdit_);
    }
    openvr_pair::overlay::ui::TooltipOnHover(
        "Outbound OSC target port. Edits write to profiles\\oscrouter.json\n"
        "and push to the live driver immediately when SteamVR is running.");

    DrawConnectedModules(ctx);

    ImGui::Separator();

    ImGui::Text("Packets sent: %llu  Bytes: %llu  Dropped: %llu  Routes: %u",
        (unsigned long long)lastStats_.packets_sent,
        (unsigned long long)lastStats_.bytes_sent,
        (unsigned long long)lastStats_.packets_dropped,
        (unsigned)lastStats_.active_routes);

    ImGui::Separator();
    DrawRouteTable();
    ImGui::Separator();
    DrawTestPublish();
}

void RouterTab::DrawConnectedModules(openvr_pair::overlay::ShellContext &ctx)
{
    struct Entry {
        const char *flag;
        const char *label;
        const char *summary;
    };
    static const Entry kEntries[] = {
        { "enable_facetracking.flag", "Face Tracking",
          "legacy and v2 avatar parameters" },
        { "enable_captions.flag",     "Captions",
          "chatbox text and transcripts" },
    };

    ImGui::Spacing();
    ImGui::Text("Connected modules:");
    if (ImGui::BeginTable("oscrouter_connected", 3,
            ImGuiTableFlags_BordersOuter | ImGuiTableFlags_RowBg |
            ImGuiTableFlags_SizingStretchProp)) {
        ImGui::TableSetupColumn("Module",  ImGuiTableColumnFlags_WidthStretch, 3.0f);
        ImGui::TableSetupColumn("Status",  ImGuiTableColumnFlags_WidthStretch, 1.5f);
        ImGui::TableSetupColumn("Sends",   ImGuiTableColumnFlags_WidthStretch, 4.0f);
        ImGui::TableHeadersRow();

        const auto &palette = openvr_pair::overlay::ui::GetPalette();
        for (const auto &e : kEntries) {
            const bool enabled = ctx.IsFlagPresent(e.flag);
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::TextUnformatted(e.label);
            ImGui::TableSetColumnIndex(1);
            if (enabled) {
                ImGui::TextColored(palette.statusOk, "enabled");
            } else {
                ImGui::TextColored(palette.statusIdle, "disabled");
            }
            ImGui::TableSetColumnIndex(2);
            ImGui::TextUnformatted(e.summary);
        }
        ImGui::EndTable();
    }
    ImGui::TextDisabled(
        "These features send OSC through the router, which merges them into\n"
        "one stable connection to VRChat.");
}

void RouterTab::DrawRouteTable()
{
    ImGui::Text("Active routes:");
    if (ImGui::BeginTable("routes", 3,
            ImGuiTableFlags_BordersOuter | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp)) {
        ImGui::TableSetupColumn("Pattern",    ImGuiTableColumnFlags_WidthStretch, 5.0f);
        ImGui::TableSetupColumn("Matched",    ImGuiTableColumnFlags_WidthStretch, 1.5f);
        ImGui::TableSetupColumn("Dropped",    ImGuiTableColumnFlags_WidthStretch, 1.5f);
        ImGui::TableHeadersRow();

        for (uint32_t i = 0; i < OscRouterStatsReader::RouteSlotCount(); ++i) {
            protocol::OscRouterRouteSlot slot;
            if (!statsReader_.ReadRoute(i, slot)) continue;
            if (!slot.active) continue;

            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::TextUnformatted(slot.address_pattern);
            ImGui::TableSetColumnIndex(1);
            ImGui::Text("%llu", (unsigned long long)slot.match_count.load(std::memory_order_relaxed));
            ImGui::TableSetColumnIndex(2);
            ImGui::Text("%llu", (unsigned long long)slot.drop_count.load(std::memory_order_relaxed));
        }
        ImGui::EndTable();
    }
}

void RouterTab::DrawTestPublish()
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
        TrySendTestPublish();
    }
    if (testStatus_[0] != '\0') {
        ImGui::TextUnformatted(testStatus_);
    }
}

void RouterTab::SendPortChanged(int newPort)
{
    // Persist first so the value survives a crash before the driver round-
    // trips. WriteProfileSendPort is best-effort -- a write failure leaves
    // the existing oscrouter.json intact and the UI keeps the new value in
    // memory so the next edit will retry the write.
    WriteProfileSendPort(newPort);

    // Push live if the IPC pipe is open. When the driver isn't running yet,
    // the on-disk write above is sufficient -- the driver reads send_port
    // from oscrouter.json at init time.
    EnsureIpc();
    if (!ipc_.IsConnected()) return;

    protocol::Request req(protocol::RequestSetOscRouterConfig);
    req.setOscRouterConfig.send_port = (uint16_t)newPort;
    for (int i = 0; i < 6; ++i) req.setOscRouterConfig._reserved[i] = 0;
    try {
        ipc_.SendBlocking(req);
    } catch (...) {
        ipc_.Close();
        ipcConnectAttempted_ = false;
    }
}

void RouterTab::TrySendTestPublish()
{
    testStatus_[0] = '\0';
    EnsureIpc();

    if (!ipc_.IsConnected()) {
        snprintf(testStatus_, sizeof(testStatus_),
            "Not connected to driver. Is OSC Router enabled?");
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
        const char *src = testAddress_;
        for (; n < sizeof(req.oscPublish.address) - 1 && src[n]; ++n)
            req.oscPublish.address[n] = src[n];
        req.oscPublish.address[n] = '\0';
    }
    req.oscPublish.typetag[0] = ','; req.oscPublish.typetag[1] = 'f'; req.oscPublish.typetag[2] = '\0';
    req.oscPublish.arg_len = 4;
    memcpy(req.oscPublish.arg_bytes, argBytes, 4);

    try {
        protocol::Response resp = ipc_.SendBlocking(req);
        if (resp.type == protocol::ResponseSuccess) {
            snprintf(testStatus_, sizeof(testStatus_), "Sent %s = %.4f", testAddress_, fval);
        } else {
            snprintf(testStatus_, sizeof(testStatus_), "Driver rejected publish (queue full?)");
        }
    } catch (const std::exception &e) {
        snprintf(testStatus_, sizeof(testStatus_), "IPC error: %s", e.what());
        ipc_.Close();
        ipcConnectAttempted_ = false;
    }
}
