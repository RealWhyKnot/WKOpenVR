#include "FacetrackingPlugin.h"

#include "AdvancedTab.h"
#include "CalibrationTab.h"
#include "DebugLogging.h"
#include "IPCClient.h"
#include "Logging.h"
#include "LogsSection.h"
#include "ModuleSources.h"
#include "ModulesTab.h"
#include "Profiles.h"
#include "SettingsTab.h"
#include "ShellContext.h"
#include "ShellFooter.h"
#include "UiHelpers.h"
#include "BuildStamp.h"

#include <imgui/imgui.h>

#include <chrono>
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <exception>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

using Clock = std::chrono::steady_clock;

namespace {

bool StartsWith(const std::string &value, const char *prefix)
{
    return prefix && value.rfind(prefix, 0) == 0;
}

bool IsDriverWaitError(const std::string &error)
{
    return StartsWith(error, "FaceTracking IPC:")
        || StartsWith(error, "Driver connection:")
        || StartsWith(error, "Not connected");
}

} // namespace

FacetrackingPlugin::FacetrackingPlugin()
{
    observed_ipc_generation_ = ipc_.ConnectionGeneration();
}

void FacetrackingPlugin::OnStart(openvr_pair::overlay::ShellContext &)
{
    FtOverlayVerbose.store(openvr_pair::common::IsDebugLoggingEnabled(), std::memory_order_relaxed);

    FtOpenLogFile();
    FT_LOG_OVL("FaceTracking overlay plugin starting (build %s channel=%s)",
        FACETRACKING_BUILD_STAMP, FACETRACKING_BUILD_CHANNEL);

    profile_.Load();

    // Seed sources.json on first run; load the catalogue for Tick() to use.
    sources_catalogue_ = facetracking::EnsureSourcesCatalogue();

    try {
        ipc_.Connect();
        FT_LOG_OVL("[ipc] connected on startup");
        PushConfigToDriver();
    } catch (const std::exception &e) {
        FT_LOG_OVL("[ipc] initial connect failed: %s", e.what());
        last_error_ = std::string("FaceTracking IPC: ") + e.what();
    }

    const auto now = Clock::now();
    last_connection_check_ = now;
    last_save_             = now;
}

void FacetrackingPlugin::OnShutdown(openvr_pair::overlay::ShellContext &)
{
    profile_.Save();
    ipc_.Close();
    FT_LOG_OVL("FaceTracking overlay plugin shutting down");
    FtLogFlush();
}

void FacetrackingPlugin::Tick(openvr_pair::overlay::ShellContext &)
{
    const auto now = Clock::now();

    if (now - last_connection_check_ >= std::chrono::seconds(1)) {
        MaintainDriverConnection();
        last_connection_check_ = now;
    }

    // Pull the latest host_status.json snapshot. The poller throttles itself
    // to a stat() every 500 ms and only re-reads on mtime change, so calling
    // this every frame is cheap.
    host_status_.Tick();

    // Pull the latest driver_telemetry.json snapshot (same cadence).
    driver_telemetry_.Tick();

    // Reap completed sync helpers here so process handles close even if the
    // Modules tab is not visible.
    if (auto res = sync_runner_.Poll()) {
        HandleSyncResult(*res);
    }

    // Periodic auto-save (every 60 s).
    if (now - last_save_ >= std::chrono::seconds(60)) {
        profile_.Save();
        last_save_ = now;
    }
}

std::optional<facetracking::SyncResult> FacetrackingPlugin::ConsumeSyncResult()
{
    if (completed_sync_results_.empty()) return std::nullopt;
    facetracking::SyncResult result = completed_sync_results_.front();
    completed_sync_results_.erase(completed_sync_results_.begin());
    return result;
}

void FacetrackingPlugin::HandleSyncResult(const facetracking::SyncResult &result)
{
    if (result.ok && !result.installed_uuid.empty()) {
        FT_LOG_OVL("[modules] sync completed: uuid='%s' ver='%s'",
            result.installed_uuid.c_str(), result.installed_version.c_str());
    }

    sources_catalogue_ = facetracking::EnsureSourcesCatalogue();
    if (!result.source_id.empty()) {
        bool changed = false;
        for (auto &src : sources_catalogue_.sources) {
            if (src.id != result.source_id) continue;
            src.last_checked_at = facetracking::NowIso8601();
            src.last_sync_error = result.ok ? std::string{} : result.message;
            changed = true;
            break;
        }
        if (changed) facetracking::SaveSourcesCatalogue(sources_catalogue_);
    }

    if (result.ok && !result.installed_uuid.empty()) {
        ReconcileEnabledModulesWithInstalled(result.installed_uuid);
    }

    completed_sync_results_.push_back(result);
    if (completed_sync_results_.size() > 8)
        completed_sync_results_.erase(completed_sync_results_.begin());
}

void FacetrackingPlugin::PushConfigToDriver()
{
    if (!ipc_.IsConnected()) {
        last_error_ = "Not connected to the FaceTracking driver. Is SteamVR running?";
        return;
    }
    try {
        protocol::Request req(protocol::RequestSetFaceTrackingConfig);
        auto &cfg = req.setFaceTrackingConfig;
        const auto &p = profile_.current;

        cfg.master_enabled             = 1;
        cfg.eyelid_sync_enabled        = p.eyelid_sync_enabled        ? 1 : 0;
        cfg.eyelid_sync_preserve_winks = p.eyelid_sync_preserve_winks ? 1 : 0;
        cfg.vergence_lock_enabled      = p.vergence_lock_enabled       ? 1 : 0;
        cfg.continuous_calib_mode      = static_cast<uint8_t>(p.continuous_calib_mode);
        cfg.output_osc_enabled         = p.output_osc_enabled          ? 1 : 0;
        cfg._reserved_native           = 0;
        cfg._reserved1                 = 0;
        cfg.eyelid_sync_strength       = static_cast<uint8_t>(p.eyelid_sync_strength);
        cfg.vergence_lock_strength     = static_cast<uint8_t>(p.vergence_lock_strength);
        cfg.gaze_smoothing             = static_cast<uint8_t>(p.gaze_smoothing);
        cfg.openness_smoothing         = static_cast<uint8_t>(p.openness_smoothing);
        // osc_port / osc_host are deprecated; the router owns the UDP socket.
        // Leave them zeroed. The driver ignores them once output_osc_enabled
        // routes through the in-process PublishOsc path.
        cfg.osc_port   = 0;
        cfg._reserved2 = 0;
        cfg.osc_host[0] = '\0';

        // Active module = first enabled entry (backend is single-active for
        // now; once the host learns to run multiple, this entry should grow
        // to carry the full list via a protocol bump). Empty list = empty
        // string = host picks automatically.
        const std::string &primary = p.enabled_module_uuids.empty()
            ? std::string{}
            : p.enabled_module_uuids.front();
        std::snprintf(cfg.active_module_uuid, sizeof(cfg.active_module_uuid), "%s", primary.c_str());

        auto resp = ipc_.SendBlocking(req);
        if (resp.type != protocol::ResponseSuccess) {
            last_error_ = "Driver rejected SetFaceTrackingConfig (type=" +
                          std::to_string(resp.type) + ")";
            FT_LOG_OVL("[ipc] driver rejected config push: type=%d", (int)resp.type);
            return;
        }
        last_error_.clear();
        FT_LOG_OVL("[ipc] config pushed: osc_enabled=%d eyelid=%d vergence=%d calib_mode=%d",
            (int)cfg.output_osc_enabled, (int)cfg.eyelid_sync_enabled,
            (int)cfg.vergence_lock_enabled, (int)cfg.continuous_calib_mode);
    } catch (const std::exception &e) {
        last_error_ = std::string("IPC error: ") + e.what();
        FT_LOG_OVL("[ipc] PushConfigToDriver failed: %s", e.what());
    }
}

void FacetrackingPlugin::SendCalibrationCommand(protocol::FaceCalibrationOp op)
{
    if (!ipc_.IsConnected()) {
        last_error_ = "Not connected to the FaceTracking driver.";
        return;
    }
    try {
        protocol::Request req(protocol::RequestSetFaceCalibrationCommand);
        req.setFaceCalibrationCommand.op = static_cast<uint8_t>(op);
        std::memset(req.setFaceCalibrationCommand._reserved, 0,
            sizeof(req.setFaceCalibrationCommand._reserved));
        auto resp = ipc_.SendBlocking(req);
        if (resp.type != protocol::ResponseSuccess) {
            last_error_ = "Driver rejected calibration command (op=" +
                          std::to_string((int)op) + " type=" +
                          std::to_string(resp.type) + ")";
            FT_LOG_OVL("[ipc] driver rejected calib command op=%d: type=%d",
                (int)op, (int)resp.type);
            return;
        }
        last_error_.clear();
        FT_LOG_OVL("[ipc] calibration command sent: op=%d", (int)op);
    } catch (const std::exception &e) {
        last_error_ = std::string("IPC error: ") + e.what();
        FT_LOG_OVL("[ipc] SendCalibrationCommand(op=%d) failed: %s", (int)op, e.what());
    }
}

void FacetrackingPlugin::SendEnabledModules(const std::vector<std::string> &uuids)
{
    // Persist the full list so the order survives across sessions. Backend
    // currently consumes only the first entry; remaining entries are kept
    // for the future multi-run host upgrade.
    profile_.current.enabled_module_uuids = uuids;
    profile_.Save();

    const std::string &primary = uuids.empty() ? std::string{} : uuids.front();

    if (!ipc_.IsConnected()) {
        // Config will be pushed on the next successful heartbeat reconnect.
        return;
    }
    try {
        protocol::Request req(protocol::RequestSetFaceActiveModule);
        std::snprintf(req.setFaceActiveModule.uuid, sizeof(req.setFaceActiveModule.uuid), "%s", primary.c_str());
        std::memset(req.setFaceActiveModule._reserved, 0,
            sizeof(req.setFaceActiveModule._reserved));
        auto resp = ipc_.SendBlocking(req);
        if (resp.type != protocol::ResponseSuccess) {
            last_error_ = "Driver rejected SetFaceActiveModule (type=" +
                          std::to_string(resp.type) + ")";
            FT_LOG_OVL("[ipc] driver rejected active-module set: type=%d", (int)resp.type);
            return;
        }
        last_error_.clear();
        FT_LOG_OVL("[ipc] enabled modules set: count=%zu primary='%s'",
            uuids.size(), primary.c_str());
    } catch (const std::exception &e) {
        last_error_ = std::string("IPC error: ") + e.what();
        FT_LOG_OVL("[ipc] SendEnabledModules failed: %s", e.what());
    }
}

void FacetrackingPlugin::ReconcileEnabledModulesWithInstalled(const std::string &preferred_uuid)
{
    const std::vector<facetracking::InstalledModule> installed =
        facetracking::ScanInstalledModules();
    std::unordered_set<std::string> installedUuids;
    installedUuids.reserve(installed.size());
    for (const auto &m : installed) {
        if (!m.uuid.empty()) installedUuids.insert(m.uuid);
    }

    const auto current = profile_.current.enabled_module_uuids;
    std::vector<std::string> next;
    next.reserve(current.size() + 1);

    bool changed = false;
    for (const auto &uuid : current) {
        if (installedUuids.find(uuid) == installedUuids.end()) {
            changed = true;
            FT_LOG_OVL("[modules] pruning enabled module missing from disk: uuid='%s'",
                uuid.c_str());
            continue;
        }
        if (std::find(next.begin(), next.end(), uuid) == next.end())
            next.push_back(uuid);
        else
            changed = true;
    }

    bool forceResend = false;
    if (!preferred_uuid.empty() &&
        installedUuids.find(preferred_uuid) != installedUuids.end()) {
        auto it = std::find(next.begin(), next.end(), preferred_uuid);
        if (it == next.end()) {
            next.insert(next.begin(), preferred_uuid);
            changed = true;
            FT_LOG_OVL("[modules] auto-enabled newly installed module: uuid='%s'",
                preferred_uuid.c_str());
        } else {
            forceResend = (it == next.begin());
        }
    }

    if (changed || forceResend) {
        SendEnabledModules(next);
    }
}

void FacetrackingPlugin::MaintainDriverConnection()
{
    // Track consecutive heartbeat failures and connect attempts so chronic
    // disconnects produce a periodic `[ipc][retry-status]` summary rather
    // than per-tick failure spam, AND so a successful reconnection emits a
    // matching `[ipc][retry-status] recovered` line indicating how long the
    // connection was down. Cleared on successful handshake.
    static int s_consecutiveFailures = 0;
    static std::chrono::steady_clock::time_point s_lastRetryStatusLog{};
    static std::chrono::steady_clock::time_point s_firstFailureTp{};

    try {
        if (!ipc_.IsConnected()) {
            ipc_.Connect();
            FT_LOG_OVL("[ipc] connected from heartbeat");
        }

        auto resp = ipc_.SendBlocking(protocol::Request(protocol::RequestHandshake));
        if (resp.type != protocol::ResponseHandshake ||
            resp.protocol.version != protocol::Version) {
            last_error_ = "FaceTracking driver protocol mismatch during heartbeat";
            FT_LOG_OVL("[ipc] heartbeat mismatch: type=%d driverVer=%u overlayVer=%u",
                (int)resp.type, resp.protocol.version, protocol::Version);
            return;
        }

        const uint64_t gen = ipc_.ConnectionGeneration();
        if (gen != observed_ipc_generation_) {
            observed_ipc_generation_ = gen;
            FT_LOG_OVL("[ipc] generation changed to %llu; re-sending config",
                (unsigned long long)gen);
            PushConfigToDriver();
        }

        // Clear stale connection-error banners.
        if (last_error_.find("FaceTracking IPC") == 0 ||
            last_error_.find("Not connected")    == 0 ||
            last_error_.find("Driver connection:") == 0) {
            last_error_.clear();
        }
        // Recovery edge: emit a recovered annotation if we just came off a
        // failure streak. Captures the down-duration so a triage reader can
        // see how long the driver-side was unreachable.
        if (s_consecutiveFailures > 0) {
            const auto downSec = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - s_firstFailureTp).count();
            FT_LOG_OVL("[ipc][retry-status] recovered after %d failed attempts, down ~%.1fs",
                s_consecutiveFailures, downSec);
            s_consecutiveFailures = 0;
        }
    } catch (const std::exception &e) {
        last_error_ = std::string("Driver connection: ") + e.what();
        if (s_consecutiveFailures == 0) {
            s_firstFailureTp = std::chrono::steady_clock::now();
        }
        ++s_consecutiveFailures;
        // First failure logs verbatim; subsequent failures throttle to a
        // single line per 10 s containing attempt count + elapsed downtime.
        const auto nowTp = std::chrono::steady_clock::now();
        if (s_consecutiveFailures == 1) {
            FT_LOG_OVL("[ipc] heartbeat failed: %s", e.what());
            s_lastRetryStatusLog = nowTp;
        } else if (nowTp - s_lastRetryStatusLog >= std::chrono::seconds(10)) {
            const auto downSec = std::chrono::duration<double>(
                nowTp - s_firstFailureTp).count();
            FT_LOG_OVL("[ipc][retry-status] %d consecutive failures (down ~%.1fs): %s",
                s_consecutiveFailures, downSec, e.what());
            s_lastRetryStatusLog = nowTp;
        }
        ipc_.Close();
    }
}

void FacetrackingPlugin::DrawStatusBanner()
{
    if (!last_error_.empty() && !IsDriverWaitError(last_error_)) {
        openvr_pair::overlay::ui::DrawErrorBanner(
            "Face Tracking driver problem", last_error_.c_str());
    }
}

void FacetrackingPlugin::DrawTab(openvr_pair::overlay::ShellContext &ctx)
{
    DrawStatusBanner();

    if (ImGui::BeginTabBar("ft_tabs")) {
        if (ImGui::BeginTabItem("Settings"))    {
            facetracking::ui::DrawSettingsTab(*this);
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Calibration")) {
            facetracking::ui::DrawCalibrationTab(*this);
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Modules"))     {
            facetracking::ui::DrawModulesTab(*this);
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Advanced"))    {
            facetracking::ui::DrawAdvancedTab(*this);
            ImGui::EndTabItem();
        }
        // Logs appear in the umbrella's global Logs tab via DrawLogsSection.
        ImGui::EndTabBar();
    }

    openvr_pair::overlay::ShellFooterStatus footer;
    footer.driverConnected = ipc_.IsConnected();
    footer.vrConnected     = ctx.vrConnected;
    footer.driverLabel     = "FaceTracking driver";
    footer.buildStamp      = FACETRACKING_BUILD_STAMP;
    openvr_pair::overlay::DrawShellFooter(footer);
}

void FacetrackingPlugin::DrawLogsSection(openvr_pair::overlay::ShellContext &)
{
    facetracking::ui::DrawLogsSection(*this);
}

void FacetrackingPlugin::OnDebugLoggingChanged(bool enabled)
{
    FtOverlayVerbose.store(enabled, std::memory_order_relaxed);
}

namespace openvr_pair::overlay {

std::unique_ptr<FeaturePlugin> CreateFaceTrackingPlugin()
{
    return std::make_unique<FacetrackingPlugin>();
}

} // namespace openvr_pair::overlay
