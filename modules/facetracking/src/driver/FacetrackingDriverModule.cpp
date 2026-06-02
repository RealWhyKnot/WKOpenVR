#define _CRT_SECURE_NO_DEPRECATE
#include "CalibrationEngine.h"
#include "EyelidSync.h"
#include "FaceFrameReader.h"
#include "FaceOscPublisher.h"
#include "FaceSignalProcessor.h"
#include "FaceTrackingDevice.h"
#include "HostSupervisor.h"
#include "Logging.h"
#include "VergenceLock.h"

#include "DriverModule.h"
#include "FeatureFlags.h"
#include "Protocol.h"
#include "ServerTrackedDeviceProvider.h"
#include "Win32Paths.h"

#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <openvr_driver.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

namespace facetracking {
namespace {

// -----------------------------------------------------------------------
// Telemetry sidecar helpers
// -----------------------------------------------------------------------

// %LocalAppDataLow%/WKOpenVR/facetracking/
static std::wstring ResolveTelemetryDir()
{
    return openvr_pair::common::WkOpenVrSubdirectoryPath(L"facetracking", true);
}

// Atomically write `content` to `final_path` via a .tmp rename.
static void AtomicWriteFile(const std::wstring &final_path, const std::string &content)
{
    std::wstring tmp_path = final_path + L".tmp";

    HANDLE h = CreateFileW(tmp_path.c_str(),
        GENERIC_WRITE, 0, nullptr,
        CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
    if (h == INVALID_HANDLE_VALUE) return;

    DWORD written = 0;
    WriteFile(h, content.data(), static_cast<DWORD>(content.size()), &written, nullptr);
    CloseHandle(h);

    if (written == static_cast<DWORD>(content.size())) {
        MoveFileExW(tmp_path.c_str(), final_path.c_str(), MOVEFILE_REPLACE_EXISTING);
    } else {
        DeleteFileW(tmp_path.c_str());
    }
}

// Build driver_telemetry.json from current state.
static std::string BuildTelemetryJson(
    DWORD                     pid,
    uint64_t                  frames_processed,
    uint64_t                  frames_read,
    uint64_t                  osc_messages_sent,
    uint64_t                  osc_messages_dropped,
    const std::string        &active_module_uuid,
    bool                      vergence_enabled,
    float                     focus_m,
    float                     ipd_m,
    const CalibrationEngine  &calib)
{
    // Timestamp.
    SYSTEMTIME st{};
    GetSystemTime(&st);
    char tsz[32];
    snprintf(tsz, sizeof(tsz),
        "%04d-%02d-%02dT%02d:%02d:%02dZ",
        st.wYear, st.wMonth, st.wDay,
        st.wHour, st.wMinute, st.wSecond);

    // Unix epoch seconds (rough: days-since-epoch * 86400 + intra-day).
    // We use the FILETIME -> unix conversion (FILETIME epoch = 1601-01-01).
    FILETIME ft{};
    SystemTimeToFileTime(&st, &ft);
    ULARGE_INTEGER u;
    u.LowPart  = ft.dwLowDateTime;
    u.HighPart = ft.dwHighDateTime;
    const int64_t unix_s = static_cast<int64_t>(u.QuadPart / 10000000ULL) - 11644473600LL;

    std::ostringstream o;
    o << "{\n";
    o << "  \"schema_version\": 1,\n";
    o << "  \"driver_pid\": " << pid << ",\n";
    o << "  \"wrote_at\": \"" << tsz << "\",\n";
    o << "  \"wrote_at_unix\": " << unix_s << ",\n";
    o << "  \"frames_processed\": " << frames_processed << ",\n";
    o << "  \"frames_read\": " << frames_read << ",\n";
    o << "  \"osc_messages_sent\": " << osc_messages_sent << ",\n";
    o << "  \"osc_messages_dropped\": " << osc_messages_dropped << ",\n";
    o << "  \"active_module_uuid\": \"" << active_module_uuid << "\",\n";
    o << "  \"vergence\": {\n";
    o << "    \"enabled\": " << (vergence_enabled ? "true" : "false") << ",\n";
    o << "    \"focus_distance_m\": " << focus_m << ",\n";
    o << "    \"ipd_m\": " << ipd_m << "\n";
    o << "  },\n";

    // shape_readiness: 63 expressions (indices 0-62) then 2 eye-openness
    // slots (indices 63-64 in CalibrationEngine's kIdxOpenL/kIdxOpenR).
    o << "  \"shape_readiness\": [";
    for (int i = 0; i < 65; ++i) {
        if (i > 0) o << ", ";
        o << (calib.IsShapeWarm(i) ? "true" : "false");
    }
    o << "]\n";
    o << "}\n";
    return o.str();
}

// -----------------------------------------------------------------------
// Host-exe path resolution
// -----------------------------------------------------------------------

// Resolve the path to the C# host relative to the driver resources directory.
// SteamVR exposes the driver path via IVRProperties on the driver context; we
// fall back to a search-order heuristic if the property isn't available yet.
std::string ResolveHostExePath(vr::IVRDriverContext *driverContext)
{
    // Attempt to get the install directory from SteamVR.
    char buf[MAX_PATH] = {};
    vr::ETrackedPropertyError err = vr::TrackedProp_Success;
    vr::CVRPropertyHelpers *props = vr::VRProperties();
    if (props) {
        vr::PropertyContainerHandle_t systemContainer =
            props->TrackedDeviceToPropertyContainer(vr::k_unTrackedDeviceIndex_Hmd);
        (void)systemContainer; // unused if the driver context isn't initialised yet
    }

    // Use the driver's own module path as the anchor.
    // GetModuleFileNameA on our DLL gives us the driver DLL path; strip to the
    // containing directory and navigate to resources/facetracking/host/.
    HMODULE hSelf = nullptr;
    GetModuleHandleExA(
        GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS |
        GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
        reinterpret_cast<LPCSTR>(&ResolveHostExePath),
        &hSelf);

    if (hSelf) {
        char dllPath[MAX_PATH] = {};
        GetModuleFileNameA(hSelf, dllPath, MAX_PATH);
        std::string path(dllPath);
        // Walk up to the driver root. DLL lives at
        //   <driver_root>/bin/win64/driver_wkopenvr.dll
        // so we pop the filename, then "win64", then "bin" -- three pops -- to
        // reach <driver_root>, and then append resources/facetracking/host/.
        // The original code popped only twice and landed at <driver_root>/bin,
        // producing a phantom <driver_root>/bin/resources/... path that does
        // not exist on disk; CreateProcessW returned err=3 (PATH_NOT_FOUND)
        // and the host never spawned.
        for (int up = 0; up < 3; ++up) {
            auto sep = path.find_last_of("/\\");
            if (sep == std::string::npos) break;
            path = path.substr(0, sep);
        }
        path += "\\resources\\facetracking\\host\\WKOpenVR.FaceModuleHost.exe";
        return path;
    }
    return {};
}

class FacetrackingDriverModule final : public DriverModule
{
public:
    const char *Name()        const override { return "FaceTracking"; }
    uint32_t    FeatureMask() const override { return pairdriver::kFeatureFaceTracking; }
    const char *PipeName()    const override { return OPENVR_PAIRDRIVER_FACETRACKING_PIPE_NAME; }

    bool Init(DriverModuleContext &context) override
    {
        FtDrvOpenLogFile();
        FT_LOG_DRV("[module] Init()", 0);

        provider_       = context.provider;
        driver_context_ = context.driverContext;

        // Resolve the telemetry output path once at init. CreateDirectory is
        // idempotent so this is safe to call even if the dir already exists.
        std::wstring tdir = ResolveTelemetryDir();
        if (!tdir.empty()) {
            CreateDirectoryW(tdir.c_str(), nullptr); // ignore ERROR_ALREADY_EXISTS
            telemetry_path_ = tdir + L"\\driver_telemetry.json";
            osc_filter_ = FaceOscAddressFilter(tdir + L"\\avatar_parameters.txt");
        }

        // Create the shmem ring through the reader so we map the segment
        // exactly once. The driver owns the lifecycle; the C# host opens
        // the existing segment via MemoryMappedFile.OpenExisting. The prior
        // form did shmem_.Create() AND reader_.Open() on the same name,
        // producing two independent mappings of the same backing store and
        // leaking a HANDLE pair on shutdown.
        if (!reader_.Create(OPENVR_PAIRDRIVER_FACETRACKING_SHMEM_NAME)) {
            FT_LOG_DRV("[module] failed to create shmem segment '%s'",
                OPENVR_PAIRDRIVER_FACETRACKING_SHMEM_NAME);
            return false;
        }
        FT_LOG_DRV("[module] shmem segment created", 0);

        // Native SteamVR tracker output is not registered by default. VRChat
        // face output uses OSC, and registering a generic tracker creates a
        // visible floor tracker with no useful role for the default path.
        FT_LOG_DRV("[module] native SteamVR sink tracker registration skipped; OSC output is the default path", 0);

        // Build host path and start supervisor.
        std::string host_path = ResolveHostExePath(driver_context_);
        FT_LOG_DRV("[module] host exe: %s", host_path.c_str());
        supervisor_ = std::make_unique<HostSupervisor>(host_path);
        // Pre-spawn sweep: if a prior SteamVR session left a wedged host
        // process (singleton mutex held, pipe unresponsive), terminate it
        // before Start() so connect-first attach does not lock onto a dead
        // peer.
        supervisor_->CleanupStaleHostIfWedged();
        if (!supervisor_->Start()) {
            FT_LOG_DRV("[module] host initial spawn failed; supervisor monitor will retry", 0);
        }

        // Start the frame worker thread.
        worker_stop_.store(false, std::memory_order_release);
        worker_ = std::thread([this]{ WorkerLoop(); });

        FT_LOG_DRV("[module] Init complete", 0);
        return true;
    }

    void Shutdown() override
    {
        FT_LOG_DRV("[module] Shutdown()", 0);

        worker_stop_.store(true, std::memory_order_release);
        if (worker_.joinable()) worker_.join();

        if (supervisor_) supervisor_->Stop();
        supervisor_.reset();

        calib_.Save();
        reader_.Close();
        device_.reset();

        provider_       = nullptr;
        driver_context_ = nullptr;

        FT_LOG_DRV("[module] shutdown complete", 0);
    }

    bool HandleRequest(const protocol::Request &req, protocol::Response &resp) override
    {
        switch (req.type) {
        case protocol::RequestSetFaceTrackingConfig: {
            std::lock_guard<std::mutex> lk(config_mutex_);
            config_ = req.setFaceTrackingConfig;
            // Forward active module selection to supervisor.
            if (supervisor_) {
                supervisor_->SetActiveModuleUuid(config_.active_module_uuid);
            }
            resp.type = protocol::ResponseSuccess;
            return true;
        }
        case protocol::RequestSetFaceCalibrationCommand: {
            const protocol::FaceCalibrationOp op =
                (protocol::FaceCalibrationOp)req.setFaceCalibrationCommand.op;
            std::lock_guard<std::mutex> lk(config_mutex_);
            calib_.Reset(op);
            resp.type = protocol::ResponseSuccess;
            return true;
        }
        case protocol::RequestSetFaceActiveModule: {
            if (supervisor_) {
                supervisor_->SetActiveModuleUuid(req.setFaceActiveModule.uuid);
            }
            resp.type = protocol::ResponseSuccess;
            return true;
        }
        case protocol::RequestFaceHostRestart: {
            FT_LOG_DRV("[module] host restart requested by overlay", 0);
            if (supervisor_) supervisor_->Restart();
            resp.type = protocol::ResponseSuccess;
            return true;
        }
        default:
            return false;
        }
    }

private:
    ServerTrackedDeviceProvider *provider_       = nullptr;
    vr::IVRDriverContext        *driver_context_ = nullptr;

    FaceFrameReader                  reader_;

    std::unique_ptr<FaceTrackingDevice> device_;
    std::unique_ptr<HostSupervisor>     supervisor_;

    CalibrationEngine calib_;
    VergenceLock      vergence_;
    EyelidSync        eyelid_;
    FaceSignalProcessor signal_processor_;

    // Config cache -- written by HandleRequest, read by WorkerLoop.
    protocol::FaceTrackingConfig config_{};
    mutable std::mutex           config_mutex_;

    std::atomic<bool> worker_stop_{ false };
    std::thread       worker_;

    // Telemetry sidecar state.
    std::wstring telemetry_path_;
    std::chrono::steady_clock::time_point last_telemetry_write_{};
    uint64_t frames_processed_ = 0;
    uint64_t frames_read_ = 0;
    uint64_t osc_messages_sent_ = 0;
    uint64_t osc_messages_dropped_ = 0;
    FaceOscAddressFilter osc_filter_;

    // Diagnostics state: OSC output transition tracking.
    bool     osc_was_enabled_     = false;
    bool     osc_first_publish_   = false;
    uint32_t all_zero_frames_     = 0;
    bool     all_zero_warned_     = false;
    bool     native_unavailable_warned_ = false;

    // -----------------------------------------------------------------------
    // Worker thread: polls shmem, runs the filter pipeline, publishes.
    // -----------------------------------------------------------------------
    void WorkerLoop()
    {
        FT_LOG_DRV("[worker] started", 0);

        uint64_t last_idx = 0;
        const DWORD self_pid = GetCurrentProcessId();

        // Wedge-detector: rising edge bookkeeping so the warning logs only
        // once per wedge episode and the restart fires only once per
        // detection. Cleared as soon as the heartbeat or state recovers.
        bool wedge_restart_pending = false;
        auto last_wedge_restart_time = std::chrono::steady_clock::time_point{};

        while (!worker_stop_.load(std::memory_order_acquire)) {
            // Heartbeat-based wedge detector: an active host that is alive
            // but no longer publishing frames is just as bad as a dead one.
            // The host's state byte plus heartbeat-age give the driver a way
            // to distinguish "idle, not pushing -- fine" from "wedged".
            //
            // Pre-heartbeat (legacy) hosts write zero for both fields; in
            // that case HostState() is HostStateLegacy and HeartbeatAgeMs()
            // is UINT64_MAX, so the check below is a no-op.
            const uint32_t host_state = reader_.HostState();
            const uint64_t hb_age_ms  = reader_.HeartbeatAgeMs();
            uint64_t       wedge_threshold_ms = UINT64_MAX;
            if (host_state == protocol::HostStatePublishing)
                wedge_threshold_ms = 2000;
            else if (host_state == protocol::HostStateIdle)
                wedge_threshold_ms = 5000;
            // HostStateDraining and HostStateLegacy intentionally never wedge-kill.

            const bool is_wedged = (host_state != protocol::HostStateLegacy) &&
                                   (hb_age_ms != UINT64_MAX) &&
                                   (hb_age_ms > wedge_threshold_ms);
            if (is_wedged) {
                // Rate-limit restarts to once per 10 s; otherwise a host
                // that takes a few seconds to come back up could chain
                // restarts and never settle.
                const auto now = std::chrono::steady_clock::now();
                const bool restart_allowed = !wedge_restart_pending ||
                    std::chrono::duration_cast<std::chrono::seconds>(
                        now - last_wedge_restart_time).count() >= 10;
                if (restart_allowed) {
                    FT_LOG_DRV("[worker] WEDGE: host_state=%u heartbeat_age=%llums "
                               "threshold=%llums -- restarting host",
                               host_state,
                               static_cast<unsigned long long>(hb_age_ms),
                               static_cast<unsigned long long>(wedge_threshold_ms));
                    if (supervisor_) supervisor_->Restart();
                    // Zero the heartbeat fields so the stale value left by
                    // the dead host does not re-trigger the detector
                    // before the new host writes its first tick.
                    reader_.ResetHostLiveness();
                    wedge_restart_pending     = true;
                    last_wedge_restart_time   = now;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                continue;
            } else if (wedge_restart_pending && hb_age_ms != UINT64_MAX) {
                // New host wrote a fresh heartbeat -- clear the rate-limit
                // so a future wedge gets handled promptly.
                FT_LOG_DRV("[worker] wedge cleared: heartbeat resumed (age=%llums state=%u)",
                           static_cast<unsigned long long>(hb_age_ms), host_state);
                wedge_restart_pending = false;
            }

            uint64_t idx = reader_.LastPublishIndex();
            if (idx == last_idx) {
                // No new frame; sleep briefly so we don't busy-spin.
                std::this_thread::sleep_for(std::chrono::milliseconds(2));

                // Still write telemetry even on idle ticks so the overlay
                // doesn't see a stale file during low-motion periods.
                MaybeWriteTelemetry(self_pid);
                continue;
            }
            last_idx = idx;

            protocol::FaceTrackingFrameBody frame{};
            if (!reader_.TryRead(frame)) continue;
            ++frames_read_;

            // Snapshot config under lock.
            protocol::FaceTrackingConfig cfg;
            {
                std::lock_guard<std::mutex> lk(config_mutex_);
                cfg = config_;
            }

            // Continuous calibration ingestion + normalization.
            if (cfg.continuous_calib_mode > 0) {
                calib_.IngestFrame(frame);
                calib_.Normalize(frame);
            }

            // Vergence lock.
            if (cfg.vergence_lock_enabled) {
                vergence_.Apply(frame, cfg.vergence_lock_strength);
            }

            // Eyelid sync.
            if (cfg.eyelid_sync_enabled) {
                eyelid_.Apply(frame,
                    cfg.eyelid_sync_strength,
                    cfg.eyelid_sync_preserve_winks != 0);
            }

            signal_processor_.Apply(frame, cfg);

            // Publish to SteamVR inputs.
            if (cfg._reserved_native && device_) {
                device_->PublishFrame(frame);
            } else if (cfg._reserved_native && !native_unavailable_warned_) {
                FT_LOG_DRV("[facetracking] native output requested but no SteamVR sink tracker is registered", 0);
                native_unavailable_warned_ = true;
            }

            // Forward face data to the OSC router. PublishOsc is non-blocking
            // and returns false silently when the router is not active, so
            // gating on output_osc_enabled is the only check we need here.
            const bool osc_enabled = (cfg.output_osc_enabled != 0);
            if (osc_enabled != osc_was_enabled_) {
                if (osc_enabled) {
                    FT_LOG_DRV("[facetracking] OSC output enabled", 0);
                    osc_first_publish_ = false;
                } else {
                    FT_LOG_DRV("[facetracking] OSC output disabled", 0);
                }
                osc_was_enabled_ = osc_enabled;
            }

            if (osc_enabled) {
                const bool eye_valid  = (frame.flags & 0x1u) != 0;
                const bool expr_valid = (frame.flags & 0x2u) != 0;

                // All-zero expression guard.
                bool has_nonzero = false;
                if (expr_valid) {
                    for (uint32_t i = 0; i < protocol::FACETRACKING_EXPRESSION_COUNT; ++i) {
                        if (frame.expressions[i] != 0.f) { has_nonzero = true; break; }
                    }
                }
                if (eye_valid) {
                    has_nonzero = has_nonzero ||
                        frame.eye_openness_l != 0.f || frame.eye_openness_r != 0.f;
                }

                if (!has_nonzero) {
                    if (++all_zero_frames_ >= 60 && !all_zero_warned_) {
                        FT_LOG_DRV("[facetracking] expression frame all-zero for 60+ frames -- module may not be delivering data", 0);
                        all_zero_warned_ = true;
                    }
                } else {
                    all_zero_frames_ = 0;
                    all_zero_warned_ = false;
                }

                if (osc_filter_.ReloadIfChanged()) {
                    FT_LOG_DRV("[facetracking] avatar OSC allowlist loaded: %u addresses",
                        (unsigned)osc_filter_.AllowedCount());
                }

                FaceOscPublishCounts counts = PublishFaceFrameOsc(frame, &osc_filter_);
                osc_messages_sent_ += counts.sent;
                osc_messages_dropped_ += counts.dropped;

                if (!osc_first_publish_ && counts.sent > 0) {
                    FT_LOG_DRV("[facetracking] first OSC publish: sent=%u JawOpen=%.3f LeftEyeLid=%.3f flags=0x%x",
                        (unsigned)counts.sent,
                        frame.expressions[26], // index 26 = JawOpen
                        frame.eye_openness_l,
                        (unsigned)frame.flags);
                    osc_first_publish_ = true;
                }
            }

            ++frames_processed_;
            MaybeWriteTelemetry(self_pid);
        }

        FT_LOG_DRV("[worker] stopped", 0);
    }

    void MaybeWriteTelemetry(DWORD pid)
    {
        if (telemetry_path_.empty()) return;

        const auto now = std::chrono::steady_clock::now();
        if (now - last_telemetry_write_ < std::chrono::milliseconds(500)) return;
        last_telemetry_write_ = now;

        protocol::FaceTrackingConfig cfg;
        {
            std::lock_guard<std::mutex> lk(config_mutex_);
            cfg = config_;
        }

        const bool verg_enabled = (cfg.vergence_lock_enabled != 0);
        const float focus_m = verg_enabled ? vergence_.LastFocusDistanceM() : 0.f;
        const float ipd_m   = verg_enabled ? vergence_.LastIpdM()           : 0.f;

        std::string json = BuildTelemetryJson(
            pid, frames_processed_, frames_read_,
            osc_messages_sent_, osc_messages_dropped_,
            cfg.active_module_uuid,
            verg_enabled, focus_m, ipd_m,
            calib_);

        AtomicWriteFile(telemetry_path_, json);
    }
};

} // namespace

std::unique_ptr<DriverModule> CreateDriverModule()
{
    return std::make_unique<FacetrackingDriverModule>();
}

} // namespace facetracking
