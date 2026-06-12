#define _CRT_SECURE_NO_DEPRECATE
#include "EyelidSync.h"
#include "FaceFrameReader.h"
#include "FaceOscPublisher.h"
#include "FaceSignalProcessor.h"
#include "FaceTrackingDevice.h"
#include "HostSupervisor.h"
#include "Logging.h"
#include "VergenceLock.h"

#include "DriverModule.h"
#include "DebugLogging.h"
#include "FeatureFlags.h"
#include "ModulePerf.h"
#include "ModuleRegistry.h"
#include "Protocol.h"
#include "ServerTrackedDeviceProvider.h"
#include "Win32Paths.h"

#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <openvr_driver.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace facetracking {
namespace {

using FaceShapeTuningArray = std::array<protocol::FaceShapeTuningParams, protocol::FACETRACKING_EXPRESSION_COUNT>;
using FaceExpressionArray = std::array<float, protocol::FACETRACKING_EXPRESSION_COUNT>;

// -----------------------------------------------------------------------
// Telemetry sidecar helpers
// -----------------------------------------------------------------------

// %LocalAppDataLow%/WKOpenVR/facetracking/
static std::wstring ResolveTelemetryDir()
{
	return openvr_pair::common::WkOpenVrSubdirectoryPath(L"facetracking", true);
}

// Atomically write `content` to `final_path` via a .tmp rename.
static bool AtomicWriteFile(const std::wstring& final_path, const std::string& content)
{
	std::wstring tmp_path = final_path + L".tmp";

	HANDLE h = CreateFileW(tmp_path.c_str(), GENERIC_WRITE, 0, nullptr, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
	if (h == INVALID_HANDLE_VALUE) return false;

	DWORD written = 0;
	const BOOL write_ok = WriteFile(h, content.data(), static_cast<DWORD>(content.size()), &written, nullptr);
	CloseHandle(h);

	if (!write_ok || written != static_cast<DWORD>(content.size())) {
		DeleteFileW(tmp_path.c_str());
		return false;
	}
	if (!MoveFileExW(tmp_path.c_str(), final_path.c_str(), MOVEFILE_REPLACE_EXISTING)) {
		DeleteFileW(tmp_path.c_str());
		return false;
	}
	return true;
}

static protocol::FaceShapeTuningParams DefaultShapeTuning()
{
	return protocol::FaceShapeTuningParams{protocol::FACETRACKING_SHAPE_TUNING_DEFAULT_PERCENT,
	                                       protocol::FACETRACKING_SHAPE_TUNING_DEFAULT_MIN_PERCENT,
	                                       protocol::FACETRACKING_SHAPE_TUNING_DEFAULT_MAX_PERCENT};
}

static protocol::FaceShapeTuningParams NormalizeShapeTuning(const protocol::FaceShapeTuning& tuning)
{
	protocol::FaceShapeTuningParams params{};
	params.scale_percent = std::min<uint16_t>(tuning.scale_percent, protocol::FACETRACKING_SHAPE_TUNING_MAX_PERCENT);

	if (tuning.min_percent == 0 && tuning.max_percent == 0) {
		params.min_percent = protocol::FACETRACKING_SHAPE_TUNING_DEFAULT_MIN_PERCENT;
		params.max_percent = protocol::FACETRACKING_SHAPE_TUNING_DEFAULT_MAX_PERCENT;
	}
	else {
		params.min_percent = std::min<uint16_t>(tuning.min_percent, protocol::FACETRACKING_SHAPE_TUNING_MAX_PERCENT);
		params.max_percent = std::min<uint16_t>(tuning.max_percent, protocol::FACETRACKING_SHAPE_TUNING_MAX_PERCENT);
		if (params.min_percent > params.max_percent) std::swap(params.min_percent, params.max_percent);
	}

	return params;
}

static void AppendExpressionArrayJson(std::ostringstream& o, const FaceExpressionArray& values)
{
	o << "[";
	for (size_t i = 0; i < values.size(); ++i) {
		if (i != 0) o << ",";
		o << values[i];
	}
	o << "]";
}

// Build driver_telemetry.json from current state.
static std::string BuildTelemetryJson(DWORD pid, uint64_t frames_processed, uint64_t frames_read,
                                      uint64_t osc_messages_sent, uint64_t osc_messages_dropped,
                                      const std::string& active_module_uuid, bool vergence_enabled, float focus_m,
                                      float ipd_m, bool shape_values_valid, uint64_t shape_values_frame,
                                      const FaceExpressionArray& pre_tuning_expressions,
                                      const FaceExpressionArray& post_tuning_expressions)
{
	// Timestamp.
	SYSTEMTIME st{};
	GetSystemTime(&st);
	char tsz[32];
	snprintf(tsz, sizeof(tsz), "%04d-%02d-%02dT%02d:%02d:%02dZ", st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute,
	         st.wSecond);

	// Unix epoch seconds (rough: days-since-epoch * 86400 + intra-day).
	// We use the FILETIME -> unix conversion (FILETIME epoch = 1601-01-01).
	FILETIME ft{};
	SystemTimeToFileTime(&st, &ft);
	ULARGE_INTEGER u;
	u.LowPart = ft.dwLowDateTime;
	u.HighPart = ft.dwHighDateTime;
	const int64_t unix_s = static_cast<int64_t>(u.QuadPart / 10000000ULL) - 11644473600LL;

	std::ostringstream o;
	o << "{\n";
	o << "  \"schema_version\": 2,\n";
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
	o << "  \"shape_values\": {\n";
	o << "    \"valid\": " << (shape_values_valid ? "true" : "false") << ",\n";
	o << "    \"frame\": " << shape_values_frame << ",\n";
	o << "    \"pre_tuning\": ";
	AppendExpressionArrayJson(o, pre_tuning_expressions);
	o << ",\n";
	o << "    \"post_tuning\": ";
	AppendExpressionArrayJson(o, post_tuning_expressions);
	o << "\n";
	o << "  }\n";
	o << "}\n";
	return o.str();
}

static const char* FaceCalibrationOpName(protocol::FaceCalibrationOp op)
{
	switch (op) {
		case protocol::FaceCalibBegin:
			return "begin";
		case protocol::FaceCalibEnd:
			return "end";
		case protocol::FaceCalibSave:
			return "save";
		case protocol::FaceCalibResetAll:
			return "reset-all";
		case protocol::FaceCalibResetEye:
			return "reset-eye";
		case protocol::FaceCalibResetExpr:
			return "reset-expr";
		default:
			return "unknown";
	}
}

static std::string FixedString(const char* value, size_t capacity)
{
	if (!value || capacity == 0) return {};
	size_t n = 0;
	while (n < capacity && value[n] != '\0')
		++n;
	return std::string(value, n);
}

// -----------------------------------------------------------------------
// Host-exe path resolution
// -----------------------------------------------------------------------

// Resolve the path to the C# host relative to the driver resources directory.
// SteamVR exposes the driver path via IVRProperties on the driver context; we
// fall back to a search-order heuristic if the property isn't available yet.
std::string ResolveHostExePath(vr::IVRDriverContext* driverContext)
{
	// Attempt to get the install directory from SteamVR.
	char buf[MAX_PATH] = {};
	vr::ETrackedPropertyError err = vr::TrackedProp_Success;
	vr::CVRPropertyHelpers* props = vr::VRProperties();
	if (props) {
		vr::PropertyContainerHandle_t systemContainer =
		    props->TrackedDeviceToPropertyContainer(vr::k_unTrackedDeviceIndex_Hmd);
		(void)systemContainer; // unused if the driver context isn't initialised yet
	}

	// Use the driver's own module path as the anchor.
	// GetModuleFileNameA on our DLL gives us the driver DLL path; strip to the
	// containing directory and navigate to resources/facetracking/host/.
	HMODULE hSelf = nullptr;
	GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
	                   reinterpret_cast<LPCSTR>(&ResolveHostExePath), &hSelf);

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
	const char* Name() const override { return "FaceTracking"; }
	uint32_t FeatureMask() const override { return pairdriver::kFeatureFaceTracking; }
	const char* PipeName() const override
	{
		return openvr_pair::common::modules::PipeName(openvr_pair::common::modules::ModuleId::FaceTracking);
	}

	bool Init(DriverModuleContext& context) override
	{
		FtDrvOpenLogFile();
		FT_LOG_DRV("[module] Init()", 0);

		provider_ = context.provider;
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
			FT_LOG_DRV("[module] failed to create shmem segment '%s'", OPENVR_PAIRDRIVER_FACETRACKING_SHMEM_NAME);
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
		worker_ = std::thread([this] { WorkerLoop(); });

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

		reader_.Close();
		device_.reset();

		provider_ = nullptr;
		driver_context_ = nullptr;

		FT_LOG_DRV("[module] shutdown complete", 0);
	}

	bool HandleRequest(const protocol::Request& req, protocol::Response& resp) override
	{
		switch (req.type) {
			case protocol::RequestSetFaceTrackingConfig: {
				std::lock_guard<std::mutex> lk(config_mutex_);
				const protocol::FaceTrackingConfig old = config_;
				config_ = req.setFaceTrackingConfig;
				const std::string old_uuid =
				    FixedString(old.active_module_uuid, protocol::FACETRACKING_MODULE_UUID_LEN);
				const std::string new_uuid =
				    FixedString(config_.active_module_uuid, protocol::FACETRACKING_MODULE_UUID_LEN);
				FT_LOG_DRV("[module] config update: master=%u->%u osc=%u->%u native=%u->%u "
				           "module='%s'->'%s' calib=%u->%u eyelid=%u/%u/mode=%u->%u/%u/mode=%u "
				           "vergence=%u/%u->%u/%u smooth(gaze=%u->%u open=%u->%u) "
				           "corr=0x%02x->0x%02x corr_strengths=0x%04x->0x%04x osc_port=%u->%u",
				           (unsigned)old.master_enabled, (unsigned)config_.master_enabled,
				           (unsigned)old.output_osc_enabled, (unsigned)config_.output_osc_enabled,
				           (unsigned)old._reserved_native, (unsigned)config_._reserved_native, old_uuid.c_str(),
				           new_uuid.c_str(), (unsigned)old.continuous_calib_mode,
				           (unsigned)config_.continuous_calib_mode, (unsigned)old.eyelid_sync_enabled,
				           (unsigned)old.eyelid_sync_strength, (unsigned)old.eyelid_sync_mode,
				           (unsigned)config_.eyelid_sync_enabled, (unsigned)config_.eyelid_sync_strength,
				           (unsigned)config_.eyelid_sync_mode, (unsigned)old.vergence_lock_enabled,
				           (unsigned)old.vergence_lock_strength, (unsigned)config_.vergence_lock_enabled,
				           (unsigned)config_.vergence_lock_strength, (unsigned)old.gaze_smoothing,
				           (unsigned)config_.gaze_smoothing, (unsigned)old.openness_smoothing,
				           (unsigned)config_.openness_smoothing, (unsigned)old.expression_correction_flags,
				           (unsigned)config_.expression_correction_flags, (unsigned)old.expression_correction_strengths,
				           (unsigned)config_.expression_correction_strengths, (unsigned)old.osc_port,
				           (unsigned)config_.osc_port);
				// Forward active module selection to supervisor.
				if (supervisor_) {
					supervisor_->SetActiveModuleUuid(config_.active_module_uuid);
				}
				else {
					FT_LOG_DRV("[module] config update could not reach host supervisor: supervisor unavailable", 0);
				}
				resp.type = protocol::ResponseSuccess;
				return true;
			}
			case protocol::RequestSetFaceCalibrationCommand: {
				const protocol::FaceCalibrationOp op = (protocol::FaceCalibrationOp)req.setFaceCalibrationCommand.op;
				std::lock_guard<std::mutex> lk(config_mutex_);
				FT_LOG_DRV("[module] calibration command ignored: op=%s(%u)", FaceCalibrationOpName(op), (unsigned)op);
				resp.type = protocol::ResponseSuccess;
				return true;
			}
			case protocol::RequestSetFaceActiveModule: {
				const std::string uuid =
				    FixedString(req.setFaceActiveModule.uuid, protocol::FACETRACKING_MODULE_UUID_LEN);
				FT_LOG_DRV("[module] active module request uuid='%s'", uuid.c_str());
				if (supervisor_) {
					supervisor_->SetActiveModuleUuid(req.setFaceActiveModule.uuid);
				}
				else {
					FT_LOG_DRV("[module] active module request could not reach host supervisor: supervisor unavailable",
					           0);
				}
				resp.type = protocol::ResponseSuccess;
				return true;
			}
			case protocol::RequestSetFaceShapeTuning: {
				const auto& tune = req.setFaceShapeTuning;
				std::lock_guard<std::mutex> lk(config_mutex_);
				if (tune.index == protocol::FACETRACKING_SHAPE_TUNING_RESET_INDEX) {
					shape_tuning_percent_.fill(DefaultShapeTuning());
					FT_LOG_DRV("[module] face shape tuning reset", 0);
					resp.type = protocol::ResponseSuccess;
					return true;
				}
				if (tune.index >= protocol::FACETRACKING_EXPRESSION_COUNT) {
					FT_LOG_DRV("[module] face shape tuning rejected: index=%u scale=%u min=%u max=%u",
					           (unsigned)tune.index, (unsigned)tune.scale_percent, (unsigned)tune.min_percent,
					           (unsigned)tune.max_percent);
					resp.type = protocol::ResponseInvalid;
					return true;
				}
				const protocol::FaceShapeTuningParams params = NormalizeShapeTuning(tune);
				const protocol::FaceShapeTuningParams old = shape_tuning_percent_[tune.index];
				shape_tuning_percent_[tune.index] = params;
				if (old.scale_percent != params.scale_percent || old.min_percent != params.min_percent ||
				    old.max_percent != params.max_percent) {
					FT_LOG_DRV("[module] face shape tuning update: index=%u scale=%u%%->%u%% min=%u%%->%u%% "
					           "max=%u%%->%u%%",
					           (unsigned)tune.index, (unsigned)old.scale_percent, (unsigned)params.scale_percent,
					           (unsigned)old.min_percent, (unsigned)params.min_percent, (unsigned)old.max_percent,
					           (unsigned)params.max_percent);
				}
				resp.type = protocol::ResponseSuccess;
				return true;
			}
			case protocol::RequestFaceHostRestart: {
				FT_LOG_DRV("[module] host restart requested by overlay", 0);
				if (supervisor_) {
					supervisor_->Restart();
				}
				else {
					FT_LOG_DRV("[module] host restart ignored: supervisor unavailable", 0);
				}
				resp.type = protocol::ResponseSuccess;
				return true;
			}
			default:
				return false;
		}
	}

private:
	ServerTrackedDeviceProvider* provider_ = nullptr;
	vr::IVRDriverContext* driver_context_ = nullptr;

	FaceFrameReader reader_;

	std::unique_ptr<FaceTrackingDevice> device_;
	std::unique_ptr<HostSupervisor> supervisor_;

	VergenceLock vergence_;
	EyelidSync eyelid_;
	FaceSignalProcessor signal_processor_;

	// Config cache -- written by HandleRequest, read by WorkerLoop.
	protocol::FaceTrackingConfig config_{};
	FaceShapeTuningArray shape_tuning_percent_ = [] {
		FaceShapeTuningArray values{};
		values.fill(DefaultShapeTuning());
		return values;
	}();
	mutable std::mutex config_mutex_;

	std::atomic<bool> worker_stop_{false};
	std::thread worker_;

	// Telemetry sidecar state.
	std::wstring telemetry_path_;
	std::chrono::steady_clock::time_point last_telemetry_write_{};
	uint64_t frames_processed_ = 0;
	uint64_t frames_read_ = 0;
	uint64_t osc_messages_sent_ = 0;
	uint64_t osc_messages_dropped_ = 0;
	bool latest_shape_values_valid_ = false;
	uint64_t latest_shape_values_frame_ = 0;
	FaceExpressionArray latest_pre_tuning_expressions_{};
	FaceExpressionArray latest_post_tuning_expressions_{};
	FaceOscAddressFilter osc_filter_;
	bool telemetry_write_failed_ = false;

	// Diagnostics state: OSC output transition tracking.
	bool osc_was_enabled_ = false;
	bool osc_first_publish_ = false;
	uint32_t all_zero_frames_ = 0;
	bool all_zero_warned_ = false;
	bool native_unavailable_warned_ = false;
	bool first_frame_diag_logged_ = false;
	std::chrono::steady_clock::time_point last_diag_log_{};
	uint64_t diag_frames_ = 0;
	uint64_t diag_eye_valid_ = 0;
	uint64_t diag_expr_valid_ = 0;
	uint64_t diag_zero_expr_ = 0;
	uint64_t diag_read_failures_ = 0;
	uint64_t diag_osc_attempted_ = 0;
	uint64_t diag_osc_sent_ = 0;
	uint64_t diag_osc_dropped_ = 0;
	uint64_t diag_osc_filtered_ = 0;
	uint64_t diag_osc_deduped_ = 0;
	uint64_t diag_osc_remapped_ = 0;

	// Face-frame anomaly diagnostics. Accumulated per period and emitted on the
	// 5s tick so the log stays low-noise. The anomaly fields capture the worst
	// frame in the window; the pre-correction snapshot captures the most recent
	// frame so the period line can show, per shape, what the module sent vs what
	// the correction pass produced.
	uint32_t diag_max_oob_ = 0;      // expressions outside [0,1] (worst frame)
	uint32_t diag_max_nan_ = 0;      // non-finite expressions (worst frame)
	float diag_max_expr_val_ = 0.0f; // largest expression magnitude (>1 => bulge)
	int diag_max_expr_idx_ = -1;     // which shape held diag_max_expr_val_
	float diag_min_gaze_len_ = 9.0f; // smallest gaze unit-length (0 => dead eyes)
	float diag_max_gaze_len_ = 0.0f; // largest gaze unit-length (>1 => overshoot)
	float diag_pre_jaw_ = 0.0f;
	float diag_pre_mouthClose_ = 0.0f;
	float diag_pre_smileL_ = 0.0f;
	float diag_pre_sadL_ = 0.0f;
	float diag_pre_cheekL_ = 0.0f;
	float diag_pre_browInnerL_ = 0.0f;
	float diag_pre_eyeWideL_ = 0.0f;

	FaceOscPublishCounts MaybePublishOsc(const protocol::FaceTrackingConfig& cfg,
	                                     const protocol::FaceTrackingFrameBody& frame, bool eye_valid,
	                                     bool expr_nonzero)
	{
		const bool osc_enabled = (cfg.output_osc_enabled != 0);
		if (osc_enabled != osc_was_enabled_) {
			if (osc_enabled) {
				FT_LOG_DRV("[facetracking] OSC output enabled", 0);
				osc_first_publish_ = false;
			}
			else {
				FT_LOG_DRV("[facetracking] OSC output disabled", 0);
			}
			osc_was_enabled_ = osc_enabled;
		}

		FaceOscPublishCounts counts{};
		if (!osc_enabled) return counts;

		bool has_nonzero = expr_nonzero;
		if (eye_valid) {
			has_nonzero = has_nonzero || frame.eye_openness_l != 0.f || frame.eye_openness_r != 0.f;
		}

		if (!has_nonzero) {
			if (++all_zero_frames_ >= 60 && !all_zero_warned_) {
				FT_LOG_DRV(
				    "[facetracking] expression frame all-zero for 60+ frames -- module may not be delivering data", 0);
				all_zero_warned_ = true;
			}
		}
		else {
			all_zero_frames_ = 0;
			all_zero_warned_ = false;
		}

		if (osc_filter_.ReloadIfChanged()) {
			FT_LOG_DRV("[facetracking] avatar OSC allowlist status=%s addresses=%u",
			           FaceOscAddressFilterLoadStatusName(osc_filter_.LastLoadStatus()),
			           (unsigned)osc_filter_.AllowedCount());
		}

		// On the first publish, also capture the exact addresses emitted so we
		// can log a one-shot manifest (what the avatar is actually driven with).
		std::vector<std::string> manifest;
		counts = PublishFaceFrameOsc(frame, &osc_filter_, osc_first_publish_ ? nullptr : &manifest);
		osc_messages_sent_ += counts.sent;
		osc_messages_dropped_ += counts.dropped;

		if (!osc_first_publish_ && counts.attempted > 0) {
			FT_LOG_DRV("[facetracking] first OSC publish: attempted=%u sent=%u drop=%u "
			           "filtered=%u deduped=%u remapped=%u JawOpen=%.3f LeftEyeLid=%.3f flags=0x%x",
			           (unsigned)counts.attempted, (unsigned)counts.sent, (unsigned)counts.dropped,
			           (unsigned)counts.filtered, (unsigned)counts.deduped, (unsigned)counts.remapped,
			           frame.expressions[26], // index 26 = JawOpen
			           frame.eye_openness_l, (unsigned)frame.flags);
			LogOscManifest(manifest);
			osc_first_publish_ = true;
		}

		return counts;
	}

	// One-shot dump of the distinct OSC addresses emitted in the first frame.
	// Never repeats, so it adds no ongoing noise; it shows which
	// parameters drive the avatar (and whether legacy + v2 families are both
	// sent, which can over-drive shapes on avatars that map both families).
	void LogOscManifest(const std::vector<std::string>& emitted)
	{
		std::vector<std::string> uniq(emitted.begin(), emitted.end());
		std::sort(uniq.begin(), uniq.end());
		uniq.erase(std::unique(uniq.begin(), uniq.end()), uniq.end());
		unsigned legacy = 0;
		unsigned v2 = 0;
		for (const auto& a : uniq) {
			if (a.find("/v2/") != std::string::npos) {
				++v2;
			}
			else {
				++legacy;
			}
		}
		FT_LOG_DRV("[facetracking][osc-manifest] distinct=%u legacy=%u v2=%u (full list follows)",
		           (unsigned)uniq.size(), legacy, v2);
		std::string line;
		unsigned col = 0;
		for (const auto& a : uniq) {
			line += a;
			line += ' ';
			if (++col >= 8) {
				FT_LOG_DRV("[facetracking][osc-manifest]   %s", line.c_str());
				line.clear();
				col = 0;
			}
		}
		if (!line.empty()) {
			FT_LOG_DRV("[facetracking][osc-manifest]   %s", line.c_str());
		}
	}

	void ResetDebugPeriodCounters()
	{
		diag_frames_ = 0;
		diag_eye_valid_ = 0;
		diag_expr_valid_ = 0;
		diag_zero_expr_ = 0;
		diag_read_failures_ = 0;
		diag_osc_attempted_ = 0;
		diag_osc_sent_ = 0;
		diag_osc_dropped_ = 0;
		diag_osc_filtered_ = 0;
		diag_osc_deduped_ = 0;
		diag_osc_remapped_ = 0;
		diag_max_oob_ = 0;
		diag_max_nan_ = 0;
		diag_max_expr_val_ = 0.0f;
		diag_max_expr_idx_ = -1;
		diag_min_gaze_len_ = 9.0f;
		diag_max_gaze_len_ = 0.0f;
	}

	// -----------------------------------------------------------------------
	// Worker thread: polls shmem, runs the filter pipeline, publishes.
	// -----------------------------------------------------------------------
	void WorkerLoop()
	{
		openvr_pair::common::moduleperf::ScopedThreadRegistration perfRegistration(
		    openvr_pair::common::modules::ModuleId::FaceTracking, "face-frame-worker");
		FT_LOG_DRV("[worker] started", 0);

		uint64_t last_idx = 0;
		const DWORD self_pid = GetCurrentProcessId();
		uint64_t same_index_polls = 0;
		auto last_no_frame_log = std::chrono::steady_clock::time_point{};

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
			const uint64_t hb_age_ms = reader_.HeartbeatAgeMs();
			uint64_t wedge_threshold_ms = UINT64_MAX;
			if (host_state == protocol::HostStatePublishing)
				wedge_threshold_ms = 2000;
			else if (host_state == protocol::HostStateIdle)
				wedge_threshold_ms = 5000;
			// HostStateDraining and HostStateLegacy intentionally never wedge-kill.

			const bool is_wedged = (host_state != protocol::HostStateLegacy) && (hb_age_ms != UINT64_MAX) &&
			                       (hb_age_ms > wedge_threshold_ms);
			if (is_wedged) {
				// Rate-limit restarts to once per 10 s; otherwise a host
				// that takes a few seconds to come back up could chain
				// restarts and never settle.
				const auto now = std::chrono::steady_clock::now();
				const bool restart_allowed =
				    !wedge_restart_pending ||
				    std::chrono::duration_cast<std::chrono::seconds>(now - last_wedge_restart_time).count() >= 10;
				if (restart_allowed) {
					FT_LOG_DRV("[worker] WEDGE: host_state=%u heartbeat_age=%llums "
					           "threshold=%llums -- restarting host",
					           host_state, static_cast<unsigned long long>(hb_age_ms),
					           static_cast<unsigned long long>(wedge_threshold_ms));
					if (supervisor_) supervisor_->Restart();
					// Zero the heartbeat fields so the stale value left by
					// the dead host does not re-trigger the detector
					// before the new host writes its first tick.
					reader_.ResetHostLiveness();
					wedge_restart_pending = true;
					last_wedge_restart_time = now;
				}
				std::this_thread::sleep_for(std::chrono::milliseconds(50));
				continue;
			}
			else if (wedge_restart_pending && hb_age_ms != UINT64_MAX) {
				// New host wrote a fresh heartbeat -- clear the rate-limit
				// so a future wedge gets handled promptly.
				FT_LOG_DRV("[worker] wedge cleared: heartbeat resumed (age=%llums state=%u)",
				           static_cast<unsigned long long>(hb_age_ms), host_state);
				wedge_restart_pending = false;
			}

			uint64_t idx = reader_.LastPublishIndex();
			if (idx == last_idx) {
				++same_index_polls;
				const auto no_frame_now = std::chrono::steady_clock::now();
				const bool host_is_fresh = hb_age_ms != UINT64_MAX && hb_age_ms <= wedge_threshold_ms;
				const bool should_log_no_frame = host_state == protocol::HostStatePublishing && host_is_fresh &&
				                                 (last_no_frame_log == std::chrono::steady_clock::time_point{} ||
				                                  no_frame_now - last_no_frame_log >= std::chrono::seconds(5));
				if (should_log_no_frame) {
					FT_LOG_DRV("[worker][diag] no new frame index while host is publishing: "
					           "publish_index=%llu same_index_polls=%llu host_state=%u hb_age_ms=%llu",
					           static_cast<unsigned long long>(idx), static_cast<unsigned long long>(same_index_polls),
					           host_state, static_cast<unsigned long long>(hb_age_ms));
					last_no_frame_log = no_frame_now;
				}
				// No new frame; sleep briefly so we don't busy-spin.
				std::this_thread::sleep_for(std::chrono::milliseconds(2));

				// Still write telemetry even on idle ticks so the overlay
				// doesn't see a stale file during low-motion periods.
				MaybeWriteTelemetry(self_pid);
				continue;
			}
			last_idx = idx;
			same_index_polls = 0;

			protocol::FaceTrackingFrameBody frame{};
			if (!reader_.TryRead(frame)) {
				const uint64_t failures = ++diag_read_failures_;
				if (failures == 1 || (failures % 256) == 0) {
					FT_LOG_DRV("[worker][diag] frame read failed after publish_index advanced: "
					           "publish_index=%llu failures=%llu host_state=%u hb_age_ms=%llu",
					           static_cast<unsigned long long>(idx), static_cast<unsigned long long>(failures),
					           host_state, static_cast<unsigned long long>(hb_age_ms));
				}
				continue;
			}
			++frames_read_;
			const uint32_t input_flags = frame.flags;
			const float input_jaw_open = frame.upstream_expressions[22];     // UnifiedExpressions.JawOpen
			const float input_mouth_closed = frame.upstream_expressions[29]; // UnifiedExpressions.MouthClosed
			const float input_eye_wide_l = frame.upstream_expressions[3];    // UnifiedExpressions.EyeWideLeft

			// Snapshot config under lock.
			protocol::FaceTrackingConfig cfg;
			FaceShapeTuningArray shapeTuning;
			{
				std::lock_guard<std::mutex> lk(config_mutex_);
				cfg = config_;
				shapeTuning = shape_tuning_percent_;
			}

			// Pre-correction (module-remapped) values for the shapes whose
			// wrongness is most visible on the avatar, captured before the
			// vergence/eyelid/signal-processor transforms so the periodic diag
			// can show module-output vs our-corrected per shape.
			diag_pre_jaw_ = frame.expressions[26];
			diag_pre_mouthClose_ = frame.expressions[40];
			diag_pre_smileL_ = frame.expressions[45];
			diag_pre_sadL_ = frame.expressions[47];
			diag_pre_cheekL_ = frame.expressions[20];
			diag_pre_browInnerL_ = frame.expressions[14];
			diag_pre_eyeWideL_ = frame.expressions[8];

			// Vergence lock.
			if (cfg.vergence_lock_enabled) {
				vergence_.Apply(frame, cfg.vergence_lock_strength);
			}

			// Eyelid sync.
			if (cfg.eyelid_sync_enabled) {
				eyelid_.Apply(frame, cfg.eyelid_sync_strength, cfg.eyelid_sync_preserve_winks != 0,
				              cfg.eyelid_sync_mode);
			}

			FaceExpressionArray preTuningExpressions{};
			signal_processor_.Apply(frame, cfg, shapeTuning.data(), preTuningExpressions.data());

			const bool eye_valid = (frame.flags & 0x1u) != 0;
			const bool expr_valid = (frame.flags & 0x2u) != 0;
			if (expr_valid) {
				latest_shape_values_valid_ = true;
				latest_shape_values_frame_ = frames_read_;
				latest_pre_tuning_expressions_ = preTuningExpressions;
				std::copy(frame.expressions, frame.expressions + protocol::FACETRACKING_EXPRESSION_COUNT,
				          latest_post_tuning_expressions_.begin());
			}

			// Publish to SteamVR inputs.
			if (cfg._reserved_native && device_) {
				device_->PublishFrame(frame);
			}
			else if (cfg._reserved_native && !native_unavailable_warned_) {
				FT_LOG_DRV("[facetracking] native output requested but no SteamVR sink tracker is registered", 0);
				native_unavailable_warned_ = true;
			}

			FaceOscPublishCounts counts{};
			const bool debug_logging_enabled = openvr_pair::common::IsDebugLoggingEnabled();
			bool expr_nonzero = false;
			if ((cfg.output_osc_enabled != 0 || debug_logging_enabled) && expr_valid) {
				for (uint32_t i = 0; i < protocol::FACETRACKING_EXPRESSION_COUNT; ++i) {
					if (frame.expressions[i] != 0.f) {
						expr_nonzero = true;
						break;
					}
				}
			}
			counts = MaybePublishOsc(cfg, frame, eye_valid, expr_nonzero);

			if (debug_logging_enabled) {
				++diag_frames_;
				if (eye_valid) ++diag_eye_valid_;
				if (expr_valid) ++diag_expr_valid_;
				if (expr_valid && !expr_nonzero) ++diag_zero_expr_;
				diag_osc_attempted_ += counts.attempted;
				diag_osc_sent_ += counts.sent;
				diag_osc_dropped_ += counts.dropped;
				diag_osc_filtered_ += counts.filtered;
				diag_osc_deduped_ += counts.deduped;
				diag_osc_remapped_ += counts.remapped;

				// Per-frame anomaly scan -- out-of-range / non-finite expressions
				// and gaze unit-length drift are the usual sources of a bulging or
				// spazzing avatar. Keep only the worst frame of the period.
				{
					uint32_t oob = 0;
					uint32_t nan = 0;
					float maxVal = 0.0f;
					int maxIdx = -1;
					for (uint32_t i = 0; i < protocol::FACETRACKING_EXPRESSION_COUNT; ++i) {
						const float v = frame.expressions[i];
						if (!std::isfinite(v)) {
							++nan;
							continue;
						}
						if (v < 0.0f || v > 1.0f) ++oob;
						const float mag = v < 0.0f ? -v : v;
						if (mag > maxVal) {
							maxVal = mag;
							maxIdx = (int)i;
						}
					}
					if (oob > diag_max_oob_) diag_max_oob_ = oob;
					if (nan > diag_max_nan_) diag_max_nan_ = nan;
					if (maxVal > diag_max_expr_val_) {
						diag_max_expr_val_ = maxVal;
						diag_max_expr_idx_ = maxIdx;
					}
					if (eye_valid) {
						const float* g[2] = {frame.eye_gaze_l, frame.eye_gaze_r};
						for (int e = 0; e < 2; ++e) {
							const float len = std::sqrt(g[e][0] * g[e][0] + g[e][1] * g[e][1] + g[e][2] * g[e][2]);
							if (std::isfinite(len)) {
								if (len < diag_min_gaze_len_) diag_min_gaze_len_ = len;
								if (len > diag_max_gaze_len_) diag_max_gaze_len_ = len;
							}
						}
					}
				}

				if (!first_frame_diag_logged_) {
					first_frame_diag_logged_ = true;
					FT_LOG_DRV("[facetracking][diag] first-frame flags_in=0x%x flags_out=0x%x "
					           "host_state=%u hb_age_ms=%llu module='%s' osc=%u calib=%u "
					           "eyelid=%u/%u/mode=%u vergence=%u/%u corr=0x%02x smooth(gaze=%u open=%u) "
					           "jaw(raw=%.3f out=%.3f mouthClosedRaw=%.3f) "
					           "eyeOpen=(%.3f,%.3f) gazeL=(%.3f,%.3f,%.3f) "
					           "pupil=(%.3f,%.3f) rawEyeWideL=%.3f allowlist=%u allowlist_status=%s",
					           (unsigned)input_flags, (unsigned)frame.flags, host_state,
					           static_cast<unsigned long long>(hb_age_ms), cfg.active_module_uuid,
					           (unsigned)cfg.output_osc_enabled, (unsigned)cfg.continuous_calib_mode,
					           (unsigned)cfg.eyelid_sync_enabled, (unsigned)cfg.eyelid_sync_strength,
					           (unsigned)cfg.eyelid_sync_mode, (unsigned)cfg.vergence_lock_enabled,
					           (unsigned)cfg.vergence_lock_strength, (unsigned)cfg.expression_correction_flags,
					           (unsigned)cfg.gaze_smoothing, (unsigned)cfg.openness_smoothing, input_jaw_open,
					           frame.expressions[26], input_mouth_closed, frame.eye_openness_l, frame.eye_openness_r,
					           frame.eye_gaze_l[0], frame.eye_gaze_l[1], frame.eye_gaze_l[2], frame.pupil_dilation_l,
					           frame.pupil_dilation_r, input_eye_wide_l, (unsigned)osc_filter_.AllowedCount(),
					           FaceOscAddressFilterLoadStatusName(osc_filter_.LastLoadStatus()));
				}

				const auto diag_now = std::chrono::steady_clock::now();
				if (last_diag_log_ == std::chrono::steady_clock::time_point{}) {
					last_diag_log_ = diag_now;
				}
				else if (diag_now - last_diag_log_ >= std::chrono::seconds(5)) {
					FT_LOG_DRV(
					    "[facetracking][diag] period frames=%llu eye_valid=%llu expr_valid=%llu "
					    "zero_expr=%llu host_state=%u hb_age_ms=%llu module='%s' "
					    "osc=%u osc_delta=(attempt=%llu sent=%llu drop=%llu filtered=%llu deduped=%llu remapped=%llu) "
					    "total_osc=(sent=%llu drop=%llu) allowlist=%u allowlist_status=%s read_fail=%llu "
					    "calib=%u eyelid=%u/mode=%u vergence=%u corr=0x%02x "
					    "last_jaw=%.3f last_eye=(%.3f,%.3f) last_pupil=(%.3f,%.3f)",
					    static_cast<unsigned long long>(diag_frames_), static_cast<unsigned long long>(diag_eye_valid_),
					    static_cast<unsigned long long>(diag_expr_valid_),
					    static_cast<unsigned long long>(diag_zero_expr_), host_state,
					    static_cast<unsigned long long>(hb_age_ms), cfg.active_module_uuid,
					    (unsigned)cfg.output_osc_enabled, static_cast<unsigned long long>(diag_osc_attempted_),
					    static_cast<unsigned long long>(diag_osc_sent_),
					    static_cast<unsigned long long>(diag_osc_dropped_),
					    static_cast<unsigned long long>(diag_osc_filtered_),
					    static_cast<unsigned long long>(diag_osc_deduped_),
					    static_cast<unsigned long long>(diag_osc_remapped_),
					    static_cast<unsigned long long>(osc_messages_sent_),
					    static_cast<unsigned long long>(osc_messages_dropped_), (unsigned)osc_filter_.AllowedCount(),
					    FaceOscAddressFilterLoadStatusName(osc_filter_.LastLoadStatus()),
					    static_cast<unsigned long long>(diag_read_failures_), (unsigned)cfg.continuous_calib_mode,
					    (unsigned)cfg.eyelid_sync_enabled, (unsigned)cfg.eyelid_sync_mode,
					    (unsigned)cfg.vergence_lock_enabled, (unsigned)cfg.expression_correction_flags,
					    frame.expressions[26], frame.eye_openness_l, frame.eye_openness_r, frame.pupil_dilation_l,
					    frame.pupil_dilation_r);

					// Per-shape module-output vs our-corrected for the avatar's
					// most-visible shapes (latest frame). A large pre->post swing
					// points at our corrections; a wrong pre points upstream.
					FT_LOG_DRV("[facetracking][shapes] jaw=%.3f->%.3f mouthClose=%.3f->%.3f smileL=%.3f->%.3f "
					           "sadL=%.3f->%.3f cheekL=%.3f->%.3f browInnerL=%.3f->%.3f eyeWideL=%.3f->%.3f "
					           "eyeOpen=(%.3f,%.3f) gazeL=(%.3f,%.3f,%.3f) gazeR=(%.3f,%.3f,%.3f)",
					           diag_pre_jaw_, frame.expressions[26], diag_pre_mouthClose_, frame.expressions[40],
					           diag_pre_smileL_, frame.expressions[45], diag_pre_sadL_, frame.expressions[47],
					           diag_pre_cheekL_, frame.expressions[20], diag_pre_browInnerL_, frame.expressions[14],
					           diag_pre_eyeWideL_, frame.expressions[8], frame.eye_openness_l, frame.eye_openness_r,
					           frame.eye_gaze_l[0], frame.eye_gaze_l[1], frame.eye_gaze_l[2], frame.eye_gaze_r[0],
					           frame.eye_gaze_r[1], frame.eye_gaze_r[2]);

					// Health: worst-frame anomalies + the OSC family/filter state.
					// filter_active=0 means fail-open: BOTH the legacy and v2
					// families are sent unfiltered with dedup off, which can
					// over-drive an avatar that maps both parameter families.
					const bool diag_filter_active = osc_filter_.Active();
					FT_LOG_DRV(
					    "[facetracking][health] worst_oob=%u worst_nan=%u max_expr=%s:%.3f gaze_len=[%.3f..%.3f] "
					    "filter_active=%d families=%s allowlist=%u status=%s",
					    (unsigned)diag_max_oob_, (unsigned)diag_max_nan_,
					    diag_max_expr_idx_ >= 0 ? facetracking::FaceExpressionOscName((uint32_t)diag_max_expr_idx_)
					                            : "n/a",
					    diag_max_expr_val_, diag_min_gaze_len_, diag_max_gaze_len_, (int)diag_filter_active,
					    diag_filter_active ? "filtered" : "legacy+v2+vrcft(failopen,dedup-off)",
					    (unsigned)osc_filter_.AllowedCount(),
					    FaceOscAddressFilterLoadStatusName(osc_filter_.LastLoadStatus()));

					ResetDebugPeriodCounters();
					last_diag_log_ = diag_now;
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
		const float ipd_m = verg_enabled ? vergence_.LastIpdM() : 0.f;

		std::string json = BuildTelemetryJson(pid, frames_processed_, frames_read_, osc_messages_sent_,
		                                      osc_messages_dropped_, cfg.active_module_uuid, verg_enabled, focus_m,
		                                      ipd_m, latest_shape_values_valid_, latest_shape_values_frame_,
		                                      latest_pre_tuning_expressions_, latest_post_tuning_expressions_);

		if (!AtomicWriteFile(telemetry_path_, json)) {
			if (!telemetry_write_failed_) {
				FT_LOG_DRV("[worker][diag] driver telemetry write failed", 0);
				telemetry_write_failed_ = true;
			}
		}
		else if (telemetry_write_failed_) {
			FT_LOG_DRV("[worker][diag] driver telemetry write recovered", 0);
			telemetry_write_failed_ = false;
		}
	}
};

} // namespace

std::unique_ptr<DriverModule> CreateDriverModule()
{
	return std::make_unique<FacetrackingDriverModule>();
}

} // namespace facetracking
