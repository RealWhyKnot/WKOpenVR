#include "CalibrationMetrics.h"
#include "BuildChannel.h"
#include "BuildStamp.h"
#include "CalibrationLogLaunchContext.h"
#include "DiagnosticsLog.h"
#include "FileLog.h"
#include "LogPaths.h"
#include "Win32Paths.h"
#if WKOPENVR_BUILD_IS_DEV
#include "MotionRecording.h"
#endif
#include <cerrno>
#include <cstdio>
#include <fcntl.h>
#include <openvr.h>
#include <ostream>
#include <sstream>
#include <string>
#include <io.h>

namespace Metrics {
double TimeSpan = 30, CurrentTime = 0;

TimeSeries<Eigen::Vector3d> posOffset_rawComputed; // , rotOffset_rawComputed;
TimeSeries<Eigen::Vector3d> posOffset_currentCal;  // , rotOffset_currentCal;
TimeSeries<Eigen::Vector3d> posOffset_lastSample;  // , rotOffset_lastSample;
TimeSeries<Eigen::Vector3d> posOffset_byRelPose;

TimeSeries<double> error_rawComputed, error_currentCal, error_byRelPose, error_currentCalRelPose;
TimeSeries<double> axisIndependence;
TimeSeries<double> computationTime;
TimeSeries<double> jitterRef, jitterTarget;
TimeSeries<double> rotationConditionRatio;
TimeSeries<double> consecutiveRejections;
TimeSeries<double> samplesInBuffer;
TimeSeries<double> watchdogResetCount;
TimeSeries<double> translationDiversity;
TimeSeries<double> rotationDiversity;
TimeSeries<Eigen::Vector3d> translationAxisRangesCm;
TimeSeries<double> pairedMotionWarningCount;
TimeSeries<double> watchdogHealthySkip;
TimeSeries<double> effectivePriorMm;
TimeSeries<double> validateRmsThresholdMm;
std::string lastRejectReason;

TimeSeries<double> fallbackApplyRate;
TimeSeries<double> perIdApplyRate;
TimeSeries<double> quashApplyRate;

// true - full calibration, false - static calibration
TimeSeries<bool> calibrationApplied;

TimeSeries<bool> headMountActive;
TimeSeries<double> headMountResidualMm;
TimeSeries<double> questHmdVsProxyDeltaMm;
TimeSeries<uint32_t> snapSuppressedCount;
TimeSeries<uint32_t> driverSynthFallbackCount;
TimeSeries<bool> boundaryActive;
TimeSeries<uint32_t> chaperoneRePushCount;

// https://stackoverflow.com/a/17827724
bool IsBrowsePath(const std::wstring& path)
{
	return (path == L"." || path == L"..");
}

uint64_t CalculateDirSize(const std::wstring& path, uint64_t size = 0)
{
	WIN32_FIND_DATA data;
	HANDLE sh = NULL;
	sh = FindFirstFile((path + L"\\*").c_str(), &data);

	if (sh == INVALID_HANDLE_VALUE) {
		// We should probably return an error, but we don't for the sake of minimising memory allocations
		return size;
	}

	do {
		// skip current and parent
		if (!IsBrowsePath(data.cFileName)) {
			// if found object is ...
			if ((data.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) == FILE_ATTRIBUTE_DIRECTORY)
				// directory, then search it recursievly
				size = CalculateDirSize(path + L"\\" + data.cFileName, size);
			else
				// otherwise get object size and add it to directory size
				size += (uint64_t)(data.nFileSizeHigh * (MAXDWORD) + data.nFileSizeLow);
		}

	} while (FindNextFile(sh, &data)); // do

	FindClose(sh);

	return size;
}

double timestamp()
{
	static long long ts_start = ~0LL;

	LARGE_INTEGER ts, freq;
	QueryPerformanceCounter(&ts);
	QueryPerformanceFrequency(&freq);

	if (ts_start == ~0LL) ts_start = ts.QuadPart;

	ts.QuadPart -= ts_start;

	return ts.QuadPart / (double)freq.QuadPart;
}

void RecordTimestamp()
{
	CurrentTime = timestamp();
}

// Debug-log toggle. Default depends on the build channel:
//   - dev builds (build.ps1 without -Version, SPACECAL_BUILD_CHANNEL="dev"):
//     ON. Local development sessions reproduce issues that benefit from
//     a CSV trail without the developer having to remember to flip the
//     toggle every launch.
//   - release builds (CI tag publish, SPACECAL_BUILD_CHANNEL="release"):
//     OFF. End users only opt in when investigating a bug.
// The user can flip it either direction from the Logs tab regardless;
// this is just the boot default.
#if defined(SPACECAL_BUILD_CHANNEL_IS_DEV)
bool enableLogs = true;
#else
bool enableLogs = (std::string(SPACECAL_BUILD_CHANNEL) == "dev");
#endif
#if WKOPENVR_BUILD_IS_DEV
bool enableReplayCsv = true;
#endif

static FILE* logFile = nullptr;
static std::wstring logFilePath;
static bool logFileIsOpen = false;
static bool failedToOpenLogFile = false;
static ULONGLONG failedToOpenLogFileAtMs = 0;
static bool logWriteFailed = false;
static bool logFlushFailed = false;
static uint64_t logRowsWritten = 0;
static uint64_t logAnnotationsWritten = 0;
static uint64_t logOpenAttempts = 0;
static unsigned long lastLogErrorCode = 0;
static std::string lastLogStatus = "not opened";

// v2 wire-format addition: per-tick raw reference and target poses plus the tick
// phase. Filled by SetTickRawPoses() each tick and consumed by the field writers
// below. Defaults are an identity pose with phase=None so that any unexpected
// WriteLogEntry call (i.e. one not preceded by SetTickRawPoses) emits a syntactically
// valid row rather than uninitialized memory.
struct TickRawPoses
{
	Eigen::Vector3d refTrans = Eigen::Vector3d::Zero();
	Eigen::Quaterniond refRot = Eigen::Quaterniond::Identity();
	Eigen::Vector3d targetTrans = Eigen::Vector3d::Zero();
	Eigen::Quaterniond targetRot = Eigen::Quaterniond::Identity();
	TickPhase phase = TickPhase::None;
};
static TickRawPoses g_tickRaw;

static const char* TickPhaseName(TickPhase p)
{
	switch (p) {
		case TickPhase::None:
			return "None";
		case TickPhase::Begin:
			return "Begin";
		case TickPhase::Rotation:
			return "Rotation";
		case TickPhase::Translation:
			return "Translation";
		case TickPhase::Editing:
			return "Editing";
		case TickPhase::Continuous:
			return "Continuous";
		case TickPhase::ContinuousStandby:
			return "ContinuousStandby";
	}
	return "None";
}

void SetTickRawPoses(const Eigen::Vector3d& refTrans, const Eigen::Quaterniond& refRot,
                     const Eigen::Vector3d& targetTrans, const Eigen::Quaterniond& targetRot, TickPhase phase)
{
	g_tickRaw.refTrans = refTrans;
	g_tickRaw.refRot = refRot;
	g_tickRaw.targetTrans = targetTrans;
	g_tickRaw.targetRot = targetRot;
	g_tickRaw.phase = phase;
}

struct CsvField
{
	const char* name;
	void (*writer)(std::ostream& s);
};

static void SetLogStatus(const std::string& status, unsigned long errorCode = 0)
{
	lastLogStatus = status;
	lastLogErrorCode = errorCode;
}

#define TS_FIELD(n) {#n, [](auto& s) { s << n.last(); }}

#define TS_VECTOR_FIELD(n)                                                                                             \
	{#n ".x", [](auto& s) { s << n.last()(0); }}, {#n ".y", [](auto& s) { s << n.last()(1); }},                        \
	{                                                                                                                  \
		#n ".z", [](auto& s) {                                                                                         \
			s << n.last()(2);                                                                                          \
		}                                                                                                              \
	}

#if WKOPENVR_BUILD_IS_DEV
static const CsvField fields[] = {
    {"Timestamp", [](auto& s) { s << CurrentTime; }},

    TS_VECTOR_FIELD(posOffset_rawComputed),
    TS_VECTOR_FIELD(posOffset_currentCal),
    TS_VECTOR_FIELD(posOffset_lastSample),
    TS_VECTOR_FIELD(posOffset_byRelPose),

    TS_FIELD(error_rawComputed),
    TS_FIELD(error_currentCal),
    TS_FIELD(error_byRelPose),
    TS_FIELD(error_currentCalRelPose),
    TS_FIELD(axisIndependence),
    TS_FIELD(rotationConditionRatio),
    TS_FIELD(consecutiveRejections),
    TS_FIELD(samplesInBuffer),
    // Motion-coverage scores for the live sample buffer (0..1 each). Pushed by
    // CollectSample. The Calibration Progress popup reads these for the live
    // "Translation %" / "Rotation %" bars; logging them here lets post-hoc
    // triage see exactly what the bars showed when a one-shot calibration
    // got stuck below the auto-finish threshold.
    TS_FIELD(translationDiversity),
    TS_FIELD(rotationDiversity),
    TS_VECTOR_FIELD(translationAxisRangesCm),
    TS_FIELD(pairedMotionWarningCount),
    // Wedge-detection diagnostics. watchdogHealthySkip flags ticks where the
    // watchdog wanted to clear but couldn't (prior in healthy band).
    // effectivePriorMm is the actual prior the 1.5× gate compared against.
    // validateRmsThresholdMm is the dynamic noise-floor threshold the
    // validate gate used this tick.
    TS_FIELD(watchdogHealthySkip),
    TS_FIELD(effectivePriorMm),
    TS_FIELD(validateRmsThresholdMm),
    TS_FIELD(watchdogResetCount),
    TS_FIELD(computationTime),
    TS_FIELD(jitterRef),
    TS_FIELD(jitterTarget),
    TS_FIELD(fallbackApplyRate),
    TS_FIELD(perIdApplyRate),
    TS_FIELD(quashApplyRate),
    // String-valued reject reason. Empty when the last ComputeIncremental
    // accepted; otherwise one of: "below_floor_or_worse", "axis_variance_low",
    // "rotation_planar", "rotation_no_deltas", "translation_planar",
    // "translation_no_deltas", "validate_failed", "healthy_below_floor".
    {"reject_reason", [](auto& s) { s << lastRejectReason; }},

    {"calibrationApplied",
     [](auto& s) {
	     if (calibrationApplied.lastTs() == CurrentTime) {
		     if (calibrationApplied.last()) {
			     s << "FULL";
		     }
		     else {
			     s << "STATIC";
		     }
	     }
     }},

    // --- v2 columns: raw reference + target poses and tick phase ---------------
    // Translations are in meters, rotations are unit quaternions in (w,x,y,z) order.
    // These are written with full double precision so the replay harness can
    // reconstruct the exact `Sample` values that fed CalibrationCalc::PushSample.
    {"ref_tx",
     [](auto& s) {
	     s.precision(17);
	     s << g_tickRaw.refTrans.x();
     }},
    {"ref_ty",
     [](auto& s) {
	     s.precision(17);
	     s << g_tickRaw.refTrans.y();
     }},
    {"ref_tz",
     [](auto& s) {
	     s.precision(17);
	     s << g_tickRaw.refTrans.z();
     }},
    {"ref_qw",
     [](auto& s) {
	     s.precision(17);
	     s << g_tickRaw.refRot.w();
     }},
    {"ref_qx",
     [](auto& s) {
	     s.precision(17);
	     s << g_tickRaw.refRot.x();
     }},
    {"ref_qy",
     [](auto& s) {
	     s.precision(17);
	     s << g_tickRaw.refRot.y();
     }},
    {"ref_qz",
     [](auto& s) {
	     s.precision(17);
	     s << g_tickRaw.refRot.z();
     }},
    {"tgt_tx",
     [](auto& s) {
	     s.precision(17);
	     s << g_tickRaw.targetTrans.x();
     }},
    {"tgt_ty",
     [](auto& s) {
	     s.precision(17);
	     s << g_tickRaw.targetTrans.y();
     }},
    {"tgt_tz",
     [](auto& s) {
	     s.precision(17);
	     s << g_tickRaw.targetTrans.z();
     }},
    {"tgt_qw",
     [](auto& s) {
	     s.precision(17);
	     s << g_tickRaw.targetRot.w();
     }},
    {"tgt_qx",
     [](auto& s) {
	     s.precision(17);
	     s << g_tickRaw.targetRot.x();
     }},
    {"tgt_qy",
     [](auto& s) {
	     s.precision(17);
	     s << g_tickRaw.targetRot.y();
     }},
    {"tgt_qz",
     [](auto& s) {
	     s.precision(17);
	     s << g_tickRaw.targetRot.z();
     }},
    {"tick_phase", [](auto& s) { s << TickPhaseName(g_tickRaw.phase); }},
};
#endif

// Launch-context details are written by CalibrationLogLaunchContext.cpp.

static std::wstring InsertUniqueSuffix(const std::wstring& fileName, DWORD pid, int attempt)
{
	if (attempt == 0) return fileName;

	const size_t dot = fileName.rfind(L".txt");
	std::wstring out = dot == std::wstring::npos ? fileName : fileName.substr(0, dot);
	wchar_t suffix[64] = {};
	swprintf_s(suffix, L".%lu-%02d", static_cast<unsigned long>(pid), attempt);
	out += suffix;
	if (dot != std::wstring::npos) {
		out += fileName.substr(dot);
	}
	return out;
}

static FILE* OpenUniqueLogFile(std::wstring& outPath)
{
	std::wstring directory = openvr_pair::common::WkOpenVrLogsPath(true);
	if (directory.empty()) {
		SetLogStatus("log directory unavailable");
		return nullptr;
	}

	openvr_pair::common::DeleteOldLogFiles(directory, L"spacecal_log");

	SYSTEMTIME now{};
	GetSystemTime(&now);
	const std::wstring baseName = openvr_pair::common::TimestampedLogFileName(L"spacecal_log", now);
	if (baseName.empty()) return nullptr;

	if (!directory.empty() && directory.back() != L'\\' && directory.back() != L'/') {
		directory.push_back(L'\\');
	}

	const DWORD pid = GetCurrentProcessId();
	for (int attempt = 0; attempt < 100; ++attempt) {
		std::wstring candidate = directory + InsertUniqueSuffix(baseName, pid, attempt);
		HANDLE handle = CreateFileW(candidate.c_str(), GENERIC_WRITE, FILE_SHARE_READ, nullptr, CREATE_NEW,
		                            FILE_ATTRIBUTE_NORMAL | FILE_FLAG_WRITE_THROUGH, nullptr);
		if (handle == INVALID_HANDLE_VALUE) {
			const DWORD err = GetLastError();
			if (err == ERROR_FILE_EXISTS || err == ERROR_ALREADY_EXISTS) {
				continue;
			}
			SetLogStatus("CreateFileW failed", err);
			openvr_pair::common::DiagnosticLog("spacecal", "log_create_failed path='%ls' err=%lu", candidate.c_str(),
			                                   err);
			return nullptr;
		}

		const int fd = _open_osfhandle(reinterpret_cast<intptr_t>(handle), _O_WRONLY | _O_BINARY);
		if (fd < 0) {
			const int openErr = errno;
			CloseHandle(handle);
			SetLogStatus("file descriptor conversion failed", static_cast<unsigned long>(openErr));
			openvr_pair::common::DiagnosticLog("spacecal", "log_fd_failed path='%ls' errno=%d", candidate.c_str(),
			                                   openErr);
			return nullptr;
		}

		FILE* file = _fdopen(fd, "wb");
		if (!file) {
			const int fdopenErr = errno;
			_close(fd);
			SetLogStatus("fdopen failed", static_cast<unsigned long>(fdopenErr));
			openvr_pair::common::DiagnosticLog("spacecal", "log_fdopen_failed path='%ls' errno=%d", candidate.c_str(),
			                                   fdopenErr);
			return nullptr;
		}

		outPath = candidate;
		return file;
	}

	openvr_pair::common::DiagnosticLog("spacecal", "log_create_failed reason=name_collision_exhausted");
	SetLogStatus("name collision exhausted");
	return nullptr;
}

static bool FlushOpenLogFile()
{
	if (!logFile) return false;
	if (openvr_pair::common::FlushLogFileToDisk(logFile)) {
		logFlushFailed = false;
		return true;
	}
	if (!logFlushFailed) {
		SetLogStatus("flush failed");
		openvr_pair::common::DiagnosticLog("spacecal", "log_flush_failed path='%ls'", logFilePath.c_str());
		logFlushFailed = true;
	}
	return false;
}

static bool WriteRawLogText(const std::string& text)
{
	if (!logFile) return false;
	if (!text.empty()) {
		const size_t written = fwrite(text.data(), 1, text.size(), logFile);
		if (written != text.size()) {
			if (!logWriteFailed) {
				SetLogStatus("write failed", static_cast<unsigned long>(errno));
				openvr_pair::common::DiagnosticLog(
				    "spacecal", "log_write_failed path='%ls' wrote=%llu expected=%llu errno=%d", logFilePath.c_str(),
				    static_cast<unsigned long long>(written), static_cast<unsigned long long>(text.size()), errno);
				logWriteFailed = true;
			}
			return false;
		}
		logWriteFailed = false;
	}
	return FlushOpenLogFile();
}

static long long CurrentLogSizeBytes()
{
	if (!logFile) return -1;
	const int fd = _fileno(logFile);
	if (fd < 0) return -1;
	const intptr_t osHandle = _get_osfhandle(fd);
	if (osHandle == -1) return -1;
	LARGE_INTEGER size{};
	if (!GetFileSizeEx(reinterpret_cast<HANDLE>(osHandle), &size)) return -1;
	return size.QuadPart;
}

static void DiscardOpenLogFile()
{
	if (logFile) {
		FlushOpenLogFile();
		fclose(logFile);
	}
	logFile = nullptr;
	logFilePath.clear();
	logFileIsOpen = false;
	logWriteFailed = false;
	logFlushFailed = false;
}

// %userprofile%\LocalLow\WKOpenVR\Logs
static bool OpenLogFile()
{
	++logOpenAttempts;
	SetLogStatus("opening");
	std::wstring path;
	FILE* file = OpenUniqueLogFile(path);
	if (!file) {
		if (lastLogStatus.empty() || lastLogStatus == "opening") {
			SetLogStatus("open failed");
		}
		openvr_pair::common::DiagnosticLog("spacecal", "log_open_failed");
		return false;
	}

	logFile = file;
	logFilePath = path;
	openvr_pair::common::SetLowLatencyLogMode(logFile);
	logWriteFailed = false;
	logFlushFailed = false;
	logRowsWritten = 0;
	logAnnotationsWritten = 0;

	openvr_pair::common::DiagnosticLog("spacecal", "log_opened path='%ls' enable_logs=%d", logFilePath.c_str(),
	                                   enableLogs ? 1 : 0);

	std::ostringstream header;
	header.precision(17);

#if WKOPENVR_BUILD_IS_DEV
	if (enableReplayCsv) {
		// Wire-format version annotation. v2 added per-tick raw reference + target
		// poses (ref_t{x,y,z}, ref_q{w,x,y,z}, tgt_*) and tick_phase. The replay
		// harness rejects logs that don't begin with this banner so older captures
		// fail loud rather than being interpreted with the wrong column layout.
		header << "# spacecal_log_v2\n";
	}
	else
#endif
	{
		header << "# spacecal_debug_log\n";
	}

	// === Self-describing header ============================================
	// Triage from a debug log starts with "what build was this, on what
	// hardware, with what profile". Embed that up-front so a single log file
	// is sufficient for a bug report — no need to ask the reporter to run a
	// dozen follow-up commands.
	header << "# build_stamp=" SPACECAL_BUILD_STAMP "\n";
	header << "# build_channel=" SPACECAL_BUILD_CHANNEL "\n";
	header << "# debug_logging_effective=1\n";
	header << "# replay_recording="
#if WKOPENVR_BUILD_IS_DEV
	       << (enableReplayCsv ? "enabled format=spacecal_log_v2" : "disabled")
#else
	       << "not_compiled"
#endif
	       << " flush=per_write\n";

	if (!WriteRawLogText(header.str())) {
		DiscardOpenLogFile();
		return false;
	}
	header.str(std::string());
	header.clear();

#if WKOPENVR_BUILD_IS_DEV
	if (enableReplayCsv) {
		const spacecal::replay::RecordingRetentionPolicy retentionPolicy{};
		const auto prune = spacecal::replay::PruneRecordings(retentionPolicy);
		header << "# dev_auto_recording=enabled"
		       << " retention_files=" << retentionPolicy.maxFiles
		       << " retention_bytes=" << retentionPolicy.maxTotalBytes << " total_files=" << prune.totalFiles
		       << " total_bytes=" << prune.totalBytes << " deleted_files=" << prune.deletedFiles
		       << " freed_bytes=" << prune.freedBytes << " kept_files=" << prune.keptFiles
		       << " kept_bytes=" << prune.keptBytes << " failed_deletes=" << prune.failedDeletes << "\n";
	}
	else {
		header << "# dev_auto_recording=disabled replay_csv=0\n";
	}
#else
	header << "# dev_auto_recording=not_compiled build_channel=" SPACECAL_BUILD_CHANNEL "\n";
#endif

	// HMD identification — model + tracking system. Driver-side issues often
	// correlate to a specific HMD or runtime, so this is the first thing
	// anyone reading the log wants to know.
	if (auto vrSystem = vr::VRSystem()) {
		char buf[vr::k_unMaxPropertyStringSize] = {};
		vr::ETrackedPropertyError pe = vr::TrackedProp_Success;
		vrSystem->GetStringTrackedDeviceProperty(vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_TrackingSystemName_String,
		                                         buf, sizeof buf, &pe);
		if (pe == vr::TrackedProp_Success && buf[0]) {
			header << "# hmd_tracking_system=" << buf << "\n";
		}
		buf[0] = 0;
		vrSystem->GetStringTrackedDeviceProperty(vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_ModelNumber_String, buf,
		                                         sizeof buf, &pe);
		if (pe == vr::TrackedProp_Success && buf[0]) {
			header << "# hmd_model=" << buf << "\n";
		}
		buf[0] = 0;
		vrSystem->GetStringTrackedDeviceProperty(vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_SerialNumber_String, buf,
		                                         sizeof buf, &pe);
		if (pe == vr::TrackedProp_Success && buf[0]) {
			header << "# hmd_serial=" << buf << "\n";
		}

		// SteamVR runtime version, when reachable. Useful for filtering issues
		// to a specific Steam beta/stable lineage.
		char rtPath[MAX_PATH] = {};
		unsigned int rtLen = 0;
		vr::VR_GetRuntimePath(rtPath, MAX_PATH, &rtLen);
		if (rtLen > 0) {
			header << "# steamvr_runtime_path=" << rtPath << "\n";
		}
	}
	else {
		header << "# vr_system=unavailable_at_log_open\n";
	}

	// CPU + memory floor for context. SHGetKnownFolderPath needed Win10+
	// already so OS version reporting is just for triage; not gated.
	{
		OSVERSIONINFOEXW osv{sizeof(OSVERSIONINFOEXW)};
		using RtlGetVersionPtr = LONG(WINAPI*)(LPOSVERSIONINFOEXW);
		HMODULE ntdll = GetModuleHandleW(L"ntdll.dll");
		if (ntdll) {
			auto fn = (RtlGetVersionPtr)GetProcAddress(ntdll, "RtlGetVersion");
			if (fn && fn(&osv) == 0) {
				header << "# windows=" << osv.dwMajorVersion << "." << osv.dwMinorVersion << "." << osv.dwBuildNumber
				       << "\n";
			}
		}

		SYSTEM_INFO si{};
		GetSystemInfo(&si);
		header << "# logical_processors=" << si.dwNumberOfProcessors << "\n";
	}

	// === Launch context ====================================================
	// Captures everything that could differ between a Start-menu launch and
	// a script-launched instance. Diffing these blocks across two log files
	// (one from each launch path) tells us exactly what's environmentally
	// different about the process — without this, we're guessing about
	// elevation / parent-process / cwd / session etc.
	//
	// Triggered by the 2026-05-03 user report: tracking consistently breaks
	// when SpaceCalibrator.exe is launched by quick.ps1 -Install's auto-
	// relaunch, but works fine when the same exe is launched from the
	// Start menu. Same exe, same driver, same registry profile. Some part
	// of the process context must differ; this block captures the candidates.
	//
	// All values are best-effort — Win32 calls that fail are logged with
	// the error code rather than skipped, so a missing field doesn't hide
	// the failure mode itself.
	WriteLaunchContextBanner(header);

#if WKOPENVR_BUILD_IS_DEV
	if (enableReplayCsv) {
		// Banner divider so a human grepping the file can see where header ends
		// and the column row begins. Replay harness ignores any line starting with #.
		header << "# === columns ===\n";

		for (int i = 0; i < sizeof fields / sizeof fields[0]; i++) {
			if (i > 0) header << ",";
			header << fields[i].name;
		}
		header << "\n";
	}
#endif

	logFileIsOpen = true;
	if (!WriteRawLogText(header.str())) {
		DiscardOpenLogFile();
		return false;
	}
	openvr_pair::common::DiagnosticLog("spacecal", "log_ready path='%ls' size_bytes=%lld", logFilePath.c_str(),
	                                   CurrentLogSizeBytes());
	SetLogStatus("open");

	return true;
}

static bool CheckLogOpen()
{
	if (!enableLogs) {
		if (logFileIsOpen) {
			CloseLogFile();
		}
		logFileIsOpen = false;
		failedToOpenLogFile = false;
		failedToOpenLogFileAtMs = 0;
		SetLogStatus("debug logging disabled");

		return false;
	}

	if (failedToOpenLogFile) {
		const ULONGLONG nowMs = GetTickCount64();
		if (failedToOpenLogFileAtMs != 0 && nowMs - failedToOpenLogFileAtMs < 5000) {
			return false;
		}
		openvr_pair::common::DiagnosticLog("spacecal", "log_open_retry after_previous_failure");
		failedToOpenLogFile = false;
	}
	if (!logFileIsOpen && !OpenLogFile()) {
		failedToOpenLogFile = true;
		failedToOpenLogFileAtMs = GetTickCount64();
		return false;
	}
	failedToOpenLogFileAtMs = 0;
	return true;
}

static void WriteLogHealthSnapshotRaw(const char* reason)
{
	if (!logFile) return;
	std::ostringstream row;
	row.precision(17);
	row << "# [" << timestamp() << "] log_health"
	    << " reason=" << ((reason && reason[0]) ? reason : "unspecified") << " debug_enabled=" << (enableLogs ? 1 : 0)
	    << " open=" << (logFileIsOpen ? 1 : 0) << " failed_to_open=" << (failedToOpenLogFile ? 1 : 0)
	    << " write_failed=" << (logWriteFailed ? 1 : 0) << " flush_failed=" << (logFlushFailed ? 1 : 0)
	    << " rows=" << logRowsWritten << " annotations=" << logAnnotationsWritten
	    << " size_bytes=" << CurrentLogSizeBytes() << " open_attempts=" << logOpenAttempts
	    << " last_error=" << lastLogErrorCode << " status=" << lastLogStatus << "\n";
	if (WriteRawLogText(row.str())) {
		++logAnnotationsWritten;
	}
}

void WriteLogAnnotation(const char* s)
{
	if (!CheckLogOpen()) return;

	std::ostringstream row;
	row.precision(17);
	row << "# [" << timestamp() << "] " << s << "\n";
	if (WriteRawLogText(row.str())) {
		++logAnnotationsWritten;
	}
}

void WriteLogEntry()
{
#if WKOPENVR_BUILD_IS_DEV
	if (!enableReplayCsv) return;
	if (!CheckLogOpen()) return;

	if (logFileIsOpen) {
		std::ostringstream row;
		row.precision(17);
		for (int i = 0; i < sizeof fields / sizeof fields[0]; i++) {
			if (i > 0) row << ",";
			fields[i].writer(row);
		}
		row << "\n";
		if (WriteRawLogText(row.str())) {
			++logRowsWritten;
		}
	}
#endif
}

bool EnsureLogFileReady(const char* reason)
{
	const bool wasOpen = logFileIsOpen;
	if (!CheckLogOpen()) return false;
	if (!wasOpen || (reason && reason[0])) {
		WriteLogHealthSnapshotRaw(reason && reason[0] ? reason : "ensure_ready");
	}
	return true;
}

bool FlushLogFile()
{
	if (!logFileIsOpen || !logFile) return false;
	return FlushOpenLogFile();
}

LogHealth GetLogHealth()
{
	LogHealth health;
	health.debugEnabled = enableLogs;
	health.open = logFileIsOpen && logFile != nullptr;
	health.failedToOpen = failedToOpenLogFile;
	health.writeFailed = logWriteFailed;
	health.flushFailed = logFlushFailed;
#if WKOPENVR_BUILD_IS_DEV
	health.replayCsvEnabled = enableReplayCsv;
	health.devAutoRecording = enableLogs && enableReplayCsv;
#else
	health.replayCsvEnabled = false;
	health.devAutoRecording = false;
#endif
	health.path = logFilePath;
	health.sizeBytes = CurrentLogSizeBytes();
	health.rowsWritten = logRowsWritten;
	health.annotationsWritten = logAnnotationsWritten;
	health.openAttempts = logOpenAttempts;
	health.lastErrorCode = lastLogErrorCode;
	health.status = lastLogStatus;
	return health;
}

void WriteLogHealthSnapshot(const char* reason)
{
	if (!CheckLogOpen()) return;
	WriteLogHealthSnapshotRaw(reason);
}

void CloseLogFile()
{
	if (!logFile) {
		logFileIsOpen = false;
		failedToOpenLogFile = false;
		failedToOpenLogFileAtMs = 0;
		SetLogStatus(enableLogs ? "not opened" : "debug logging disabled");
		return;
	}

	std::ostringstream row;
	row.precision(17);
	row << "# [" << timestamp() << "] log_close\n";
	WriteRawLogText(row.str());
	FlushOpenLogFile();
	fclose(logFile);
	openvr_pair::common::DiagnosticLog("spacecal", "log_closed path='%ls'", logFilePath.c_str());
	logFile = nullptr;
	logFilePath.clear();
	logFileIsOpen = false;
	failedToOpenLogFile = false;
	failedToOpenLogFileAtMs = 0;
	logWriteFailed = false;
	logFlushFailed = false;
	SetLogStatus("closed");
}
} // namespace Metrics
