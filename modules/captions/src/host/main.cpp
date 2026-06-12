#define _CRT_SECURE_NO_DEPRECATE
// WIN32_LEAN_AND_MEAN is set by the CMakeLists compile-definitions; no
// local re-define here to avoid the macro-redefinition warning.
#include <windows.h>
#include <shlobj.h>
#include <objbase.h>
#include <delayimp.h>

#include "ActionBindings.h"
#include "Captions.h"
#include "CaptionsChatboxPacing.h"
#include "CaptionsAudioInputFile.h"
#include "CaptionsOutputPolicy.h"
#include "CaptionsRealtimeFlags.h"
#include "CaptionsSpeechModels.h"
#include "CaptionsThreadPlan.h"
#include "ChatboxText.h"
#include "ChatboxPacer.h"
#include "AudioLevel.h"
#include "EnergySpeechGate.h"
#include "HostStatus.h"
#include "Logging.h"
#include "ModelDownloader.h"
#include "RouterPublisher.h"
#include "SileroVad.h"
#include "SidecarOwnerLease.h"
#include "TranscriptText.h"
#include "WasapiCapture.h"
#include "WhisperEngine.h"
#include "WhisperPromptHistory.h"
#include "Win32Paths.h"
#include "Win32Text.h"

#include <openvr.h>

#if defined(WKOPENVR_CAPTIONS_VULKAN_ENABLED)
#include <vulkan/vulkan.h>
#endif

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <deque>
#include <cstdio>
#include <cstring>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

// ---------------------------------------------------------------------------
// Early-crash helpers: write a one-shot crash record before the logger is up.
// Uses raw Win32 only -- no C runtime, no statics, no malloc.
// ---------------------------------------------------------------------------

static void WriteEarlyCrashLog(const char* msg)
{
	std::wstring path = openvr_pair::common::WkOpenVrLogsPath(true);
	if (path.empty()) return;

	DWORD pid = GetCurrentProcessId();
	std::wstring fname = path + L"\\captions_host_crash_" + std::to_wstring((unsigned long)pid) + L".txt";

	HANDLE h = CreateFileW(fname.c_str(), GENERIC_WRITE, FILE_SHARE_READ, nullptr, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL,
	                       nullptr);
	if (h == INVALID_HANDLE_VALUE) return;

	DWORD written = 0;
	WriteFile(h, msg, (DWORD)strlen(msg), &written, nullptr);
	CloseHandle(h);
}

// ---------------------------------------------------------------------------
// Delay-load failure hook: called when DelayLoadHelper2 cannot resolve a DLL.
// Runs before any C++ object constructors for the host's own code, so we must
// not call into any initialised state -- only Win32 + WriteEarlyCrashLog.
//
// Exit codes in range 0xCEE0DC00-0xCEE0DCFF are reserved for delay-load
// failures; the low byte carries the low byte of GetLastError at the time of
// failure (e.g. 126 = ERROR_MOD_NOT_FOUND). The supervisor decodes this range.
// ---------------------------------------------------------------------------

static FARPROC WINAPI DelayLoadFailureHook(unsigned dliNotify, PDelayLoadInfo pdli)
{
	if (dliNotify == dliFailLoadLib) {
		const char* dll = (pdli && pdli->szDll) ? pdli->szDll : "<unknown>";
		DWORD err = pdli ? pdli->dwLastError : GetLastError();
		char msg[768];
		_snprintf_s(msg, sizeof msg,
		            "[captions-host] FATAL: delay-load failed for '%s' (err=%lu)\n"
		            "  Cause: an optional captions runtime DLL is missing from the per-user\n"
		            "  captions runtime folder or from PATH. Install the relevant Captions\n"
		            "  pack from the overlay, then restart the host.\n",
		            dll, (unsigned long)err);
		WriteEarlyCrashLog(msg);
		ExitProcess(0xCEE0DC00 | (err & 0xFF));
	}
	return nullptr;
}

// The MSVC linker resolves this symbol from delayimp.lib. Declaring it
// extern "C" with the exact name overrides the default null hook.
extern "C" const PfnDliHook __pfnDliFailureHook2 = DelayLoadFailureHook;

// ---------------------------------------------------------------------------
// Top-level SEH filter: catches crashes that occur after the loader but before
// or during main() -- e.g., an access violation in whisper's CUDA init path.
// ---------------------------------------------------------------------------

static LONG WINAPI TopLevelSehFilter(EXCEPTION_POINTERS* ep)
{
	if (!ep) return EXCEPTION_EXECUTE_HANDLER;
	DWORD code = ep->ExceptionRecord ? ep->ExceptionRecord->ExceptionCode : 0;
	void* addr = ep->ExceptionRecord ? ep->ExceptionRecord->ExceptionAddress : nullptr;
	char msg[512];
	_snprintf_s(msg, sizeof msg,
	            "[captions-host] FATAL SEH: code=0x%08lX addr=%p\n"
	            "  This is likely a crash inside a native inference library (whisper/ORT/CT2).\n"
	            "  Check the captions driver log for 'dll-probe' lines to identify missing DLLs.\n",
	            (unsigned long)code, addr);
	WriteEarlyCrashLog(msg);
	return EXCEPTION_EXECUTE_HANDLER;
}

// ---------------------------------------------------------------------------
// DLL probe helper: LoadLibraryW + immediate FreeLibrary to verify presence.
// Logs to the host log (call after CaptionsHostOpenLogFile).
// ---------------------------------------------------------------------------

static void ProbeDll(const wchar_t* dll_name)
{
	HMODULE h = LoadLibraryW(dll_name);
	if (h) {
		wchar_t fullpath[MAX_PATH] = {};
		GetModuleFileNameW(h, fullpath, MAX_PATH);
		FreeLibrary(h);
		TH_LOG("[captions-host] dll-probe: %ls -> FOUND at %ls", dll_name,
		       fullpath[0] ? fullpath : L"(path unavailable)");
	}
	else {
		DWORD err = GetLastError();
		TH_LOG("[captions-host] dll-probe: %ls -> MISSING (err=%lu)", dll_name, (unsigned long)err);
	}
}

// ---------------------------------------------------------------------------
// Crash-proof Vulkan device probe.
//
// ggml-vulkan v1.7.4 enumerates devices the moment whisper requests GPU, and on
// that path it THROWS when no Vulkan loader/ICD is present and calls
// GGML_ABORT() -- terminating the process -- when the loader is present but
// reports zero physical devices. So we must NOT blindly hand whisper use_gpu=true.
// This probe makes the decision first with the plain Vulkan C API (which only
// returns VkResult codes, never throws) under an SEH guard (broken drivers can
// still raise access violations). whisper is only allowed near the GPU when this
// returns > 0.
// ---------------------------------------------------------------------------

#if defined(WKOPENVR_CAPTIONS_VULKAN_ENABLED)
static int SafeVulkanDeviceCountSeh()
{
	// No C++ objects with destructors in this function: SEH __try requires it.
	int count = 0;
	__try {
		VkApplicationInfo app{};
		app.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
		app.pApplicationName = "WKOpenVR.CaptionsHost";
		app.apiVersion = VK_API_VERSION_1_0;

		VkInstanceCreateInfo ci{};
		ci.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
		ci.pApplicationInfo = &app;

		VkInstance instance = VK_NULL_HANDLE;
		if (vkCreateInstance(&ci, nullptr, &instance) != VK_SUCCESS || instance == VK_NULL_HANDLE) {
			return 0;
		}
		uint32_t n = 0;
		VkResult r = vkEnumeratePhysicalDevices(instance, &n, nullptr);
		if (r != VK_SUCCESS && r != VK_INCOMPLETE) n = 0;
		count = static_cast<int>(n);
		vkDestroyInstance(instance, nullptr);
	}
	__except (EXCEPTION_EXECUTE_HANDLER) {
		count = 0;
	}
	return count;
}
#endif

// Number of usable Vulkan GPUs, or 0 when the backend is not compiled in or no
// safe device is found. Never throws, never aborts.
static int SafeVulkanDeviceCount()
{
#if defined(WKOPENVR_CAPTIONS_VULKAN_ENABLED)
	return SafeVulkanDeviceCountSeh();
#else
	return 0;
#endif
}

static bool FileExistsA(const std::string& path);

static std::string HostFilePath(const char* filename)
{
	char buf[MAX_PATH] = {};
	GetModuleFileNameA(nullptr, buf, MAX_PATH);
	std::string path(buf);
	size_t sep = path.find_last_of("/\\");
	if (sep != std::string::npos)
		path.resize(sep + 1);
	else
		path.clear();
	path += filename;
	return path;
}

static void RegisterCaptionsManifest()
{
	auto* apps = vr::VRApplications();
	if (!apps) {
		TH_LOG("[actions] VRApplications unavailable; captions app manifest not registered");
		return;
	}

	// Remove the legacy key from SteamVR's vrappconfig on upgrade.
	const char* legacyKey = "wk.wkopenvr.translator";
	if (apps->IsApplicationInstalled(legacyKey)) {
		std::string legacyManifest = HostFilePath("translator.vrmanifest");
		if (FileExistsA(legacyManifest)) {
			vr::EVRApplicationError rmErr = apps->RemoveApplicationManifest(legacyManifest.c_str());
			TH_LOG("[actions] RemoveApplicationManifest (legacy translator) -> %d", (int)rmErr);
		}
		else {
			// Manifest file already gone; unregister by key directly if API supports it.
			// VRApplications has no RemoveApplicationManifestByKey; log and continue.
			TH_LOG("[actions] legacy translator manifest file not present; stale vrappconfig entry may persist");
		}
	}

	const char* appKey = "wk.wkopenvr.captions";
	std::string manifest = HostFilePath("captions.vrmanifest");
	if (!FileExistsA(manifest)) {
		TH_LOG("[actions] captions app manifest missing: %s", manifest.c_str());
		return;
	}

	if (!apps->IsApplicationInstalled(appKey)) {
		vr::EVRApplicationError err = apps->AddApplicationManifest(manifest.c_str());
		TH_LOG("[actions] AddApplicationManifest(%s) -> %d", manifest.c_str(), (int)err);
	}
	else {
		TH_LOG("[actions] captions app manifest already installed");
	}
}

static bool FileExistsA(const std::string& path)
{
	if (path.empty()) return false;
	DWORD attr = GetFileAttributesA(path.c_str());
	return attr != INVALID_FILE_ATTRIBUTES && (attr & FILE_ATTRIBUTE_DIRECTORY) == 0;
}

static bool DirectoryExistsA(const std::string& path)
{
	if (path.empty()) return false;
	DWORD attr = GetFileAttributesA(path.c_str());
	return attr != INVALID_FILE_ATTRIBUTES && (attr & FILE_ATTRIBUTE_DIRECTORY) != 0;
}

static long long SamplesToAudioMs(size_t samples)
{
	return static_cast<long long>((samples * 1000ULL) / 16000ULL);
}

static size_t TotalAudioSamples(const std::vector<std::vector<float>>& frames)
{
	size_t total = 0;
	for (const auto& frame : frames) {
		total += frame.size();
	}
	return total;
}

static void EnsureDirectoryTreeA(const std::string& path)
{
	if (path.empty()) return;
	std::string current;
	current.reserve(path.size());
	for (size_t i = 0; i < path.size(); ++i) {
		current.push_back(path[i]);
		const bool separator = path[i] == '\\' || path[i] == '/';
		if (!separator && i + 1 != path.size()) continue;
		if (current.size() <= 3) continue; // skip "C:\" prefix
		while (!current.empty() && (current.back() == '\\' || current.back() == '/')) {
			current.pop_back();
		}
		if (!current.empty()) CreateDirectoryA(current.c_str(), nullptr);
		if (separator) current.push_back('\\');
	}
}

static std::string CaptionsDataDir()
{
	std::wstring root = openvr_pair::common::WkOpenVrSubdirectoryPath(L"captions", true);
	return openvr_pair::common::WideToUtf8(root);
}

static std::string CaptionsRuntimeDir()
{
	std::string dir = CaptionsDataDir();
	if (dir.empty()) return {};
	return dir + "\\runtime";
}

static void AddCaptionsRuntimeSearchPath()
{
	std::string dir = CaptionsRuntimeDir();
	if (dir.empty()) return;
	EnsureDirectoryTreeA(dir);
	SetDllDirectoryA(dir.c_str());
	TH_LOG("[captions-host] runtime DLL directory: %s", dir.c_str());
}

// ---------------------------------------------------------------------------
// Config (updated at runtime via the host control pipe)
// ---------------------------------------------------------------------------

struct HostConfig
{
	std::string source_lang = "auto";
	std::string target_lang = ""; // empty = transcribe only
	std::string chatbox_address = "/chatbox/input";
	uint16_t chatbox_port = 9000;
	bool chatbox_enabled = false;
	bool notify_sound = false;
	int chatbox_split_delay_ms = captions::kCaptionsChatboxSplitDelayDefaultMs;
	bool transcript_logging = false;
	int mode = 0; // 0=PTT, 1=always-on
	uint8_t realtime_flags = captions::kCaptionsRealtimeDefaultFlags;
	uint8_t speech_model = captions::kCaptionsSpeechModelBalanced;
	bool prefer_gpu = true; // request GPU decode when a usable device exists; --no-gpu forces CPU

	// Paths: resolved once at startup. Overrideable via command-line.
	std::string whisper_model_path;
	bool whisper_model_path_overridden = false;
	std::string silero_model_path;

	bool RealtimeEnabled(uint8_t flag) const { return captions::CaptionsRealtimeFlagEnabled(realtime_flags, flag); }
};

struct OwnerLivenessConfig
{
	std::string name;
	uint64_t nonce = 0;
	bool Configured() const { return !name.empty() && nonce != 0; }
};

static std::mutex g_config_mutex;
static HostConfig g_config;

// ---------------------------------------------------------------------------
// Control pipe listener (receives config updates from the driver)
// ---------------------------------------------------------------------------

#define HOST_CONTROL_PIPE_NAME "\\\\.\\pipe\\WKOpenVR-Captions.host"

static std::atomic<bool> g_shutdown{false};
static std::string g_control_pipe_name = HOST_CONTROL_PIPE_NAME;

static bool TryParseInt(const std::string& text, int& value)
{
	try {
		size_t consumed = 0;
		int parsed = std::stoi(text, &consumed);
		if (consumed != text.size()) return false;
		value = parsed;
		return true;
	}
	catch (...) {
		return false;
	}
}

static void DispatchControlMessage(char* buf, DWORD got)
{
	buf[got] = '\0'; // caller guarantees buf has room for one extra byte
	std::string msg(buf, got);
	TH_LOG("[control] received: %s", msg.c_str());
	if (msg == "shutdown" || msg == "shutdown\n" || msg == "shutdown\r\n") {
		TH_LOG("[control] shutdown requested");
		g_shutdown.store(true, std::memory_order_release);
		return;
	}
	if (msg.rfind("config:", 0) != 0) return;

	std::lock_guard<std::mutex> lk(g_config_mutex);
	std::string body = msg.substr(7);
	size_t pos = 0;
	while (pos < body.size()) {
		size_t eq = body.find('=', pos);
		if (eq == std::string::npos) break;
		size_t comma = body.find(',', eq);
		std::string key = body.substr(pos, eq - pos);
		std::string val = body.substr(eq + 1, comma == std::string::npos ? std::string::npos : comma - eq - 1);
		while (!val.empty() && (val.back() == '\n' || val.back() == '\r'))
			val.pop_back();

		if (key == "src")
			g_config.source_lang = val;
		else if (key == "tgt")
			g_config.target_lang = val;
		else if (key == "mode") {
			int parsed = 0;
			if (TryParseInt(val, parsed)) {
				if (parsed < 0) parsed = 0;
				if (parsed > 1) parsed = 1;
				g_config.mode = parsed;
			}
		}
		else if (key == "addr")
			g_config.chatbox_address = val;
		else if (key == "port") {
			int parsed = 0;
			if (TryParseInt(val, parsed) && parsed >= 0 && parsed <= 65535) {
				g_config.chatbox_port = (uint16_t)parsed;
			}
		}
		else if (key == "notify")
			g_config.notify_sound = (val != "0");
		else if (key == "chatbox")
			g_config.chatbox_enabled = (val != "0");
		else if (key == "splitdelay") {
			int parsed = 0;
			if (TryParseInt(val, parsed)) {
				g_config.chatbox_split_delay_ms = captions::NormalizeCaptionsChatboxSplitDelayMs(parsed);
			}
		}
		else if (key == "log")
			g_config.transcript_logging = (val != "0");
		else if (key == "flags") {
			int parsed = 0;
			if (TryParseInt(val, parsed)) {
				if (parsed < 0) parsed = 0;
				if (parsed > 255) parsed = 255;
				g_config.realtime_flags = static_cast<uint8_t>(parsed);
			}
		}
		else if (key == "model") {
			int parsed = 0;
			if (TryParseInt(val, parsed)) {
				g_config.speech_model = captions::NormalizeCaptionsSpeechModel(parsed);
			}
		}

		if (comma == std::string::npos) break;
		pos = comma + 1;
	}
}

static void ControlPipeThread()
{
	// Create the pipe server once. Driver only sends one config message per
	// connection; after each message we disconnect and wait for the next
	// client without destroying the server handle -- this avoids the race
	// window where a second host process could steal the server slot.
	HANDLE pipe = CreateNamedPipeA(g_control_pipe_name.c_str(), PIPE_ACCESS_INBOUND,
	                               PIPE_TYPE_BYTE | PIPE_READMODE_BYTE | PIPE_WAIT,
	                               2, // max 2 instances: tolerates brief driver reconnect overlap
	                               0, 4096, 1000, nullptr);

	if (pipe == INVALID_HANDLE_VALUE) {
		DWORD err = GetLastError();
		if (err == ERROR_PIPE_BUSY) {
			// Another host process already owns the pipe; exit so the
			// supervisor surfaces the conflict rather than silently spinning.
			TH_LOG("[control] FATAL: pipe already owned by another host (ERROR_PIPE_BUSY); exiting (code 4)");
			CaptionsHostFlushLog();
			ExitProcess(4);
		}
		TH_LOG("[control] CreateNamedPipeA failed err=%lu; control pipe unavailable", (unsigned long)err);
		return;
	}

	char buf[4097] = {};
	while (!g_shutdown.load(std::memory_order_acquire)) {
		BOOL connected = ConnectNamedPipe(pipe, nullptr);
		if (!connected && GetLastError() != ERROR_PIPE_CONNECTED) {
			if (g_shutdown.load(std::memory_order_acquire)) break;
			Sleep(50);
			continue;
		}

		DWORD got = 0;
		if (ReadFile(pipe, buf, sizeof(buf) - 1, &got, nullptr) && got > 0) {
			DispatchControlMessage(buf, got);
		}

		DisconnectNamedPipe(pipe);
	}

	CloseHandle(pipe);
}

static void WakeControlPipe()
{
	HANDLE h = CreateFileA(g_control_pipe_name.c_str(), GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL,
	                       nullptr);
	if (h != INVALID_HANDLE_VALUE) {
		CloseHandle(h);
	}
}

static void OwnerLivenessThread(OwnerLivenessConfig config)
{
	if (!config.Configured()) return;

	namespace owner = openvr_pair::common::sidecar_owner;
	owner::LeaseReader reader;
	if (!reader.Open(config.name)) {
		TH_LOG("[owner-liveness] failed to open lease '%s'; shutting down", config.name.c_str());
		g_shutdown.store(true, std::memory_order_release);
		return;
	}

	TH_LOG("[owner-liveness] watching lease '%s'", config.name.c_str());
	while (!g_shutdown.load(std::memory_order_acquire)) {
		owner::LeaseSnapshot snapshot{};
		owner::WatchdogStatus status = owner::WatchdogStatus::Missing;
		if (reader.TryRead(snapshot)) {
			status = owner::EvaluateLeaseSnapshot(snapshot, openvr_pair::common::modules::ModuleId::Captions,
			                                      config.nonce, owner::MonotonicMillis(), 3000);
		}
		if (status != owner::WatchdogStatus::Alive) {
			TH_LOG("[owner-liveness] owner lease no longer alive: %s; shutting down",
			       owner::WatchdogStatusName(status));
			g_shutdown.store(true, std::memory_order_release);
			break;
		}
		for (int i = 0; i < 25 && !g_shutdown.load(std::memory_order_acquire); ++i) {
			Sleep(10);
		}
	}
}

// ---------------------------------------------------------------------------
// Default model paths
// ---------------------------------------------------------------------------

static std::string DefaultWhisperModelPath(uint8_t speech_model = captions::kCaptionsSpeechModelBalanced)
{
	std::string dir = ModelDownloader::DefaultModelDir();
	if (dir.empty()) return {};
	return dir + "\\" + captions::CaptionsSpeechModelFileName(speech_model);
}

static std::string RequiredWhisperModelPath(const HostConfig& cfg)
{
	return cfg.whisper_model_path_overridden ? cfg.whisper_model_path : DefaultWhisperModelPath(cfg.speech_model);
}

static std::string ActiveWhisperModelPath(const HostConfig& cfg)
{
	if (cfg.whisper_model_path_overridden) return cfg.whisper_model_path;

	std::string selected = DefaultWhisperModelPath(cfg.speech_model);
	if (FileExistsA(selected)) return selected;

	if (cfg.speech_model == captions::kCaptionsSpeechModelHighAccuracy) {
		const std::string fallback = DefaultWhisperModelPath(captions::kCaptionsSpeechModelBalanced);
		if (FileExistsA(fallback)) return fallback;
	}

	return selected;
}

static std::string DefaultSileroModelPath()
{
	std::string dir = ModelDownloader::DefaultModelDir();
	if (dir.empty()) return {};
	return dir + "\\silero_vad.onnx";
}

static std::string TranslationPair(const std::string& src, const std::string& tgt)
{
	if (tgt.empty()) return {};
	return (src == "auto" || src.empty() ? "en" : src) + "-" + tgt;
}

static std::string ResolveCaptionsModelDir(const std::string& src, const std::string& tgt)
{
	std::string dir = ModelDownloader::DefaultModelDir();
	std::string pair = TranslationPair(src, tgt);
	if (dir.empty() || pair.empty()) return {};
	return dir + "\\ct2-opus-mt-" + pair;
}

static void UpdatePackStatus(HostStatus& status, const HostConfig& cfg)
{
	const bool whisper_model = FileExistsA(RequiredWhisperModelPath(cfg));
	const bool vad_model = FileExistsA(cfg.silero_model_path);
	const bool speech_pack = whisper_model && vad_model;
	const bool vad_runtime = SileroVad::RuntimeAvailable();
	const bool translation_runtime = Captions::RuntimeAvailable();

	const std::string pair = TranslationPair(cfg.source_lang, cfg.target_lang);
	const std::string tr_model_dir = ResolveCaptionsModelDir(cfg.source_lang, cfg.target_lang);
	const bool tr_pack = pair.empty() ? true : DirectoryExistsA(tr_model_dir);

	status.SetSpeechPackInstalled(speech_pack);
	status.SetVadRuntimeAvailable(vad_runtime);
	status.SetTranslationRuntimeAvailable(translation_runtime);
	status.SetTranslationPackInstalled(tr_pack);
	status.SetActiveTranslationPair(pair);

	std::ostringstream err;
	if (!whisper_model) {
		if (cfg.speech_model == captions::kCaptionsSpeechModelHighAccuracy && !cfg.whisper_model_path_overridden)
			err << "High accuracy speech model not installed.";
		else
			err << "Whisper model not installed.";
	}
	else if (!vad_model) {
		err << "Speech VAD model not installed.";
	}
	else if (!vad_runtime) {
		err << "Speech detection runtime not installed.";
	}
	else if (!pair.empty() && !translation_runtime) {
		err << "Translation runtime not installed.";
	}
	else if (!pair.empty() && !tr_pack) {
		err << "Translation pack " << pair << " not installed.";
	}
	status.SetLastError(err.str());
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

// Derive a per-user discriminator from %USERNAME% (ASCII; safe for mutex names).
static std::wstring GetUserSidString()
{
	wchar_t user[256] = {};
	DWORD len = 256;
	if (!GetUserNameW(user, &len) || len == 0) return L"unknown";
	// Truncate any trailing null GetUserNameW leaves inside len.
	return std::wstring(user);
}

// Singleton mutex held for process lifetime; prevents two CaptionsHost
// processes from coexisting under the same user session.
static HANDLE g_singletonMutex = nullptr;

int main(int argc, char** argv)
try {
	// Install SEH filter first -- catches crashes during early C++ init.
	SetUnhandledExceptionFilter(TopLevelSehFilter);

	CaptionsHostOpenLogFile();
	TH_LOG("[startup] phase=logger-open");
	TH_LOG("[main] WKOpenVR.CaptionsHost starting");

	AddCaptionsRuntimeSearchPath();

	// The host is a background sidecar. Run it below normal priority so a CPU
	// speech decode (which spins up several whisper worker threads) can never
	// starve the Windows desktop compositor or the overlay render thread. This
	// is the safety net that keeps the desktop smooth even on the CPU-fallback
	// path; the GPU path is barely affected since the GPU does the work.
	if (SetPriorityClass(GetCurrentProcess(), BELOW_NORMAL_PRIORITY_CLASS)) {
		TH_LOG("[startup] process priority = BELOW_NORMAL");
	}
	else {
		TH_LOG("[startup] SetPriorityClass(BELOW_NORMAL) failed err=%lu", (unsigned long)GetLastError());
	}

	// Probe native dependencies before any inference library call.
	// These log lines are the first thing to read when the host fails to start.
	TH_LOG("[captions-host] startup-phase=dll-probe");
#if defined(WKOPENVR_CAPTIONS_CUDA_ENABLED)
	ProbeDll(L"cudart64_13.dll");
	ProbeDll(L"cublas64_13.dll");
	ProbeDll(L"cublasLt64_13.dll");
	ProbeDll(L"nvcudart_hybrid64.dll");
	ProbeDll(L"nvcuda.dll");
#else
	TH_LOG("[captions-host] dll-probe: CUDA backend disabled in this build");
#endif
#if defined(WKOPENVR_CAPTIONS_VULKAN_ENABLED)
	ProbeDll(L"vulkan-1.dll");
#else
	TH_LOG("[captions-host] dll-probe: Vulkan backend disabled in this build");
#endif
	ProbeDll(L"onnxruntime.dll");
	ProbeDll(L"ctranslate2.dll");
	TH_LOG("[captions-host] startup-phase=dll-probe-done");

	bool self_test = false;
	std::string rejected_live_publish_arg;
	std::wstring status_path_override;
	std::wstring e2e_singleton_suffix;
	OwnerLivenessConfig owner_liveness;

	// Parse optional command-line overrides: --model <path> --silero <path>
	{
		std::lock_guard<std::mutex> lk(g_config_mutex);
		g_config.whisper_model_path = DefaultWhisperModelPath();
		g_config.whisper_model_path_overridden = false;
		g_config.silero_model_path = DefaultSileroModelPath();
		for (int i = 1; i < argc; ++i) {
			if (strcmp(argv[i], "--self-test") == 0 || strcmp(argv[i], "--healthcheck") == 0) {
				self_test = true;
			}
			else if (strcmp(argv[i], "--e2e-fake-chatbox") == 0 || strcmp(argv[i], "--e2e-fake-captions-output") == 0 ||
			         strcmp(argv[i], "--e2e-fake-translator-output") == 0) {
				rejected_live_publish_arg = argv[i];
				if (i + 1 < argc) ++i;
			}
			else if (i + 1 < argc && strcmp(argv[i], "--status-file") == 0) {
				status_path_override = openvr_pair::common::Utf8ToWide(argv[i + 1]);
				++i;
			}
			else if (i + 1 < argc && strcmp(argv[i], "--e2e-singleton-suffix") == 0) {
				e2e_singleton_suffix = openvr_pair::common::Utf8ToWide(argv[i + 1]);
				++i;
			}
			else if (i + 1 < argc && strcmp(argv[i], "--e2e-control-pipe") == 0) {
				g_control_pipe_name = argv[i + 1];
				++i;
			}
			else if (i + 1 < argc && strcmp(argv[i], "--owner-liveness") == 0) {
				owner_liveness.name = argv[i + 1];
				++i;
			}
			else if (i + 1 < argc && strcmp(argv[i], "--owner-liveness-nonce") == 0) {
				owner_liveness.nonce = _strtoui64(argv[i + 1], nullptr, 16);
				++i;
			}
			else if (i + 1 < argc && strcmp(argv[i], "--model") == 0) {
				g_config.whisper_model_path = argv[i + 1];
				g_config.whisper_model_path_overridden = true;
				++i;
			}
			else if (i + 1 < argc && strcmp(argv[i], "--silero") == 0) {
				g_config.silero_model_path = argv[i + 1];
				++i;
			}
			else if (strcmp(argv[i], "--no-gpu") == 0 || strcmp(argv[i], "--force-cpu") == 0) {
				g_config.prefer_gpu = false;
			}
		}
	}

	// Decide the whisper compute backend once, up front, with the crash-proof
	// Vulkan probe. GPU is requested only when a usable device exists and the
	// user has not forced CPU via --no-gpu.
	const int vulkan_device_count = SafeVulkanDeviceCount();
	const int whisper_threads = captions::ResolveWhisperThreadCount();
	bool whisper_use_gpu;
	{
		std::lock_guard<std::mutex> lk(g_config_mutex);
		whisper_use_gpu = (vulkan_device_count > 0) && g_config.prefer_gpu;
	}
	TH_LOG("[startup] compute backend=%s vulkan_devices=%d threads=%d", whisper_use_gpu ? "Vulkan" : "CPU",
	       vulkan_device_count, whisper_threads);

	HostStatus status(status_path_override);
	status.SetComputeBackend(whisper_use_gpu ? "Vulkan" : "CPU", whisper_use_gpu);
	status.SetPhase("config-loaded");
	{
		std::lock_guard<std::mutex> lk(g_config_mutex);
		UpdatePackStatus(status, g_config);
	}
	status.SetState(HostStatus::State::Idle);
	status.Flush();

	if (!rejected_live_publish_arg.empty()) {
		status.SetState(HostStatus::State::Error);
		status.SetPhase("unsupported-argument");
		status.SetLastError("Live OSC fake publish command is disabled.");
		status.Flush();
		TH_LOG("[startup] rejected removed live OSC fake publish argument: %s", rejected_live_publish_arg.c_str());
		CaptionsHostFlushLog();
		return 2;
	}

	if (self_test) {
		status.SetPhase("self-test-complete");
		status.Flush();
		TH_LOG("[startup] self-test complete compute=%s vulkan_devices=%d threads=%d",
		       whisper_use_gpu ? "Vulkan" : "CPU", vulkan_device_count, whisper_threads);
		CaptionsHostFlushLog();
		return 0;
	}

	// Layer 1: acquire system-wide singleton mutex before opening any IPC.
	{
		std::wstring user = GetUserSidString();
		wchar_t mname[512] = {};
		if (e2e_singleton_suffix.empty()) {
			swprintf_s(mname, L"Global\\WKOpenVR-CaptionsHost-Singleton-%ls", user.c_str());
		}
		else {
			swprintf_s(mname, L"Global\\WKOpenVR-CaptionsHost-Singleton-%ls-%ls", user.c_str(),
			           e2e_singleton_suffix.c_str());
		}

		g_singletonMutex = CreateMutexW(nullptr, TRUE, mname);
		DWORD merr = GetLastError();
		if (!g_singletonMutex) {
			status.SetPhase("singleton-failed");
			status.SetLastError("Captions singleton mutex could not be created.");
			status.Flush();
			TH_LOG("[singleton] CreateMutexW failed err=%lu; exiting", (unsigned long)merr);
			CaptionsHostFlushLog();
			return 1;
		}
		if (merr == ERROR_ALREADY_EXISTS) {
			// Another instance already holds the mutex; exit cleanly.
			status.SetPhase("singleton-owned");
			status.SetLastError("Another captions host is already running.");
			status.Flush();
			TH_LOG("[singleton] another host already owns mutex; exiting cleanly (code 3)");
			CaptionsHostFlushLog();
			CloseHandle(g_singletonMutex);
			return 3;
		}
		TH_LOG("[singleton] acquired mutex '%ls'; proceeding as sole instance", mname);
	}
	TH_LOG("[startup] phase=singleton-acquired");
	status.SetPhase("singleton-acquired");
	status.Flush();
	TH_LOG("[startup] owner-liveness configured=%d name=%s", owner_liveness.Configured() ? 1 : 0,
	       owner_liveness.name.empty() ? "(none)" : owner_liveness.name.c_str());

	TH_LOG("[startup] phase=opening-control-pipe");
	status.SetPhase("opening-control-pipe");
	status.Flush();
	std::thread owner_thread;
	if (owner_liveness.Configured()) {
		owner_thread = std::thread(OwnerLivenessThread, owner_liveness);
	}
	// Start control pipe thread.
	std::thread ctrl_thread(ControlPipeThread);

	// Initialise OpenVR with a short retry loop. The host can launch slightly
	// ahead of SteamVR's IPC being ready, in which case the very first
	// VR_Init returns VRInitError_Init_HmdNotFoundPresenceFailed (121) and
	// PTT was silently unavailable for the rest of the session. Retry every
	// 2 s for up to 30 s; bail out immediately on shutdown.
	bool vr_ok = false;
	{
		constexpr int kMaxAttempts = 15;
		constexpr auto kAttemptInterval = std::chrono::seconds(2);
		vr::EVRInitError vr_err = vr::VRInitError_None;
		for (int attempt = 1; attempt <= kMaxAttempts; ++attempt) {
			vr_err = vr::VRInitError_None;
			vr::VR_Init(&vr_err, vr::VRApplication_Background);
			if (vr_err == vr::VRInitError_None) {
				vr_ok = true;
				if (attempt > 1) {
					TH_LOG("[main] VR_Init succeeded on attempt %d/%d", attempt, kMaxAttempts);
				}
				break;
			}
			TH_LOG("[main] VR_Init attempt %d/%d failed (%d: %s); will retry in %ds", attempt, kMaxAttempts,
			       (int)vr_err, vr::VR_GetVRInitErrorAsSymbol(vr_err),
			       (int)std::chrono::duration_cast<std::chrono::seconds>(kAttemptInterval).count());
			if (g_shutdown.load(std::memory_order_acquire)) break;
			// Sleep in 100 ms slices so a shutdown signal cuts the wait short.
			const auto deadline = std::chrono::steady_clock::now() + kAttemptInterval;
			while (std::chrono::steady_clock::now() < deadline) {
				if (g_shutdown.load(std::memory_order_acquire)) break;
				std::this_thread::sleep_for(std::chrono::milliseconds(100));
			}
			if (g_shutdown.load(std::memory_order_acquire)) break;
		}
		if (!vr_ok) {
			TH_LOG("[main] VR_Init gave up after %d attempts (last err=%d: %s); "
			       "PTT will be unavailable for this session",
			       kMaxAttempts, (int)vr_err, vr::VR_GetVRInitErrorAsSymbol(vr_err));
		}
	}
	status.SetPttStatus(vr_ok, false, "", vr_ok ? "" : "VR_Init failed; push-to-talk unavailable.");
	if (vr_ok) RegisterCaptionsManifest();

	TH_LOG("[startup] phase=initializing-vad");
	status.SetPhase("initializing-vad");
	status.Flush();
	// Load Silero VAD.
	std::unique_ptr<SileroVad> vad;
	auto next_vad_load_attempt = std::chrono::steady_clock::time_point{};
	{
		std::string silero_path;
		{
			std::lock_guard<std::mutex> lk(g_config_mutex);
			silero_path = g_config.silero_model_path;
		}
		if (SileroVad::RuntimeAvailable() && FileExistsA(silero_path)) {
			vad = std::make_unique<SileroVad>();
			if (!vad->Load(silero_path)) {
				vad.reset();
				next_vad_load_attempt = std::chrono::steady_clock::now() + std::chrono::seconds(5);
				TH_LOG("[main] Silero VAD load failed; path='%s'", silero_path.c_str());
			}
		}
		else if (!FileExistsA(silero_path)) {
			TH_LOG("[main] Silero VAD model missing; path='%s'", silero_path.c_str());
		}
		else {
			TH_LOG("[main] Silero VAD runtime missing; install the speech pack to enable always-on speech detection");
		}
	}

	TH_LOG("[startup] phase=initializing-translation");
	status.SetPhase("initializing-translation");
	status.Flush();
	// Load Whisper.
	WhisperEngine whisper;
	captions::WhisperPromptHistory whisper_prompt(192);
	std::string whisper_prompt_lang_key;
	std::string loaded_whisper_model_path;
	bool whisper_load_attempted = false;
	bool whisper_load_failed = false;
	auto next_whisper_load_attempt = std::chrono::steady_clock::time_point{};
	{
		std::string model_path;
		{
			std::lock_guard<std::mutex> lk(g_config_mutex);
			model_path = ActiveWhisperModelPath(g_config);
		}
		if (FileExistsA(model_path)) {
			whisper_load_attempted = true;
			if (!whisper.Load(model_path, whisper_threads, whisper_use_gpu)) {
				whisper_load_failed = true;
				next_whisper_load_attempt = std::chrono::steady_clock::now() + std::chrono::seconds(5);
				TH_LOG("[main] Whisper model load failed; path='%s'", model_path.c_str());
			}
			else {
				loaded_whisper_model_path = model_path;
				status.SetComputeBackend(whisper.BackendInfo().device_name, whisper.BackendInfo().gpu_active);
			}
		}
		else {
			TH_LOG("[main] Whisper model missing; path='%s'", model_path.c_str());
		}
	}

	// Translation model loaded on demand when target_lang changes.
	Captions captions_engine;
	std::string loaded_src_lang;
	std::string loaded_tgt_lang;
	std::string loaded_model_dir;
	bool translation_load_failed = false;
	auto next_translation_load_attempt = std::chrono::steady_clock::time_point{};

	// Action bindings for PTT.
	ActionBindings actions;
	if (vr_ok) {
		std::string manifest = ActionBindings::ResolveManifestPath();
		if (!actions.Register(manifest)) {
			TH_LOG("[main] PTT action binding failed; push-to-talk unavailable");
		}
		status.SetPttStatus(true, actions.IsRegistered(), actions.ApplicationKey(), actions.LastError());
		status.Flush();
	}

	// Router publisher.
	RouterPublisher publisher;
	bool typing_indicator_active = false;
	auto set_typing_indicator = [&](bool active, const HostConfig& current_cfg) {
		if (!current_cfg.chatbox_enabled || !current_cfg.RealtimeEnabled(captions::kCaptionsRealtimeTypingIndicator)) {
			active = false;
		}
		if (typing_indicator_active == active) return;

		if (!publisher.PublishTyping(active)) {
			status.SetLastError("OSC router publish pipe unavailable.");
		}
		typing_indicator_active = active;
	};

	// Chatbox pacer.
	ChatboxPacer pacer(captions::kCaptionsChatboxSplitDelayDefaultMs / 1000.0);

	// VAD state machine.
	int silence_count = 0;
	bool in_speech = false;
	bool ptt_was_held = false;
	size_t speech_samples_since_open = 0;
	size_t speech_evidence_samples_since_open = 0;
	size_t speech_frames_since_open = 0;
	size_t possible_frames_since_open = 0;
	size_t total_frames_since_open = 0;
	size_t speech_feature_frames_since_open = 0;
	double speech_sum_acoustic_artifact_risk = 0.0;
	double speech_sum_speech_band_ratio = 0.0;
	double speech_sum_zero_crossing_rate = 0.0;
	double speech_sum_clipping_ratio = 0.0;
	float speech_max_vad_probability = -1.0f;
	float speech_max_frame_peak = 0.0f;
	float speech_max_frame_rms = 0.0f;
	float gate_last_frame_peak = 0.0f;
	float gate_last_frame_rms = 0.0f;
	float last_vad_probability = -1.0f;
	captions::AdaptiveSpeechGate speech_gate;
	captions::SpeechActivationWindow speech_activation;
	std::deque<std::vector<float>> preroll_frames;
	std::vector<float> speech_buf;
	bool speech_buf_has_continuation_overlap = false;
	std::string last_transcript_for_overlap;
	uint8_t last_logged_speech_model = 255;
	std::string last_logged_required_whisper_model_path;
	std::string last_logged_active_whisper_model_path;
	bool last_logged_whisper_loaded = false;
	bool last_logged_model_fallback = false;
	bool last_logged_required_exists = false;
	bool last_logged_active_exists = false;
	auto next_audio_backlog_log = std::chrono::steady_clock::time_point{};
	bool prompt_context_cleared_for_backlog = false;
	int prompt_context_quarantine_segments = 0;
	int suppression_streak = 0;
	int accepted_filler_streak = 0;
	std::string last_normalized_transcript;
	int repeated_transcript_streak = 0;
	int last_segment_risk_score = 0;
	std::string last_segment_risk_reason;
	float last_segment_speech_frame_ratio = 0.0f;
	float last_segment_possible_frame_ratio = 0.0f;
	float last_segment_acoustic_artifact_risk = 0.0f;
	float last_segment_speech_band_ratio = 0.0f;
	float last_segment_zero_crossing_rate = 0.0f;
	float last_segment_clipping_ratio = 0.0f;
	long long suppressed_transcripts = 0;
	long long suppressed_non_speech = 0;
	long long suppressed_no_speech_probability = 0;
	long long suppressed_common_hallucination = 0;
	long long suppressed_common_filler = 0;
	long long suppressed_short_weak_audio = 0;
	long long suppressed_repetitive = 0;
	long long suppressed_low_confidence = 0;
	long long suppressed_slow_short_decode = 0;
	std::string last_suppression_reason;
	auto publish_suppression_status = [&]() {
		status.SetTranscriptSuppressionDiagnostics(
		    last_suppression_reason, suppressed_transcripts, suppressed_non_speech, suppressed_no_speech_probability,
		    suppressed_common_hallucination, suppressed_common_filler, suppressed_short_weak_audio,
		    suppressed_repetitive, suppressed_low_confidence, suppressed_slow_short_decode);
	};
	auto record_suppression = [&](const char* reason) {
		last_suppression_reason = reason ? reason : "";
		++suppressed_transcripts;
		if (last_suppression_reason == "non-speech") {
			++suppressed_non_speech;
		}
		else if (last_suppression_reason == "no-speech-probability") {
			++suppressed_no_speech_probability;
		}
		else if (last_suppression_reason == "common-hallucination") {
			++suppressed_common_hallucination;
		}
		else if (last_suppression_reason == "common-filler") {
			++suppressed_common_filler;
		}
		else if (last_suppression_reason == "short-weak-audio") {
			++suppressed_short_weak_audio;
		}
		else if (last_suppression_reason == "repetitive") {
			++suppressed_repetitive;
		}
		else if (last_suppression_reason == "low-confidence") {
			++suppressed_low_confidence;
		}
		else if (last_suppression_reason == "slow-short-decode") {
			++suppressed_slow_short_decode;
		}
		publish_suppression_status();
	};
	auto clear_prompt_context = [&](const char* reason) {
		const size_t old_chars = whisper_prompt.Text().size();
		const bool had_overlap = !last_transcript_for_overlap.empty();
		whisper_prompt.Clear();
		whisper.SetInitialPrompt("");
		last_transcript_for_overlap.clear();
		status.SetPromptContextLength(0);
		if (old_chars > 0 || had_overlap) {
			TH_LOG("[main] cleared prompt context reason=%s old_chars=%zu overlap=%d", reason ? reason : "unknown",
			       old_chars, had_overlap ? 1 : 0);
		}
	};
	auto enter_prompt_quarantine = [&](const char* reason, int segments) {
		if (segments > prompt_context_quarantine_segments) {
			prompt_context_quarantine_segments = segments;
		}
		clear_prompt_context(reason);
		TH_LOG("[main] prompt context quarantine reason=%s remaining_segments=%d", reason ? reason : "unknown",
		       prompt_context_quarantine_segments);
	};
	auto publish_segment_risk_status = [&]() {
		status.SetSegmentRiskDiagnostics(
		    last_segment_risk_score, last_segment_risk_reason, last_segment_speech_frame_ratio,
		    last_segment_possible_frame_ratio, last_segment_acoustic_artifact_risk, last_segment_speech_band_ratio,
		    last_segment_zero_crossing_rate, last_segment_clipping_ratio, prompt_context_quarantine_segments);
	};
	auto reset_speech_feature_stats = [&]() {
		speech_feature_frames_since_open = 0;
		speech_sum_acoustic_artifact_risk = 0.0;
		speech_sum_speech_band_ratio = 0.0;
		speech_sum_zero_crossing_rate = 0.0;
		speech_sum_clipping_ratio = 0.0;
	};
	auto observe_speech_features = [&](const captions::AudioFrameFeatures& features) {
		++speech_feature_frames_since_open;
		speech_sum_acoustic_artifact_risk += features.artifact_risk;
		speech_sum_speech_band_ratio += features.speech_band_ratio;
		speech_sum_zero_crossing_rate += features.zero_crossing_rate;
		speech_sum_clipping_ratio += features.clipping_ratio;
	};
	auto segment_feature_average = [](double sum, size_t frames) -> float {
		return frames > 0 ? static_cast<float>(sum / static_cast<double>(frames)) : 0.0f;
	};
	publish_suppression_status();
	status.SetPromptContextLength(whisper_prompt.Text().size());
	publish_segment_risk_status();

	TH_LOG("[startup] phase=initializing-audio-capture");
	status.SetPhase("initializing-audio-capture");
	status.Flush();
	// WASAPI capture: 32 ms chunks fed through a thread-safe queue.
	std::mutex audio_mutex;
	std::vector<std::vector<float>> audio_queue;

	// Selected capture device: the overlay writes the chosen WASAPI endpoint id
	// to captions\audio_input.txt; an empty/missing file means "system default".
	// Resolve the directory once and seed the device before capture starts so
	// the first open targets the user's choice, then watch the file for changes.
	const std::wstring captions_dir = openvr_pair::common::WkOpenVrSubdirectoryPath(L"captions", true);
	const std::wstring audio_input_path = captions_dir.empty() ? std::wstring() : captions_dir + L"\\audio_input.txt";
	int64_t audio_input_mtime = audio_input_path.empty() ? 0 : openvr_pair::common::FileLastWriteTime(audio_input_path);
	bool audio_input_file_present = audio_input_mtime != 0;
	std::string selected_audio_device_id;
	auto next_device_check = std::chrono::steady_clock::now();

	WasapiCapture capture;
	{
		selected_audio_device_id = captions::ReadCaptionsInputDeviceId(captions_dir);
		if (!selected_audio_device_id.empty()) {
			TH_LOG("[main] using saved capture device id='%s'", selected_audio_device_id.c_str());
		}
		capture.SetDevice(selected_audio_device_id);
	}
	bool cap_ok = capture.Start([&](const float* pcm, size_t n) {
		std::lock_guard<std::mutex> lk(audio_mutex);
		audio_queue.push_back(std::vector<float>(pcm, pcm + n));
	});

	if (!cap_ok) TH_LOG("[main] WASAPI capture start failed");

	status.SetMicName(capture.DeviceName());
	status.SetInputDeviceDiagnostics(!selected_audio_device_id.empty(), audio_input_file_present, capture.DeviceName());
	status.SetState(HostStatus::State::Idle);
	TH_LOG("[main] capture device effective mode=%s audio_input_present=%d name='%s'",
	       selected_audio_device_id.empty() ? "system-default" : "explicit", audio_input_file_present ? 1 : 0,
	       capture.DeviceName().c_str());

	TH_LOG("[startup] phase=running");
	status.SetPhase("running");
	status.Flush();

	// ---------------------------------------------------------------------------
	// Main loop
	// ---------------------------------------------------------------------------

	while (!g_shutdown.load(std::memory_order_acquire)) {
		const auto loop_now = std::chrono::steady_clock::now();

		// Watch for a capture-device change written by the overlay (~2 Hz). A
		// changed mtime re-reads the file and hands the new endpoint id to the
		// capture thread, which re-opens on its next iteration.
		if (!audio_input_path.empty() && loop_now >= next_device_check) {
			next_device_check = loop_now + std::chrono::milliseconds(500);
			int64_t mt = openvr_pair::common::FileLastWriteTime(audio_input_path);
			if (mt != audio_input_mtime) {
				audio_input_mtime = mt;
				audio_input_file_present = mt != 0;
				selected_audio_device_id = captions::ReadCaptionsInputDeviceId(captions_dir);
				TH_LOG("[main] capture device file changed; selecting mode=%s id='%s' audio_input_present=%d",
				       selected_audio_device_id.empty() ? "system-default" : "explicit",
				       selected_audio_device_id.empty() ? "(system default)" : selected_audio_device_id.c_str(),
				       audio_input_file_present ? 1 : 0);
				capture.SetDevice(selected_audio_device_id);
				clear_prompt_context("device-change");
				in_speech = false;
				ptt_was_held = false;
				silence_count = 0;
				speech_activation.Reset();
				speech_samples_since_open = 0;
				speech_evidence_samples_since_open = 0;
				speech_frames_since_open = 0;
				possible_frames_since_open = 0;
				total_frames_since_open = 0;
				reset_speech_feature_stats();
				speech_max_vad_probability = -1.0f;
				speech_max_frame_peak = 0.0f;
				speech_max_frame_rms = 0.0f;
				preroll_frames.clear();
				speech_buf.clear();
				speech_buf_has_continuation_overlap = false;
				prompt_context_quarantine_segments = 0;
				suppression_streak = 0;
				accepted_filler_streak = 0;
				repeated_transcript_streak = 0;
				last_normalized_transcript.clear();
			}
		}

		// Poll PTT action.
		bool ptt_held = actions.Poll();

		// Drain captured audio frames.
		std::vector<std::vector<float>> frames;
		{
			std::lock_guard<std::mutex> lk(audio_mutex);
			frames.swap(audio_queue);
		}
		const size_t audio_queue_frames = frames.size();
		const size_t audio_queue_samples = TotalAudioSamples(frames);
		const long long audio_queue_ms = SamplesToAudioMs(audio_queue_samples);

		HostConfig cfg;
		{
			std::lock_guard<std::mutex> lk(g_config_mutex);
			cfg = g_config;
		}
		pacer.SetMinGapSec(captions::NormalizeCaptionsChatboxSplitDelayMs(cfg.chatbox_split_delay_ms) / 1000.0);
		if (!captions::ShouldDrainQueuedChatbox(cfg.chatbox_enabled)) {
			pacer.Clear();
		}
		UpdatePackStatus(status, cfg);
		status.SetAudioQueueDiagnostics(static_cast<long long>(audio_queue_frames), audio_queue_ms);
		if (audio_queue_ms >= 2000) {
			if (!prompt_context_cleared_for_backlog) {
				clear_prompt_context("audio-backlog");
				prompt_context_cleared_for_backlog = true;
			}
		}
		else if (audio_queue_ms < 500) {
			prompt_context_cleared_for_backlog = false;
		}
		if (audio_queue_ms >= 500 && loop_now >= next_audio_backlog_log) {
			next_audio_backlog_log = loop_now + std::chrono::seconds(2);
			TH_LOG("[audio] drained capture backlog frames=%zu samples=%zu audio_ms=%lld model=%s mode=%s state=%s",
			       audio_queue_frames, audio_queue_samples, audio_queue_ms,
			       captions::CaptionsSpeechModelName(cfg.speech_model), cfg.mode == 1 ? "always-on" : "ptt",
			       in_speech ? "listening" : "idle");
		}

		if ((!vad || !vad->IsLoaded()) && SileroVad::RuntimeAvailable() && FileExistsA(cfg.silero_model_path) &&
		    loop_now >= next_vad_load_attempt) {
			auto candidate = std::make_unique<SileroVad>();
			if (candidate->Load(cfg.silero_model_path)) {
				vad = std::move(candidate);
			}
			else {
				next_vad_load_attempt = loop_now + std::chrono::seconds(5);
				status.SetLastError("Speech VAD model failed to load.");
			}
		}

		// Update whisper language hint.
		whisper.SetLanguage(cfg.source_lang == "auto" ? "" : cfg.source_lang);
		const std::string prompt_lang_key = cfg.source_lang.empty() ? "auto" : cfg.source_lang;
		if (prompt_lang_key != whisper_prompt_lang_key) {
			clear_prompt_context("language-change");
			prompt_context_quarantine_segments = 0;
			whisper_prompt_lang_key = prompt_lang_key;
		}
		const std::string required_whisper_model_path = RequiredWhisperModelPath(cfg);
		const std::string desired_whisper_model_path = ActiveWhisperModelPath(cfg);
		if (whisper.IsLoaded() && desired_whisper_model_path != loaded_whisper_model_path) {
			TH_LOG("[main] switching Whisper model: '%s' -> '%s'", loaded_whisper_model_path.c_str(),
			       desired_whisper_model_path.c_str());
			whisper.Unload();
			loaded_whisper_model_path.clear();
			clear_prompt_context("model-change");
			prompt_context_quarantine_segments = 0;
			whisper_load_attempted = false;
			whisper_load_failed = false;
			next_whisper_load_attempt = loop_now;
		}
		if (!whisper.IsLoaded() && FileExistsA(desired_whisper_model_path) && loop_now >= next_whisper_load_attempt) {
			whisper_load_attempted = true;
			whisper_load_failed = !whisper.Load(desired_whisper_model_path, whisper_threads, whisper_use_gpu);
			if (whisper_load_failed) {
				next_whisper_load_attempt = loop_now + std::chrono::seconds(5);
				status.SetLastError("Whisper model failed to load.");
			}
			else {
				loaded_whisper_model_path = desired_whisper_model_path;
				status.SetComputeBackend(whisper.BackendInfo().device_name, whisper.BackendInfo().gpu_active);
			}
		}
		else if (!whisper.IsLoaded() && FileExistsA(desired_whisper_model_path) && whisper_load_attempted &&
		         whisper_load_failed) {
			status.SetLastError("Whisper model failed to load.");
		}
		else if (whisper.IsLoaded()) {
			whisper_load_failed = false;
		}
		const bool required_whisper_exists = FileExistsA(required_whisper_model_path);
		const bool active_whisper_exists = FileExistsA(desired_whisper_model_path);
		const bool model_fallback =
		    !cfg.whisper_model_path_overridden && required_whisper_model_path != desired_whisper_model_path;
		const std::string reported_whisper_model_path = whisper.IsLoaded() && !loaded_whisper_model_path.empty()
		                                                    ? loaded_whisper_model_path
		                                                    : desired_whisper_model_path;
		status.SetSpeechModel(cfg.speech_model, captions::CaptionsSpeechModelName(cfg.speech_model),
		                      reported_whisper_model_path, whisper.IsLoaded(), model_fallback);
		if (cfg.speech_model != last_logged_speech_model ||
		    required_whisper_model_path != last_logged_required_whisper_model_path ||
		    desired_whisper_model_path != last_logged_active_whisper_model_path ||
		    whisper.IsLoaded() != last_logged_whisper_loaded || model_fallback != last_logged_model_fallback ||
		    required_whisper_exists != last_logged_required_exists ||
		    active_whisper_exists != last_logged_active_exists) {
			TH_LOG("[config] speech-model id=%u name=%s required='%s' required_exists=%d active='%s' active_exists=%d "
			       "loaded=%d fallback=%d override=%d vad_loaded=%d vad_failures=%lld flags=%u mode=%s chatbox=%d "
			       "src=%s tgt=%s",
			       static_cast<unsigned>(cfg.speech_model), captions::CaptionsSpeechModelName(cfg.speech_model),
			       required_whisper_model_path.c_str(), required_whisper_exists ? 1 : 0,
			       desired_whisper_model_path.c_str(), active_whisper_exists ? 1 : 0, whisper.IsLoaded() ? 1 : 0,
			       model_fallback ? 1 : 0, cfg.whisper_model_path_overridden ? 1 : 0, vad && vad->IsLoaded() ? 1 : 0,
			       vad ? static_cast<long long>(vad->InferenceFailures()) : 0LL,
			       static_cast<unsigned>(cfg.realtime_flags), cfg.mode == 1 ? "always-on" : "ptt",
			       cfg.chatbox_enabled ? 1 : 0, cfg.source_lang.c_str(), cfg.target_lang.c_str());
			last_logged_speech_model = cfg.speech_model;
			last_logged_required_whisper_model_path = required_whisper_model_path;
			last_logged_active_whisper_model_path = desired_whisper_model_path;
			last_logged_whisper_loaded = whisper.IsLoaded();
			last_logged_model_fallback = model_fallback;
			last_logged_required_exists = required_whisper_exists;
			last_logged_active_exists = active_whisper_exists;
		}

		// Load/unload translation model as packs appear, disappear, or the
		// selected pair changes. Pack install/uninstall can happen while this
		// host is running, so target_lang alone is not enough to decide.
		if (cfg.target_lang.empty()) {
			if (captions_engine.IsLoaded()) {
				captions_engine.Unload();
				TH_LOG("[captions] unloaded model; target language disabled");
			}
			loaded_src_lang.clear();
			loaded_tgt_lang.clear();
			loaded_model_dir.clear();
			translation_load_failed = false;
		}
		else {
			std::string model_dir = ResolveCaptionsModelDir(cfg.source_lang, cfg.target_lang);
			const bool runtime_available = Captions::RuntimeAvailable();
			const bool model_available = !model_dir.empty() && DirectoryExistsA(model_dir);

			if (!runtime_available || !model_available) {
				if (captions_engine.IsLoaded()) {
					captions_engine.Unload();
					TH_LOG("[captions] unloaded model for %s->%s; runtime_available=%d model_available=%d",
					       cfg.source_lang.c_str(), cfg.target_lang.c_str(), runtime_available ? 1 : 0,
					       model_available ? 1 : 0);
				}
				loaded_src_lang.clear();
				loaded_tgt_lang.clear();
				loaded_model_dir.clear();
				translation_load_failed = false;
			}
			else if (!captions_engine.IsLoaded() || cfg.source_lang != loaded_src_lang ||
			         cfg.target_lang != loaded_tgt_lang || model_dir != loaded_model_dir) {
				if (loop_now < next_translation_load_attempt) {
					if (translation_load_failed) status.SetLastError("Translation model failed to load.");
				}
				else {
					if (captions_engine.Load(model_dir)) {
						loaded_src_lang = cfg.source_lang;
						loaded_tgt_lang = cfg.target_lang;
						loaded_model_dir = model_dir;
						translation_load_failed = false;
					}
					else {
						loaded_src_lang.clear();
						loaded_tgt_lang.clear();
						loaded_model_dir.clear();
						translation_load_failed = true;
						next_translation_load_attempt = loop_now + std::chrono::seconds(5);
						status.SetLastError("Translation model failed to load.");
					}
				}
			}
		}

		const bool always_on = (cfg.mode == 1);
		const bool extended_timing = cfg.RealtimeEnabled(captions::kCaptionsRealtimeExtendedTiming);
		if (typing_indicator_active &&
		    (!cfg.chatbox_enabled || !cfg.RealtimeEnabled(captions::kCaptionsRealtimeTypingIndicator))) {
			set_typing_indicator(false, cfg);
		}
		if (!always_on && in_speech) {
			in_speech = false;
			silence_count = 0;
			speech_activation.Reset();
			speech_samples_since_open = 0;
			speech_evidence_samples_since_open = 0;
			speech_frames_since_open = 0;
			possible_frames_since_open = 0;
			total_frames_since_open = 0;
			reset_speech_feature_stats();
			speech_max_vad_probability = -1.0f;
			speech_max_frame_peak = 0.0f;
			speech_max_frame_rms = 0.0f;
			preroll_frames.clear();
			speech_buf.clear();
			speech_buf_has_continuation_overlap = false;
			set_typing_indicator(false, cfg);
			status.SetState(HostStatus::State::Idle);
		}

		for (const auto& frame : frames) {
			bool continue_after_transcribe = false;
			const char* transcribe_reason = "unknown";

			// VAD gate (PTT mode skips it). Silero is the primary gate; the
			// frame peak is a conservative fallback for capture paths where the
			// model stays below threshold despite clear microphone input.
			if (always_on) {
				float prob = -1.0f;
				if (vad && vad->IsLoaded()) {
					prob = vad->Feed(frame.data(), frame.size());
				}
				last_vad_probability = prob;
				const captions::AudioFrameFeatures frame_features =
				    captions::ComputeAudioFrameFeatures(frame.data(), frame.size());
				const float peak = frame_features.peak;
				const float rms = frame_features.rms;
				gate_last_frame_peak = peak;
				gate_last_frame_rms = rms;
				const bool speech_frame = speech_gate.IsSpeech(prob, peak, rms);
				const bool possible_speech_frame = speech_gate.IsPossibleSpeech(prob, peak, rms);
				if (!in_speech) {
					speech_activation.Push(speech_frame, possible_speech_frame);
				}
				if (!in_speech && speech_activation.ShouldOpen()) {
					const int activation_speech_frames = speech_activation.SpeechFrames();
					const int activation_possible_frames = speech_activation.PossibleFrames();
					const size_t activation_evidence_samples =
					    static_cast<size_t>(activation_speech_frames) * frame.size();
					in_speech = true;
					if (vad) vad->Reset();
					speech_buf.clear();
					speech_buf_has_continuation_overlap = false;
					speech_samples_since_open = 0;
					speech_evidence_samples_since_open = activation_evidence_samples;
					speech_frames_since_open = 0;
					possible_frames_since_open = 0;
					total_frames_since_open = 0;
					reset_speech_feature_stats();
					speech_max_vad_probability = -1.0f;
					speech_max_frame_peak = 0.0f;
					speech_max_frame_rms = 0.0f;
					for (const auto& preroll : preroll_frames) {
						speech_buf.insert(speech_buf.end(), preroll.begin(), preroll.end());
					}
					preroll_frames.clear();
					speech_activation.Reset();
					status.SetState(HostStatus::State::Listening);
					set_typing_indicator(true, cfg);
					const char* open_reason = prob >= 0.5f ? "vad" : "input-level";
					TH_LOG("[vad] speech gate opened reason=%s model=%s speech_frames=%d possible_frames=%d peak=%.3f "
					       "peak_threshold=%.3f rms=%.3f rms_threshold=%.3f noise_peak=%.3f noise_rms=%.3f prob=%.3f "
					       "evidence_ms=%lld preroll_frames=%zu",
					       open_reason, captions::CaptionsSpeechModelName(cfg.speech_model), activation_speech_frames,
					       activation_possible_frames, peak, speech_gate.SpeechPeakThreshold(), rms,
					       speech_gate.SpeechRmsThreshold(), speech_gate.AmbientPeak(), speech_gate.AmbientRms(), prob,
					       SamplesToAudioMs(activation_evidence_samples), preroll_frames.size());
				}
				if (speech_frame) {
					if (in_speech) {
						silence_count = 0;
					}
				}
				else if (in_speech && speech_gate.IsSilence(prob, peak, rms)) {
					++silence_count;
					if (silence_count >= captions::AlwaysOnSilenceFrames(extended_timing)) {
						in_speech = false;
						speech_activation.Reset();
						set_typing_indicator(false, cfg);
						status.SetState(HostStatus::State::Transcribing);
						transcribe_reason = "silence";
						goto transcribe;
					}
				}
				else if (in_speech && possible_speech_frame && silence_count > 0) {
					--silence_count;
				}
				if (in_speech) {
					speech_buf.insert(speech_buf.end(), frame.begin(), frame.end());
					speech_samples_since_open += frame.size();
					++total_frames_since_open;
					if (speech_frame) {
						speech_evidence_samples_since_open += frame.size();
						++speech_frames_since_open;
					}
					if (possible_speech_frame) {
						++possible_frames_since_open;
					}
					observe_speech_features(frame_features);
					speech_max_vad_probability = std::max(speech_max_vad_probability, prob);
					speech_max_frame_peak = std::max(speech_max_frame_peak, peak);
					speech_max_frame_rms = std::max(speech_max_frame_rms, rms);
					if (speech_samples_since_open >= captions::AlwaysOnMaxSpeechSamples(extended_timing)) {
						continue_after_transcribe = true;
						status.SetState(HostStatus::State::Transcribing);
						transcribe_reason = "max-duration";
						goto transcribe;
					}
				}
				else {
					if (!possible_speech_frame) {
						speech_gate.ObserveAmbient(peak, rms);
					}
					preroll_frames.push_back(frame);
					while (preroll_frames.size() >
					       static_cast<size_t>(captions::AlwaysOnPrerollFrames(extended_timing))) {
						preroll_frames.pop_front();
					}
				}
				continue;
			}

			// PTT mode: collect while held, flush on release.
			if (!always_on) {
				if (ptt_held) {
					if (!ptt_was_held) {
						speech_buf.clear();
						speech_buf_has_continuation_overlap = false;
						speech_samples_since_open = 0;
						speech_evidence_samples_since_open = 0;
						speech_frames_since_open = 0;
						possible_frames_since_open = 0;
						total_frames_since_open = 0;
						reset_speech_feature_stats();
						speech_max_vad_probability = -1.0f;
						speech_max_frame_peak = 0.0f;
						speech_max_frame_rms = 0.0f;
						status.SetState(HostStatus::State::Listening);
						set_typing_indicator(true, cfg);
					}
					speech_buf.insert(speech_buf.end(), frame.begin(), frame.end());
					speech_samples_since_open += frame.size();
					speech_evidence_samples_since_open += frame.size();
					++speech_frames_since_open;
					++possible_frames_since_open;
					++total_frames_since_open;
					const captions::AudioFrameFeatures frame_features =
					    captions::ComputeAudioFrameFeatures(frame.data(), frame.size());
					const float peak = frame_features.peak;
					const float rms = frame_features.rms;
					gate_last_frame_peak = peak;
					gate_last_frame_rms = rms;
					observe_speech_features(frame_features);
					speech_max_frame_peak = std::max(speech_max_frame_peak, peak);
					speech_max_frame_rms = std::max(speech_max_frame_rms, rms);
					ptt_was_held = true;
				}
				else if (ptt_was_held) {
					ptt_was_held = false;
					set_typing_indicator(false, cfg);
					status.SetState(HostStatus::State::Transcribing);
					transcribe_reason = "ptt-release";
					goto transcribe;
				}
			}
			continue;

		transcribe:
			const size_t segment_gate_samples = cfg.RealtimeEnabled(captions::kCaptionsRealtimeSpeechEvidenceGate)
			                                        ? speech_evidence_samples_since_open
			                                        : speech_samples_since_open;
			const long long segment_audio_ms = SamplesToAudioMs(speech_buf.size());
			const long long segment_evidence_ms = SamplesToAudioMs(segment_gate_samples);
			const float current_speech_peak_threshold = speech_gate.SpeechPeakThreshold();
			const float current_speech_rms_threshold = speech_gate.SpeechRmsThreshold();
			const size_t segment_total_frames = total_frames_since_open;
			const size_t segment_speech_frames = speech_frames_since_open;
			const size_t segment_possible_frames = possible_frames_since_open;
			const float segment_speech_frame_ratio =
			    captions::SpeechGateRatio(segment_speech_frames, segment_total_frames);
			const float segment_possible_frame_ratio =
			    captions::SpeechGateRatio(segment_possible_frames, segment_total_frames);
			const size_t segment_feature_frames = speech_feature_frames_since_open;
			const float segment_acoustic_artifact_risk =
			    segment_feature_average(speech_sum_acoustic_artifact_risk, segment_feature_frames);
			const float segment_speech_band_ratio =
			    segment_feature_average(speech_sum_speech_band_ratio, segment_feature_frames);
			const float segment_zero_crossing_rate =
			    segment_feature_average(speech_sum_zero_crossing_rate, segment_feature_frames);
			const float segment_clipping_ratio =
			    segment_feature_average(speech_sum_clipping_ratio, segment_feature_frames);
			const bool segment_has_enough_evidence = captions::SpeechSegmentShouldTranscribe(
			    segment_gate_samples, always_on, speech_max_vad_probability, speech_max_frame_peak,
			    current_speech_peak_threshold, speech_max_frame_rms, current_speech_rms_threshold);
			const bool segment_has_usable_shape = captions::SpeechSegmentHasUsableShape(
			    segment_total_frames, segment_speech_frames, segment_possible_frames, segment_gate_samples, always_on,
			    speech_max_vad_probability, speech_max_frame_peak, current_speech_peak_threshold, speech_max_frame_rms,
			    current_speech_rms_threshold);
			if (!segment_has_enough_evidence || !segment_has_usable_shape) {
				const bool adapted_rejected_noise =
				    always_on && speech_gate.ObserveRejectedSegment(speech_max_vad_probability, speech_max_frame_peak,
				                                                    speech_max_frame_rms);
				const char* drop_reason = segment_has_enough_evidence ? "speech-shape" : transcribe_reason;
				last_segment_risk_score = segment_has_usable_shape ? 0 : 5;
				last_segment_risk_reason = segment_has_usable_shape ? "" : "speech-shape";
				last_segment_speech_frame_ratio = segment_speech_frame_ratio;
				last_segment_possible_frame_ratio = segment_possible_frame_ratio;
				last_segment_acoustic_artifact_risk = segment_acoustic_artifact_risk;
				last_segment_speech_band_ratio = segment_speech_band_ratio;
				last_segment_zero_crossing_rate = segment_zero_crossing_rate;
				last_segment_clipping_ratio = segment_clipping_ratio;
				publish_segment_risk_status();
				status.SetLastSegmentDiagnostics(drop_reason, segment_audio_ms, segment_evidence_ms, 0,
				                                 speech_max_vad_probability, speech_max_frame_peak,
				                                 current_speech_peak_threshold, speech_max_frame_rms,
				                                 current_speech_rms_threshold);
				TH_LOG("[speech] dropped segment reason=%s model=%s audio_ms=%lld evidence_ms=%lld always_on=%d "
				       "vad=%.3f peak=%.3f peak_threshold=%.3f rms=%.3f rms_threshold=%.3f frames=%zu "
				       "speech_frames=%zu possible_frames=%zu speech_ratio=%.3f possible_ratio=%.3f "
				       "acoustic_risk=%.3f speech_band=%.3f zcr=%.3f clipping=%.3f adapted_noise=%d queue_ms=%lld",
				       drop_reason, captions::CaptionsSpeechModelName(cfg.speech_model), segment_audio_ms,
				       segment_evidence_ms, always_on ? 1 : 0, speech_max_vad_probability, speech_max_frame_peak,
				       current_speech_peak_threshold, speech_max_frame_rms, current_speech_rms_threshold,
				       segment_total_frames, segment_speech_frames, segment_possible_frames, segment_speech_frame_ratio,
				       segment_possible_frame_ratio, segment_acoustic_artifact_risk, segment_speech_band_ratio,
				       segment_zero_crossing_rate, segment_clipping_ratio, adapted_rejected_noise ? 1 : 0,
				       audio_queue_ms);
				in_speech = false;
				speech_buf.clear();
				speech_buf_has_continuation_overlap = false;
				speech_samples_since_open = 0;
				speech_evidence_samples_since_open = 0;
				speech_frames_since_open = 0;
				possible_frames_since_open = 0;
				total_frames_since_open = 0;
				reset_speech_feature_stats();
				silence_count = 0;
				speech_activation.Reset();
				speech_max_vad_probability = -1.0f;
				speech_max_frame_peak = 0.0f;
				speech_max_frame_rms = 0.0f;
				status.SetState(HostStatus::State::Idle);
				continue;
			}

			{
				const bool resume_after_transcribe = continue_after_transcribe && always_on;
				const bool segment_had_continuation_overlap = speech_buf_has_continuation_overlap;
				const float segment_max_vad_probability = speech_max_vad_probability;
				const float segment_max_frame_peak = speech_max_frame_peak;
				const float segment_max_frame_rms = speech_max_frame_rms;
				const float segment_speech_peak_threshold = current_speech_peak_threshold;
				const float segment_speech_rms_threshold = current_speech_rms_threshold;
				const size_t segment_speech_evidence_samples = speech_evidence_samples_since_open;
				const size_t segment_frame_count = segment_total_frames;
				const size_t segment_speech_frame_count = segment_speech_frames;
				const size_t segment_possible_frame_count = segment_possible_frames;
				std::vector<float> segment_pcm = speech_buf;
				std::vector<float> continuation_overlap;
				if (resume_after_transcribe) {
					continuation_overlap = captions::CopyTrailingSamples(
					    segment_pcm, captions::AlwaysOnContinuationOverlapSamples(extended_timing));
				}
				TH_LOG("[speech] segment closed reason=%s model=%s audio_ms=%lld gate_ms=%lld raw_evidence_ms=%lld "
				       "vad=%.3f peak=%.3f peak_threshold=%.3f rms=%.3f rms_threshold=%.3f frames=%zu "
				       "speech_frames=%zu possible_frames=%zu speech_ratio=%.3f possible_ratio=%.3f resume=%d "
				       "overlap=%d acoustic_risk=%.3f speech_band=%.3f zcr=%.3f clipping=%.3f queue_ms=%lld",
				       transcribe_reason, captions::CaptionsSpeechModelName(cfg.speech_model), segment_audio_ms,
				       segment_evidence_ms, SamplesToAudioMs(segment_speech_evidence_samples),
				       segment_max_vad_probability, segment_max_frame_peak, segment_speech_peak_threshold,
				       segment_max_frame_rms, segment_speech_rms_threshold, segment_frame_count,
				       segment_speech_frame_count, segment_possible_frame_count, segment_speech_frame_ratio,
				       segment_possible_frame_ratio, resume_after_transcribe ? 1 : 0,
				       segment_had_continuation_overlap ? 1 : 0, segment_acoustic_artifact_risk,
				       segment_speech_band_ratio, segment_zero_crossing_rate, segment_clipping_ratio, audio_queue_ms);

				std::string detected_lang;
				std::string transcript;
				WhisperTranscriptResult whisper_result;
				bool prompt_context_enabled = false;
				bool prompt_context_used = false;
				size_t prompt_chars = 0;
				if (whisper.IsLoaded()) {
					prompt_context_enabled = cfg.RealtimeEnabled(captions::kCaptionsRealtimePromptContext);
					if (!prompt_context_enabled && !whisper_prompt.Text().empty()) {
						whisper_prompt.Clear();
					}
					const bool prompt_context_allowed =
					    prompt_context_enabled &&
					    captions::TranscriptShouldUsePromptContextForDecode(
					        always_on, segment_audio_ms, segment_evidence_ms, segment_max_vad_probability,
					        segment_speech_frame_ratio, segment_acoustic_artifact_risk,
					        prompt_context_quarantine_segments);
					const std::string prompt_text = prompt_context_allowed ? whisper_prompt.Text() : std::string();
					prompt_context_used = !prompt_text.empty();
					prompt_chars = prompt_text.size();
					whisper.SetInitialPrompt(prompt_text);
					WhisperDecodeOptions decode_options;
					decode_options.use_no_speech_threshold =
					    cfg.RealtimeEnabled(captions::kCaptionsRealtimeWhisperNoSpeechGate);
					decode_options.no_speech_threshold = captions::TranscriptNoSpeechProbabilityThreshold();
					whisper_result = whisper.TranscribeDetailed(segment_pcm, decode_options);
					detected_lang = whisper_result.detected_lang;
					transcript = whisper_result.text;
				}
				else if (whisper_load_attempted && whisper_load_failed) {
					status.SetLastError("Whisper model failed to load.");
				}

				if (cfg.RealtimeEnabled(captions::kCaptionsRealtimeOverlapCleanup) &&
				    segment_had_continuation_overlap && !transcript.empty() && !last_transcript_for_overlap.empty()) {
					std::string trimmed =
					    captions::RemoveOverlappingTranscriptPrefix(last_transcript_for_overlap, transcript);
					if (trimmed.size() != transcript.size()) {
						TH_LOG("[main] trimmed overlapping transcript prefix: before=%zu after=%zu", transcript.size(),
						       trimmed.size());
					}
					transcript = trimmed;
				}

				if (resume_after_transcribe) {
					speech_buf = std::move(continuation_overlap);
					speech_buf_has_continuation_overlap = !speech_buf.empty();
					in_speech = true;
				}
				else {
					speech_buf.clear();
					speech_buf_has_continuation_overlap = false;
					in_speech = false;
				}
				speech_samples_since_open = 0;
				speech_evidence_samples_since_open = 0;
				speech_frames_since_open = 0;
				possible_frames_since_open = 0;
				total_frames_since_open = 0;
				reset_speech_feature_stats();
				silence_count = 0;
				speech_activation.Reset();
				speech_max_vad_probability = -1.0f;
				speech_max_frame_peak = 0.0f;
				speech_max_frame_rms = 0.0f;
				status.SetLastSegmentDiagnostics(transcribe_reason, segment_audio_ms, segment_evidence_ms,
				                                 whisper_result.decode_ms, segment_max_vad_probability,
				                                 segment_max_frame_peak, segment_speech_peak_threshold,
				                                 segment_max_frame_rms, segment_speech_rms_threshold);
				const double decode_ratio =
				    segment_audio_ms > 0 ? static_cast<double>(whisper_result.decode_ms) / segment_audio_ms : 0.0;
				TH_LOG("[main] transcript model=%s active='%s' reason=%s lang=%s audio_ms=%lld evidence_ms=%lld "
				       "decode_ms=%lld decode_ratio=%.2f prompt=%d prompt_chars=%zu prompt_quarantine=%d "
				       "nospeech=%.3f avglog=%.3f tokens=%d segments=%d peak=%.3f peak_threshold=%.3f rms=%.3f "
				       "rms_threshold=%.3f speech_ratio=%.3f possible_ratio=%.3f queue_ms=%lld resume=%d overlap=%d "
				       "acoustic_risk=%.3f speech_band=%.3f zcr=%.3f clipping=%.3f text=%s",
				       captions::CaptionsSpeechModelName(cfg.speech_model), reported_whisper_model_path.c_str(),
				       transcribe_reason, detected_lang.c_str(), segment_audio_ms, segment_evidence_ms,
				       whisper_result.decode_ms, decode_ratio, prompt_context_used ? 1 : 0, prompt_chars,
				       prompt_context_quarantine_segments, whisper_result.max_no_speech_probability,
				       whisper_result.average_token_log_probability, whisper_result.token_count,
				       whisper_result.segment_count, segment_max_frame_peak, segment_speech_peak_threshold,
				       segment_max_frame_rms, segment_speech_rms_threshold, segment_speech_frame_ratio,
				       segment_possible_frame_ratio, audio_queue_ms, resume_after_transcribe ? 1 : 0,
				       segment_had_continuation_overlap ? 1 : 0, segment_acoustic_artifact_risk,
				       segment_speech_band_ratio, segment_zero_crossing_rate, segment_clipping_ratio,
				       transcript.c_str());

				std::string publish_transcript = captions::CleanTranscriptForPublish(transcript);
				const std::string normalized_publish_transcript = captions::TranscriptLowerWords(publish_transcript);
				const int candidate_same_text_count =
				    !normalized_publish_transcript.empty() &&
				            normalized_publish_transcript == last_normalized_transcript
				        ? repeated_transcript_streak + 1
				        : 1;
				const bool skip_non_speech = !transcript.empty() && publish_transcript.empty();
				const bool skip_no_speech = cfg.RealtimeEnabled(captions::kCaptionsRealtimeWhisperNoSpeechGate) &&
				                            captions::TranscriptShouldSuppressByNoSpeechProbability(
				                                publish_transcript, always_on, whisper_result.max_no_speech_probability,
				                                whisper_result.average_token_log_probability);
				captions::TranscriptRiskInput risk_input;
				risk_input.cleaned_text = publish_transcript;
				risk_input.always_on = always_on;
				risk_input.max_vad_probability = segment_max_vad_probability;
				risk_input.max_frame_peak = segment_max_frame_peak;
				risk_input.speech_peak_threshold = segment_speech_peak_threshold;
				risk_input.no_speech_probability = whisper_result.max_no_speech_probability;
				risk_input.average_token_log_probability = whisper_result.average_token_log_probability;
				risk_input.token_count = whisper_result.token_count;
				risk_input.max_frame_rms = segment_max_frame_rms;
				risk_input.speech_rms_threshold = segment_speech_rms_threshold;
				risk_input.audio_ms = segment_audio_ms;
				risk_input.evidence_ms = segment_evidence_ms;
				risk_input.decode_ratio = decode_ratio;
				risk_input.prompt_chars = prompt_chars;
				risk_input.recent_suppression_count = suppression_streak;
				risk_input.recent_same_text_count = candidate_same_text_count;
				risk_input.speech_frame_ratio = segment_speech_frame_ratio;
				risk_input.acoustic_artifact_risk = segment_acoustic_artifact_risk;
				risk_input.speech_band_ratio = segment_speech_band_ratio;
				risk_input.zero_crossing_rate = segment_zero_crossing_rate;
				risk_input.clipping_ratio = segment_clipping_ratio;
				const captions::TranscriptRiskResult risk_result =
				    cfg.RealtimeEnabled(captions::kCaptionsRealtimeConfidenceFilter)
				        ? captions::TranscriptRiskScore(risk_input)
				        : captions::TranscriptRiskResult{};
				const captions::TranscriptSuppressionReason confidence_suppression_reason = risk_result.reason;
				const bool skip_confidence = risk_result.suppress;
				last_segment_risk_score = risk_result.score;
				last_segment_risk_reason = risk_result.reason == captions::TranscriptSuppressionReason::None
				                               ? ""
				                               : captions::TranscriptSuppressionReasonName(risk_result.reason);
				last_segment_speech_frame_ratio = segment_speech_frame_ratio;
				last_segment_possible_frame_ratio = segment_possible_frame_ratio;
				last_segment_acoustic_artifact_risk = segment_acoustic_artifact_risk;
				last_segment_speech_band_ratio = segment_speech_band_ratio;
				last_segment_zero_crossing_rate = segment_zero_crossing_rate;
				last_segment_clipping_ratio = segment_clipping_ratio;
				publish_segment_risk_status();
				if (skip_non_speech || skip_no_speech || skip_confidence) {
					const bool adapted_rejected_noise =
					    always_on && speech_gate.ObserveRejectedSegment(segment_max_vad_probability,
					                                                    segment_max_frame_peak, segment_max_frame_rms);
					const char* suppression_reason =
					    skip_non_speech  ? "non-speech"
					    : skip_no_speech ? "no-speech-probability"
					                     : captions::TranscriptSuppressionReasonName(confidence_suppression_reason);
					if (last_segment_risk_reason.empty()) {
						last_segment_risk_score = std::max(last_segment_risk_score, 5);
						last_segment_risk_reason = suppression_reason;
						publish_segment_risk_status();
					}
					++suppression_streak;
					accepted_filler_streak = 0;
					if (!normalized_publish_transcript.empty()) {
						last_normalized_transcript = normalized_publish_transcript;
						repeated_transcript_streak = candidate_same_text_count;
					}
					record_suppression(suppression_reason);
					enter_prompt_quarantine("suppressed-transcript", 3);
					TH_LOG("[main] suppressed transcript reason=%s model=%s audio_ms=%lld evidence_ms=%lld "
					       "decode_ms=%lld vad=%.3f peak=%.3f peak_threshold=%.3f rms=%.3f rms_threshold=%.3f "
					       "nospeech=%.3f avglog=%.3f tokens=%d risk_score=%d same_text=%d streak=%d "
					       "prompt_quarantine=%d acoustic_risk=%.3f speech_band=%.3f zcr=%.3f clipping=%.3f "
					       "adapted_noise=%d queue_ms=%lld raw='%s'",
					       suppression_reason, captions::CaptionsSpeechModelName(cfg.speech_model), segment_audio_ms,
					       segment_evidence_ms, whisper_result.decode_ms, segment_max_vad_probability,
					       segment_max_frame_peak, segment_speech_peak_threshold, segment_max_frame_rms,
					       segment_speech_rms_threshold, whisper_result.max_no_speech_probability,
					       whisper_result.average_token_log_probability, whisper_result.token_count, risk_result.score,
					       candidate_same_text_count, suppression_streak, prompt_context_quarantine_segments,
					       segment_acoustic_artifact_risk, segment_speech_band_ratio, segment_zero_crossing_rate,
					       segment_clipping_ratio, adapted_rejected_noise ? 1 : 0, audio_queue_ms, transcript.c_str());
					publish_transcript.clear();
					status.SetLastTranscript("");
					status.SetLastTranslation("");
				}
				else if (!publish_transcript.empty()) {
					status.SetLastTranscript(publish_transcript);
					suppression_streak = 0;
					last_normalized_transcript = normalized_publish_transcript;
					repeated_transcript_streak = candidate_same_text_count;
					bool cleared_overlap_for_filler = false;
					const bool prompt_context_accepted =
					    cfg.RealtimeEnabled(captions::kCaptionsRealtimePromptContext) &&
					    captions::TranscriptShouldUpdatePromptContext(
					        publish_transcript, always_on, segment_max_vad_probability, segment_max_frame_peak,
					        segment_speech_peak_threshold, whisper_result.max_no_speech_probability,
					        whisper_result.average_token_log_probability, whisper_result.token_count,
					        segment_max_frame_rms, segment_speech_rms_threshold, segment_evidence_ms);
					if (prompt_context_accepted) {
						if (prompt_context_quarantine_segments > 0) {
							TH_LOG("[main] prompt context quarantine exited reason=high-confidence-transcript "
							       "remaining_segments=%d",
							       prompt_context_quarantine_segments);
						}
						prompt_context_quarantine_segments = 0;
						whisper_prompt.Observe(publish_transcript);
					}
					else if (prompt_context_quarantine_segments > 0) {
						--prompt_context_quarantine_segments;
					}
					if (captions::TranscriptLooksLikeCommonFiller(publish_transcript)) {
						++accepted_filler_streak;
						if (accepted_filler_streak >= 2) {
							enter_prompt_quarantine("repeated-filler", 2);
							cleared_overlap_for_filler = true;
						}
					}
					else {
						accepted_filler_streak = 0;
					}
					if (!cleared_overlap_for_filler) {
						last_transcript_for_overlap = publish_transcript;
					}
				}
				else {
					status.SetLastTranscript("");
					accepted_filler_streak = 0;
					if (prompt_context_quarantine_segments > 0) {
						--prompt_context_quarantine_segments;
					}
				}
				status.SetPromptContextLength(whisper_prompt.Text().size());
				publish_segment_risk_status();

				// Translation step.
				std::string output = publish_transcript;
				const std::string effective_src_lang =
				    (cfg.source_lang.empty() || cfg.source_lang == "auto") ? detected_lang : cfg.source_lang;
				const bool translation_requested =
				    !cfg.target_lang.empty() && (effective_src_lang.empty() || effective_src_lang != cfg.target_lang);
				if (!publish_transcript.empty() && translation_requested) {
					if (captions_engine.IsLoaded()) {
						status.SetState(HostStatus::State::Translating);
						const auto translation_start = std::chrono::steady_clock::now();
						output = captions_engine.Translate(publish_transcript, effective_src_lang, cfg.target_lang);
						const long long translation_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
						                                     std::chrono::steady_clock::now() - translation_start)
						                                     .count();
						status.SetLastTranslation(output);
						TH_LOG("[main] translation src=%s tgt=%s ms=%lld text=%s", effective_src_lang.c_str(),
						       cfg.target_lang.c_str(), translation_ms, output.c_str());
					}
					else {
						output.clear();
						status.SetLastTranslation("");
						status.SetLastError("Translation model is not loaded; skipped chatbox publish.");
						TH_LOG("[main] translation requested for %s->%s but model is not loaded; skipped publish",
						       effective_src_lang.c_str(), cfg.target_lang.c_str());
					}
				}
				else if (publish_transcript.empty() || !translation_requested) {
					status.SetLastTranslation("");
				}

				if (!output.empty()) {
					status.IncrementCaptionsCompleted();
				}
				if (captions::ShouldPublishChatbox(cfg.chatbox_enabled, output)) {
					if (cfg.RealtimeEnabled(captions::kCaptionsRealtimeChatboxSplitting)) {
						const std::vector<std::string> chunks = captions::SplitTextForChatbox(output);
						if (chunks.size() > 1) {
							TH_LOG("[main] split chatbox output into %zu chunks", chunks.size());
						}
						for (const auto& chunk : chunks) {
							pacer.Enqueue(chunk, true, cfg.notify_sound);
						}
					}
					else {
						pacer.Enqueue(output, true, cfg.notify_sound);
					}
				}
				status.SetState(resume_after_transcribe ? HostStatus::State::Listening : HostStatus::State::Idle);
			}
		}

		// Drain pacer and publish.
		ChatboxPacer::Entry entry;
		while (pacer.Dequeue(entry)) {
			if (!captions::ShouldDrainQueuedChatbox(cfg.chatbox_enabled)) {
				continue;
			}
			status.SetState(HostStatus::State::Sending);
			bool sent = publisher.PublishChatbox(cfg.chatbox_address, entry.text, entry.send_immediate, entry.notify);
			if (sent) {
				status.IncrementPacketsSent();
				TH_LOG("[main] published: '%s'", entry.text.c_str());
			}
			else {
				status.SetLastError("OSC router publish pipe unavailable.");
			}
			status.SetState(HostStatus::State::Idle);
		}

		if (in_speech) {
			status.SetState(HostStatus::State::Listening);
		}

		status.SetMicName(capture.DeviceName());
		status.SetInputDeviceDiagnostics(!selected_audio_device_id.empty(), audio_input_file_present,
		                                 capture.DeviceName());
		status.SetAudioLevel(capture.Level());
		status.SetFramesCaptured(static_cast<long long>(capture.FramesCaptured()));
		status.SetSpeechGateDiagnostics(gate_last_frame_peak, gate_last_frame_rms, speech_gate.AmbientPeak(),
		                                speech_gate.AmbientRms(), speech_gate.SpeechPeakThreshold(),
		                                speech_gate.SpeechRmsThreshold());
		status.SetVadDiagnostics(vad && vad->IsLoaded(), last_vad_probability,
		                         vad ? static_cast<long long>(vad->InferenceFailures()) : 0,
		                         vad ? vad->LastError() : std::string());
		status.SetPromptContextLength(whisper_prompt.Text().size());
		publish_segment_risk_status();
		status.MaybeFlush();

		Sleep(10); // 10 ms main-loop cadence
	}

	if (typing_indicator_active) {
		publisher.PublishTyping(false);
		typing_indicator_active = false;
	}

	TH_LOG("[main] shutting down");
	capture.Stop();
	status.SetState(HostStatus::State::Idle);
	status.Flush();

	if (vr_ok) vr::VR_Shutdown();

	g_shutdown.store(true, std::memory_order_release);
	WakeControlPipe();
	if (ctrl_thread.joinable()) ctrl_thread.join();
	if (owner_thread.joinable()) owner_thread.join();

	CaptionsHostFlushLog();
	return 0;
}
catch (const std::exception& e) {
	TH_LOG("[crash] main threw std::exception: %s", e.what());
	CaptionsHostFlushLog();
	return 1;
}
catch (...) {
	TH_LOG("[crash] main threw unknown exception");
	CaptionsHostFlushLog();
	return 1;
}
