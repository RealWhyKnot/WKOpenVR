#define _CRT_SECURE_NO_DEPRECATE
// WIN32_LEAN_AND_MEAN is set by the CMakeLists compile-definitions; no
// local re-define here to avoid the macro-redefinition warning.
#include <windows.h>
#include <shlobj.h>
#include <objbase.h>
#include <delayimp.h>

#include "ActionBindings.h"
#include "Captions.h"
#include "CaptionsAudioInputFile.h"
#include "CaptionsOutputPolicy.h"
#include "ChatboxText.h"
#include "ChatboxPacer.h"
#include "AudioLevel.h"
#include "EnergySpeechGate.h"
#include "HostStatus.h"
#include "Logging.h"
#include "ModelDownloader.h"
#include "RouterPublisher.h"
#include "SileroVad.h"
#include "TranscriptText.h"
#include "WasapiCapture.h"
#include "WhisperEngine.h"
#include "WhisperPromptHistory.h"
#include "Win32Paths.h"
#include "Win32Text.h"

#include <openvr.h>

#include <algorithm>
#include <atomic>
#include <chrono>
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
	bool transcript_logging = false;
	int mode = 0; // 0=PTT, 1=always-on

	// Paths: resolved once at startup. Overrideable via command-line.
	std::string whisper_model_path;
	std::string silero_model_path;
};

static std::mutex g_config_mutex;
static HostConfig g_config;

// ---------------------------------------------------------------------------
// Control pipe listener (receives config updates from the driver)
// ---------------------------------------------------------------------------

#define HOST_CONTROL_PIPE_NAME "\\\\.\\pipe\\WKOpenVR-Captions.host"

static std::atomic<bool> g_shutdown{false};

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
		else if (key == "log")
			g_config.transcript_logging = (val != "0");

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
	HANDLE pipe =
	    CreateNamedPipeA(HOST_CONTROL_PIPE_NAME, PIPE_ACCESS_INBOUND, PIPE_TYPE_BYTE | PIPE_READMODE_BYTE | PIPE_WAIT,
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

// ---------------------------------------------------------------------------
// Default model paths
// ---------------------------------------------------------------------------

static std::string DefaultWhisperModelPath()
{
	std::string dir = ModelDownloader::DefaultModelDir();
	if (dir.empty()) return {};
	return dir + "\\ggml-base.bin";
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
	const bool whisper_model = FileExistsA(cfg.whisper_model_path);
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

static int RunE2eFakePublish(const std::string& text, HostStatus& status)
{
	status.SetPhase("e2e-fake-publishing");
	status.SetLastTranscript(text);
	status.SetLastTranslation(text);
	status.SetState(HostStatus::State::Sending);
	status.Flush();

	RouterPublisher publisher;
	bool sent = false;
	for (int i = 0; i < 8 && !sent; ++i) {
		sent = publisher.PublishChatbox("/chatbox/input", text, true, false);
		if (!sent) Sleep(100);
	}

	if (sent) {
		status.IncrementPacketsSent();
		status.SetState(HostStatus::State::Idle);
		status.SetPhase("e2e-fake-complete");
		status.Flush();
		TH_LOG("[e2e] fake captions output published: %s", text.c_str());
		CaptionsHostFlushLog();
		return 0;
	}

	status.SetState(HostStatus::State::Error);
	status.SetPhase("e2e-fake-failed");
	status.SetLastError("OSC router publish pipe unavailable.");
	status.Flush();
	TH_LOG("[e2e] fake captions output failed: router publish pipe unavailable");
	CaptionsHostFlushLog();
	return 2;
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
	ProbeDll(L"onnxruntime.dll");
	ProbeDll(L"ctranslate2.dll");
	TH_LOG("[captions-host] startup-phase=dll-probe-done");

	bool self_test = false;
	bool e2e_fake_publish = false;
	std::string e2e_fake_text = "WKOpenVR fake captions";
	std::wstring status_path_override;

	// Parse optional command-line overrides: --model <path> --silero <path>
	{
		std::lock_guard<std::mutex> lk(g_config_mutex);
		g_config.whisper_model_path = DefaultWhisperModelPath();
		g_config.silero_model_path = DefaultSileroModelPath();
		for (int i = 1; i < argc; ++i) {
			if (strcmp(argv[i], "--self-test") == 0 || strcmp(argv[i], "--healthcheck") == 0) {
				self_test = true;
			}
			else if (i + 1 < argc && (strcmp(argv[i], "--e2e-fake-chatbox") == 0 ||
			                          strcmp(argv[i], "--e2e-fake-captions-output") == 0 ||
			                          strcmp(argv[i], "--e2e-fake-translator-output") == 0)) {
				if (strcmp(argv[i], "--e2e-fake-translator-output") == 0) {
					fprintf(stderr, "[captions-host] warning: --e2e-fake-translator-output is deprecated; use "
					                "--e2e-fake-captions-output\n");
				}
				e2e_fake_publish = true;
				e2e_fake_text = argv[i + 1];
				++i;
			}
			else if (i + 1 < argc && strcmp(argv[i], "--status-file") == 0) {
				status_path_override = openvr_pair::common::Utf8ToWide(argv[i + 1]);
				++i;
			}
			else if (i + 1 < argc && strcmp(argv[i], "--model") == 0) {
				g_config.whisper_model_path = argv[i + 1];
				++i;
			}
			else if (i + 1 < argc && strcmp(argv[i], "--silero") == 0) {
				g_config.silero_model_path = argv[i + 1];
				++i;
			}
		}
	}

	HostStatus status(status_path_override);
	status.SetPhase("config-loaded");
	{
		std::lock_guard<std::mutex> lk(g_config_mutex);
		UpdatePackStatus(status, g_config);
	}
	status.SetState(HostStatus::State::Idle);
	status.Flush();

	if (e2e_fake_publish) {
		return RunE2eFakePublish(e2e_fake_text, status);
	}

	if (self_test) {
		status.SetPhase("self-test-complete");
		status.Flush();
		TH_LOG("[startup] self-test complete");
		CaptionsHostFlushLog();
		return 0;
	}

	// Layer 1: acquire system-wide singleton mutex before opening any IPC.
	{
		std::wstring user = GetUserSidString();
		wchar_t mname[512] = {};
		swprintf_s(mname, L"Global\\WKOpenVR-CaptionsHost-Singleton-%ls", user.c_str());

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

	TH_LOG("[startup] phase=opening-control-pipe");
	status.SetPhase("opening-control-pipe");
	status.Flush();
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
	captions::WhisperPromptHistory whisper_prompt;
	std::string whisper_prompt_lang_key;
	bool whisper_load_attempted = false;
	bool whisper_load_failed = false;
	auto next_whisper_load_attempt = std::chrono::steady_clock::time_point{};
	{
		std::string model_path;
		{
			std::lock_guard<std::mutex> lk(g_config_mutex);
			model_path = g_config.whisper_model_path;
		}
		if (FileExistsA(model_path)) {
			whisper_load_attempted = true;
			if (!whisper.Load(model_path)) {
				whisper_load_failed = true;
				next_whisper_load_attempt = std::chrono::steady_clock::now() + std::chrono::seconds(5);
				TH_LOG("[main] Whisper model load failed; path='%s'", model_path.c_str());
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
		if (!current_cfg.chatbox_enabled) active = false;
		if (typing_indicator_active == active) return;

		if (!publisher.PublishTyping(active)) {
			status.SetLastError("OSC router publish pipe unavailable.");
		}
		typing_indicator_active = active;
	};

	// Chatbox pacer (1.2 s minimum gap).
	ChatboxPacer pacer(1.2);

	// VAD state machine.
	int silence_count = 0;
	bool in_speech = false;
	bool ptt_was_held = false;
	size_t speech_samples_since_open = 0;
	size_t speech_evidence_samples_since_open = 0;
	float speech_max_vad_probability = -1.0f;
	float speech_max_frame_peak = 0.0f;
	captions::AdaptiveSpeechGate speech_gate;
	captions::SpeechActivationWindow speech_activation;
	std::deque<std::vector<float>> preroll_frames;
	std::vector<float> speech_buf;
	bool speech_buf_has_continuation_overlap = false;
	std::string last_transcript_for_overlap;

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
	auto next_device_check = std::chrono::steady_clock::now();

	WasapiCapture capture;
	{
		std::string initial_device = captions::ReadCaptionsInputDeviceId(captions_dir);
		if (!initial_device.empty()) {
			TH_LOG("[main] using saved capture device id='%s'", initial_device.c_str());
		}
		capture.SetDevice(initial_device);
	}
	bool cap_ok = capture.Start([&](const float* pcm, size_t n) {
		std::lock_guard<std::mutex> lk(audio_mutex);
		audio_queue.push_back(std::vector<float>(pcm, pcm + n));
	});

	if (!cap_ok) TH_LOG("[main] WASAPI capture start failed");

	status.SetMicName(capture.DeviceName());
	status.SetState(HostStatus::State::Idle);

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
				std::string id = captions::ReadCaptionsInputDeviceId(captions_dir);
				TH_LOG("[main] capture device file changed; selecting id='%s'",
				       id.empty() ? "(system default)" : id.c_str());
				capture.SetDevice(id);
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

		HostConfig cfg;
		{
			std::lock_guard<std::mutex> lk(g_config_mutex);
			cfg = g_config;
		}
		UpdatePackStatus(status, cfg);

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
			whisper_prompt.Clear();
			whisper.SetInitialPrompt("");
			whisper_prompt_lang_key = prompt_lang_key;
			last_transcript_for_overlap.clear();
		}
		if (!whisper.IsLoaded() && FileExistsA(cfg.whisper_model_path) && loop_now >= next_whisper_load_attempt) {
			whisper_load_attempted = true;
			whisper_load_failed = !whisper.Load(cfg.whisper_model_path);
			if (whisper_load_failed) {
				next_whisper_load_attempt = loop_now + std::chrono::seconds(5);
				status.SetLastError("Whisper model failed to load.");
			}
		}
		else if (!whisper.IsLoaded() && FileExistsA(cfg.whisper_model_path) && whisper_load_attempted &&
		         whisper_load_failed) {
			status.SetLastError("Whisper model failed to load.");
		}
		else if (whisper.IsLoaded()) {
			whisper_load_failed = false;
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
		if (typing_indicator_active && !cfg.chatbox_enabled) {
			set_typing_indicator(false, cfg);
		}
		if (!always_on && in_speech) {
			in_speech = false;
			silence_count = 0;
			speech_activation.Reset();
			speech_samples_since_open = 0;
			speech_evidence_samples_since_open = 0;
			speech_max_vad_probability = -1.0f;
			speech_max_frame_peak = 0.0f;
			preroll_frames.clear();
			speech_buf.clear();
			speech_buf_has_continuation_overlap = false;
			set_typing_indicator(false, cfg);
			status.SetState(HostStatus::State::Idle);
		}

		for (const auto& frame : frames) {
			bool continue_after_transcribe = false;

			// VAD gate (PTT mode skips it). Silero is the primary gate; the
			// frame peak is a conservative fallback for capture paths where the
			// model stays below threshold despite clear microphone input.
			if (always_on) {
				float prob = -1.0f;
				if (vad && vad->IsLoaded()) {
					prob = vad->Feed(frame.data(), frame.size());
				}
				const float peak = captions::ComputeBufferPeak(frame.data(), frame.size());
				const bool speech_frame = speech_gate.IsSpeech(prob, peak);
				const bool possible_speech_frame = speech_gate.IsPossibleSpeech(prob, peak);
				if (!in_speech) {
					speech_activation.Push(speech_frame, possible_speech_frame);
				}
				if (!in_speech && speech_activation.ShouldOpen()) {
					const size_t activation_evidence_samples =
					    static_cast<size_t>(speech_activation.PossibleFrames()) * frame.size();
					in_speech = true;
					if (vad) vad->Reset();
					speech_buf.clear();
					speech_buf_has_continuation_overlap = false;
					speech_samples_since_open = 0;
					speech_evidence_samples_since_open = activation_evidence_samples;
					speech_max_vad_probability = -1.0f;
					speech_max_frame_peak = 0.0f;
					for (const auto& preroll : preroll_frames) {
						speech_buf.insert(speech_buf.end(), preroll.begin(), preroll.end());
					}
					preroll_frames.clear();
					speech_activation.Reset();
					status.SetState(HostStatus::State::Listening);
					set_typing_indicator(true, cfg);
					if (prob < 0.5f) {
						TH_LOG("[vad] speech gate opened by input level peak=%.3f threshold=%.3f noise=%.3f prob=%.3f",
						       peak, speech_gate.SpeechPeakThreshold(), speech_gate.AmbientPeak(), prob);
					}
				}
				if (speech_frame) {
					if (in_speech) {
						silence_count = 0;
					}
				}
				else if (in_speech && speech_gate.IsSilence(prob, peak)) {
					++silence_count;
					if (silence_count >= captions::AlwaysOnSilenceFrames()) {
						in_speech = false;
						speech_activation.Reset();
						set_typing_indicator(false, cfg);
						status.SetState(HostStatus::State::Transcribing);
						goto transcribe;
					}
				}
				else if (in_speech && possible_speech_frame && silence_count > 0) {
					--silence_count;
				}
				if (in_speech) {
					speech_buf.insert(speech_buf.end(), frame.begin(), frame.end());
					speech_samples_since_open += frame.size();
					if (speech_frame || possible_speech_frame) {
						speech_evidence_samples_since_open += frame.size();
					}
					speech_max_vad_probability = std::max(speech_max_vad_probability, prob);
					speech_max_frame_peak = std::max(speech_max_frame_peak, peak);
					if (speech_samples_since_open >= captions::AlwaysOnMaxSpeechSamples()) {
						continue_after_transcribe = true;
						status.SetState(HostStatus::State::Transcribing);
						goto transcribe;
					}
				}
				else {
					if (!possible_speech_frame) {
						speech_gate.ObserveAmbient(peak);
					}
					preroll_frames.push_back(frame);
					while (preroll_frames.size() > static_cast<size_t>(captions::AlwaysOnPrerollFrames())) {
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
						speech_max_vad_probability = -1.0f;
						speech_max_frame_peak = 0.0f;
						status.SetState(HostStatus::State::Listening);
						set_typing_indicator(true, cfg);
					}
					speech_buf.insert(speech_buf.end(), frame.begin(), frame.end());
					speech_samples_since_open += frame.size();
					speech_evidence_samples_since_open += frame.size();
					ptt_was_held = true;
				}
				else if (ptt_was_held) {
					ptt_was_held = false;
					set_typing_indicator(false, cfg);
					status.SetState(HostStatus::State::Transcribing);
					goto transcribe;
				}
			}
			continue;

		transcribe:
			if (!captions::SpeechSegmentShouldTranscribe(speech_evidence_samples_since_open, always_on,
			                                             speech_max_vad_probability, speech_max_frame_peak,
			                                             speech_gate.SpeechPeakThreshold())) {
				in_speech = false;
				speech_buf.clear();
				speech_buf_has_continuation_overlap = false;
				speech_samples_since_open = 0;
				speech_evidence_samples_since_open = 0;
				silence_count = 0;
				speech_activation.Reset();
				speech_max_vad_probability = -1.0f;
				speech_max_frame_peak = 0.0f;
				status.SetState(HostStatus::State::Idle);
				continue;
			}

			{
				const bool resume_after_transcribe = continue_after_transcribe && always_on;
				const bool segment_had_continuation_overlap = speech_buf_has_continuation_overlap;
				const float segment_max_vad_probability = speech_max_vad_probability;
				const float segment_max_frame_peak = speech_max_frame_peak;
				const float segment_speech_peak_threshold = speech_gate.SpeechPeakThreshold();
				const size_t segment_speech_evidence_samples = speech_evidence_samples_since_open;
				std::vector<float> segment_pcm = speech_buf;
				std::vector<float> continuation_overlap;
				if (resume_after_transcribe) {
					continuation_overlap =
					    captions::CopyTrailingSamples(segment_pcm, captions::AlwaysOnContinuationOverlapSamples());
				}

				std::string detected_lang;
				std::string transcript;
				WhisperTranscriptResult whisper_result;
				if (whisper.IsLoaded()) {
					whisper.SetInitialPrompt(whisper_prompt.Text());
					whisper_result = whisper.TranscribeDetailed(segment_pcm);
					detected_lang = whisper_result.detected_lang;
					transcript = whisper_result.text;
				}
				else if (whisper_load_attempted && whisper_load_failed) {
					status.SetLastError("Whisper model failed to load.");
				}

				if (segment_had_continuation_overlap && !transcript.empty() && !last_transcript_for_overlap.empty()) {
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
				silence_count = 0;
				speech_activation.Reset();
				speech_max_vad_probability = -1.0f;
				speech_max_frame_peak = 0.0f;
				status.SetLastTranscript(transcript);
				TH_LOG("[main] transcript (%s nospeech=%.3f avglog=%.3f tokens=%d evidence=%zu): %s",
				       detected_lang.c_str(), whisper_result.max_no_speech_probability,
				       whisper_result.average_token_log_probability, whisper_result.token_count,
				       segment_speech_evidence_samples, transcript.c_str());

				std::string publish_transcript = captions::CleanTranscriptForPublish(transcript);
				const bool skip_non_speech = !transcript.empty() && publish_transcript.empty();
				const bool skip_confidence = captions::TranscriptShouldSuppressByConfidence(
				    publish_transcript, always_on, segment_max_vad_probability, segment_max_frame_peak,
				    segment_speech_peak_threshold, whisper_result.max_no_speech_probability,
				    whisper_result.average_token_log_probability, whisper_result.token_count);
				if (skip_non_speech || skip_confidence) {
					TH_LOG("[main] suppressed transcript as %s: raw='%s' vad=%.3f peak=%.3f nospeech=%.3f avglog=%.3f",
					       skip_non_speech ? "non-speech" : "low-confidence", transcript.c_str(),
					       segment_max_vad_probability, segment_max_frame_peak,
					       whisper_result.max_no_speech_probability, whisper_result.average_token_log_probability);
					publish_transcript.clear();
					status.SetLastTranslation("");
				}
				else if (!publish_transcript.empty()) {
					whisper_prompt.Observe(publish_transcript);
					last_transcript_for_overlap = publish_transcript;
				}

				// Translation step.
				std::string output = publish_transcript;
				const std::string effective_src_lang =
				    (cfg.source_lang.empty() || cfg.source_lang == "auto") ? detected_lang : cfg.source_lang;
				const bool translation_requested =
				    !cfg.target_lang.empty() && (effective_src_lang.empty() || effective_src_lang != cfg.target_lang);
				if (!publish_transcript.empty() && translation_requested) {
					if (captions_engine.IsLoaded()) {
						status.SetState(HostStatus::State::Translating);
						output = captions_engine.Translate(publish_transcript, effective_src_lang, cfg.target_lang);
						status.SetLastTranslation(output);
						TH_LOG("[main] translation: %s", output.c_str());
					}
					else {
						output.clear();
						status.SetLastTranslation("");
						status.SetLastError("Translation model is not loaded; skipped chatbox publish.");
						TH_LOG("[main] translation requested for %s->%s but model is not loaded; skipped publish",
						       effective_src_lang.c_str(), cfg.target_lang.c_str());
					}
				}
				else if (!translation_requested) {
					status.SetLastTranslation("");
				}

				if (!output.empty()) {
					status.IncrementCaptionsCompleted();
				}
				if (captions::ShouldPublishChatbox(cfg.chatbox_enabled, output)) {
					const std::vector<std::string> chunks = captions::SplitTextForChatbox(output);
					if (chunks.size() > 1) {
						TH_LOG("[main] split chatbox output into %zu chunks", chunks.size());
					}
					for (const auto& chunk : chunks) {
						pacer.Enqueue(chunk, true, cfg.notify_sound);
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
		status.SetAudioLevel(capture.Level());
		status.SetFramesCaptured(static_cast<long long>(capture.FramesCaptured()));
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
	if (ctrl_thread.joinable()) ctrl_thread.join();

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
