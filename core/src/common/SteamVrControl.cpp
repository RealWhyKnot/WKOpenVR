#include "SteamVrControl.h"

#include "DiagnosticsLog.h"
#include "JsonUtil.h"
#include "Win32Text.h"

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <tlhelp32.h>

#include <algorithm>
#include <cstdarg>
#include <fstream>
#include <iterator>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace openvr_pair::common::steamvr_control {

namespace {

void Log(const char* fmt, ...)
{
	va_list args;
	va_start(args, fmt);
	DiagnosticLogV("safe-mode-recovery", fmt, args);
	va_end(args);
}

std::wstring EnvW(const wchar_t* name)
{
	wchar_t buf[MAX_PATH * 2] = {};
	DWORD n = GetEnvironmentVariableW(name, buf, static_cast<DWORD>(std::size(buf)));
	if (n == 0 || n >= std::size(buf)) return {};
	return std::wstring(buf, n);
}

bool ReadFileToString(const std::wstring& path, std::string& out)
{
	std::ifstream f(path, std::ios::binary);
	if (!f) return false;
	std::ostringstream ss;
	ss << f.rdbuf();
	out = ss.str();
	return true;
}

bool WriteStringToFile(const std::wstring& path, const std::string& body)
{
	std::ofstream f(path, std::ios::binary | std::ios::trunc);
	if (!f) return false;
	f.write(body.data(), static_cast<std::streamsize>(body.size()));
	return f.good();
}

std::wstring ParentDir(const std::wstring& path)
{
	const size_t slash = path.find_last_of(L"\\/");
	return (slash == std::wstring::npos) ? std::wstring() : path.substr(0, slash);
}

// Pull the first string element of a top-level JSON array key.
std::string FirstArrayString(const picojson::value& root, const char* key)
{
	const picojson::array* arr = json::ArrayAt(root, key);
	if (!arr || arr->empty()) return {};
	const picojson::value& first = (*arr)[0];
	return first.is<std::string>() ? first.get<std::string>() : std::string();
}

bool LooksLikeImage(const wchar_t* exeName, const wchar_t* wantLowerAscii)
{
	// Case-insensitive ASCII compare of a process image name against a known
	// lowercase target (e.g. L"vrserver.exe").
	for (size_t i = 0;; ++i) {
		wchar_t c = exeName[i];
		wchar_t w = wantLowerAscii[i];
		if (c >= L'A' && c <= L'Z') c = static_cast<wchar_t>(c - L'A' + L'a');
		if (c != w) return false;
		if (w == L'\0') return true;
	}
}

// Enumerate processes and invoke `fn(pid, lowercased-image)` for SteamVR images.
template <typename Fn> void ForEachSteamVrProcess(Fn&& fn)
{
	HANDLE snap = CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0);
	if (snap == INVALID_HANDLE_VALUE) return;
	PROCESSENTRY32W entry{};
	entry.dwSize = sizeof(entry);
	static const wchar_t* kImages[] = {L"vrdashboard.exe", L"vrcompositor.exe", L"vrwebhelper.exe",
	                                   L"vrmonitor.exe",   L"vrserver.exe",     L"vrservicewrapper.exe"};
	if (Process32FirstW(snap, &entry)) {
		do {
			for (const wchar_t* img : kImages) {
				if (LooksLikeImage(entry.szExeFile, img)) {
					fn(entry.th32ProcessID, img);
					break;
				}
			}
		} while (Process32NextW(snap, &entry));
	}
	CloseHandle(snap);
}

} // namespace

SteamPaths ResolveSteamPaths()
{
	SteamPaths paths;

	// Primary: %LOCALAPPDATA%\openvr\openvrpaths.vrpath -- the canonical OpenVR
	// registration that points at the active runtime + config directories.
	const std::wstring localAppData = EnvW(L"LOCALAPPDATA");
	if (!localAppData.empty()) {
		const std::wstring vrpath = localAppData + L"\\openvr\\openvrpaths.vrpath";
		std::string body;
		if (ReadFileToString(vrpath, body)) {
			picojson::value root;
			if (json::ParseObject(root, body)) {
				const std::string configDir = FirstArrayString(root, "config");
				const std::string runtimeDir = FirstArrayString(root, "runtime");
				if (!configDir.empty()) {
					const std::wstring cfg = Utf8ToWide(configDir);
					paths.vrSettingsPath = cfg + L"\\steamvr.vrsettings";
					const std::wstring steamRoot = ParentDir(cfg); // strip trailing \config
					if (!steamRoot.empty()) paths.vrServerLogPath = steamRoot + L"\\logs\\vrserver.txt";
				}
				if (!runtimeDir.empty()) {
					paths.vrStartupExe = Utf8ToWide(runtimeDir) + L"\\bin\\win64\\vrstartup.exe";
				}
			}
		}
	}

	// Fallback: conventional default install location. Mirrors the path
	// ShellContext uses when SteamVR discovery fails so a standard install keeps
	// working even if openvrpaths.vrpath is missing.
	if (paths.vrSettingsPath.empty()) {
		const std::wstring steam = L"C:\\Program Files (x86)\\Steam";
		paths.vrSettingsPath = steam + L"\\config\\steamvr.vrsettings";
		paths.vrServerLogPath = steam + L"\\logs\\vrserver.txt";
		paths.vrStartupExe = steam + L"\\steamapps\\common\\SteamVR\\bin\\win64\\vrstartup.exe";
	}
	if (paths.vrStartupExe.empty()) {
		paths.vrStartupExe = L"C:\\Program Files (x86)\\Steam\\steamapps\\common\\SteamVR\\bin\\win64\\vrstartup.exe";
	}

	paths.ok = !paths.vrSettingsPath.empty();
	return paths;
}

bool ParseVrServerSafeModeBlock(const std::string& logText, std::vector<std::string>& blocked)
{
	blocked.clear();
	static const std::string kPrefix = "Not loading driver ";
	static const std::string kSuffix = " because it was blocked by a previous safe mode event";
	static const std::string kSafeModeMarker = "Using safe mode";

	if (logText.find(kSafeModeMarker) == std::string::npos) return false;

	size_t pos = 0;
	while (true) {
		const size_t at = logText.find(kPrefix, pos);
		if (at == std::string::npos) break;
		const size_t nameStart = at + kPrefix.size();
		const size_t suf = logText.find(kSuffix, nameStart);
		if (suf == std::string::npos) {
			pos = nameStart;
			continue;
		}
		std::string name = logText.substr(nameStart, suf - nameStart);
		pos = suf + kSuffix.size();
		// A real match keeps the driver name on one line.
		if (name.empty() || name.find('\n') != std::string::npos || name.find('\r') != std::string::npos) continue;
		if (std::find(blocked.begin(), blocked.end(), name) == blocked.end()) blocked.push_back(name);
	}
	return !blocked.empty();
}

bool ReadSafeModeBlockedDrivers(const std::wstring& vrServerLogPath, std::vector<std::string>& blocked)
{
	blocked.clear();
	std::string body;
	if (!ReadFileToString(vrServerLogPath, body)) return false;
	// The safe-mode lines are emitted at startup (top of the current session's
	// log); cap to a generous prefix so a long session log stays cheap to scan.
	constexpr size_t kScanCap = 256 * 1024;
	if (body.size() > kScanCap) body.resize(kScanCap);
	return ParseVrServerSafeModeBlock(body, blocked);
}

bool ClearSafeModeInVrSettingsJson(const std::string& inputJson, const std::vector<std::string>& driverNames,
                                   std::string& outputJson)
{
	picojson::value root;
	std::string err;
	if (!json::Parse(root, inputJson, &err) || !root.is<picojson::object>()) return false;

	picojson::object& obj = root.get<picojson::object>();

	auto ensureObject = [](picojson::object& parent, const std::string& key) -> picojson::object& {
		auto it = parent.find(key);
		if (it == parent.end() || !it->second.is<picojson::object>()) {
			parent[key] = picojson::value(picojson::object());
		}
		return parent[key].get<picojson::object>();
	};

	// Global safe-mode flag off.
	ensureObject(obj, "steamvr")["enableSafeMode"] = picojson::value(false);

	// Per-driver block cleared for every driver SteamVR locked out.
	for (const std::string& name : driverNames) {
		if (name.empty()) continue;
		ensureObject(obj, "driver_" + name)["blocked_by_safe_mode"] = picojson::value(false);
	}

	outputJson = root.serialize(true);
	return true;
}

std::wstring BackupVrSettings(const std::wstring& vrSettingsPath)
{
	SYSTEMTIME st{};
	GetLocalTime(&st);
	wchar_t stamp[32] = {};
	swprintf(stamp, std::size(stamp), L"%04u%02u%02u-%02u%02u%02u", st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute,
	         st.wSecond);
	std::wstring backup = vrSettingsPath + L".wkopenvr-safe-mode-" + stamp + L".bak";
	if (!CopyFileW(vrSettingsPath.c_str(), backup.c_str(), FALSE)) {
		Log("backup of '%ls' failed err=%lu", vrSettingsPath.c_str(), GetLastError());
		return {};
	}
	return backup;
}

bool ClearSafeMode(const SteamPaths& paths, const std::vector<std::string>& driverNames)
{
	if (paths.vrSettingsPath.empty()) return false;

	std::string body;
	if (!ReadFileToString(paths.vrSettingsPath, body)) {
		Log("could not read steamvr.vrsettings at '%ls'", paths.vrSettingsPath.c_str());
		return false;
	}

	const std::wstring backup = BackupVrSettings(paths.vrSettingsPath);
	if (backup.empty()) return false;

	std::string transformed;
	if (!ClearSafeModeInVrSettingsJson(body, driverNames, transformed)) {
		Log("steamvr.vrsettings was not valid JSON; leaving it untouched");
		return false;
	}

	if (!WriteStringToFile(paths.vrSettingsPath, transformed)) {
		Log("failed to write cleared steamvr.vrsettings at '%ls'", paths.vrSettingsPath.c_str());
		return false;
	}
	Log("cleared safe mode for %zu driver(s); backup at '%ls'", driverNames.size(), backup.c_str());
	return true;
}

bool IsVrServerRunning()
{
	bool found = false;
	ForEachSteamVrProcess([&](DWORD, const wchar_t* image) {
		if (LooksLikeImage(image, L"vrserver.exe")) found = true;
	});
	return found;
}

void StopVrServer()
{
	// Collect target PIDs first, then terminate monitor-before-server so a still
	// live vrmonitor cannot re-spawn vrserver in the gap.
	std::vector<std::pair<DWORD, std::wstring>> targets;
	ForEachSteamVrProcess([&](DWORD pid, const wchar_t* image) { targets.emplace_back(pid, std::wstring(image)); });

	auto killImage = [&](const wchar_t* image) {
		for (const auto& t : targets) {
			if (t.second != image) continue;
			HANDLE proc = OpenProcess(PROCESS_TERMINATE | SYNCHRONIZE, FALSE, t.first);
			if (!proc) continue;
			TerminateProcess(proc, 1);
			WaitForSingleObject(proc, 3000);
			CloseHandle(proc);
			Log("terminated %ls (pid=%lu)", image, t.first);
		}
	};

	killImage(L"vrdashboard.exe");
	killImage(L"vrcompositor.exe");
	killImage(L"vrwebhelper.exe");
	killImage(L"vrmonitor.exe");
	killImage(L"vrservicewrapper.exe");
	killImage(L"vrserver.exe");
}

bool LaunchSteamVr(const SteamPaths& paths)
{
	if (paths.vrStartupExe.empty()) return false;
	if (GetFileAttributesW(paths.vrStartupExe.c_str()) == INVALID_FILE_ATTRIBUTES) {
		Log("vrstartup.exe not found at '%ls'", paths.vrStartupExe.c_str());
		return false;
	}

	std::wstring cmd = L"\"" + paths.vrStartupExe + L"\"";
	std::wstring workingDir = ParentDir(paths.vrStartupExe);

	STARTUPINFOW si{};
	si.cb = sizeof(si);
	PROCESS_INFORMATION pi{};
	if (!CreateProcessW(paths.vrStartupExe.c_str(), cmd.data(), nullptr, nullptr, FALSE, CREATE_NO_WINDOW, nullptr,
	                    workingDir.empty() ? nullptr : workingDir.c_str(), &si, &pi)) {
		Log("CreateProcessW(vrstartup.exe) failed err=%lu", GetLastError());
		return false;
	}
	CloseHandle(pi.hThread);
	CloseHandle(pi.hProcess);
	Log("relaunched SteamVR via vrstartup.exe (pid=%lu)", pi.dwProcessId);
	return true;
}

std::vector<const module_safety::ModuleSpec*> FindUncontainedCrashCulprits()
{
	std::vector<const module_safety::ModuleSpec*> culprits;
	size_t count = 0;
	const module_safety::ModuleSpec* specs = module_safety::Specs(&count);
	for (size_t i = 0; i < count; ++i) {
		const module_safety::ModuleSpec& spec = specs[i];
		if (module_safety::HasSuspectMarker(spec)) culprits.push_back(&spec);
	}
	return culprits;
}

LoopGuardDecision EvaluateLoopGuard(LoopGuardState prev, long long nowEpoch, unsigned maxAttempts,
                                    long long windowSeconds)
{
	LoopGuardDecision decision;
	const bool windowElapsed = prev.windowStartEpoch == 0 || (nowEpoch - prev.windowStartEpoch) >= windowSeconds ||
	                           nowEpoch < prev.windowStartEpoch;
	if (windowElapsed) {
		decision.allowed = true;
		decision.next = {1u, nowEpoch};
		return decision;
	}
	if (prev.count < maxAttempts) {
		decision.allowed = true;
		decision.next = {prev.count + 1u, prev.windowStartEpoch};
		return decision;
	}
	decision.allowed = false;
	decision.next = prev;
	return decision;
}

namespace {
std::wstring LoopGuardPath()
{
	return module_safety::RootPath(true) + L"\\recovery_attempts.state";
}
} // namespace

bool ReadLoopGuardState(LoopGuardState& out)
{
	out = {};
	std::string body;
	if (!ReadFileToString(LoopGuardPath(), body)) return false;
	std::istringstream ss(body);
	long long count = 0;
	long long window = 0;
	if (!(ss >> count >> window)) return false;
	out.count = count < 0 ? 0u : static_cast<unsigned>(count);
	out.windowStartEpoch = window;
	return true;
}

bool WriteLoopGuardState(const LoopGuardState& state)
{
	std::ostringstream ss;
	ss << state.count << ' ' << state.windowStartEpoch << '\n';
	return WriteStringToFile(LoopGuardPath(), ss.str());
}

long long NowEpochSeconds()
{
	FILETIME ft{};
	GetSystemTimeAsFileTime(&ft);
	ULARGE_INTEGER u{};
	u.LowPart = ft.dwLowDateTime;
	u.HighPart = ft.dwHighDateTime;
	// 100ns ticks since 1601-01-01 -> seconds since 1970-01-01.
	constexpr unsigned long long kEpochDelta = 116444736000000000ULL;
	if (u.QuadPart < kEpochDelta) return 0;
	return static_cast<long long>((u.QuadPart - kEpochDelta) / 10000000ULL);
}

} // namespace openvr_pair::common::steamvr_control
