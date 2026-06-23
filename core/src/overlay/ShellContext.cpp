#include "ShellContext.h"

#include "DiagnosticsLog.h"
#include "ModuleRegistry.h"
#include "ModuleSafety.h"
#include "PowerShellCommand.h"
#include "ShellSettings.h"
#include "ShellUiLogic.h"
#include "Win32Paths.h"
#include "Win32Text.h"

#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <shellapi.h>

#include <cctype>
#include <fstream>
#include <string>
#include <string_view>
#include <sstream>
#include <utility>
#include <vector>

namespace openvr_pair::overlay {
namespace {

namespace module_registry = openvr_pair::common::modules;

struct PendingToggle
{
	std::string flagFileName;
	bool wantPresent;
	HANDLE process;
};

std::vector<PendingToggle> g_pendingToggles;
constexpr const char* kDesktopDefaultModuleSetting = "desktop_default_module";
constexpr const char* kModuleTabOrderSetting = "module_tab_order";
const char* FallbackDesktopDefaultModule()
{
	return module_registry::FlagFileName(module_registry::ModuleId::QuestApp);
}

double ShellNowSeconds()
{
	return static_cast<double>(GetTickCount64()) / 1000.0;
}

// Returns the directory containing the running exe, without a trailing slash.
// Falls back to an empty string on failure.
std::wstring ExeDir()
{
	wchar_t buf[MAX_PATH + 1] = {};
	DWORD len = GetModuleFileNameW(nullptr, buf, MAX_PATH);
	if (len == 0 || len >= MAX_PATH) return {};
	std::wstring path(buf, len);
	auto sep = path.find_last_of(L"\\/");
	return (sep != std::wstring::npos) ? path.substr(0, sep) : path;
}

// Read a REG_SZ value from the given hive/key/value. Returns empty on failure.
std::wstring ReadRegString(HKEY hive, const wchar_t* subkey, const wchar_t* valueName)
{
	HKEY hk = nullptr;
	if (RegOpenKeyExW(hive, subkey, 0, KEY_READ, &hk) != ERROR_SUCCESS) return {};
	DWORD type = 0, size = 0;
	if (RegQueryValueExW(hk, valueName, nullptr, &type, nullptr, &size) != ERROR_SUCCESS || type != REG_SZ ||
	    size == 0) {
		RegCloseKey(hk);
		return {};
	}
	std::wstring value(size / sizeof(wchar_t), L'\0');
	if (RegQueryValueExW(hk, valueName, nullptr, nullptr, reinterpret_cast<LPBYTE>(value.data()), &size) !=
	    ERROR_SUCCESS) {
		RegCloseKey(hk);
		return {};
	}
	RegCloseKey(hk);
	// Strip trailing NUL that RegQueryValueEx includes in `size`
	while (!value.empty() && value.back() == L'\0')
		value.pop_back();
	return value;
}

// Finds the Steam install root by trying three registry locations in order.
// Returns empty if none are found.
std::wstring FindSteamInstallPath()
{
	// 32-bit view first (most common install location on 64-bit Windows)
	std::wstring p = ReadRegString(HKEY_LOCAL_MACHINE, L"SOFTWARE\\WOW6432Node\\Valve\\Steam", L"InstallPath");
	if (!p.empty()) return p;
	p = ReadRegString(HKEY_LOCAL_MACHINE, L"SOFTWARE\\Valve\\Steam", L"InstallPath");
	if (!p.empty()) return p;
	// HKCU fallback (per-user install)
	p = ReadRegString(HKEY_CURRENT_USER, L"Software\\Valve\\Steam", L"SteamPath");
	if (!p.empty()) {
		// SteamPath uses forward slashes on some installs; normalise.
		for (wchar_t& ch : p)
			if (ch == L'/') ch = L'\\';
	}
	return p;
}

// Checks whether `candidate` is the SteamVR library root by testing for the
// SteamVR common directory inside it.
bool IsSteamVRRoot(const std::wstring& candidate)
{
	if (candidate.empty()) return false;
	DWORD attr = GetFileAttributesW((candidate + L"\\steamapps\\common\\SteamVR").c_str());
	return attr != INVALID_FILE_ATTRIBUTES && (attr & FILE_ATTRIBUTE_DIRECTORY);
}

// Minimal libraryfolders.vdf scanner. Reads the file line by line and grabs
// values from lines that look like:  "path"   "C:\\some\\path"
// Tests each path for the presence of steamapps\common\SteamVR.
// Returns the SteamVR install root, or empty if not found.
std::wstring FindSteamVRRootFromVDF(const std::wstring& steamPath)
{
	if (steamPath.empty()) return {};

	// The Steam install itself is always a library root.
	if (IsSteamVRRoot(steamPath)) return steamPath;

	std::wstring vdfPath = steamPath + L"\\config\\libraryfolders.vdf";
	std::ifstream vdf(vdfPath);
	if (!vdf.is_open()) return {};

	std::string line;
	while (std::getline(vdf, line)) {
		// Find a line containing the key "path" (case-insensitive enough for VDF)
		auto kpos = line.find("\"path\"");
		if (kpos == std::string::npos) {
			// Also accept "Path" with capital P just in case
			kpos = line.find("\"Path\"");
		}
		if (kpos == std::string::npos) continue;
		// Find the value: the next quoted string after the key
		auto q1 = line.find('"', kpos + 6);
		if (q1 == std::string::npos) continue;
		auto q2 = line.find('"', q1 + 1);
		if (q2 == std::string::npos) continue;
		std::string raw = line.substr(q1 + 1, q2 - q1 - 1);
		// VDF uses \\ escape; convert to single backslash
		std::string unescaped;
		unescaped.reserve(raw.size());
		for (size_t i = 0; i < raw.size(); ++i) {
			if (raw[i] == '\\' && i + 1 < raw.size() && raw[i + 1] == '\\') {
				unescaped += '\\';
				++i;
			}
			else {
				unescaped += raw[i];
			}
		}
		std::wstring candidate = openvr_pair::common::Utf8ToWide(unescaped);
		if (candidate.empty()) continue;
		if (IsSteamVRRoot(candidate)) return candidate;
	}
	return {};
}

// Returns the SteamVR install root (the directory that contains
// steamapps\common\SteamVR), or empty if discovery fails entirely.
std::wstring DiscoverSteamVRRoot()
{
	std::wstring steamPath = FindSteamInstallPath();
	std::wstring root = FindSteamVRRootFromVDF(steamPath);
	return root; // may be empty
}

std::string TrimAscii(std::string value)
{
	size_t begin = 0;
	while (begin < value.size() &&
	       (value[begin] == ' ' || value[begin] == '\t' || value[begin] == '\r' || value[begin] == '\n')) {
		++begin;
	}
	size_t end = value.size();
	while (end > begin &&
	       (value[end - 1] == ' ' || value[end - 1] == '\t' || value[end - 1] == '\r' || value[end - 1] == '\n')) {
		--end;
	}
	return value.substr(begin, end - begin);
}

bool IsValidModuleFlagFileName(const std::string& value)
{
	if (value.rfind("enable_", 0) != 0) return false;
	const std::string suffix = ".flag";
	if (value.size() <= suffix.size() || value.compare(value.size() - suffix.size(), suffix.size(), suffix) != 0) {
		return false;
	}
	for (const char ch : value) {
		const unsigned char c = static_cast<unsigned char>(ch);
		if (std::isalnum(c) || ch == '_' || ch == '-' || ch == '.') continue;
		return false;
	}
	return value.find('\\') == std::string::npos && value.find('/') == std::string::npos;
}

std::string NormalizeDesktopDefaultModuleFlag(std::string value)
{
	value = TrimAscii(std::move(value));
	return IsValidModuleFlagFileName(value) ? value : FallbackDesktopDefaultModule();
}

std::string AutoDisabledReasonLabel(const std::string& reason)
{
	if (reason == "init_exception") return "Init fault";
	if (reason == "request_exception") return "Request fault";
	if (reason == "interface_exception") return "Interface fault";
	if (reason == "pose_exception") return "Pose fault";
	if (reason == "shutdown_exception") return "Shutdown fault";
	if (reason == "unclean_exit_during_module_operation") return "Crash during module operation";
	if (reason == "repeated_unclean_driver_exit") return "Repeated unclean exits";
	if (reason == "pose_pipeline") return "Pose pipeline";
	return reason.empty() ? std::string("Crash safety") : reason;
}

} // namespace

std::string Narrow(const std::wstring& value)
{
	return openvr_pair::common::WideToUtf8(value);
}

ShellContext CreateShellContext()
{
	ShellContext ctx;

	// --- Install dir: prefer exe's own directory over the hard-coded fallback.
	std::wstring exeDir = ExeDir();
	ctx.installDir = exeDir.empty() ? L"C:\\Program Files\\WKOpenVR" : exeDir;

	ctx.profileRoot = openvr_pair::common::WkOpenVrSubdirectoryPath(L"profiles", true);
	ctx.logRoot = openvr_pair::common::WkOpenVrLogsPath(true);
	ctx.desktopDefaultModuleFlagFileName = NormalizeDesktopDefaultModuleFlag(
	    ReadShellSetting(ctx.profileRoot, kDesktopDefaultModuleSetting, FallbackDesktopDefaultModule()));
	ctx.moduleTabOrder = ParseModuleTabOrderSetting(ReadShellSetting(ctx.profileRoot, kModuleTabOrderSetting, ""));

	// --- Driver resources dir: discover SteamVR via registry + libraryfolders.vdf.
	// Fall back to the hard-coded path if any step fails so that known-good
	// installs continue to work even if the discovery logic hits an edge case.
	static const std::wstring kFallbackResources =
	    L"C:\\Program Files (x86)\\Steam\\steamapps\\common\\SteamVR\\drivers\\01wkopenvr\\resources";
	std::wstring resources;
	std::wstring steamvrRoot = DiscoverSteamVRRoot();
	if (!steamvrRoot.empty()) {
		resources = steamvrRoot + L"\\steamapps\\common\\SteamVR\\drivers\\01wkopenvr\\resources";
	}
	if (resources.empty()) {
		resources = kFallbackResources;
	}
	ctx.driverResourceDirs.push_back(resources);
	openvr_pair::common::DiagnosticLog("shell",
	                                   "context_paths install='%s' profile_root='%s' log_root='%s' steamvr_root='%s' "
	                                   "resources='%s' resources_fallback=%d",
	                                   Narrow(ctx.installDir).c_str(), Narrow(ctx.profileRoot).c_str(),
	                                   Narrow(ctx.logRoot).c_str(), Narrow(steamvrRoot).c_str(),
	                                   Narrow(resources).c_str(), steamvrRoot.empty() ? 1 : 0);
	return ctx;
}

std::string ShellContext::DesktopDefaultModuleFlagFileName() const
{
	return desktopDefaultModuleFlagFileName.empty() ? std::string(FallbackDesktopDefaultModule())
	                                                : desktopDefaultModuleFlagFileName;
}

bool ShellContext::SetDesktopDefaultModuleFlagFileName(const char* flagFileName)
{
	if (!flagFileName) return false;
	const std::string normalized = NormalizeDesktopDefaultModuleFlag(flagFileName);
	if (!WriteShellSetting(profileRoot, kDesktopDefaultModuleSetting, normalized)) {
		SetStatus("Desktop default module was not saved.");
		return false;
	}
	desktopDefaultModuleFlagFileName = normalized;
	SetStatus("Desktop default module saved.");
	return true;
}

std::vector<std::string> ShellContext::ModuleTabOrder() const
{
	return moduleTabOrder;
}

bool ShellContext::SetModuleTabOrder(const std::vector<std::string>& order)
{
	std::vector<std::string_view> available;
	available.reserve(order.size());
	for (const std::string& flag : order) {
		available.push_back(flag);
	}
	std::vector<std::string> normalized = ResolveModuleTabOrder(order, available);
	const std::string serialized = SerializeModuleTabOrderSetting(normalized);
	if (!WriteShellSetting(profileRoot, kModuleTabOrderSetting, serialized)) {
		SetStatus("Module tab order was not saved.");
		return false;
	}
	moduleTabOrder = std::move(normalized);
	SetStatus("Module tab order saved.", 2.0);
	return true;
}

std::wstring ShellContext::FlagPath(const char* flagFileName) const
{
	if (driverResourceDirs.empty() || !flagFileName) return {};
	std::wstring flag = openvr_pair::common::Utf8ToWide(flagFileName);
	if (flag.empty()) return {};
	return driverResourceDirs.front() + L"\\" + flag;
}

bool ShellContext::IsFlagPresent(const char* flagFileName) const
{
	for (const auto& dir : driverResourceDirs) {
		std::wstring path = dir + L"\\";
		std::wstring flag = openvr_pair::common::Utf8ToWide(flagFileName);
		if (flag.empty()) continue;
		path += flag;
		DWORD attr = GetFileAttributesW(path.c_str());
		if (attr != INVALID_FILE_ATTRIBUTES && !(attr & FILE_ATTRIBUTE_DIRECTORY)) return true;
	}
	return false;
}

bool ShellContext::IsModuleAutoDisabled(const char* flagFileName) const
{
	const auto* spec = openvr_pair::common::module_safety::FindByFlagFileName(flagFileName ? flagFileName : "");
	return spec ? openvr_pair::common::module_safety::HasAutoDisabledMarker(*spec) : false;
}

std::string ShellContext::ModuleAutoDisabledReason(const char* flagFileName) const
{
	const auto* spec = openvr_pair::common::module_safety::FindByFlagFileName(flagFileName ? flagFileName : "");
	if (!spec || !openvr_pair::common::module_safety::HasAutoDisabledMarker(*spec)) return {};
	return AutoDisabledReasonLabel(openvr_pair::common::module_safety::AutoDisabledReason(*spec));
}

// Maps an enable_<feature>.flag filename to the user-facing label used in
// the matching Start Menu shortcut filename + description.
const wchar_t* ShortcutLabelFor(const char* flagFileName)
{
	if (!flagFileName) return nullptr;
	const module_registry::ModuleInfo* module = module_registry::FindByFlagFileName(flagFileName);
	return module ? module->shortcut_label : nullptr;
}

// Per-feature CLI arg attached to the shortcut. Windows Search dedupes
// Start-Menu entries by the exact (target + arguments) tuple.
const wchar_t* ShortcutArgFor(const char* flagFileName)
{
	if (!flagFileName) return L"";
	const module_registry::ModuleInfo* module = module_registry::FindByFlagFileName(flagFileName);
	return module ? module->shortcut_argument : L"";
}

bool ShellContext::SetFlagPresent(const char* flagFileName, bool present)
{
	std::wstring path = FlagPath(flagFileName);
	if (path.empty()) {
		openvr_pair::common::DiagnosticLog("shell", "set_flag_no_path flag='%s' want_present=%d",
		                                   flagFileName ? flagFileName : "(null)", present ? 1 : 0);
		return false;
	}
	openvr_pair::common::DiagnosticLog("shell", "set_flag_start flag='%s' want_present=%d path='%s'",
	                                   flagFileName ? flagFileName : "(null)", present ? 1 : 0, Narrow(path).c_str());

	std::wstring parent = path.substr(0, path.find_last_of(L"\\/"));

	// Shortcut targets: a Start Menu .lnk pointing at the umbrella exe so
	// Windows search surfaces WKOpenVR when the user types the feature
	// name. The installer drops the umbrella shortcut at install time;
	// these per-feature aliases are managed by this helper so they track
	// the enabled set even after the user toggles features at runtime.
	//
	// $env:ProgramData is resolved by PowerShell at runtime via Join-Path
	// (single-quoting it on the C++ side would prevent expansion). The
	// path components are passed as literal single-quoted strings so any
	// spaces and backslashes survive verbatim.
	const wchar_t* label = ShortcutLabelFor(flagFileName);
	const wchar_t* scArg = ShortcutArgFor(flagFileName);
	std::wstring smRelative = L"Microsoft\\Windows\\Start Menu\\Programs\\WKOpenVR";
	std::wstring scFileName;
	std::wstring exePath;
	std::wstring desc;
	if (label) {
		scFileName = std::wstring(L"WKOpenVR - ") + label + L".lnk";
		exePath = installDir + L"\\WKOpenVR.exe";
		desc = std::wstring(label) + L" in WKOpenVR";
	}

	std::wstring command;
	if (present) {
		// New-Item does NOT accept -LiteralPath in Windows PowerShell 5.1 (the
		// shipping host). It only has -Path, which is fine here because driver
		// resources paths never contain wildcards. Set-Content does accept
		// -LiteralPath; we keep that to defeat any accidental wildcard parse on
		// the destination filename.
		command = L"New-Item -ItemType Directory -Force -Path " + common::QuotePowerShellLiteral(parent) +
		          L" | Out-Null; Set-Content -LiteralPath " + common::QuotePowerShellLiteral(path) +
		          L" -Value enabled -NoNewline";
		if (label) {
			// COM-creating a shortcut from PowerShell is the simplest way to
			// produce a .lnk without shipping a separate helper exe. The
			// shortcut points at the installed WKOpenVR.exe (resolved via
			// the ShellContext install dir captured at process start), uses
			// the same exe for its icon, and tags a description so search
			// previews read "<Feature> in WKOpenVR".
			command += L"; $smDir = Join-Path $env:ProgramData " + common::QuotePowerShellLiteral(smRelative);
			command += L"; New-Item -ItemType Directory -Force -Path $smDir | Out-Null";
			command += L"; $scPath = Join-Path $smDir " + common::QuotePowerShellLiteral(scFileName);
			command += L"; $wsh = New-Object -ComObject WScript.Shell";
			command += L"; $sc = $wsh.CreateShortcut($scPath)";
			command += L"; $sc.TargetPath = " + common::QuotePowerShellLiteral(exePath);
			command += L"; $sc.Arguments = " + common::QuotePowerShellLiteral(scArg);
			command += L"; $sc.IconLocation = " + common::QuotePowerShellLiteral(exePath + L",0");
			command += L"; $sc.Description = " + common::QuotePowerShellLiteral(desc);
			command += L"; $sc.Save()";
		}
	}
	else {
		command = L"if (Test-Path -LiteralPath " + common::QuotePowerShellLiteral(path) +
		          L") { Remove-Item -LiteralPath " + common::QuotePowerShellLiteral(path) + L" -Force }";
		if (label) {
			command += L"; $smDir = Join-Path $env:ProgramData " + common::QuotePowerShellLiteral(smRelative);
			command += L"; $scPath = Join-Path $smDir " + common::QuotePowerShellLiteral(scFileName);
			command += L"; if (Test-Path -LiteralPath $scPath) { Remove-Item -LiteralPath $scPath -Force }";
		}
	}

	// -EncodedCommand expects base64-encoded UTF-16 LE. We use it instead of
	// -Command '<script>' because ShellExecuteEx with the runas verb routes
	// through UAC's launcher, which re-parses lpParameters and silently strips
	// single-quoted blocks. The symptom was an elevated powershell.exe that
	// dropped to an interactive prompt with no -Command -- the helper would
	// hang indefinitely while the Modules tab showed "Enabling..." forever.
	// EncodedCommand has no special characters in the cmd line so re-parsing
	// is a no-op.
	std::wstring args =
	    L"-NoProfile -ExecutionPolicy Bypass -EncodedCommand " + common::EncodePowerShellCommand(command);

	SHELLEXECUTEINFOW sei{};
	sei.cbSize = sizeof(sei);
	sei.fMask = SEE_MASK_NOCLOSEPROCESS;
	sei.lpVerb = L"runas";
	sei.lpFile = L"powershell.exe";
	sei.lpParameters = args.c_str();
	sei.nShow = SW_HIDE;
	if (!ShellExecuteExW(&sei) || sei.hProcess == nullptr) {
		// User dismissed the consent prompt before it even ran, or the
		// shell refused to launch the helper. Either way there is no
		// process to wait on.
		DWORD err = GetLastError();
		openvr_pair::common::DiagnosticLog("shell", "set_flag_launch_failed flag='%s' want_present=%d error=%lu",
		                                   flagFileName ? flagFileName : "(null)", present ? 1 : 0, err);
		if (err == 0) {
			SetStatus("Module change cancelled.");
		}
		else {
			std::ostringstream oss;
			oss << "Module change was not started (admin helper error 0x" << std::hex << std::uppercase << err << ")";
			SetStatus(oss.str());
		}
		return false;
	}

	g_pendingToggles.push_back({flagFileName, present, sei.hProcess});
	openvr_pair::common::DiagnosticLog("shell", "set_flag_queued flag='%s' want_present=%d pending_count=%zu",
	                                   flagFileName ? flagFileName : "(null)", present ? 1 : 0,
	                                   g_pendingToggles.size());
	SetStatus("Module change queued. SteamVR will pick up the new state the next time it loads the driver.");
	return true;
}

bool ShellContext::IsTogglePending(const char* flagFileName) const
{
	if (!flagFileName) return false;
	for (const auto& entry : g_pendingToggles) {
		if (entry.flagFileName == flagFileName) return true;
	}
	return false;
}

void ShellContext::TickToggles()
{
	for (auto it = g_pendingToggles.begin(); it != g_pendingToggles.end();) {
		if (WaitForSingleObject(it->process, 0) != WAIT_OBJECT_0) {
			++it;
			continue;
		}
		DWORD exitCode = 0;
		if (!GetExitCodeProcess(it->process, &exitCode)) {
			exitCode = 1;
		}
		const bool present = IsFlagPresent(it->flagFileName.c_str());
		openvr_pair::common::DiagnosticLog(
		    "shell", "set_flag_finished flag='%s' want_present=%d present=%d exit=0x%08lX", it->flagFileName.c_str(),
		    it->wantPresent ? 1 : 0, present ? 1 : 0, static_cast<unsigned long>(exitCode));
		if (exitCode == 0 && present == it->wantPresent) {
			const bool wasAutoDisabled = it->wantPresent && IsModuleAutoDisabled(it->flagFileName.c_str());
			if (it->wantPresent && openvr_pair::common::module_safety::ClearAutoDisabledForFlag(it->flagFileName)) {
				if (wasAutoDisabled) {
					openvr_pair::common::DiagnosticLog("shell", "module_safety_reenable flag='%s'",
					                                   it->flagFileName.c_str());
				}
			}
			SetStatus("Module change applied. SteamVR will pick up the new state the next time it loads the driver.");
		}
		else if (present == it->wantPresent) {
			std::ostringstream oss;
			if (exitCode == 0) {
				oss << (it->wantPresent ? "Enable did not apply: flag file was not created."
				                        : "Disable did not apply: flag file was still present.");
			}
			else {
				oss << (it->wantPresent ? "Enable completed, but helper exit was non-zero (0x"
				                        : "Disable completed, but helper exit was non-zero (0x");
				oss << std::hex << std::uppercase << exitCode << ").";
			}
			oss << " Check that the helper process could write Program Files and that SteamVR is not holding"
			    << (it->wantPresent ? " the driver folder open." : " the module state files.");
			SetStatus(oss.str());
		}
		else {
			SetStatus(std::string(it->wantPresent
			                          ? "Enable did not apply -- helper finished but state is still disabled."
			                          : "Disable did not apply -- helper finished but state is still enabled."));
		}
		CloseHandle(it->process);
		it = g_pendingToggles.erase(it);
	}
}

void ShellContext::TickStatus()
{
	if (ShouldClearTransientStatus(ShellNowSeconds(), statusClearAtSeconds)) {
		ClearStatus();
	}
}

void ShellContext::ClearStatus()
{
	status.clear();
	statusClearAtSeconds = 0.0;
}

void ShellContext::SetStatus(std::string message, double ttlSeconds)
{
	status = std::move(message);
	if (status.empty()) {
		statusClearAtSeconds = 0.0;
		return;
	}
	if (ttlSeconds < 0.0) {
		ttlSeconds = kShellStatusDefaultSeconds;
	}
	statusClearAtSeconds = ttlSeconds > 0.0 ? ShellNowSeconds() + ttlSeconds : 0.0;
}

} // namespace openvr_pair::overlay
