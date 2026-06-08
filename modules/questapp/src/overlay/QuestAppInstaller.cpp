#include "QuestAppInstaller.h"

#include "AdbSetupWizard.h"
#include "QuestCompanionProtocol.h"
#include "QuestAppCatalog.h"
#include "Win32Paths.h"
#include "Win32Text.h"

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <winhttp.h>

#include <algorithm>
#include <chrono>
#include <cctype>
#include <filesystem>
#include <sstream>
#include <string>

namespace wkopenvr::questapp {
namespace {

std::string Narrow(const std::wstring& value)
{
	return openvr_pair::common::WideToUtf8(value);
}

std::wstring QuoteForCommandLine(const std::wstring& value)
{
	std::wstring out = L"\"";
	for (wchar_t ch : value) {
		if (ch == L'"')
			out += L"\\\"";
		else
			out += ch;
	}
	out += L"\"";
	return out;
}

OperationResult RunPowerShellScript(const std::wstring& scriptPath, const std::vector<std::wstring>& args,
                                    DWORD timeoutMs)
{
	OperationResult result;
	if (scriptPath.empty() || GetFileAttributesW(scriptPath.c_str()) == INVALID_FILE_ATTRIBUTES) {
		result.message = "Install script is missing.";
		return result;
	}

	std::wstring command = L"powershell.exe -NoProfile -ExecutionPolicy Bypass -File ";
	command += QuoteForCommandLine(scriptPath);
	for (const auto& arg : args) {
		command += L" ";
		command += QuoteForCommandLine(arg);
	}

	STARTUPINFOW si{};
	si.cb = sizeof(si);
	si.dwFlags = STARTF_USESHOWWINDOW;
	si.wShowWindow = SW_HIDE;
	PROCESS_INFORMATION pi{};

	std::wstring mutableCommand = command;
	if (!CreateProcessW(nullptr, mutableCommand.data(), nullptr, nullptr, FALSE, CREATE_NO_WINDOW, nullptr, nullptr,
	                    &si, &pi)) {
		std::ostringstream oss;
		oss << "Could not start PowerShell installer (error 0x" << std::hex << std::uppercase << GetLastError() << ").";
		result.message = oss.str();
		return result;
	}

	const DWORD wait = WaitForSingleObject(pi.hProcess, timeoutMs);
	if (wait == WAIT_TIMEOUT) {
		TerminateProcess(pi.hProcess, 124);
		CloseHandle(pi.hThread);
		CloseHandle(pi.hProcess);
		result.message = "Installer timed out.";
		return result;
	}

	DWORD exitCode = 1;
	GetExitCodeProcess(pi.hProcess, &exitCode);
	CloseHandle(pi.hThread);
	CloseHandle(pi.hProcess);

	result.ok = (wait == WAIT_OBJECT_0 && exitCode == 0);
	if (result.ok) {
		result.message = "Platform-tools installed.";
	}
	else {
		std::ostringstream oss;
		oss << "Platform-tools installer failed (exit " << exitCode << ").";
		result.message = oss.str();
	}
	return result;
}

bool ContainsNoCase(std::string text, std::string needle)
{
	std::transform(text.begin(), text.end(), text.begin(),
	               [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
	std::transform(needle.begin(), needle.end(), needle.begin(),
	               [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
	return text.find(needle) != std::string::npos;
}

std::string StripEndpointPort(const std::string& endpoint)
{
	const size_t colon = endpoint.find(':');
	return colon == std::string::npos ? endpoint : endpoint.substr(0, colon);
}

std::string ExtractRouteSourceIp(const std::string& routeOutput)
{
	std::istringstream fields(routeOutput);
	std::string token;
	while (fields >> token) {
		if (token != "src") continue;
		std::string ip;
		if (fields >> ip) return ip;
	}
	return {};
}

std::string UrlEncode(const std::string& value)
{
	static constexpr char kHex[] = "0123456789ABCDEF";
	std::string out;
	out.reserve(value.size());
	for (unsigned char c : value) {
		const bool safe = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '-' ||
		                  c == '_' || c == '.' || c == '~';
		if (safe) {
			out.push_back(static_cast<char>(c));
		}
		else {
			out.push_back('%');
			out.push_back(kHex[(c >> 4) & 0x0F]);
			out.push_back(kHex[c & 0x0F]);
		}
	}
	return out;
}

std::vector<std::string> CompanionServiceCommand(const QuestAppConfig& cfg, const QuestCompanionSettings* settings)
{
	std::vector<std::string> args = {
	    "shell",
	    "am",
	    "start-foreground-service",
	    "-n",
	    "org.wkopenvr.quest/.QuestCompanionService",
	    "--es",
	    "wkopenvr_pairing_key",
	    cfg.pairingKey,
	};
	if (!settings) return args;

	args.push_back("--ez");
	args.push_back("wkopenvr_auto_launch_enabled");
	args.push_back(settings->autoLaunchEnabled ? "true" : "false");
	if (!settings->selectedPackage.empty()) {
		args.push_back("--es");
		args.push_back("wkopenvr_selected_package");
		args.push_back(settings->selectedPackage);
	}
	if (!settings->selectedActivity.empty()) {
		args.push_back("--es");
		args.push_back("wkopenvr_selected_activity");
		args.push_back(settings->selectedActivity);
	}
	return args;
}

std::vector<std::string> TargetAdbArgs(std::string serial, std::vector<std::string> args)
{
	if (!serial.empty()) {
		args.insert(args.begin(), serial);
		args.insert(args.begin(), "-s");
	}
	return args;
}

std::string FindPreferredQuestSerial(AdbController& adb, bool usbOnly)
{
	adb.RefreshResolvedBinaryPath();
	auto devices = adb.Run({"devices", "-l"}, std::chrono::seconds(8));
	if (devices.timedOut || devices.exitCode != 0) return {};

	std::string serial = FindAuthorizedUsbQuestSerial(devices.out);
	if (!serial.empty() || usbOnly) return serial;
	return FindAuthorizedWifiQuestSerial(devices.out);
}

std::wstring CompanionConfigPath(const QuestAppConfig& cfg, const QuestCompanionSettings& settings)
{
	std::string path = "/config?key=" + UrlEncode(cfg.pairingKey) +
	                   "&autoLaunch=" + std::string(settings.autoLaunchEnabled ? "true" : "false");
	if (!settings.selectedPackage.empty()) {
		path += "&package=" + UrlEncode(settings.selectedPackage);
	}
	if (!settings.selectedActivity.empty()) {
		path += "&activity=" + UrlEncode(settings.selectedActivity);
	}
	return openvr_pair::common::Utf8ToWide(path);
}

OperationResult HttpGetCompanion(const std::wstring& host, const std::wstring& path, std::string& body)
{
	OperationResult out;
	HINTERNET session = WinHttpOpen(L"WKOpenVR Quest App/1.0", WINHTTP_ACCESS_TYPE_DEFAULT_PROXY, WINHTTP_NO_PROXY_NAME,
	                                WINHTTP_NO_PROXY_BYPASS, 0);
	if (!session) {
		out.message = "Could not create HTTP session.";
		return out;
	}

	HINTERNET connect = WinHttpConnect(session, host.c_str(), 39789, 0);
	if (!connect) {
		WinHttpCloseHandle(session);
		out.message = "Could not connect to companion endpoint.";
		return out;
	}

	HINTERNET request =
	    WinHttpOpenRequest(connect, L"GET", path.c_str(), nullptr, WINHTTP_NO_REFERER, WINHTTP_DEFAULT_ACCEPT_TYPES, 0);
	if (!request) {
		WinHttpCloseHandle(connect);
		WinHttpCloseHandle(session);
		out.message = "Could not create companion HTTP request.";
		return out;
	}

	BOOL ok = WinHttpSendRequest(request, WINHTTP_NO_ADDITIONAL_HEADERS, 0, WINHTTP_NO_REQUEST_DATA, 0, 0, 0);
	if (ok) ok = WinHttpReceiveResponse(request, nullptr);

	DWORD status = 0;
	DWORD statusSize = sizeof(status);
	if (ok) {
		ok = WinHttpQueryHeaders(request, WINHTTP_QUERY_STATUS_CODE | WINHTTP_QUERY_FLAG_NUMBER,
		                         WINHTTP_HEADER_NAME_BY_INDEX, &status, &statusSize, WINHTTP_NO_HEADER_INDEX);
	}

	if (ok) {
		body.clear();
		for (;;) {
			DWORD available = 0;
			if (!WinHttpQueryDataAvailable(request, &available) || available == 0) break;
			std::string chunk;
			chunk.resize(available);
			DWORD read = 0;
			if (!WinHttpReadData(request, chunk.data(), available, &read)) break;
			chunk.resize(read);
			body += chunk;
		}
	}

	WinHttpCloseHandle(request);
	WinHttpCloseHandle(connect);
	WinHttpCloseHandle(session);

	if (!ok) {
		out.message = "Companion HTTP request failed.";
		return out;
	}
	if (status != 200) {
		std::ostringstream oss;
		oss << "Companion rejected settings query (HTTP " << status << ").";
		out.message = oss.str();
		return out;
	}
	out.ok = true;
	out.message = "Companion settings loaded.";
	return out;
}

OperationResult TryHttpCompanion(const QuestAppConfig& cfg, const std::wstring& path, std::string& body)
{
	if (cfg.companionHost.empty()) {
		return {false, "Companion Wi-Fi address is not known yet."};
	}
	return HttpGetCompanion(openvr_pair::common::Utf8ToWide(cfg.companionHost), path, body);
}

bool RefreshCompanionHostFromAdb(AdbController& adb, QuestAppConfig& cfg)
{
	adb.RefreshResolvedBinaryPath();

	auto devices = adb.Run({"devices", "-l"}, std::chrono::seconds(8));
	if (devices.timedOut || devices.exitCode != 0) return false;

	std::string host = StripEndpointPort(FindAuthorizedWifiQuestSerial(devices.out));
	if (host.empty()) {
		const std::string usbSerial = FindAuthorizedUsbQuestSerial(devices.out);
		if (!usbSerial.empty()) {
			auto route = adb.Run({"-s", usbSerial, "shell", "ip", "route"}, std::chrono::seconds(5));
			if (!route.timedOut && route.exitCode == 0) {
				host = ExtractRouteSourceIp(route.out);
			}
		}
	}

	if (host.empty()) return false;
	if (cfg.companionHost != host) {
		cfg.companionHost = host;
		SaveQuestAppConfig(cfg);
	}
	return true;
}

} // namespace

std::wstring QuestAppDataDir(bool create)
{
	return openvr_pair::common::WkOpenVrSubdirectoryPath(L"questapp", create);
}

std::wstring PlatformToolsDir(bool create)
{
	return openvr_pair::common::WkOpenVrSubdirectoryPath(L"questapp\\platform-tools", create);
}

std::wstring CompanionApkPath(const openvr_pair::overlay::ShellContext& context)
{
	return context.installDir + L"\\resources\\questapp\\WKOpenVRQuestCompanion.apk";
}

std::wstring InstallPlatformToolsScriptPath(const openvr_pair::overlay::ShellContext& context)
{
	return context.installDir + L"\\resources\\questapp\\install-platform-tools.ps1";
}

bool PlatformToolsInstalled()
{
	const std::wstring path = PlatformToolsDir(false) + L"\\adb.exe";
	return GetFileAttributesW(path.c_str()) != INVALID_FILE_ATTRIBUTES;
}

bool CompanionApkAvailable(const openvr_pair::overlay::ShellContext& context)
{
	const std::wstring path = CompanionApkPath(context);
	return GetFileAttributesW(path.c_str()) != INVALID_FILE_ATTRIBUTES;
}

OperationResult InstallPlatformTools(const openvr_pair::overlay::ShellContext& context)
{
	const std::wstring root = QuestAppDataDir(true);
	if (root.empty()) return {false, "Could not create Quest App data folder."};
	const std::wstring script = InstallPlatformToolsScriptPath(context);
	return RunPowerShellScript(script, {root}, 5 * 60 * 1000);
}

OperationResult InstallCompanionApp(const openvr_pair::overlay::ShellContext& context, AdbController& adb,
                                    QuestAppConfig& cfg)
{
	OperationResult out;
	if (!CompanionApkAvailable(context)) {
		out.message = "Companion APK is not bundled in this build yet.";
		return out;
	}
	if (!PlatformToolsInstalled()) {
		out.message = "Install ADB platform-tools first.";
		return out;
	}

	adb.RefreshResolvedBinaryPath();

	const bool mustClearHeadsetKey = cfg.companionInstalled && !IsValidPairingKey(cfg.pairingKey);
	const std::string usbSerial = FindPreferredQuestSerial(adb, true);
	if (usbSerial.empty()) {
		out.message = "Connect an authorized USB Quest to install the companion.";
		return out;
	}
	if (mustClearHeadsetKey) {
		auto uninstall =
		    adb.Run(TargetAdbArgs(usbSerial, {"uninstall", "org.wkopenvr.quest"}), std::chrono::seconds(30));
		const bool alreadyGone =
		    ContainsNoCase(uninstall.out, "not installed") || ContainsNoCase(uninstall.err, "not installed") ||
		    ContainsNoCase(uninstall.out, "unknown package") || ContainsNoCase(uninstall.err, "unknown package");
		if (uninstall.timedOut || (uninstall.exitCode != 0 && !alreadyGone)) {
			out.message = "Could not remove the old companion before creating a new install key.";
			return out;
		}
		cfg.companionInstalled = false;
	}

	if (!IsValidPairingKey(cfg.pairingKey)) {
		cfg.pairingKey = GeneratePairingKey();
	}

	const std::string apk = Narrow(CompanionApkPath(context));
	auto install = adb.Run(TargetAdbArgs(usbSerial, {"install", "-r", apk}), std::chrono::minutes(2));
	if (install.timedOut || install.exitCode != 0 ||
	    (!ContainsNoCase(install.out, "success") && !ContainsNoCase(install.err, "success"))) {
		out.message = "Companion install failed.";
		return out;
	}

	auto configure = adb.Run(TargetAdbArgs(usbSerial, CompanionServiceCommand(cfg, nullptr)), std::chrono::seconds(10));
	if (configure.timedOut || configure.exitCode != 0) {
		out.message = "Companion installed, but the initial key setup did not confirm.";
		return out;
	}

	cfg.companionInstalled = true;
	RefreshCompanionHostFromAdb(adb, cfg);
	SaveQuestAppConfig(cfg);
	out.ok = true;
	out.message = "Companion app installed and paired.";
	return out;
}

OperationResult SyncCompanionConfig(AdbController& adb, const QuestAppConfig& cfg,
                                    const QuestCompanionSettings& settings)
{
	OperationResult out;
	if (!cfg.companionInstalled) {
		out.message = "Install the companion app first.";
		return out;
	}
	if (!IsValidPairingKey(cfg.pairingKey)) {
		out.message = "Install key is missing. Reinstall the companion to create a new key.";
		return out;
	}

	QuestAppConfig mutableCfg = cfg;
	std::string body;
	OperationResult http = TryHttpCompanion(mutableCfg, CompanionConfigPath(mutableCfg, settings), body);
	if (!http.ok && RefreshCompanionHostFromAdb(adb, mutableCfg)) {
		http = TryHttpCompanion(mutableCfg, CompanionConfigPath(mutableCfg, settings), body);
	}
	if (http.ok) {
		out.ok = true;
		out.message = "Companion settings sent over Wi-Fi.";
		return out;
	}

	adb.RefreshResolvedBinaryPath();
	const std::string serial = FindPreferredQuestSerial(adb, false);
	auto result = adb.Run(TargetAdbArgs(serial, CompanionServiceCommand(cfg, &settings)), std::chrono::seconds(10));
	if (result.timedOut || result.exitCode != 0) {
		out.message = "Companion settings did not confirm over Wi-Fi or ADB.";
		return out;
	}
	out.ok = true;
	out.message = "Companion settings sent to headset.";
	return out;
}

SettingsQueryResult QueryCompanionSettings(AdbController& adb, const QuestAppConfig& cfg)
{
	SettingsQueryResult out;
	if (!cfg.companionInstalled) {
		out.result.message = "Install the companion app first.";
		return out;
	}
	if (!IsValidPairingKey(cfg.pairingKey)) {
		out.result.message = "Install key is missing. Reinstall the companion to create a new key.";
		return out;
	}

	QuestAppConfig mutableCfg = cfg;
	adb.RefreshResolvedBinaryPath();
	std::string body;
	std::wstring path = L"/settings?key=" + openvr_pair::common::Utf8ToWide(cfg.pairingKey);
	out.result = TryHttpCompanion(mutableCfg, path, body);
	if (!out.result.ok && RefreshCompanionHostFromAdb(adb, mutableCfg)) {
		out.result = TryHttpCompanion(mutableCfg, path, body);
	}
	if (!out.result.ok) {
		const std::string serial = FindPreferredQuestSerial(adb, false);
		auto forward = adb.Run(TargetAdbArgs(serial, {"forward", "tcp:39789", "tcp:39789"}), std::chrono::seconds(10));
		if (forward.timedOut || forward.exitCode != 0) {
			out.result.message = "Could not reach the companion over Wi-Fi or ADB.";
			return out;
		}
		out.result = HttpGetCompanion(L"127.0.0.1", path, body);
	}
	if (!out.result.ok) return out;

	if (!ParseCompanionSettingsJson(body, out.settings)) {
		out.result.ok = false;
		out.result.message = "Companion settings response was not valid JSON.";
		return out;
	}
	return out;
}

OperationResult UninstallCompanionApp(AdbController& adb, QuestAppConfig& cfg)
{
	OperationResult out;
	adb.RefreshResolvedBinaryPath();
	const std::string serial = FindPreferredQuestSerial(adb, false);
	auto uninstall = adb.Run(TargetAdbArgs(serial, {"uninstall", "org.wkopenvr.quest"}), std::chrono::seconds(30));
	if (uninstall.timedOut || uninstall.exitCode != 0) {
		out.message = "Companion uninstall did not confirm.";
		return out;
	}

	cfg.companionInstalled = false;
	cfg.pairingKey.clear();
	SaveQuestAppConfig(cfg);
	out.ok = true;
	out.message = "Companion app uninstalled. Reinstalling will create a new key.";
	return out;
}

std::vector<QuestLaunchTarget> QueryInstalledPackages(AdbController& adb)
{
	std::vector<QuestLaunchTarget> out;
	adb.RefreshResolvedBinaryPath();
	const std::string serial = FindPreferredQuestSerial(adb, false);
	auto result = adb.Run(TargetAdbArgs(serial, {"shell", "pm", "list", "packages"}), std::chrono::seconds(10));
	if (result.timedOut || result.exitCode != 0) return out;

	std::istringstream lines(result.out);
	std::string line;
	while (std::getline(lines, line)) {
		while (!line.empty() && (line.back() == '\r' || line.back() == '\n')) {
			line.pop_back();
		}
		constexpr const char* prefix = "package:";
		if (line.rfind(prefix, 0) != 0) continue;
		const std::string pkg = line.substr(std::char_traits<char>::length(prefix));
		if (pkg.empty()) continue;
		out.push_back({pkg, pkg, {}, false});
	}
	return out;
}

} // namespace wkopenvr::questapp
