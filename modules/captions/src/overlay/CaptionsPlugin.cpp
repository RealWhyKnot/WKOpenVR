#include "CaptionsPlugin.h"
#include "CaptionsConfig.h"
#include "ShellContext.h"
#include "CaptionsIpcClient.h"
#include "CaptionsTab.h"
#include "UiHelpers.h"
#include "Win32Paths.h"

#include <imgui/imgui.h>

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

#include <chrono>
#include <cstdio>
#include <cstring>
#include <exception>
#include <sstream>
#include <string>
#include <vector>

using Clock = std::chrono::steady_clock;

namespace {

std::wstring WidenAscii(const std::string& value)
{
	return std::wstring(value.begin(), value.end());
}

std::wstring QuoteArg(const std::wstring& value)
{
	std::wstring out;
	out.reserve(value.size() + 2);
	out.push_back(L'"');
	unsigned backslashes = 0;
	for (wchar_t ch : value) {
		if (ch == L'\\') {
			++backslashes;
			continue;
		}
		if (ch == L'"') {
			out.append(backslashes * 2 + 1, L'\\');
			out.push_back(ch);
			backslashes = 0;
			continue;
		}
		if (backslashes) {
			out.append(backslashes, L'\\');
			backslashes = 0;
		}
		out.push_back(ch);
	}
	if (backslashes) {
		out.append(backslashes * 2, L'\\');
	}
	out.push_back(L'"');
	return out;
}

bool FileExists(const std::wstring& path)
{
	if (path.empty()) return false;
	DWORD attr = GetFileAttributesW(path.c_str());
	return attr != INVALID_FILE_ATTRIBUTES && (attr & FILE_ATTRIBUTE_DIRECTORY) == 0;
}

// Write the chosen capture endpoint id to the host-readable audio_input.txt in
// the shared captions dir. Atomic tmp+rename so the host never reads a partial
// id. An empty id is written as an empty file (host treats it as "default").
void WriteAudioInputFile(const std::string& deviceId)
{
	std::wstring dir = openvr_pair::common::WkOpenVrSubdirectoryPath(L"captions", true);
	if (dir.empty()) return;
	std::wstring path = dir + L"\\audio_input.txt";
	std::wstring tmp = path + L".tmp";

	HANDLE h = CreateFileW(tmp.c_str(), GENERIC_WRITE, 0, nullptr, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
	if (h == INVALID_HANDLE_VALUE) return;
	DWORD written = 0;
	BOOL ok = TRUE;
	if (!deviceId.empty()) {
		ok = WriteFile(h, deviceId.data(), (DWORD)deviceId.size(), &written, nullptr);
	}
	CloseHandle(h);
	if (!ok) {
		DeleteFileW(tmp.c_str());
		return;
	}
	MoveFileExW(tmp.c_str(), path.c_str(), MOVEFILE_REPLACE_EXISTING);
}

} // namespace

CaptionsPlugin::CaptionsPlugin()
{
	observed_ipc_generation_ = ipc_.ConnectionGeneration();
	// Hydrate runtime-mutable settings from disk. Missing file -> defaults
	// (the constructor in the header already seeds reasonable defaults).
	CaptionsConfig loaded = LoadCaptionsConfig();
	sidecar_enabled_ = loaded.sidecar_enabled;
	mode_ = loaded.mode;
	always_on_consented_ = loaded.always_on_consented;
	source_lang_ = loaded.source_lang;
	target_lang_ = loaded.target_lang;
	chatbox_enabled_ = loaded.chatbox_enabled;
	chatbox_address_ = loaded.chatbox_address;
	notify_sound_ = loaded.notify_sound;
	realtime_flags_ = loaded.realtime_flags;
	speech_model_ = loaded.speech_model;
	input_device_ = loaded.input_device;
	// Mirror the saved selection to the host-readable file in case it drifted
	// from captions.txt (e.g. a manual edit or an interrupted earlier write).
	WriteAudioInputFile(input_device_);
}

void CaptionsPlugin::Persist()
{
	CaptionsConfig cfg;
	cfg.sidecar_enabled = sidecar_enabled_;
	cfg.mode = mode_;
	cfg.always_on_consented = always_on_consented_;
	cfg.source_lang = source_lang_;
	cfg.target_lang = target_lang_;
	cfg.chatbox_enabled = chatbox_enabled_;
	cfg.chatbox_address = chatbox_address_;
	cfg.notify_sound = notify_sound_;
	cfg.realtime_flags = realtime_flags_;
	cfg.speech_model = speech_model_;
	cfg.input_device = input_device_;
	SaveCaptionsConfig(cfg);
}

void CaptionsPlugin::SetMode(int m)
{
	mode_ = m;
	Persist();
}
void CaptionsPlugin::SetAlwaysOnConsented(bool v)
{
	always_on_consented_ = v;
	Persist();
}
void CaptionsPlugin::SetSourceLang(const std::string& s)
{
	source_lang_ = s;
	Persist();
}
void CaptionsPlugin::SetTargetLang(const std::string& s)
{
	target_lang_ = s;
	Persist();
}
void CaptionsPlugin::SetSidecarEnabled(bool v)
{
	sidecar_enabled_ = v;
	Persist();
}
void CaptionsPlugin::SetChatboxEnabled(bool v)
{
	chatbox_enabled_ = v;
	Persist();
}
void CaptionsPlugin::SetChatboxAddress(const std::string& s)
{
	chatbox_address_ = s;
	Persist();
}
void CaptionsPlugin::SetNotifySound(bool v)
{
	notify_sound_ = v;
	Persist();
}
void CaptionsPlugin::SetRealtimeOption(uint8_t flag, bool enabled)
{
	realtime_flags_ = captions::SetCaptionsRealtimeFlag(realtime_flags_, flag, enabled);
	Persist();
}
void CaptionsPlugin::SetRealtimeOptionMask(uint8_t mask, bool enabled)
{
	realtime_flags_ = captions::SetCaptionsRealtimeMask(realtime_flags_, mask, enabled);
	Persist();
}
void CaptionsPlugin::SetSpeechModel(uint8_t model)
{
	speech_model_ = captions::NormalizeCaptionsSpeechModel(model);
	Persist();
}
void CaptionsPlugin::SetInputDevice(const std::string& endpointId)
{
	input_device_ = endpointId;
	Persist();                          // captions.txt (UI state across restarts)
	WriteAudioInputFile(input_device_); // host-readable file the capture loop polls
}

void CaptionsPlugin::OnStart(openvr_pair::overlay::ShellContext& ctx)
{
	RefreshPackResourcePaths(ctx);

	try {
		ipc_.Connect();
		PushConfigToDriver();
	}
	catch (const std::exception& e) {
		last_error_ = std::string("Captions IPC: ") + e.what();
	}

	last_connection_check_ = Clock::now();
}

void CaptionsPlugin::OnShutdown(openvr_pair::overlay::ShellContext&)
{
	if (pack_process_) {
		CloseHandle(static_cast<HANDLE>(pack_process_));
		pack_process_ = nullptr;
	}
	ipc_.Close();
}

void CaptionsPlugin::Tick(openvr_pair::overlay::ShellContext&)
{
	PollPackAction();

	const auto now = Clock::now();
	if (now - last_connection_check_ >= std::chrono::seconds(1)) {
		MaintainDriverConnection();
		last_connection_check_ = now;
		PollSupervisorStatus();
	}
	host_status_.Tick();
	const auto& snap = host_status_.Snapshot();
	if (snap.valid) {
		preview_history_.Observe(snap.captions_completed, snap.last_transcript, snap.last_translation);
	}
}

void CaptionsPlugin::DrawTab(openvr_pair::overlay::ShellContext& ctx)
{
	RefreshPackResourcePaths(ctx);
	DrawStatusBanner();
	captions::ui::DrawCaptionsTab(*this);
}

void CaptionsPlugin::PushConfigToDriver()
{
	if (!ipc_.IsConnected()) {
		last_error_ = "Not connected to the Captions driver.";
		return;
	}
	try {
		protocol::Request req(protocol::RequestSetCaptionsConfig);
		auto& cfg = req.setCaptionsConfig;
		memset(&cfg, 0, sizeof(cfg));

		cfg.master_enabled = sidecar_enabled_ ? 1 : 0;
		cfg.mode = static_cast<uint8_t>(mode_);
		cfg.notify_sound = notify_sound_ ? 1 : 0;
		cfg.chatbox_enabled = chatbox_enabled_ ? 1 : 0;
		cfg.realtime_flags = realtime_flags_;
		cfg.speech_model = speech_model_;
		cfg.chatbox_port = 9000;

		std::snprintf(cfg.source_lang, sizeof(cfg.source_lang), "%s", source_lang_.c_str());
		std::snprintf(cfg.target_lang, sizeof(cfg.target_lang), "%s", target_lang_.c_str());
		std::snprintf(cfg.chatbox_address, sizeof(cfg.chatbox_address), "%s", chatbox_address_.c_str());

		auto resp = ipc_.SendBlocking(req);
		if (resp.type != protocol::ResponseSuccess) {
			last_error_ = "Driver rejected Captions config (type=" + std::to_string(resp.type) + ")";
			return;
		}
		last_error_.clear();
	}
	catch (const std::exception& e) {
		last_error_ = std::string("IPC error: ") + e.what();
		ipc_.Close();
	}
}

void CaptionsPlugin::SendRestartHost()
{
	if (!ipc_.IsConnected()) return;
	try {
		protocol::Request req(protocol::RequestCaptionsRestartHost);
		ipc_.SendBlocking(req);
	}
	catch (...) {
		ipc_.Close();
	}
}

void CaptionsPlugin::InstallSpeechPack()
{
	StartPackAction(captions::CaptionsSpeechModelPackId(speech_model_), false);
}

void CaptionsPlugin::UninstallSpeechPack()
{
	StartPackAction(captions::CaptionsSpeechModelPackId(speech_model_), true);
}

void CaptionsPlugin::InstallTranslationPack()
{
	const std::string pack = CurrentTranslationPackId();
	if (pack.empty()) {
		pack_status_ = "No managed translation pack for the selected pair.";
		return;
	}
	StartPackAction(pack, false);
}

void CaptionsPlugin::UninstallTranslationPack()
{
	const std::string pack = CurrentTranslationPackId();
	if (pack.empty()) {
		pack_status_ = "No managed translation pack for the selected pair.";
		return;
	}
	StartPackAction(pack, true);
}

std::string CaptionsPlugin::CurrentTranslationPackId() const
{
	if (target_lang_.empty()) return {};

	const std::string src = (source_lang_.empty() || source_lang_ == "auto") ? "en" : source_lang_;
	if (src != "en" || target_lang_ == "en") return {};

	static const char* kManagedTargets[] = {"de", "es", "fr", "ru", "zh"};
	for (const char* target : kManagedTargets) {
		if (target_lang_ == target) {
			return std::string("translation-en-") + target_lang_;
		}
	}
	return {};
}

bool CaptionsPlugin::HasManagedTranslationPack() const
{
	return !CurrentTranslationPackId().empty();
}

void CaptionsPlugin::RefreshPackResourcePaths(const openvr_pair::overlay::ShellContext& ctx)
{
	if (ctx.driverResourceDirs.empty()) return;
	const std::wstring root = ctx.driverResourceDirs.front() + L"\\captions\\host\\resources";
	pack_script_path_ = root + L"\\install-captions-pack.ps1";
	pack_manifest_path_ = root + L"\\captions-packs.json";
}

void CaptionsPlugin::StartPackAction(const std::string& pack_id, bool uninstall)
{
	if (pack_process_) return;
	if (pack_id.empty()) {
		pack_status_ = "No captions pack selected.";
		return;
	}
	if (!FileExists(pack_script_path_)) {
		pack_status_ = "Captions pack installer is missing from the host resources.";
		return;
	}
	if (!FileExists(pack_manifest_path_)) {
		pack_status_ = "Captions pack manifest is missing from the host resources.";
		return;
	}

	std::wstring command = L"powershell.exe -NoProfile -ExecutionPolicy Bypass -File ";
	command += QuoteArg(pack_script_path_);
	command += L" -PackId ";
	command += QuoteArg(WidenAscii(pack_id));
	command += L" -Manifest ";
	command += QuoteArg(pack_manifest_path_);
	if (uninstall) command += L" -Uninstall";

	STARTUPINFOW si{};
	si.cb = sizeof(si);
	PROCESS_INFORMATION pi{};
	std::vector<wchar_t> cmd(command.begin(), command.end());
	cmd.push_back(L'\0');

	BOOL ok =
	    CreateProcessW(nullptr, cmd.data(), nullptr, nullptr, FALSE, CREATE_NO_WINDOW, nullptr, nullptr, &si, &pi);

	if (!ok) {
		std::ostringstream ss;
		ss << "Could not start captions pack installer (Win32 " << GetLastError() << ").";
		pack_status_ = ss.str();
		return;
	}

	CloseHandle(pi.hThread);
	pack_process_ = pi.hProcess;
	pack_status_ = uninstall ? "Uninstalling captions pack..." : "Installing captions pack...";
}

void CaptionsPlugin::PollPackAction()
{
	if (!pack_process_) return;

	HANDLE h = static_cast<HANDLE>(pack_process_);
	DWORD code = STILL_ACTIVE;
	if (!GetExitCodeProcess(h, &code) || code == STILL_ACTIVE) return;

	CloseHandle(h);
	pack_process_ = nullptr;

	if (code == 0) {
		pack_status_ = "Captions pack action completed.";
		SendRestartHost();
	}
	else {
		std::ostringstream ss;
		ss << "Captions pack action failed (exit " << code << "). See captions_pack_install.log.";
		pack_status_ = ss.str();
	}
}

void CaptionsPlugin::PollSupervisorStatus()
{
	if (!ipc_.IsConnected()) return;
	try {
		auto resp = ipc_.SendBlocking(protocol::Request(protocol::RequestCaptionsGetSupervisorStatus));
		if (resp.type == protocol::ResponseCaptionsSupervisorStatus) {
			host_status_.SetSupervisorStatus(resp.captionsSupervisorStatus.host_halted != 0,
			                                 resp.captionsSupervisorStatus.last_exit_code,
			                                 resp.captionsSupervisorStatus.last_exit_description);
		}
	}
	catch (...) {
		// Non-fatal; host_halted remains at its last known value.
	}
}

void CaptionsPlugin::MaintainDriverConnection()
{
	try {
		if (!ipc_.IsConnected()) {
			ipc_.Connect();
		}

		auto resp = ipc_.SendBlocking(protocol::Request(protocol::RequestHandshake));
		if (resp.type != protocol::ResponseHandshake || resp.protocol.version != protocol::Version) {
			last_error_ = "Captions driver protocol mismatch";
			return;
		}

		const uint64_t gen = ipc_.ConnectionGeneration();
		if (gen != observed_ipc_generation_) {
			observed_ipc_generation_ = gen;
			PushConfigToDriver();
		}

		if (last_error_.find("Captions IPC") == 0 || last_error_.find("Not connected") == 0 ||
		    last_error_.find("Driver connection:") == 0) {
			last_error_.clear();
		}
	}
	catch (const std::exception& e) {
		last_error_ = std::string("Driver connection: ") + e.what();
		ipc_.Close();
	}
}

void CaptionsPlugin::DrawStatusBanner()
{
	if (!sidecar_enabled_) {
		openvr_pair::overlay::ui::DrawInfoBanner(
		    "Captions sidecar is off",
		    "Speech recognition is stopped. Turn on Run captions sidecar when you want live captions.");
		return;
	}

	if (!last_error_.empty()) {
		openvr_pair::overlay::ui::DrawErrorBanner("Captions driver problem", last_error_.c_str());
	}
}

namespace openvr_pair::overlay {

std::unique_ptr<FeaturePlugin> CreateCaptionsPlugin()
{
	return std::make_unique<CaptionsPlugin>();
}

} // namespace openvr_pair::overlay
