#define _CRT_SECURE_NO_DEPRECATE
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <shlobj.h>
#include <objbase.h>

#include "HostStatus.h"
#include "Logging.h"
#include "Win32Paths.h"

#include <cstdio>
#include <cstring>
#include <sstream>
#include <string>

static std::string EscapeJson(const std::string& s)
{
	std::string out;
	out.reserve(s.size() + 4);
	for (char c : s) {
		switch (c) {
			case '"':
				out += "\\\"";
				break;
			case '\\':
				out += "\\\\";
				break;
			case '\n':
				out += "\\n";
				break;
			case '\r':
				out += "\\r";
				break;
			case '\t':
				out += "\\t";
				break;
			default:
				if (static_cast<unsigned char>(c) < 0x20) {
					char esc[8];
					snprintf(esc, sizeof(esc), "\\u%04x", (unsigned char)c);
					out += esc;
				}
				else {
					out += c;
				}
		}
	}
	return out;
}

HostStatus::HostStatus(const std::wstring& status_path)
{
	WritePath(status_path);
}

void HostStatus::WritePath(const std::wstring& status_path)
{
	if (!status_path.empty()) {
		status_path_ = status_path;
		TH_LOG("[status] path=%ls", status_path_.c_str());
		return;
	}

	std::wstring root = openvr_pair::common::WkOpenVrSubdirectoryPath(L"captions", true);
	if (root.empty()) {
		TH_LOG("[status] failed to resolve captions status directory");
		return;
	}
	status_path_ = root + L"\\host_status.json";
	TH_LOG("[status] path=%ls", status_path_.c_str());
}

void HostStatus::SetState(State s) noexcept
{
	state_ = s;
}
void HostStatus::SetMicName(const std::string& name)
{
	mic_name_ = name;
}
void HostStatus::SetLastTranscript(const std::string& t)
{
	last_transcript_ = t;
}
void HostStatus::SetLastTranslation(const std::string& t)
{
	last_translation_ = t;
}
void HostStatus::SetLastError(const std::string& e)
{
	last_error_ = e;
}
void HostStatus::SetPhase(const std::string& phase)
{
	phase_ = phase;
}
void HostStatus::SetPttStatus(bool available, bool registered, const std::string& app_key, const std::string& error)
{
	ptt_available_ = available;
	ptt_registered_ = registered;
	ptt_app_key_ = app_key;
	ptt_error_ = error;
}
void HostStatus::SetSpeechPackInstalled(bool installed) noexcept
{
	speech_pack_installed_ = installed;
}
void HostStatus::SetVadRuntimeAvailable(bool available) noexcept
{
	vad_runtime_available_ = available;
}
void HostStatus::SetTranslationRuntimeAvailable(bool available) noexcept
{
	translation_runtime_available_ = available;
}
void HostStatus::SetTranslationPackInstalled(bool installed) noexcept
{
	translation_pack_installed_ = installed;
}
void HostStatus::SetActiveTranslationPair(const std::string& pair)
{
	active_translation_pair_ = pair;
}
void HostStatus::IncrementCaptionsCompleted() noexcept
{
	++captions_completed_;
}
void HostStatus::IncrementPacketsSent() noexcept
{
	++packets_sent_;
}

void HostStatus::MaybeFlush()
{
	const long long now = static_cast<long long>(GetTickCount64());
	if (now - last_flush_tick_ < 1000) return;
	last_flush_tick_ = now;
	DoFlush();
}

void HostStatus::Flush()
{
	last_flush_tick_ = static_cast<long long>(GetTickCount64());
	DoFlush();
}

void HostStatus::DoFlush()
{
	if (status_path_.empty()) return;

	std::ostringstream o;
	o << "{\n";
	o << "  \"schema_version\": 1,\n";
	o << "  \"host_pid\": " << (long long)GetCurrentProcessId() << ",\n";
	o << "  \"state\": " << (int)state_ << ",\n";
	o << "  \"phase\": \"" << EscapeJson(phase_) << "\",\n";
	o << "  \"mic_name\": \"" << EscapeJson(mic_name_) << "\",\n";
	o << "  \"last_transcript\": \"" << EscapeJson(last_transcript_) << "\",\n";
	o << "  \"last_translation\": \"" << EscapeJson(last_translation_) << "\",\n";
	o << "  \"last_error\": \"" << EscapeJson(last_error_) << "\",\n";
	o << "  \"ptt_available\": " << (ptt_available_ ? "true" : "false") << ",\n";
	o << "  \"ptt_registered\": " << (ptt_registered_ ? "true" : "false") << ",\n";
	o << "  \"ptt_app_key\": \"" << EscapeJson(ptt_app_key_) << "\",\n";
	o << "  \"ptt_error\": \"" << EscapeJson(ptt_error_) << "\",\n";
	o << "  \"speech_pack_installed\": " << (speech_pack_installed_ ? "true" : "false") << ",\n";
	o << "  \"vad_runtime_available\": " << (vad_runtime_available_ ? "true" : "false") << ",\n";
	o << "  \"translation_runtime_available\": " << (translation_runtime_available_ ? "true" : "false") << ",\n";
	o << "  \"translation_pack_installed\": " << (translation_pack_installed_ ? "true" : "false") << ",\n";
	o << "  \"active_translation_pair\": \"" << EscapeJson(active_translation_pair_) << "\",\n";
	o << "  \"captions_completed\": " << captions_completed_ << ",\n";
	o << "  \"packets_sent\": " << packets_sent_ << ",\n";
	o << "  \"frames_written\": 0,\n";
	o << "  \"frames_read\": 0,\n";
	o << "  \"osc_messages_sent\": " << packets_sent_ << ",\n";
	o << "  \"last_exit_code\": 0,\n";
	o << "  \"last_restart_time\": \"\"\n";
	o << "}\n";
	std::string json = o.str();

	std::wstring tmp = status_path_ + L".tmp";
	HANDLE h = CreateFileW(tmp.c_str(), GENERIC_WRITE, 0, nullptr, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
	if (h == INVALID_HANDLE_VALUE) {
		TH_LOG("[status] CreateFileW failed err=%lu path=%ls", (unsigned long)GetLastError(), tmp.c_str());
		return;
	}

	DWORD written = 0;
	WriteFile(h, json.data(), static_cast<DWORD>(json.size()), &written, nullptr);
	CloseHandle(h);

	if (written == static_cast<DWORD>(json.size())) {
		if (!MoveFileExW(tmp.c_str(), status_path_.c_str(), MOVEFILE_REPLACE_EXISTING)) {
			TH_LOG("[status] MoveFileExW failed err=%lu path=%ls", (unsigned long)GetLastError(), status_path_.c_str());
		}
	}
	else {
		TH_LOG("[status] partial write path=%ls written=%lu expected=%zu", tmp.c_str(), (unsigned long)written,
		       json.size());
		DeleteFileW(tmp.c_str());
	}
}
