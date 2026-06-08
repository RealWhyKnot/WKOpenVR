#define _CRT_SECURE_NO_DEPRECATE
#include "CaptionsConfig.h"

#include "Win32Paths.h"

#include <cstdio>
#include <cstring>
#include <string>

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

namespace {
std::wstring ConfigDir()
{
	return openvr_pair::common::WkOpenVrSubdirectoryPath(L"profiles", true);
}

std::wstring ConfigPath()
{
	std::wstring dir = ConfigDir();
	if (dir.empty()) return {};
	return dir + L"\\captions.txt";
}
} // namespace

CaptionsConfig LoadCaptionsConfig()
{
	CaptionsConfig cfg;
	std::wstring path = ConfigPath();
	if (path.empty()) return cfg;

	FILE* f = _wfopen(path.c_str(), L"r");
	if (!f) return cfg;

	char line[512];
	while (fgets(line, sizeof line, f)) {
		size_t len = strlen(line);
		while (len > 0 && (line[len - 1] == '\n' || line[len - 1] == '\r')) {
			line[--len] = '\0';
		}
		char* eq = strchr(line, '=');
		if (!eq) continue;
		*eq = '\0';
		const char* key = line;
		const char* val = eq + 1;

		if (strcmp(key, "sidecar_enabled") == 0) {
			cfg.sidecar_enabled = (atoi(val) != 0);
		}
		else if (strcmp(key, "mode") == 0) {
			int n = atoi(val);
			// Two valid modes today; clamp defensively against a hand-edit
			// setting an unsupported value rather than letting the UI render
			// an out-of-range radio button.
			if (n < 0) n = 0;
			if (n > 1) n = 1;
			cfg.mode = n;
		}
		else if (strcmp(key, "always_on_consented") == 0) {
			cfg.always_on_consented = (atoi(val) != 0);
		}
		else if (strcmp(key, "source_lang") == 0) {
			cfg.source_lang = val;
		}
		else if (strcmp(key, "target_lang") == 0) {
			cfg.target_lang = val;
		}
		else if (strcmp(key, "chatbox_enabled") == 0) {
			cfg.chatbox_enabled = (atoi(val) != 0);
		}
		else if (strcmp(key, "chatbox_address") == 0) {
			cfg.chatbox_address = val;
		}
		else if (strcmp(key, "notify_sound") == 0) {
			cfg.notify_sound = (atoi(val) != 0);
		}
		else if (strcmp(key, "realtime_flags") == 0) {
			int n = atoi(val);
			if (n < 0) n = 0;
			if (n > 255) n = 255;
			cfg.realtime_flags = static_cast<uint8_t>(n);
		}
		else if (strcmp(key, "speech_model") == 0) {
			cfg.speech_model = captions::NormalizeCaptionsSpeechModel(atoi(val));
		}
		else if (strcmp(key, "input_device") == 0) {
			cfg.input_device = val;
		}
	}
	fclose(f);
	return cfg;
}

void SaveCaptionsConfig(const CaptionsConfig& cfg)
{
	std::wstring path = ConfigPath();
	if (path.empty()) return;

	std::string body;
	body.reserve(256);
	auto appendf = [&](const char* fmt, auto&&... args) {
		char buf[512];
		int n = std::snprintf(buf, sizeof buf, fmt, std::forward<decltype(args)>(args)...);
		if (n > 0) body.append(buf, (size_t)n);
	};
	appendf("sidecar_enabled=%d\n", cfg.sidecar_enabled ? 1 : 0);
	appendf("mode=%d\n", cfg.mode);
	appendf("always_on_consented=%d\n", cfg.always_on_consented ? 1 : 0);
	appendf("source_lang=%s\n", cfg.source_lang.c_str());
	appendf("target_lang=%s\n", cfg.target_lang.c_str());
	appendf("chatbox_enabled=%d\n", cfg.chatbox_enabled ? 1 : 0);
	appendf("chatbox_address=%s\n", cfg.chatbox_address.c_str());
	appendf("notify_sound=%d\n", cfg.notify_sound ? 1 : 0);
	appendf("realtime_flags=%u\n", static_cast<unsigned>(cfg.realtime_flags));
	appendf("speech_model=%u\n", static_cast<unsigned>(cfg.speech_model));
	appendf("input_device=%s\n", cfg.input_device.c_str());

	std::wstring tmpPath = path + L".tmp";
	HANDLE h = CreateFileW(tmpPath.c_str(), GENERIC_WRITE, 0, nullptr, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
	if (h == INVALID_HANDLE_VALUE) return;
	DWORD written = 0;
	BOOL ok = WriteFile(h, body.data(), (DWORD)body.size(), &written, nullptr);
	CloseHandle(h);
	if (!ok || written != (DWORD)body.size()) {
		DeleteFileW(tmpPath.c_str());
		return;
	}
	MoveFileExW(tmpPath.c_str(), path.c_str(), MOVEFILE_REPLACE_EXISTING);
}
