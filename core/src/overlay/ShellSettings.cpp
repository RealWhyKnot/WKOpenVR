#include "ShellSettings.h"

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include <cstdio>
#include <map>
#include <utility>

namespace openvr_pair::overlay {
namespace {

std::wstring ShellSettingsPath(const std::wstring& profileRoot)
{
	if (profileRoot.empty()) return {};
	return profileRoot + L"\\shell.txt";
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

std::map<std::string, std::string> ReadShellSettings(const std::wstring& profileRoot)
{
	std::map<std::string, std::string> settings;
	const std::wstring path = ShellSettingsPath(profileRoot);
	if (path.empty()) return settings;

	FILE* f = nullptr;
	if (_wfopen_s(&f, path.c_str(), L"rb") != 0 || !f) return settings;

	char lineBuf[512] = {};
	while (fgets(lineBuf, sizeof(lineBuf), f)) {
		std::string line(lineBuf);
		const size_t eq = line.find('=');
		if (eq == std::string::npos) continue;
		std::string key = TrimAscii(line.substr(0, eq));
		std::string value = TrimAscii(line.substr(eq + 1));
		if (!key.empty()) {
			settings[key] = value;
		}
	}
	fclose(f);
	return settings;
}

bool WriteShellSettings(const std::wstring& profileRoot, const std::map<std::string, std::string>& settings)
{
	const std::wstring path = ShellSettingsPath(profileRoot);
	if (path.empty()) return false;
	CreateDirectoryW(profileRoot.c_str(), nullptr);

	FILE* f = nullptr;
	if (_wfopen_s(&f, path.c_str(), L"wb") != 0 || !f) return false;
	for (const auto& entry : settings) {
		fprintf(f, "%s=%s\n", entry.first.c_str(), entry.second.c_str());
	}
	fclose(f);
	return true;
}

} // namespace

std::string ReadShellSetting(const std::wstring& profileRoot, const char* key, const char* fallback)
{
	if (!key || !key[0]) return fallback ? fallback : "";
	const std::map<std::string, std::string> settings = ReadShellSettings(profileRoot);
	const auto it = settings.find(key);
	if (it == settings.end()) return fallback ? fallback : "";
	return it->second;
}

bool WriteShellSetting(const std::wstring& profileRoot, const char* key, const std::string& value)
{
	if (!key || !key[0]) return false;
	std::map<std::string, std::string> settings = ReadShellSettings(profileRoot);
	settings[key] = value;
	return WriteShellSettings(profileRoot, settings);
}

} // namespace openvr_pair::overlay
