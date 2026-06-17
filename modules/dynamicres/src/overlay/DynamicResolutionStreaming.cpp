#include "DynamicResolutionStreaming.h"

#include "JsonUtil.h"
#include "Win32Paths.h"

#include <windows.h>

#include <algorithm>
#include <cctype>
#include <fstream>
#include <sstream>

namespace wkopenvr::dynamicres {
namespace {

std::string TrimAscii(std::string value)
{
	size_t first = 0;
	while (first < value.size() && std::isspace(static_cast<unsigned char>(value[first]))) {
		++first;
	}
	size_t last = value.size();
	while (last > first && std::isspace(static_cast<unsigned char>(value[last - 1]))) {
		--last;
	}
	return value.substr(first, last - first);
}

std::string LowerAscii(std::string value)
{
	for (char& ch : value) {
		ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
	}
	return value;
}

std::string NormalizeCodec(std::string value)
{
	value = LowerAscii(TrimAscii(std::move(value)));
	std::string out;
	out.reserve(value.size());
	for (char ch : value) {
		if (ch == ' ' || ch == '-' || ch == '_') continue;
		out.push_back(ch);
	}
	return out;
}

std::wstring ProgramDataPath()
{
	wchar_t buffer[32767] = {};
	constexpr DWORD capacity = static_cast<DWORD>(sizeof(buffer) / sizeof(buffer[0]));
	const DWORD len = GetEnvironmentVariableW(L"PROGRAMDATA", buffer, capacity);
	if (len > 0 && len < capacity) {
		return std::wstring(buffer, len);
	}
	return L"C:\\ProgramData";
}

bool ReadTextFile(const std::wstring& path, std::string& out)
{
	std::ifstream in(path, std::ios::binary);
	if (!in) return false;
	std::ostringstream ss;
	ss << in.rdbuf();
	out = ss.str();
	return true;
}

} // namespace

bool ParseVirtualDesktopStreamerSettings(const std::string& body, VirtualDesktopStreamerSettings& out,
                                         std::string* error)
{
	picojson::value root;
	std::string parseError;
	if (!openvr_pair::common::json::ParseObject(root, body, &parseError)) {
		if (error) *error = parseError;
		out.parsed = false;
		out.error = parseError;
		return false;
	}

	out.parsed = true;
	out.error.clear();
	out.codecName = TrimAscii(openvr_pair::common::json::StringAt(root, "CodecName"));
	out.deviceName = TrimAscii(openvr_pair::common::json::StringAt(root, "DeviceName"));
	out.preferredCodec = openvr_pair::common::json::IntAt(root, "PreferredCodec", -1);
	if (error) error->clear();
	return true;
}

std::wstring DefaultVirtualDesktopStreamerSettingsPath()
{
	std::wstring base = ProgramDataPath();
	if (!base.empty() && base.back() != L'\\' && base.back() != L'/') {
		base.push_back(L'\\');
	}
	base += L"Virtual Desktop\\StreamerSettings.json";
	return base;
}

VirtualDesktopStreamerSettings LoadVirtualDesktopStreamerSettings()
{
	VirtualDesktopStreamerSettings settings;
	settings.path = DefaultVirtualDesktopStreamerSettingsPath();
	settings.lastWriteTime = openvr_pair::common::FileLastWriteTime(settings.path);

	std::string body;
	if (!ReadTextFile(settings.path, body)) {
		settings.settingsFilePresent = false;
		settings.error = "settings file not found";
		return settings;
	}

	settings.settingsFilePresent = true;
	std::string error;
	if (!ParseVirtualDesktopStreamerSettings(body, settings, &error)) {
		settings.error = error.empty() ? "settings parse failed" : error;
	}
	return settings;
}

bool VirtualDesktopCodecAllowsDefaultAction(const VirtualDesktopStreamerSettings& settings)
{
	if (!settings.parsed) return false;
	const std::string codec = NormalizeCodec(settings.codecName);
	return codec == "h.264" || codec == "h264" || codec == "h.264+" || codec == "h264+" || codec == "x264" ||
	       codec == "avc";
}

std::string VirtualDesktopCodecLabel(const VirtualDesktopStreamerSettings& settings)
{
	if (!settings.settingsFilePresent) return "Settings file not found";
	if (!settings.parsed) return settings.error.empty() ? "Settings parse failed" : settings.error;
	if (!settings.codecName.empty()) return settings.codecName;
	if (settings.preferredCodec >= 0) return "PreferredCodec " + std::to_string(settings.preferredCodec);
	return "Codec not reported";
}

} // namespace wkopenvr::dynamicres
