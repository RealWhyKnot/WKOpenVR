#pragma once

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

#include <string>

// Cross-process contract for the user's chosen capture device.
//
// The overlay writes the selected WASAPI endpoint id to
//   %LocalAppDataLow%\WKOpenVR\captions\audio_input.txt
// and the host reads it here. An absent or empty file means "system default"
// (the historical behaviour). This mirrors the model-path convention noted in
// ProtocolPayloads.h: device selection is a host-resolved resource carried via
// a local file rather than over the size-constrained config IPC.
//
// Both sides derive the captions directory independently via
// WkOpenVrSubdirectoryPath(L"captions"), the same way host_status.json is
// shared, so no path is hard-coded here.
namespace captions {

inline constexpr const wchar_t* kAudioInputFileName = L"audio_input.txt";

// Trim surrounding whitespace / CR / LF from raw file contents and return the
// endpoint id. Pure; unit-tested without touching disk.
inline std::string ParseAudioInputDeviceId(const std::string& raw)
{
	size_t begin = 0;
	size_t end = raw.size();
	auto is_trim = [](char c) { return c == ' ' || c == '\t' || c == '\r' || c == '\n'; };
	while (begin < end && is_trim(raw[begin])) ++begin;
	while (end > begin && is_trim(raw[end - 1])) --end;
	return raw.substr(begin, end - begin);
}

// Read the selected endpoint id from <captionsDir>\audio_input.txt. Returns ""
// (system default) when the directory is empty, the file is missing, or the
// file is blank. Never throws.
inline std::string ReadCaptionsInputDeviceId(const std::wstring& captionsDir)
{
	if (captionsDir.empty()) return {};
	std::wstring path = captionsDir + L"\\" + kAudioInputFileName;

	HANDLE h = CreateFileW(path.c_str(), GENERIC_READ, FILE_SHARE_READ | FILE_SHARE_WRITE, nullptr, OPEN_EXISTING,
	                       FILE_ATTRIBUTE_NORMAL, nullptr);
	if (h == INVALID_HANDLE_VALUE) return {};

	std::string raw;
	char buf[1024];
	DWORD got = 0;
	while (ReadFile(h, buf, sizeof(buf), &got, nullptr) && got > 0) {
		raw.append(buf, got);
		if (raw.size() > 4096) break; // endpoint ids are short; guard against junk
	}
	CloseHandle(h);
	return ParseAudioInputDeviceId(raw);
}

} // namespace captions
