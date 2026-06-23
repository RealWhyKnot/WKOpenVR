#include "PowerShellCommand.h"

#include "Win32Text.h"

#include <cstdint>
#include <vector>

namespace openvr_pair::common {

std::wstring QuotePowerShellLiteral(const std::wstring& value)
{
	std::wstring out = L"'";
	for (wchar_t ch : value) {
		if (ch == L'\'')
			out += L"''";
		else
			out += ch;
	}
	out += L"'";
	return out;
}

std::wstring EncodePowerShellCommand(const std::wstring& script)
{
	std::vector<unsigned char> bytes;
	bytes.reserve(script.size() * 2);
	for (wchar_t ch : script) {
		bytes.push_back(static_cast<unsigned char>(ch & 0xFF));
		bytes.push_back(static_cast<unsigned char>((ch >> 8) & 0xFF));
	}

	static constexpr char kBase64[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
	std::wstring out;
	out.reserve(((bytes.size() + 2) / 3) * 4);
	size_t i = 0;
	while (i + 3 <= bytes.size()) {
		const uint32_t v =
		    (uint32_t(bytes[i]) << 16) | (uint32_t(bytes[i + 1]) << 8) | uint32_t(bytes[i + 2]);
		out += static_cast<wchar_t>(kBase64[(v >> 18) & 0x3F]);
		out += static_cast<wchar_t>(kBase64[(v >> 12) & 0x3F]);
		out += static_cast<wchar_t>(kBase64[(v >> 6) & 0x3F]);
		out += static_cast<wchar_t>(kBase64[v & 0x3F]);
		i += 3;
	}
	if (i < bytes.size()) {
		uint32_t v = uint32_t(bytes[i]) << 16;
		const size_t rem = bytes.size() - i;
		if (rem == 2) v |= uint32_t(bytes[i + 1]) << 8;
		out += static_cast<wchar_t>(kBase64[(v >> 18) & 0x3F]);
		out += static_cast<wchar_t>(kBase64[(v >> 12) & 0x3F]);
		out += static_cast<wchar_t>(rem == 2 ? kBase64[(v >> 6) & 0x3F] : L'=');
		out += L'=';
	}
	return out;
}

std::string EncodePowerShellCommandUtf8(const std::wstring& script)
{
	return WideToUtf8(EncodePowerShellCommand(script));
}

} // namespace openvr_pair::common
