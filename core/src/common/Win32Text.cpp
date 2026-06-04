#include "Win32Text.h"

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

namespace openvr_pair::common {

std::string WideToUtf8(std::wstring_view value)
{
	if (value.empty()) return {};
	const int needed =
	    WideCharToMultiByte(CP_UTF8, 0, value.data(), static_cast<int>(value.size()), nullptr, 0, nullptr, nullptr);
	if (needed <= 0) return {};

	std::string out(static_cast<size_t>(needed), '\0');
	WideCharToMultiByte(CP_UTF8, 0, value.data(), static_cast<int>(value.size()), out.data(), needed, nullptr, nullptr);
	return out;
}

std::wstring Utf8ToWide(std::string_view value)
{
	if (value.empty()) return {};
	const int needed = MultiByteToWideChar(CP_UTF8, 0, value.data(), static_cast<int>(value.size()), nullptr, 0);
	if (needed <= 0) return {};

	std::wstring out(static_cast<size_t>(needed), L'\0');
	MultiByteToWideChar(CP_UTF8, 0, value.data(), static_cast<int>(value.size()), out.data(), needed);
	return out;
}

} // namespace openvr_pair::common
