#include "Win32Errors.h"

#include "Win32Text.h"

namespace openvr_pair::common {

std::string FormatWin32Error(DWORD error)
{
	LPWSTR buffer = nullptr;
	const DWORD size = FormatMessageW(
	    FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS, nullptr, error,
	    MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), reinterpret_cast<LPWSTR>(&buffer), 0, nullptr);

	std::string message;
	if (buffer && size > 0) {
		message = WideToUtf8(std::wstring_view(buffer, size));
	}
	if (buffer) {
		LocalFree(buffer);
	}
	return message;
}

} // namespace openvr_pair::common
