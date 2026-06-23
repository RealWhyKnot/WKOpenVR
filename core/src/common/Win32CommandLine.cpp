#include "Win32CommandLine.h"

namespace openvr_pair::common {

std::wstring QuoteCommandLineArg(const std::wstring& value)
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

} // namespace openvr_pair::common
