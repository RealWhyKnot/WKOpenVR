#pragma once

#include <string>
#include <string_view>

namespace openvr_pair::common {

// ASCII-only on purpose: std::isspace is locale-sensitive and also strips
// vertical tab and form feed, which config and log parsers here treat as data.
inline bool IsAsciiSpace(char c)
{
	return c == ' ' || c == '\t' || c == '\r' || c == '\n';
}

inline std::string TrimAscii(std::string_view value)
{
	size_t begin = 0;
	while (begin < value.size() && IsAsciiSpace(value[begin])) {
		++begin;
	}
	size_t end = value.size();
	while (end > begin && IsAsciiSpace(value[end - 1])) {
		--end;
	}
	return std::string(value.substr(begin, end - begin));
}

inline bool StartsWith(std::string_view value, std::string_view prefix)
{
	return value.size() >= prefix.size() && value.compare(0, prefix.size(), prefix) == 0;
}

inline bool EndsWith(std::string_view value, std::string_view suffix)
{
	return value.size() >= suffix.size() && value.compare(value.size() - suffix.size(), suffix.size(), suffix) == 0;
}

} // namespace openvr_pair::common
