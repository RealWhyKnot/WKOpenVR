#pragma once

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

#include <chrono>
#include <string>
#include <string_view>

namespace openvr_pair::common {

std::wstring TimestampedLogFileName(std::wstring_view prefix, const SYSTEMTIME& utc);
void DeleteOldLogFiles(const std::wstring& directory, std::wstring_view prefix,
                       std::chrono::hours maxAge = std::chrono::hours(24));
std::wstring TimestampedLogPath(std::wstring_view prefix);

} // namespace openvr_pair::common
