#pragma once

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

#include <string>
#include <string_view>

namespace openvr_pair::common {

std::wstring LocalAppDataLowPath();
bool EnsureDirectory(const std::wstring& path);
std::wstring WkOpenVrRootPath(bool create = true);
std::wstring WkOpenVrSubdirectoryPath(std::wstring_view relative, bool create = true);
std::wstring WkOpenVrLogsPath(bool create = true);
int64_t FileLastWriteTime(const std::wstring& path);

} // namespace openvr_pair::common
