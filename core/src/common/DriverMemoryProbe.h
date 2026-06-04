#pragma once

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

#include <cstddef>
#include <string>

namespace openvr_pair::common {

bool IsReadableMemoryRange(const void* addr, size_t bytes);
std::string DescribeVirtualQueryRegion(const char* tag, const void* addr);

} // namespace openvr_pair::common
