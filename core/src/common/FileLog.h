#pragma once

#include <cstdio>

namespace openvr_pair::common {

void SetLowLatencyLogMode(FILE* file);
bool FlushLogFileToDisk(FILE* file);

} // namespace openvr_pair::common
