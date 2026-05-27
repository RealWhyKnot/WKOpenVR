#include "FileLog.h"

#include <cstdint>
#include <cstdio>

#if defined(_WIN32)
#include <io.h>

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#endif

namespace openvr_pair::common {

void SetLowLatencyLogMode(FILE* file)
{
	if (!file) return;
	if (file == stdin || file == stdout || file == stderr) return;
	setvbuf(file, nullptr, _IONBF, 0);
}

bool FlushLogFileToDisk(FILE* file)
{
	if (!file) return false;
	if (fflush(file) != 0) return false;
	if (file == stdin || file == stdout || file == stderr) return true;

#if defined(_WIN32)
	const int fd = _fileno(file);
	if (fd < 0) return false;
	const intptr_t osHandle = _get_osfhandle(fd);
	if (osHandle == -1) return false;
	return FlushFileBuffers(reinterpret_cast<HANDLE>(osHandle)) != 0;
#else
	return true;
#endif
}

} // namespace openvr_pair::common
