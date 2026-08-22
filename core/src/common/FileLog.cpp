#define _CRT_SECURE_NO_DEPRECATE
#include "FileLog.h"

#include "DebugLogging.h"
#include "LogPaths.h"

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <string>

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

FILE* OpenModuleLogFile(const wchar_t* timestampedPrefix, const char* fallbackFileName)
{
	if (!IsDebugLoggingEnabled()) return nullptr;

	// _wfopen so a user name with non-ASCII characters resolves correctly.
	const std::wstring path = TimestampedLogPath(timestampedPrefix);
	if (!path.empty()) {
		if (FILE* file = _wfopen(path.c_str(), L"a")) {
			SetLowLatencyLogMode(file);
			return file;
		}
	}

	FILE* file = fopen(fallbackFileName, "a");
	if (!file) file = stderr;
	SetLowLatencyLogMode(file);
	return file;
}

tm LocalTimeForLog()
{
	const auto now = std::chrono::system_clock::now();
	const auto nowTime = std::chrono::system_clock::to_time_t(now);
	tm value{};
	localtime_s(&value, &nowTime);
	return value;
}

} // namespace openvr_pair::common
