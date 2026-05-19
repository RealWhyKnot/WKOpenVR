#define _CRT_SECURE_NO_DEPRECATE
#include "Logging.h"

#include "DebugLogging.h"
#include "LogPaths.h"

#include <chrono>

FILE *LogFile = nullptr;

void OpenLogFile()
{
	if (!openvr_pair::common::IsDebugLoggingEnabled()) return;
	if (LogFile) return;

	std::wstring path = openvr_pair::common::TimestampedLogPath(L"inputhealth_log");
	if (!path.empty()) {
		LogFile = _wfopen(path.c_str(), L"a");
		if (LogFile) return;
	}
	LogFile = fopen("openvr_inputhealth.log", "a");
	if (!LogFile) {
		LogFile = stderr;
	}
}

bool EnsureLogFileOpen()
{
	if (!openvr_pair::common::IsDebugLoggingEnabled()) return false;
	if (!LogFile) OpenLogFile();
	return LogFile != nullptr;
}

tm TimeForLog()
{
	auto now = std::chrono::system_clock::now();
	auto nowTime = std::chrono::system_clock::to_time_t(now);
	tm value;
	localtime_s(&value, &nowTime);
	return value;
}

void LogFlush()
{
	if (LogFile) fflush(LogFile);
}
