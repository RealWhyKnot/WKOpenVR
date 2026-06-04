#define _CRT_SECURE_NO_DEPRECATE
#include "Logging.h"

#include "DebugLogging.h"
#include "FileLog.h"
#include "LogPaths.h"

#include <chrono>

FILE* OrDrvLogFile = nullptr;

void OrDrvOpenLogFile()
{
	if (!openvr_pair::common::IsDebugLoggingEnabled()) return;
	if (OrDrvLogFile) return;

	std::wstring path = openvr_pair::common::TimestampedLogPath(L"oscrouter_log");
	if (!path.empty()) {
		FILE* f = _wfopen(path.c_str(), L"a");
		if (f) {
			OrDrvLogFile = f;
			openvr_pair::common::SetLowLatencyLogMode(OrDrvLogFile);
			return;
		}
	}
	FILE* f = fopen("oscrouter_drv.log", "a");
	OrDrvLogFile = f ? f : stderr;
	openvr_pair::common::SetLowLatencyLogMode(OrDrvLogFile);
}

bool OrDrvEnsureLogFileOpen()
{
	if (!openvr_pair::common::IsDebugLoggingEnabled()) return false;
	if (!OrDrvLogFile) OrDrvOpenLogFile();
	return OrDrvLogFile != nullptr;
}

tm OrDrvTimeForLog()
{
	auto now = std::chrono::system_clock::now();
	auto nowTime = std::chrono::system_clock::to_time_t(now);
	tm value{};
	localtime_s(&value, &nowTime);
	return value;
}

void OrDrvLogFlush()
{
	if (OrDrvLogFile) openvr_pair::common::FlushLogFileToDisk(OrDrvLogFile);
}
