#define _CRT_SECURE_NO_DEPRECATE
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <shlobj.h>
#include <objbase.h>

#include "Logging.h"
#include "DebugLogging.h"
#include "FileLog.h"
#include "LogPaths.h"

#include <cerrno>
#include <atomic>
#include <cstdarg>
#include <cstdio>
#include <mutex>
#include <string>

namespace {

std::mutex g_logMutex;
FILE* g_logFile = nullptr;

} // namespace

void TrDrvOpenLogFile()
{
	std::lock_guard<std::mutex> lk(g_logMutex);
	if (!openvr_pair::common::IsDebugLoggingEnabled()) return;
	if (g_logFile) return;

	std::wstring path = openvr_pair::common::TimestampedLogPath(L"captions_drv_log");
	int openErrno = 0;
	if (!path.empty()) {
		g_logFile = _wfopen(path.c_str(), L"w");
		if (g_logFile) {
			openvr_pair::common::SetLowLatencyLogMode(g_logFile);
			return;
		}
		openErrno = errno;
	}

	g_logFile = fopen("captions_drv.log", "a");
	if (!g_logFile) g_logFile = stderr;
	openvr_pair::common::SetLowLatencyLogMode(g_logFile);
	if (g_logFile) {
		fprintf(g_logFile,
		        "[log-open] captions driver log using fallback path; primary_errno=%d primary_path_empty=%d\n",
		        openErrno, path.empty() ? 1 : 0);
		openvr_pair::common::FlushLogFileToDisk(g_logFile);
	}
}

void TrLogFlushDrv()
{
	std::lock_guard<std::mutex> lk(g_logMutex);
	if (g_logFile) openvr_pair::common::FlushLogFileToDisk(g_logFile);
}

void TrDrvLogV(const char* fmt, va_list args)
{
	if (!openvr_pair::common::IsDebugLoggingEnabled()) return;

	char buf[1024];
	vsnprintf(buf, sizeof(buf), fmt, args);

	std::lock_guard<std::mutex> lk(g_logMutex);
	if (!g_logFile) {
		std::wstring path = openvr_pair::common::TimestampedLogPath(L"captions_drv_log");
		if (!path.empty()) g_logFile = _wfopen(path.c_str(), L"w");
		if (!g_logFile) g_logFile = fopen("captions_drv.log", "a");
		if (!g_logFile) g_logFile = stderr;
		openvr_pair::common::SetLowLatencyLogMode(g_logFile);
	}
	if (g_logFile) {
		fputs(buf, g_logFile);
		fputs("\n", g_logFile);
		openvr_pair::common::FlushLogFileToDisk(g_logFile);
	}
}

void TrDrvLog(const char* fmt, ...)
{
	va_list ap;
	va_start(ap, fmt);
	TrDrvLogV(fmt, ap);
	va_end(ap);
}
