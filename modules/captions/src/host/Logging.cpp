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
#include <cstdarg>
#include <cstdio>
#include <mutex>
#include <string>

namespace {

std::mutex g_logMutex;
FILE* g_logFile = nullptr;

FILE* OpenTimestampedLog()
{
	std::wstring path = openvr_pair::common::TimestampedLogPath(L"captions_host_log");
	int openErrno = 0;
	if (!path.empty()) {
		FILE* f = _wfopen(path.c_str(), L"a");
		if (f) {
			openvr_pair::common::SetLowLatencyLogMode(f);
			return f;
		}
		openErrno = errno;
	}

	FILE* f = fopen("captions_host.log", "a");
	if (!f) return nullptr;
	openvr_pair::common::SetLowLatencyLogMode(f);
	fprintf(f, "[log-open] captions host log using fallback path; primary_errno=%d primary_path_empty=%d\n", openErrno,
	        path.empty() ? 1 : 0);
	openvr_pair::common::FlushLogFileToDisk(f);
	return f;
}

} // namespace

void CaptionsHostOpenLogFile()
{
	std::lock_guard<std::mutex> lk(g_logMutex);
	if (!openvr_pair::common::IsDebugLoggingEnabled()) return;
	if (g_logFile) return;

	g_logFile = OpenTimestampedLog();
}

void CaptionsHostFlushLog()
{
	std::lock_guard<std::mutex> lk(g_logMutex);
	if (g_logFile) openvr_pair::common::FlushLogFileToDisk(g_logFile);
}

void CaptionsHostLog(const char* fmt, ...)
{
	if (!openvr_pair::common::IsDebugLoggingEnabled()) return;

	char buf[2048];
	va_list ap;
	va_start(ap, fmt);
	vsnprintf(buf, sizeof(buf), fmt, ap);
	va_end(ap);

	std::lock_guard<std::mutex> lk(g_logMutex);
	if (!g_logFile) {
		g_logFile = OpenTimestampedLog();
	}
	if (g_logFile) {
		fputs(buf, g_logFile);
		fputs("\n", g_logFile);
		openvr_pair::common::FlushLogFileToDisk(g_logFile);
	}
}
