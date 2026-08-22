#define _CRT_SECURE_NO_DEPRECATE
#include "Logging.h"

#include "DebugLogging.h"
#include "FileLog.h"
#include "LogPaths.h"

#include <cerrno>
#include <chrono>
#include <cstdarg>

FILE* LogFile = nullptr;

namespace {

bool g_logFileOwned = false;

void AdoptLogFile(FILE* file, bool owned)
{
	LogFile = file;
	g_logFileOwned = owned;
	openvr_pair::common::SetLowLatencyLogMode(LogFile);
}

} // namespace

void OpenLogFile()
{
	if (!openvr_pair::common::IsDebugLoggingEnabled()) return;
	if (LogFile) return;

	// Prefer the LocalAppDataLow path so the user's diagnostic flow ("diff
	// overlay log + driver log side-by-side") works without having to hunt
	// for the driver log under whatever cwd vrserver happened to inherit
	// from Steam (typically the Steam install dir, sometimes non-writable).
	std::wstring path = openvr_pair::common::TimestampedLogPath(L"driver_log");
	int openErrno = 0;
	if (!path.empty()) {
		// _wfopen takes a wide path, which lets us cope with usernames
		// containing non-ASCII characters that fopen would mangle.
		FILE* file = _wfopen(path.c_str(), L"a");
		if (file) {
			AdoptLogFile(file, true);
			return;
		}
		openErrno = errno;
	}

	// Fallback: legacy behavior. Better than nothing if SHGetKnownFolderPath
	// isn't available or AppDataLow isn't writable.
	FILE* file = fopen("wkopenvr_driver.log", "a");
	AdoptLogFile(file ? file : stderr, file != nullptr);
	if (LogFile) {
		fprintf(LogFile, "[log-open] driver log using fallback path; primary_errno=%d primary_path_empty=%d\n",
		        openErrno, path.empty() ? 1 : 0);
		LogFlush();
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
	auto tm = localtime_s(&value, &nowTime);
	return value;
}

void LogFlush()
{
	if (LogFile) openvr_pair::common::FlushLogFileToDisk(LogFile);
}

void LogLine(const char* fmt, ...)
{
	if (!EnsureLogFileOpen()) return;
	// Format first so the timestamp and the message reach the file in one
	// call; two writers interleaving mid-line makes the log unreadable.
	char line[4096];
	va_list args;
	va_start(args, fmt);
	const int written = vsnprintf(line, sizeof line, fmt, args);
	va_end(args);
	if (written < 0) return;
	tm logNow = TimeForLog();
	fprintf(LogFile, "[%02d:%02d:%02d] %s\n", logNow.tm_hour, logNow.tm_min, logNow.tm_sec, line);
	LogFlush();
}

void CloseLogFile()
{
	if (!LogFile) return;
	LogFlush();
	if (g_logFileOwned) {
		fclose(LogFile);
	}
	LogFile = nullptr;
	g_logFileOwned = false;
}
