#pragma once

#include <cstdio>
#include <ctime>

// Per-plugin overlay log for Smoothing. Lives at
// %LocalAppDataLow%\WKOpenVR\Logs\smoothing_log.<ts>.txt next to the
// InputHealth inputhealth_log and SC's spacecal_log so all three plugins'
// diagnostics are co-located. Headerless: each line is a single timestamped
// event written via the LOG() macro.

namespace smoothing::logging {

extern FILE *LogFile;

void OpenLogFile();
bool EnsureLogFileOpen();
tm   TimeForLog();
void LogFlush();

} // namespace smoothing::logging

#ifndef SM_LOG
#define SM_LOG(fmt, ...) do { \
	if (smoothing::logging::EnsureLogFileOpen()) { \
		tm logNow = smoothing::logging::TimeForLog(); \
		fprintf(smoothing::logging::LogFile, "[%02d:%02d:%02d] " fmt "\n", \
			logNow.tm_hour, logNow.tm_min, logNow.tm_sec, ##__VA_ARGS__); \
		smoothing::logging::LogFlush(); \
	} \
} while (0)
#endif
