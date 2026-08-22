#pragma once

#include <cstdio>
#include <ctime>

namespace openvr_pair::common {

void SetLowLatencyLogMode(FILE* file);
bool FlushLogFileToDisk(FILE* file);

// Opens a module's timestamped log under %LocalAppDataLow%\WKOpenVR\Logs, or
// returns nullptr while debug logging is off. On failure it falls back to
// fallbackFileName in the working directory, then to stderr, so a diagnostic
// line is never dropped silently.
FILE* OpenModuleLogFile(const wchar_t* timestampedPrefix, const char* fallbackFileName);

// Local wall clock, broken down for the "[hh:mm:ss] " prefix every module log
// writes.
tm LocalTimeForLog();

// Deferred-flush policy for the diagnostic log. The logger keeps the file in
// low-latency mode so each fwrite reaches the OS page cache immediately (so an
// app crash loses nothing), but the *device sync* (FlushFileBuffers, inside
// FlushLogFileToDisk) is expensive. Per-line syncing ran ~22x/sec in live
// sessions and was the bulk of dev-build log I/O. ShouldFlushLog batches the
// sync to a time-or-size threshold: the crash tail is bounded to one interval
// of data still sitting unsynced in the page cache, which only matters on power
// loss, not on a normal process crash. Pure + header-only for unit testing
// (tests/test_continuous_persist.cpp covers it alongside the persist cadence).
inline constexpr long long kLogFlushIntervalMs = 250;
inline constexpr long long kLogFlushBytes = 64 * 1024;

constexpr bool ShouldFlushLog(long long bytesSinceFlush, long long msSinceFlush,
                              long long intervalMs = kLogFlushIntervalMs, long long bytesThreshold = kLogFlushBytes)
{
	return bytesSinceFlush >= bytesThreshold || msSinceFlush >= intervalMs;
}

} // namespace openvr_pair::common
