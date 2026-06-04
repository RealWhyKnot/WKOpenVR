#pragma once

#include <atomic>
#include <cstdio>
#include <ctime>

extern FILE* FtLogFile;

// Runtime verbose flag.  When false, FT_LOG_OVL_VERBOSE lines are suppressed.
// Set to true on dev-channel builds and by the "Verbose overlay logging"
// checkbox in the Logs tab.
extern std::atomic<bool> FtOverlayVerbose;

void FtOpenLogFile();
bool FtEnsureLogFileOpen();
tm FtTimeForLog();
void FtLogFlush();

// Overlay-side face-tracking log macro. The "OVL" suffix mirrors the
// driver-side FT_LOG_DRV so log lines identify their origin clearly.
#ifndef FT_LOG_OVL
#define FT_LOG_OVL(fmt, ...)                                                                                           \
	do {                                                                                                               \
		if (FtEnsureLogFileOpen()) {                                                                                   \
			tm _t = FtTimeForLog();                                                                                    \
			fprintf(FtLogFile, "[%02d:%02d:%02d] [ft/ovl] " fmt "\n", _t.tm_hour, _t.tm_min, _t.tm_sec,                \
			        ##__VA_ARGS__);                                                                                    \
			FtLogFlush();                                                                                              \
		}                                                                                                              \
	} while (0)
#endif

// Verbose-gated variant -- compiled out at the call site when the flag is off.
#ifndef FT_LOG_OVL_VERBOSE
#define FT_LOG_OVL_VERBOSE(fmt, ...)                                                                                   \
	do {                                                                                                               \
		if (FtOverlayVerbose.load(std::memory_order_relaxed)) {                                                        \
			FT_LOG_OVL(fmt, ##__VA_ARGS__);                                                                            \
		}                                                                                                              \
	} while (0)
#endif
