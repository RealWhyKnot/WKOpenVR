#pragma once

#include <cstdio>
#include <ctime>

#ifdef FACETRACKING_TESTS
// Stub out the logging infrastructure so CalibrationEngine.cpp links inside
// the gtest binary without pulling in Logging.cpp (which opens files, calls
// SHGetKnownFolderPath, etc.).
inline FILE* FtDrvLogFile = nullptr;
inline void FtDrvOpenLogFile() {}
inline bool FtDrvEnsureLogFileOpen()
{
	return false;
}
inline tm FtDrvTimeForLog()
{
	return {};
}
inline void FtDrvLogFlush() {}
#define FT_LOG_DRV(fmt, ...)                                                                                           \
	do {                                                                                                               \
	} while (0)
#else

extern FILE* FtDrvLogFile;

void FtDrvOpenLogFile();
bool FtDrvEnsureLogFileOpen();
tm FtDrvTimeForLog();
void FtDrvLogFlush();

// FT_LOG_DRV -- driver-side face-tracking logger.
// Prefixed differently from the umbrella LOG macro to avoid collision; the
// umbrella's LOG is defined in core/src/driver/Logging.h and is not available
// from inside a feature module's static lib.
#ifndef FT_LOG_DRV
#define FT_LOG_DRV(fmt, ...)                                                                                           \
	do {                                                                                                               \
		if (FtDrvEnsureLogFileOpen()) {                                                                                \
			tm _ftNow = FtDrvTimeForLog();                                                                             \
			fprintf(FtDrvLogFile, "[%02d:%02d:%02d] " fmt "\n", _ftNow.tm_hour, _ftNow.tm_min, _ftNow.tm_sec,          \
			        ##__VA_ARGS__);                                                                                    \
			FtDrvLogFlush();                                                                                           \
		}                                                                                                              \
	} while (0)
#endif

#endif // FACETRACKING_TESTS
