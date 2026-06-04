#pragma once

#include <cstdio>
#include <ctime>

#ifdef OSCROUTER_TESTS
// In the gtest binary there is no log file infrastructure.
inline FILE* OrDrvLogFile = nullptr;
inline void OrDrvOpenLogFile() {}
inline bool OrDrvEnsureLogFileOpen()
{
	return false;
}
inline tm OrDrvTimeForLog()
{
	return {};
}
inline void OrDrvLogFlush() {}
#define OR_LOG(fmt, ...)                                                                                               \
	do {                                                                                                               \
	} while (0)
#else

extern FILE* OrDrvLogFile;

void OrDrvOpenLogFile();
bool OrDrvEnsureLogFileOpen();
tm OrDrvTimeForLog();
void OrDrvLogFlush();

#ifndef OR_LOG
#define OR_LOG(fmt, ...)                                                                                               \
	do {                                                                                                               \
		if (OrDrvEnsureLogFileOpen()) {                                                                                \
			tm _orNow = OrDrvTimeForLog();                                                                             \
			fprintf(OrDrvLogFile, "[%02d:%02d:%02d] [oscrouter] " fmt "\n", _orNow.tm_hour, _orNow.tm_min,             \
			        _orNow.tm_sec, ##__VA_ARGS__);                                                                     \
			OrDrvLogFlush();                                                                                           \
		}                                                                                                              \
	} while (0)
#endif

#endif // OSCROUTER_TESTS
