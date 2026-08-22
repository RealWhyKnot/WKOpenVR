#pragma once

#include <cstdio>
#include <ctime>

extern FILE* LogFile;

void OpenLogFile();
bool EnsureLogFileOpen();
tm TimeForLog();
void LogFlush();
void CloseLogFile();

// Emits one timestamped line. Defined per host binary: the driver DLL
// writes the driver log, the in-app desktop host writes the shared
// diagnostics log, so shared driver sources can log from either.
void LogLine(const char* fmt, ...);

#ifndef LOG
#define LOG(fmt, ...) LogLine(fmt, ##__VA_ARGS__)
#endif

#ifndef TRACE
#define TRACE LOG
#endif
