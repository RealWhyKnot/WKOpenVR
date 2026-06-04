#pragma once

#include <cstdarg>
#include <cstdio>

// Open the per-session log file (called once in Init). Subsequent TR_LOG_DRV
// calls append to it. Noop if already open.
void TrDrvOpenLogFile();
void TrLogFlushDrv();

// printf-style logging macro. The format string is a literal; arguments are
// forwarded to snprintf and written to the driver log. The trailing (0) is a
// sentinel so the compiler sees at least one non-format argument and can apply
// the format-string attribute on MSVC without warning.
#define TR_LOG_DRV(fmt, ...) TrDrvLog((fmt), ##__VA_ARGS__)

void TrDrvLog(const char* fmt, ...);

// va_list variant for callers that already have args captured (e.g. a base
// class's LogV hook). Same behavior as TrDrvLog otherwise.
void TrDrvLogV(const char* fmt, va_list args);
