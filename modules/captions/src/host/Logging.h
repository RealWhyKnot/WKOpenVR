#pragma once

#include <string>

// Opens a timestamped log file under %LocalAppDataLow%\WKOpenVR\Logs\.
// Safe to call multiple times; subsequent calls are noops.
void CaptionsHostOpenLogFile();
void CaptionsHostFlushLog();

void CaptionsHostLog(const char* fmt, ...);

#define TH_LOG(fmt, ...) CaptionsHostLog((fmt), ##__VA_ARGS__)
