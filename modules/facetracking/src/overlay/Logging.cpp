#define _CRT_SECURE_NO_DEPRECATE
#include "Logging.h"

#include "DebugLogging.h"
#include "FileLog.h"
#include "LogPaths.h"

#include <atomic>
#include <chrono>

FILE *FtLogFile = nullptr;

// Verbose flag. Default false; set true by the Logs tab checkbox and forced
// true on dev-channel builds (see LogsSection.cpp).
std::atomic<bool> FtOverlayVerbose{ false };

void FtOpenLogFile()
{
    if (!openvr_pair::common::IsDebugLoggingEnabled()) return;
    if (FtLogFile) return;

    std::wstring path = openvr_pair::common::TimestampedLogPath(L"facetracking_log");
    if (!path.empty()) {
        FtLogFile = _wfopen(path.c_str(), L"a");
        if (FtLogFile) {
            openvr_pair::common::SetLowLatencyLogMode(FtLogFile);
            return;
        }
    }
    FtLogFile = fopen("openvr_facetracking.log", "a");
    if (!FtLogFile) FtLogFile = stderr;
    openvr_pair::common::SetLowLatencyLogMode(FtLogFile);
}

bool FtEnsureLogFileOpen()
{
    if (!openvr_pair::common::IsDebugLoggingEnabled()) return false;
    if (!FtLogFile) FtOpenLogFile();
    return FtLogFile != nullptr;
}

tm FtTimeForLog()
{
    auto now     = std::chrono::system_clock::now();
    auto nowTime = std::chrono::system_clock::to_time_t(now);
    tm   val{};
    localtime_s(&val, &nowTime);
    return val;
}

void FtLogFlush()
{
    if (FtLogFile) openvr_pair::common::FlushLogFileToDisk(FtLogFile);
}
