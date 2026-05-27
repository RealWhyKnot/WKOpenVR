#define _CRT_SECURE_NO_DEPRECATE
#include "Logging.h"

#include "DebugLogging.h"
#include "FileLog.h"
#include "LogPaths.h"

#include <cerrno>
#include <chrono>

FILE *FtDrvLogFile = nullptr;

void FtDrvOpenLogFile()
{
    if (!openvr_pair::common::IsDebugLoggingEnabled()) return;
    if (FtDrvLogFile) return;

    std::wstring path = openvr_pair::common::TimestampedLogPath(L"facetracking_drv_log");
    int openErrno = 0;
    if (!path.empty()) {
        FILE *f = _wfopen(path.c_str(), L"a");
        if (f) {
            FtDrvLogFile = f;
            openvr_pair::common::SetLowLatencyLogMode(FtDrvLogFile);
            return;
        }
        openErrno = errno;
    }
    FILE *f = fopen("facetracking_drv.log", "a");
    FtDrvLogFile = f ? f : stderr;
    openvr_pair::common::SetLowLatencyLogMode(FtDrvLogFile);
    if (FtDrvLogFile) {
        fprintf(FtDrvLogFile,
            "[log-open] facetracking driver log using fallback path; primary_errno=%d primary_path_empty=%d\n",
            openErrno, path.empty() ? 1 : 0);
        FtDrvLogFlush();
    }
}

bool FtDrvEnsureLogFileOpen()
{
    if (!openvr_pair::common::IsDebugLoggingEnabled()) return false;
    if (!FtDrvLogFile) FtDrvOpenLogFile();
    return FtDrvLogFile != nullptr;
}

tm FtDrvTimeForLog()
{
    auto now     = std::chrono::system_clock::now();
    auto nowTime = std::chrono::system_clock::to_time_t(now);
    tm value{};
    localtime_s(&value, &nowTime);
    return value;
}

void FtDrvLogFlush()
{
    if (FtDrvLogFile) openvr_pair::common::FlushLogFileToDisk(FtDrvLogFile);
}
