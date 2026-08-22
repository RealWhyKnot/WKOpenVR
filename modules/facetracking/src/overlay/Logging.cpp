#include "Logging.h"

#include "DebugLogging.h"
#include "FileLog.h"

#include <atomic>

FILE* FtLogFile = nullptr;

// Verbose flag. Default false; set true by the Logs tab checkbox and forced
// true on dev-channel builds (see LogsSection.cpp).
std::atomic<bool> FtOverlayVerbose{false};

void FtOpenLogFile()
{
	if (FtLogFile) return;
	FtLogFile = openvr_pair::common::OpenModuleLogFile(L"facetracking_log", "openvr_facetracking.log");
}

bool FtEnsureLogFileOpen()
{
	if (!openvr_pair::common::IsDebugLoggingEnabled()) return false;
	if (!FtLogFile) FtOpenLogFile();
	return FtLogFile != nullptr;
}

tm FtTimeForLog()
{
	return openvr_pair::common::LocalTimeForLog();
}

void FtLogFlush()
{
	if (FtLogFile) openvr_pair::common::FlushLogFileToDisk(FtLogFile);
}
