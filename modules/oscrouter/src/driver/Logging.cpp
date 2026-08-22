#include "Logging.h"

#include "DebugLogging.h"
#include "FileLog.h"

FILE* OrDrvLogFile = nullptr;

void OrDrvOpenLogFile()
{
	if (OrDrvLogFile) return;
	OrDrvLogFile = openvr_pair::common::OpenModuleLogFile(L"oscrouter_log", "oscrouter_drv.log");
}

bool OrDrvEnsureLogFileOpen()
{
	if (!openvr_pair::common::IsDebugLoggingEnabled()) return false;
	if (!OrDrvLogFile) OrDrvOpenLogFile();
	return OrDrvLogFile != nullptr;
}

tm OrDrvTimeForLog()
{
	return openvr_pair::common::LocalTimeForLog();
}

void OrDrvLogFlush()
{
	if (OrDrvLogFile) openvr_pair::common::FlushLogFileToDisk(OrDrvLogFile);
}
