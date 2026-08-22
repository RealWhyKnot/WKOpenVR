#include "Logging.h"

#include "DebugLogging.h"
#include "FileLog.h"

FILE* LogFile = nullptr;

void OpenLogFile()
{
	if (LogFile) return;
	LogFile = openvr_pair::common::OpenModuleLogFile(L"inputhealth_log", "openvr_inputhealth.log");
}

bool EnsureLogFileOpen()
{
	if (!openvr_pair::common::IsDebugLoggingEnabled()) return false;
	if (!LogFile) OpenLogFile();
	return LogFile != nullptr;
}

tm TimeForLog()
{
	return openvr_pair::common::LocalTimeForLog();
}

void LogFlush()
{
	if (LogFile) openvr_pair::common::FlushLogFileToDisk(LogFile);
}
