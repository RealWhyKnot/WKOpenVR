#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace spacecal::ui_logs {

struct LogFileEntry
{
	std::wstring fullPath;
	std::string name;
	uint64_t sizeBytes = 0;
	uint64_t mtimeFileTime = 0;
};

struct LogsPanelState
{
	std::vector<LogFileEntry> files;
	bool listBuilt = false;
	int selectedIdx = -1;
	std::string copyHint;
	double copyHintExpireTime = 0.0;
};

LogsPanelState& LogsState();
void RebuildLogsList(LogsPanelState& state);
std::wstring ResolveLogsDirectory(const LogsPanelState& state);
void DrawLogFileList(LogsPanelState& state);
void DrawSelectedLogActions(LogsPanelState& state);

} // namespace spacecal::ui_logs
