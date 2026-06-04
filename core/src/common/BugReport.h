#pragma once

#include <cstddef>
#include <string>
#include <string_view>
#include <vector>

namespace openvr_pair::common {

struct BugReportOptions
{
	std::wstring logRoot;
	std::string version;
	size_t maxLogFiles = 8;
	size_t maxBytesPerLog = 256 * 1024;
	size_t maxIssueLogChars = 3500;
};

struct BugReportResult
{
	bool success = false;
	std::string error;
	std::wstring reportDirectory;
	std::wstring reportFile;
	std::vector<std::wstring> sourceLogs;
	std::string issueUrl;
	std::string issueBody;
};

std::string SanitizeBugReportText(std::string_view text);
std::string BuildBugReportIssueUrl(std::string_view version, std::string_view logExcerpt);
BugReportResult CreateBugReport(const BugReportOptions& options);

} // namespace openvr_pair::common
