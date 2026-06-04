#include "BugReport.h"

#include "Win32Paths.h"
#include "Win32Text.h"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <ctime>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <regex>
#include <sstream>

namespace openvr_pair::common {

namespace {

constexpr const char* kIssueBase = "https://github.com/RealWhyKnot/WKOpenVR/issues/new";

std::string RegexReplace(std::string text, const char* pattern, const char* replacement)
{
	try {
		return std::regex_replace(text, std::regex(pattern), replacement);
	}
	catch (const std::regex_error&) {
		return text;
	}
}

std::string UrlEncode(std::string_view text)
{
	static constexpr char kHex[] = "0123456789ABCDEF";
	std::string out;
	out.reserve(text.size() * 3);
	for (unsigned char c : text) {
		if (std::isalnum(c) || c == '-' || c == '_' || c == '.' || c == '~') {
			out.push_back(static_cast<char>(c));
		}
		else {
			out.push_back('%');
			out.push_back(kHex[(c >> 4) & 0x0f]);
			out.push_back(kHex[c & 0x0f]);
		}
	}
	return out;
}

std::string TimestampForFileName()
{
	const auto now = std::chrono::system_clock::now();
	const std::time_t tt = std::chrono::system_clock::to_time_t(now);
	std::tm tm{};
#ifdef _WIN32
	gmtime_s(&tm, &tt);
#else
	gmtime_r(&tt, &tm);
#endif
	char buf[32];
	std::snprintf(buf, sizeof buf, "%04d%02d%02d-%02d%02d%02d", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
	              tm.tm_hour, tm.tm_min, tm.tm_sec);
	return buf;
}

std::string TimestampForReport()
{
	const auto now = std::chrono::system_clock::now();
	const std::time_t tt = std::chrono::system_clock::to_time_t(now);
	std::tm tm{};
#ifdef _WIN32
	gmtime_s(&tm, &tt);
#else
	gmtime_r(&tt, &tm);
#endif
	char buf[40];
	std::snprintf(buf, sizeof buf, "%04d-%02d-%02d %02d:%02d:%02d UTC", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
	              tm.tm_hour, tm.tm_min, tm.tm_sec);
	return buf;
}

std::string ReadFilePrefix(const std::filesystem::path& path, size_t maxBytes)
{
	std::ifstream in(path, std::ios::binary);
	if (!in) return {};
	std::string data(maxBytes, '\0');
	in.read(data.data(), static_cast<std::streamsize>(data.size()));
	data.resize(static_cast<size_t>(in.gcount()));
	return data;
}

std::wstring ReportRootFromLogs(const std::wstring& logRoot)
{
	if (!logRoot.empty()) {
		std::filesystem::path p(logRoot);
		if (p.has_parent_path()) {
			return (p.parent_path() / L"BugReports").wstring();
		}
	}
	return (std::filesystem::path(WkOpenVrLogsPath(true)).parent_path() / L"BugReports").wstring();
}

std::vector<std::filesystem::directory_entry> FindLatestLogs(const std::wstring& logRoot, size_t maxLogFiles)
{
	std::vector<std::filesystem::directory_entry> entries;
	if (logRoot.empty()) return entries;

	std::error_code ec;
	for (const auto& entry : std::filesystem::directory_iterator(logRoot, ec)) {
		if (ec) break;
		if (!entry.is_regular_file(ec) || ec) continue;
		if (entry.path().extension() != L".txt") continue;
		const auto size = entry.file_size(ec);
		if (ec || size == 0) continue;
		entries.push_back(entry);
	}

	std::sort(entries.begin(), entries.end(), [](const auto& a, const auto& b) {
		std::error_code ea;
		std::error_code eb;
		const auto ta = a.last_write_time(ea);
		const auto tb = b.last_write_time(eb);
		if (ea || eb) return a.path().filename().wstring() > b.path().filename().wstring();
		return ta > tb;
	});

	if (entries.size() > maxLogFiles) {
		entries.resize(maxLogFiles);
	}
	return entries;
}

std::string TruncateForIssue(std::string text, size_t maxChars)
{
	if (text.size() <= maxChars) return text;
	if (maxChars < 128) return text.substr(0, maxChars);
	text.resize(maxChars - 80);
	text += "\n\n[truncated for browser URL; attach the generated report text file]\n";
	return text;
}

void AddQueryParam(std::string& url, const char* key, std::string_view value)
{
	url.push_back(url.find('?') == std::string::npos ? '?' : '&');
	url += key;
	url.push_back('=');
	url += UrlEncode(value);
}

} // namespace

std::string SanitizeBugReportText(std::string_view text)
{
	std::string out(text);

	out = RegexReplace(out, R"(([A-Za-z]:[\\/]+Users[\\/]+)[^\\/\s:'"]+)", "$1<user>");
	out = RegexReplace(out, R"((%USERPROFILE%[\\/]+)[^\\/\s:'"]+)", "$1<user>");
	out = RegexReplace(out, R"(\b[A-Za-z0-9._%+\-]+@[A-Za-z0-9.\-]+\.[A-Za-z]{2,}\b)", "<email>");
	out = RegexReplace(out, R"(\b(?:10|192\.168|172\.(?:1[6-9]|2[0-9]|3[0-1]))(?:\.[0-9]{1,3}){2}\b)", "<private-ip>");
	out = RegexReplace(out, R"(\b(?:ghp|gho|ghu|ghs|ghr)_[A-Za-z0-9_]{20,}\b)", "<github-token>");
	out = RegexReplace(out, R"(\bgithub_pat_[A-Za-z0-9_]{20,}\b)", "<github-token>");
	out = RegexReplace(out, R"((serial\s*=\s*')[^']+('))", "$1<device-serial>$2");
	out = RegexReplace(out, R"re((serial\s*=\s*")[^"]+("))re", "$1<device-serial>$2");
	out = RegexReplace(out, R"(\bLHR-[A-Za-z0-9_-]+\b)", "<device-serial>");
	out = RegexReplace(out, R"(\b(?:access|auth|refresh|secret|token)[_-]?token\s*[:=]\s*[A-Za-z0-9._\-+/=]{12,}\b)",
	                   "<token>");

	return out;
}

std::string BuildBugReportIssueUrl(std::string_view version, std::string_view logExcerpt)
{
	const std::string title = "[bug] ";
	const std::string whatHappened = "Created with the in-app Report bug button.\n\n"
	                                 "Steps to reproduce:\n"
	                                 "1. \n"
	                                 "2. \n\n"
	                                 "Expected:\n\n"
	                                 "Actual:\n";
	const std::string environment = "WKOpenVR version: " + std::string(version) +
	                                "\n"
	                                "SteamVR / headset / tracker details: ";
	const std::string extra =
	    "The in-app report flow prepared a sanitized report text file from the newest logs. "
	    "Attach that generated .txt file before submitting if the browser did not include enough detail here.";

	std::string url(kIssueBase);
	AddQueryParam(url, "template", "bug_report.yml");
	AddQueryParam(url, "title", title);
	AddQueryParam(url, "what-happened", whatHappened);
	AddQueryParam(url, "version", version);
	AddQueryParam(url, "environment", environment);
	AddQueryParam(url, "logs", logExcerpt);
	AddQueryParam(url, "extra", extra);
	return url;
}

BugReportResult CreateBugReport(const BugReportOptions& options)
{
	BugReportResult result;

	const std::wstring reportRoot = ReportRootFromLogs(options.logRoot);
	const std::string stamp = TimestampForFileName();
	const std::wstring reportDir =
	    (std::filesystem::path(reportRoot) / Utf8ToWide("WKOpenVR-bug-report-" + stamp)).wstring();
	const std::wstring reportFile =
	    (std::filesystem::path(reportDir) / Utf8ToWide("WKOpenVR-bug-report-" + stamp + ".txt")).wstring();

	std::error_code ec;
	std::filesystem::create_directories(reportDir, ec);
	if (ec) {
		result.error = "Could not create bug report folder.";
		return result;
	}

	const auto logs = FindLatestLogs(options.logRoot, options.maxLogFiles);
	std::ostringstream report;
	std::ostringstream issueLogs;

	report << "WKOpenVR bug report\n";
	report << "Generated: " << TimestampForReport() << "\n";
	report << "Version: " << options.version << "\n";
	report << "Privacy: user paths, private IPs, email addresses, tokens, and device serials are redacted.\n\n";

	if (logs.empty()) {
		report << "No log files were found.\n";
		issueLogs << "No log files were found.\n";
	}
	else {
		report << "Included logs:\n";
		for (const auto& entry : logs) {
			result.sourceLogs.push_back(entry.path().wstring());
			report << "- " << WideToUtf8(entry.path().filename().wstring()) << "\n";
		}
		report << "\n";

		for (const auto& entry : logs) {
			const std::string name = WideToUtf8(entry.path().filename().wstring());
			const std::string raw = ReadFilePrefix(entry.path(), options.maxBytesPerLog);
			const std::string sanitized = SanitizeBugReportText(raw);
			report << "===== " << name << " =====\n";
			report << sanitized;
			if (!sanitized.empty() && sanitized.back() != '\n') report << "\n";
			if (raw.size() >= options.maxBytesPerLog) {
				report << "[file truncated at " << options.maxBytesPerLog << " bytes]\n";
			}
			report << "\n";

			if (issueLogs.tellp() < static_cast<std::streampos>(options.maxIssueLogChars)) {
				issueLogs << "===== " << name << " =====\n";
				issueLogs << sanitized.substr(0, 1200);
				if (sanitized.size() > 1200) issueLogs << "\n[file excerpt truncated]\n";
				issueLogs << "\n";
			}
		}
	}

	result.issueBody = report.str();
	result.issueUrl =
	    BuildBugReportIssueUrl(options.version, TruncateForIssue(issueLogs.str(), options.maxIssueLogChars));

	std::ofstream out(reportFile, std::ios::binary);
	if (!out) {
		result.error = "Could not write bug report file.";
		return result;
	}
	out << result.issueBody;
	out.close();
	if (!out) {
		result.error = "Could not finish bug report file.";
		return result;
	}

	result.success = true;
	result.reportDirectory = reportDir;
	result.reportFile = reportFile;
	return result;
}

} // namespace openvr_pair::common
