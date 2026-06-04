#include <gtest/gtest.h>

#include "BugReport.h"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>

using openvr_pair::common::BugReportOptions;
using openvr_pair::common::CreateBugReport;
using openvr_pair::common::SanitizeBugReportText;

namespace {

std::filesystem::path UniqueTempDir()
{
	const auto now = std::chrono::steady_clock::now().time_since_epoch().count();
	return std::filesystem::temp_directory_path() / ("wkopenvr_bug_report_test_" + std::to_string(now));
}

void WriteText(const std::filesystem::path& path, const std::string& text)
{
	std::ofstream out(path, std::ios::binary);
	out << text;
}

std::string ReadText(const std::filesystem::path& path)
{
	std::ifstream in(path, std::ios::binary);
	return std::string(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}

} // namespace

TEST(BugReport, SanitizesPersonalData)
{
	const std::string raw = "C:\\Users\\Alice\\AppData\\LocalLow\\WKOpenVR\\Logs\\spacecal_log.txt\n"
	                        "user alice@example.com endpoint=192.168.1.42 token=ghp_abcdefghijklmnopqrstuvwxyz\n"
	                        "serial='LHR-10268F5C'\n";

	const std::string sanitized = SanitizeBugReportText(raw);

	EXPECT_EQ(sanitized.find("Alice"), std::string::npos);
	EXPECT_EQ(sanitized.find("alice@example.com"), std::string::npos);
	EXPECT_EQ(sanitized.find("192.168.1.42"), std::string::npos);
	EXPECT_EQ(sanitized.find("ghp_abcdefghijklmnopqrstuvwxyz"), std::string::npos);
	EXPECT_EQ(sanitized.find("LHR-10268F5C"), std::string::npos);
	EXPECT_NE(sanitized.find("C:\\Users\\<user>"), std::string::npos);
	EXPECT_NE(sanitized.find("<email>"), std::string::npos);
	EXPECT_NE(sanitized.find("<private-ip>"), std::string::npos);
	EXPECT_NE(sanitized.find("<github-token>"), std::string::npos);
	EXPECT_NE(sanitized.find("serial='<device-serial>'"), std::string::npos);
}

TEST(BugReport, CreatesReportAndPrefilledIssueUrl)
{
	const std::filesystem::path root = UniqueTempDir();
	const std::filesystem::path logs = root / "Logs";
	std::filesystem::create_directories(logs);

	WriteText(logs / "driver_log.2026-05-27T01-00-00.txt",
	          "driver path C:\\Users\\Bob\\AppData\\LocalLow\\WKOpenVR\\Logs\\driver_log.txt\n");
	WriteText(logs / "spacecal_log.2026-05-27T01-00-01.txt",
	          "serial='LHR-ABC123' quest=<private-ip> email bob@example.com\n");

	BugReportOptions options;
	options.logRoot = logs.wstring();
	options.version = "2026.5.27.0-TEST";
	options.maxLogFiles = 4;
	options.maxBytesPerLog = 4096;

	const auto result = CreateBugReport(options);
	ASSERT_TRUE(result.success) << result.error;
	EXPECT_EQ(result.sourceLogs.size(), 2u);
	EXPECT_NE(result.issueUrl.find("template=bug_report.yml"), std::string::npos);
	EXPECT_NE(result.issueUrl.find("version=2026.5.27.0-TEST"), std::string::npos);
	EXPECT_NE(result.issueUrl.find("logs="), std::string::npos);

	const std::string report = ReadText(result.reportFile);
	EXPECT_NE(report.find("WKOpenVR bug report"), std::string::npos);
	EXPECT_NE(report.find("2026.5.27.0-TEST"), std::string::npos);
	EXPECT_EQ(report.find("Bob"), std::string::npos);
	EXPECT_EQ(report.find("bob@example.com"), std::string::npos);
	EXPECT_EQ(report.find("LHR-ABC123"), std::string::npos);
	EXPECT_NE(report.find("<email>"), std::string::npos);
	EXPECT_NE(report.find("<device-serial>"), std::string::npos);

	std::filesystem::remove_all(root);
}
