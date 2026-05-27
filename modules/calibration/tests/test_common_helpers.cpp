#include "FileLog.h"
#include "JsonUtil.h"
#include "LogPaths.h"

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

#include <gtest/gtest.h>

#include <chrono>
#include <cstdio>
#include <fstream>
#include <string>

namespace {

std::wstring MakeTempDir()
{
	wchar_t base[MAX_PATH] = {};
	if (!GetTempPathW(MAX_PATH, base)) return {};

	wchar_t path[MAX_PATH] = {};
	if (!GetTempFileNameW(base, L"wko", 0, path)) return {};
	DeleteFileW(path);
	if (!CreateDirectoryW(path, nullptr)) return {};
	return path;
}

void WriteFileText(const std::wstring &path)
{
	std::ofstream out(path, std::ios::binary);
	out << "x";
}

void SetFileAgeHours(const std::wstring &path, uint64_t hoursAgo)
{
	FILETIME now{};
	GetSystemTimeAsFileTime(&now);
	ULARGE_INTEGER stamp{};
	stamp.LowPart = now.dwLowDateTime;
	stamp.HighPart = now.dwHighDateTime;
	stamp.QuadPart -= hoursAgo * 3600ULL * 10ULL * 1000ULL * 1000ULL;

	FILETIME ft{};
	ft.dwLowDateTime = stamp.LowPart;
	ft.dwHighDateTime = stamp.HighPart;

	HANDLE file = CreateFileW(
		path.c_str(),
		FILE_WRITE_ATTRIBUTES,
		FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE,
		nullptr,
		OPEN_EXISTING,
		FILE_ATTRIBUTE_NORMAL,
		nullptr);
	ASSERT_NE(file, INVALID_HANDLE_VALUE);
	EXPECT_TRUE(SetFileTime(file, nullptr, nullptr, &ft));
	CloseHandle(file);
}

bool Exists(const std::wstring &path)
{
	return GetFileAttributesW(path.c_str()) != INVALID_FILE_ATTRIBUTES;
}

} // namespace

TEST(JsonUtil, ParsesBomPrefixedObjectsAndTypedFallbacks)
{
	picojson::value root;
	std::string err;
	ASSERT_TRUE(openvr_pair::common::json::ParseObject(
		root,
		std::string("\xEF\xBB\xBF{\"name\":\"tracker\",\"enabled\":true,\"count\":7}"),
		&err)) << err;

	EXPECT_EQ("tracker", openvr_pair::common::json::StringAt(root, "name"));
	EXPECT_TRUE(openvr_pair::common::json::BoolAt(root, "enabled"));
	EXPECT_EQ(7, openvr_pair::common::json::IntAt(root, "count"));
	EXPECT_EQ("fallback", openvr_pair::common::json::StringAt(root, "missing", "fallback"));
	EXPECT_FALSE(openvr_pair::common::json::BoolAt(root, "name"));
	EXPECT_EQ(12.5, openvr_pair::common::json::NumberAt(root, "missing_number", 12.5));
}

TEST(LogPaths, BuildsStableTimestampedFileNames)
{
	SYSTEMTIME time{};
	time.wYear = 2026;
	time.wMonth = 5;
	time.wDay = 13;
	time.wHour = 4;
	time.wMinute = 5;
	time.wSecond = 6;

	EXPECT_EQ(
		L"spacecal_log.2026-05-13T04-05-06.txt",
		openvr_pair::common::TimestampedLogFileName(L"spacecal_log", time));
}

TEST(LogPaths, DeleteOldLogFilesOnlyDeletesMatchingPrefix)
{
	std::wstring dir = MakeTempDir();
	ASSERT_FALSE(dir.empty());

	const std::wstring oldMatch = dir + L"\\overlay_log.old.txt";
	const std::wstring newMatch = dir + L"\\overlay_log.new.txt";
	const std::wstring oldOther = dir + L"\\driver_log.old.txt";

	WriteFileText(oldMatch);
	WriteFileText(newMatch);
	WriteFileText(oldOther);

	SetFileAgeHours(oldMatch, 48);
	SetFileAgeHours(newMatch, 1);
	SetFileAgeHours(oldOther, 48);

	openvr_pair::common::DeleteOldLogFiles(dir, L"overlay_log", std::chrono::hours(24));

	EXPECT_FALSE(Exists(oldMatch));
	EXPECT_TRUE(Exists(newMatch));
	EXPECT_TRUE(Exists(oldOther));

	DeleteFileW(newMatch.c_str());
	DeleteFileW(oldOther.c_str());
	RemoveDirectoryW(dir.c_str());
}

TEST(FileLog, FlushLogFileToDiskAcceptsOpenFile)
{
	std::wstring dir = MakeTempDir();
	ASSERT_FALSE(dir.empty());

	const std::wstring path = dir + L"\\flush.txt";
	FILE* file = _wfopen(path.c_str(), L"wb");
	ASSERT_NE(file, nullptr);

	openvr_pair::common::SetLowLatencyLogMode(file);
	ASSERT_EQ(1u, fwrite("x", 1, 1, file));
	EXPECT_TRUE(openvr_pair::common::FlushLogFileToDisk(file));

	fclose(file);
	DeleteFileW(path.c_str());
	RemoveDirectoryW(dir.c_str());
}
