#include "FileLog.h"
#include "JsonUtil.h"
#include "LogPaths.h"
#include "ProcessPerfLog.h"
#include "RuntimeHealthSummary.h"

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

TEST(ProcessPerfLog, CalculatesCpuPercentAgainstWholeCpu)
{
	constexpr uint64_t kOneSecond100ns = 10ULL * 1000ULL * 1000ULL;

	EXPECT_DOUBLE_EQ(
		100.0,
		openvr_pair::common::CalculateProcessCpuPercentOneCore(kOneSecond100ns, 1000));
	EXPECT_DOUBLE_EQ(
		25.0,
		openvr_pair::common::CalculateProcessCpuPercentTotal(kOneSecond100ns, 1000, 4));
	EXPECT_DOUBLE_EQ(
		100.0,
		openvr_pair::common::CalculateProcessCpuPercentTotal(8 * kOneSecond100ns, 1000, 4));
	EXPECT_DOUBLE_EQ(
		0.0,
		openvr_pair::common::CalculateProcessCpuPercentTotal(kOneSecond100ns, 0, 4));
}

TEST(ProcessPerfLog, ThrottlesBySampleInterval)
{
	EXPECT_TRUE(openvr_pair::common::ShouldTakeProcessPerfSample(0, 100, 10000));
	EXPECT_FALSE(openvr_pair::common::ShouldTakeProcessPerfSample(100, 9999, 10000));
	EXPECT_TRUE(openvr_pair::common::ShouldTakeProcessPerfSample(100, 10100, 10000));
	EXPECT_TRUE(openvr_pair::common::ShouldTakeProcessPerfSample(500, 100, 10000));
}

TEST(ProcessPerfLog, CollectsCurrentProcessSnapshot)
{
	openvr_pair::common::ProcessPerfSnapshot snapshot{};
	ASSERT_TRUE(openvr_pair::common::CollectProcessPerfSnapshot(snapshot));

	EXPECT_NE(0u, snapshot.processId);
	EXPECT_GE(snapshot.logicalProcessors, 1u);
	EXPECT_TRUE(snapshot.cpuTimeValid || snapshot.memoryValid || snapshot.handleCountValid);
	if (snapshot.memoryValid) {
		EXPECT_GT(snapshot.workingSetBytes, 0u);
	}
}

TEST(ProcessPerfLog, FormatsStableLogFields)
{
	openvr_pair::common::ProcessPerfSample sample{};
	sample.snapshot.processId = 1234;
	sample.snapshot.logicalProcessors = 8;
	sample.snapshot.memoryValid = true;
	sample.snapshot.workingSetBytes = 128ULL * 1024ULL * 1024ULL;
	sample.snapshot.privateBytes = 64ULL * 1024ULL * 1024ULL;
	sample.snapshot.peakWorkingSetBytes = 256ULL * 1024ULL * 1024ULL;
	sample.snapshot.handleCountValid = true;
	sample.snapshot.handleCount = 55;
	sample.cpuValid = true;
	sample.intervalMs = 10000;
	sample.processCpuMs = 250;
	sample.cpuPctTotal = 0.31;
	sample.cpuPctOneCore = 2.50;

	const std::string line = openvr_pair::common::FormatProcessPerfSample("overlay", sample);
	EXPECT_NE(std::string::npos, line.find("role=overlay"));
	EXPECT_NE(std::string::npos, line.find("cpu_pct_total=0.31"));
	EXPECT_NE(std::string::npos, line.find("cpu_pct_one_core=2.50"));
	EXPECT_NE(std::string::npos, line.find("working_set_mb=128.00"));
	EXPECT_NE(std::string::npos, line.find("private_mb=64.00"));
	EXPECT_NE(std::string::npos, line.find("handles=55"));
}

TEST(RuntimeHealthSummary, FormatsRuntimeHealthJson)
{
	openvr_pair::common::ResetRuntimeHealthSummaryForTests();

	openvr_pair::common::ProcessPerfSample process{};
	process.snapshot.processId = 1234;
	process.snapshot.logicalProcessors = 8;
	process.snapshot.memoryValid = true;
	process.snapshot.workingSetBytes = 128ULL * 1024ULL * 1024ULL;
	process.snapshot.privateBytes = 64ULL * 1024ULL * 1024ULL;
	process.snapshot.handleCountValid = true;
	process.snapshot.handleCount = 55;
	process.cpuValid = true;
	process.cpuPctTotal = 0.31;
	process.cpuPctOneCore = 2.50;
	openvr_pair::common::RecordRuntimeProcessSample("overlay", process);

	openvr_pair::common::RuntimeCompositorTimingSample frame{};
	frame.frameIndex = 42;
	frame.framePresents = 2;
	frame.droppedFrames = 1;
	frame.mispresentedFrames = 1;
	frame.reprojectionFlags = 1;
	frame.clientFrameIntervalMs = 11.1;
	frame.totalRenderGpuMs = 6.4;
	frame.hmdPoseValid = false;
	openvr_pair::common::RecordRuntimeCompositorTiming(frame);

	openvr_pair::common::RuntimePoseHealthSample pose{};
	pose.refPoseAgeMs = 3.0;
	pose.targetPoseAgeMs = 5.0;
	pose.refPoseGapMs = 11.0;
	pose.targetPoseGapMs = 22.0;
	pose.stale = true;
	pose.jump = true;
	openvr_pair::common::RecordRuntimePoseHealth(pose);

	openvr_pair::common::RuntimeCalibrationHealthSample calibration{};
	calibration.valid = true;
	calibration.sampleCount = 60;
	calibration.validSampleCount = 60;
	calibration.pairedSampleCount = 58;
	calibration.trackingHealthPass = false;
	calibration.shadowDynamicPass = false;
	calibration.residualRmsMm = 2.5;
	openvr_pair::common::RecordRuntimeCalibrationHealth(calibration);

	const std::string body = openvr_pair::common::FormatRuntimeHealthSummaryForTests();
	picojson::value root;
	std::string err;
	ASSERT_TRUE(openvr_pair::common::json::ParseObject(root, body, &err)) << err;

	EXPECT_NE(std::string::npos, body.find("\"role\": \"overlay\""));
	EXPECT_NE(std::string::npos, body.find("\"working_set_mb\": 128.000"));
	EXPECT_NE(std::string::npos, body.find("\"dropped_frames\": 1"));
	EXPECT_NE(std::string::npos, body.find("\"hmd_pose_invalid_frames\": 1"));
	EXPECT_NE(std::string::npos, body.find("\"stale_samples\": 1"));
	EXPECT_NE(std::string::npos, body.find("\"jump_samples\": 1"));
	EXPECT_NE(std::string::npos, body.find("\"tracking_health_pass\": false"));

	openvr_pair::common::ResetRuntimeHealthSummaryForTests();
}
