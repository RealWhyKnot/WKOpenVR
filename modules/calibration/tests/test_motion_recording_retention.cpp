#include <gtest/gtest.h>

#include "MotionRecording.h"
#include "Win32Text.h"

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

namespace replay = spacecal::replay;

namespace {

replay::LogFileEntry Entry(uint64_t sizeBytes)
{
	replay::LogFileEntry entry;
	entry.sizeBytes = sizeBytes;
	return entry;
}

} // namespace

TEST(MotionRecordingRetentionTest, KeepsNewestFilesWithinFileLimit)
{
	std::vector<replay::LogFileEntry> entries = {
	    Entry(100), Entry(200), Entry(300), Entry(400), Entry(500),
	};
	const replay::RecordingRetentionPolicy policy{3, 5000};

	const auto plan = replay::PlanRecordingRetention(entries, policy);

	EXPECT_EQ((std::vector<std::size_t>{3, 4}), plan.deleteIndexes);
	EXPECT_EQ(3u, plan.keptFiles);
	EXPECT_EQ(600u, plan.keptBytes);
	EXPECT_EQ(900u, plan.deletedBytes);
}

TEST(MotionRecordingRetentionTest, PrunesOldestFilesToStayUnderByteLimit)
{
	std::vector<replay::LogFileEntry> entries = {
	    Entry(30), Entry(30), Entry(30), Entry(30), Entry(30),
	};
	const replay::RecordingRetentionPolicy policy{5, 100};

	const auto plan = replay::PlanRecordingRetention(entries, policy);

	EXPECT_EQ((std::vector<std::size_t>{3, 4}), plan.deleteIndexes);
	EXPECT_EQ(3u, plan.keptFiles);
	EXPECT_EQ(90u, plan.keptBytes);
	EXPECT_EQ(60u, plan.deletedBytes);
}

TEST(MotionRecordingRetentionTest, KeepsNewestOversizedRecording)
{
	std::vector<replay::LogFileEntry> entries = {
	    Entry(250),
	    Entry(25),
	    Entry(25),
	};
	const replay::RecordingRetentionPolicy policy{5, 100};

	const auto plan = replay::PlanRecordingRetention(entries, policy);

	EXPECT_EQ((std::vector<std::size_t>{1, 2}), plan.deleteIndexes);
	EXPECT_EQ(1u, plan.keptFiles);
	EXPECT_EQ(250u, plan.keptBytes);
	EXPECT_EQ(50u, plan.deletedBytes);
}

TEST(MotionRecordingReplayTest, ContinuousReplayCapsSampleWindow)
{
	replay::LoadedRecording recording;
	recording.rows.reserve(1000);

	for (int i = 0; i < 1000; ++i) {
		replay::ReplayRow row;
		row.timestamp = static_cast<double>(i) / 90.0;
		row.ref.rot.setIdentity();
		row.target.rot.setIdentity();
		row.ref.trans =
		    Eigen::Vector3d(static_cast<double>(i % 50) * 0.001, 1.60 + static_cast<double>((i / 50) % 10) * 0.001,
		                    static_cast<double>((i * 3) % 70) * 0.001);
		row.target.trans = row.ref.trans + Eigen::Vector3d(0.15, 0.0, -0.05);
		recording.rows.push_back(std::move(row));
	}

	replay::ReplayOptions options;
	options.continuous = true;
	options.maxContinuousSamples = 25;

	const auto result = replay::RunReplay(recording, options);

	EXPECT_TRUE(result.succeeded) << result.error;
	EXPECT_EQ(1000, result.rowsReplayed);
	EXPECT_EQ(1000u, result.trace.size());
	EXPECT_LE(result.maxSamplesInWindow, 25);
}

TEST(MotionRecordingReplayTest, ReplayLocalRecordingsWhenRequested)
{
	const char* enabled = std::getenv("WKOPENVR_REPLAY_RECORDINGS");
	if (!enabled || std::string(enabled) != "1") {
		GTEST_SKIP() << "Set WKOPENVR_REPLAY_RECORDINGS=1 to replay retained local recordings.";
	}

	const auto files = replay::ListRecordings();
	if (files.empty()) {
		GTEST_SKIP() << "No retained spacecal_log recordings found.";
	}

	replay::ReplayOptions options;
	options.continuous = true;

	std::size_t replayed = 0;
	std::size_t skippedEmpty = 0;
	for (const auto& file : files) {
		SCOPED_TRACE(file.name);
		const std::string path = openvr_pair::common::WideToUtf8(file.fullPath);
		const auto recording = replay::LoadRecording(path);
		ASSERT_TRUE(recording.error.empty()) << recording.error;
		if (recording.rows.empty()) {
			++skippedEmpty;
			std::cout << "[replay] " << file.name << " skipped=no_replayable_rows\n";
			continue;
		}

		const auto result = replay::RunReplay(recording, options);
		EXPECT_TRUE(result.succeeded) << result.error;
		EXPECT_GT(result.rowsReplayed, 0);
		std::cout << "[replay] " << file.name << " rows=" << result.rowsReplayed << " accepts=" << result.accepts
		          << " rejects=" << result.rejects << " final_error_mm=" << result.finalErrorMm << "\n";
		++replayed;
	}

	EXPECT_GT(replayed, 0u) << "No retained recordings contained replayable rows; skipped " << skippedEmpty
	                        << " empty recordings.";
}
