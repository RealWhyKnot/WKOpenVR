#include <gtest/gtest.h>

#include "MotionRecording.h"
#include "Win32Text.h"

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cmath>
#include <filesystem>
#include <fstream>
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

TEST(MotionRecordingReplayTest, LoadsV3SampleHealthAndReportsShadowQuality)
{
	const std::filesystem::path path = std::filesystem::temp_directory_path() / "wkopenvr_replay_v3_sample_health.csv";
	std::filesystem::remove(path);

	{
		std::ofstream out(path);
		ASSERT_TRUE(out) << path.string();
		out << "# spacecal_log_v3\n";
		out << "Timestamp,ref_tx,ref_ty,ref_tz,ref_qw,ref_qx,ref_qy,ref_qz,"
		       "tgt_tx,tgt_ty,tgt_tz,tgt_qw,tgt_qx,tgt_qy,tgt_qz,tick_phase,"
		       "sample_observed,sample_accepted,sample_paired_motion_valid,"
		       "sample_ref_connected,sample_tgt_connected,sample_ref_pose_valid,sample_tgt_pose_valid,"
		       "sample_ref_tracking_result,sample_tgt_tracking_result,"
		       "sample_ref_age_ms,sample_tgt_age_ms,sample_ref_gap_ms,sample_tgt_gap_ms,"
		       "sample_ref_speed_mps,sample_tgt_speed_mps,sample_ref_ang_speed_radps,sample_tgt_ang_speed_radps,"
		       "sample_ref_zero_pose,sample_tgt_zero_pose,sample_ref_unchanged,sample_tgt_unchanged,"
		       "sample_stale,sample_jump\n";
		for (int i = 0; i < 30; ++i) {
			const double t = static_cast<double>(i) / 60.0;
			const double yaw = static_cast<double>(i) * 0.035;
			const double qw = std::cos(yaw * 0.5);
			const double qy = std::sin(yaw * 0.5);
			const bool rejected = i == 5;
			const bool stale = i == 10;
			const bool pairedMotionValid = i != 12;
			const bool unchanged = i == 14;
			const bool zeroPose = i == 16;
			const bool highMotion = i == 18;
			const int refPoseValid = rejected ? 0 : 1;
			const int refTracking = rejected ? 101 : 200;
			const double linearSpeed = highMotion ? 2.0 : 0.25;
			const double angularSpeed = highMotion ? 4.0 : 0.50;
			out << t << "," << (0.01 * i) << "," << (1.60 + 0.001 * (i % 5)) << "," << (0.004 * (i % 7)) << "," << qw
			    << ",0," << qy << ",0," << (0.01 * i + 0.15) << "," << (1.60 + 0.001 * (i % 5)) << ","
			    << (0.004 * (i % 7) - 0.05) << "," << qw << ",0," << qy << ",0,Continuous,1," << (rejected ? 0 : 1)
			    << "," << (pairedMotionValid ? 1 : 0) << ",1,1," << refPoseValid << ",1," << refTracking << ",200,"
			    << (stale ? 175.0 : 8.0) << "," << (stale ? 185.0 : 9.0) << "," << (stale ? 220.0 : 16.0) << ","
			    << (stale ? 230.0 : 16.5) << "," << linearSpeed << "," << linearSpeed << "," << angularSpeed << ","
			    << angularSpeed << "," << (zeroPose ? 1 : 0) << ",0," << (unchanged ? 1 : 0) << ",0," << (stale ? 1 : 0)
			    << ",0\n";
		}
	}

	const auto recording = replay::LoadRecording(path.string());
	std::filesystem::remove(path);
	ASSERT_TRUE(recording.error.empty()) << recording.error;
	ASSERT_EQ(3, recording.formatVersion);
	ASSERT_EQ(30u, recording.rows.size());
	EXPECT_TRUE(recording.rows[5].hasSampleDiagnostics);
	EXPECT_FALSE(recording.rows[5].sampleAccepted);
	EXPECT_TRUE(recording.rows[10].sample.trackingPoseStale);
	EXPECT_FALSE(recording.rows[12].sample.pairedMotionValid);
	EXPECT_TRUE(recording.rows[14].sample.refPoseUnchanged);
	EXPECT_TRUE(recording.rows[16].sample.refZeroPose);

	replay::ReplayOptions options;
	options.continuous = false;
	options.qualityReportInterval = 10;
	const auto result = replay::RunReplay(recording, options);

	EXPECT_TRUE(result.succeeded) << result.error;
	EXPECT_EQ(30, result.rowsReplayed);
	EXPECT_EQ(30, result.sampleRowsObserved);
	EXPECT_EQ(29, result.sampleRowsAccepted);
	EXPECT_EQ(1, result.sampleRowsRejected);
	EXPECT_EQ(1, result.sampleRowsStrictUnhealthy);
	EXPECT_EQ(1, result.sampleRowsStale);
	EXPECT_EQ(1, result.sampleRowsPairedMotionInvalid);
	EXPECT_EQ(1, result.sampleRowsZeroPose);
	EXPECT_EQ(1, result.sampleRowsUnchanged);
	EXPECT_EQ(1, result.sampleRowsHighMotion);
	EXPECT_GT(result.qualityReports, 0);
	EXPECT_TRUE(result.finalQuality.available);
	EXPECT_FALSE(result.finalQuality.strictSamplesPass);
	EXPECT_GE(result.finalQuality.staleSampleCount, 1);
	EXPECT_GE(result.finalQuality.zeroPoseSampleCount, 1);
	EXPECT_GE(result.finalQuality.unchangedPoseSampleCount, 1);
	EXPECT_GE(result.finalQuality.highMotionSampleCount, 1);
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
		          << " rejects=" << result.rejects << " final_error_mm=" << result.finalErrorMm
		          << " sample_rows_accepted=" << result.sampleRowsAccepted
		          << " sample_rows_rejected=" << result.sampleRowsRejected
		          << " strict_unhealthy=" << result.sampleRowsStrictUnhealthy << " stale=" << result.sampleRowsStale
		          << " unchanged=" << result.sampleRowsUnchanged << " high_motion=" << result.sampleRowsHighMotion
		          << " quality_reports=" << result.qualityReports << " shadow_accepts=" << result.shadowWouldAccept
		          << " shadow_rejects=" << result.shadowWouldReject
		          << " final_shadow_reason=" << result.finalQuality.shadowRejectReason << "\n";
		++replayed;
	}

	EXPECT_GT(replayed, 0u) << "No retained recordings contained replayable rows; skipped " << skippedEmpty
	                        << " empty recordings.";
}
