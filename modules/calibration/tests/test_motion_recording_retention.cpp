#include <gtest/gtest.h>

#include "MotionRecording.h"
#include "Win32Text.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cmath>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace replay = spacecal::replay;

namespace {

replay::LogFileEntry Entry(uint64_t sizeBytes)
{
	replay::LogFileEntry entry;
	entry.sizeBytes = sizeBytes;
	return entry;
}

std::string TrimAscii(std::string value)
{
	while (!value.empty() && std::isspace(static_cast<unsigned char>(value.back()))) {
		value.pop_back();
	}
	std::size_t first = 0;
	while (first < value.size() && std::isspace(static_cast<unsigned char>(value[first]))) {
		++first;
	}
	if (first > 0) value.erase(0, first);
	return value;
}

std::vector<std::string> SplitReplayEnvList(const std::string& raw)
{
	std::vector<std::string> out;
	std::string cur;
	for (char c : raw) {
		if (c == ';' || c == '|') {
			cur = TrimAscii(std::move(cur));
			if (!cur.empty()) out.push_back(cur);
			cur.clear();
		}
		else {
			cur.push_back(c);
		}
	}
	cur = TrimAscii(std::move(cur));
	if (!cur.empty()) out.push_back(cur);
	return out;
}

bool EnvFlag(const char* name, bool fallback)
{
	const char* raw = std::getenv(name);
	if (!raw) return fallback;
	std::string value = TrimAscii(raw);
	for (char& c : value) {
		c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
	}
	if (value == "1" || value == "true" || value == "yes" || value == "on") return true;
	if (value == "0" || value == "false" || value == "no" || value == "off") return false;
	return fallback;
}

double EnvDouble(const char* name, double fallback)
{
	const char* raw = std::getenv(name);
	if (!raw) return fallback;
	try {
		return std::stod(TrimAscii(raw));
	}
	catch (...) {
		return fallback;
	}
}

// Parse a tracking-style env override. Accepts the numeric enum value (0-3) or a
// case-insensitive name ("manual", "continuous", "locked"/"lockedwithrecovery",
// "hard"/"hardtrackerlock"). Anything else returns the fallback.
TrackingStyle EnvTrackingStyle(const char* name, TrackingStyle fallback)
{
	const char* raw = std::getenv(name);
	if (!raw) return fallback;
	std::string value = TrimAscii(raw);
	for (char& c : value) {
		c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
	}
	if (value == "0" || value == "manual") return TrackingStyle::Manual;
	if (value == "1" || value == "continuous") return TrackingStyle::Continuous;
	if (value == "2" || value == "locked" || value == "lockedwithrecovery" || value == "locked_with_recovery")
		return TrackingStyle::LockedWithRecovery;
	if (value == "3" || value == "hard" || value == "hardtrackerlock" || value == "hard_tracker_lock")
		return TrackingStyle::HardTrackerLock;
	return fallback;
}

std::size_t EnvSize(const char* name, std::size_t fallback)
{
	const char* raw = std::getenv(name);
	if (!raw) return fallback;
	char* end = nullptr;
	const unsigned long long parsed = std::strtoull(raw, &end, 10);
	if (end == raw) return fallback;
	return static_cast<std::size_t>(parsed);
}

std::vector<std::size_t> ReplaySampleWindows()
{
	std::vector<std::size_t> out;
	if (const char* raw = std::getenv("WKOPENVR_REPLAY_SAMPLE_WINDOWS")) {
		for (const auto& item : SplitReplayEnvList(raw)) {
			char* end = nullptr;
			const unsigned long long parsed = std::strtoull(item.c_str(), &end, 10);
			if (end != item.c_str()) out.push_back(static_cast<std::size_t>(parsed));
		}
	}
	if (out.empty()) {
		const replay::ReplayOptions defaults;
		out.push_back(EnvSize("WKOPENVR_REPLAY_MAX_SAMPLES", defaults.maxContinuousSamples));
	}
	return out;
}

struct ReplayInput
{
	std::string name;
	std::string path;
};

std::vector<ReplayInput> ReplayInputs()
{
	std::vector<ReplayInput> out;
	if (const char* raw = std::getenv("WKOPENVR_REPLAY_PATHS")) {
		for (const auto& path : SplitReplayEnvList(raw)) {
			ReplayInput input;
			input.path = path;
			input.name = std::filesystem::path(path).filename().string();
			if (input.name.empty()) input.name = path;
			out.push_back(std::move(input));
		}
		return out;
	}

	for (const auto& file : replay::ListRecordings()) {
		ReplayInput input;
		input.name = file.name;
		input.path = openvr_pair::common::WideToUtf8(file.fullPath);
		out.push_back(std::move(input));
	}
	return out;
}

void AddFinite(std::vector<double>& values, double value)
{
	if (std::isfinite(value)) values.push_back(value);
}

double Percentile(std::vector<double> values, double p)
{
	if (values.empty()) return std::numeric_limits<double>::quiet_NaN();
	std::sort(values.begin(), values.end());
	const double pos = p * static_cast<double>(values.size() - 1);
	const std::size_t lo = static_cast<std::size_t>(std::floor(pos));
	const std::size_t hi = std::min<std::size_t>(lo + 1, values.size() - 1);
	const double frac = pos - static_cast<double>(lo);
	return values[lo] * (1.0 - frac) + values[hi] * frac;
}

double Percent(int count, int total)
{
	return total > 0 ? (100.0 * static_cast<double>(count) / static_cast<double>(total)) : 0.0;
}

struct QualityTraceSummary
{
	int reports = 0;
	int currentAccept = 0;
	int legacyPass = 0;
	int geometryPass = 0;
	int robustPass = 0;
	int holdoutPass = 0;
	int trackingPass = 0;
	int coreNoHoldoutAccept = 0;
	int softHoldout50Accept = 0;
	int softHoldout75Accept = 0;
	int novaRequiredAccept = 0;
	int lowResidualGeometryRescue = 0;
	std::map<std::string, int> reasons;
	std::vector<double> rmsMm;
	std::vector<double> p95Mm;
	std::vector<double> holdoutRmsMm;
	std::vector<double> holdoutP90Mm;
	std::vector<double> dynamicLimitMm;
	std::vector<double> targetSpanCm;
	std::vector<double> rotationSpanDeg;
	std::vector<double> validRotationPairs;
	std::vector<double> translationCondition;
};

QualityTraceSummary SummarizeQualityTrace(const replay::ReplayResult& result)
{
	QualityTraceSummary summary;
	for (const auto& q : result.qualityTrace) {
		++summary.reports;
		if (q.shadowWouldAccept) ++summary.currentAccept;
		if (q.legacyRmsPass) ++summary.legacyPass;
		if (q.geometryPass) ++summary.geometryPass;
		if (q.robustResidualPass) ++summary.robustPass;
		if (q.holdoutPass) ++summary.holdoutPass;
		if (q.trackingHealthPass) ++summary.trackingPass;
		if (!q.shadowWouldAccept) ++summary.reasons[q.shadowRejectReason];

		const bool coreNoHoldout = q.legacyRmsPass && q.geometryPass && q.robustResidualPass && q.trackingHealthPass;
		const bool softHoldout50 =
		    coreNoHoldout && (q.holdoutPass || (q.holdoutP90Mm <= 50.0 && q.holdoutRmsMm <= 100.0));
		const bool softHoldout75 =
		    coreNoHoldout && (q.holdoutPass || (q.holdoutP90Mm <= 75.0 && q.holdoutRmsMm <= 100.0));
		const bool lowResidualGeometryRescue = q.legacyRmsPass && !q.geometryPass && q.robustResidualPass &&
		                                       q.holdoutPass && q.trackingHealthPass && q.rmsMm <= 5.0 &&
		                                       q.p95Mm <= 10.0 && q.targetSpanM >= 0.05 && q.rotationSpanDeg >= 10.0;

		if (coreNoHoldout) ++summary.coreNoHoldoutAccept;
		if (softHoldout50) ++summary.softHoldout50Accept;
		if (softHoldout75) ++summary.softHoldout75Accept;
		if (q.shadowWouldAccept && q.novaDeltaPairsPass) ++summary.novaRequiredAccept;
		if (lowResidualGeometryRescue) ++summary.lowResidualGeometryRescue;

		AddFinite(summary.rmsMm, q.rmsMm);
		AddFinite(summary.p95Mm, q.p95Mm);
		AddFinite(summary.holdoutRmsMm, q.holdoutRmsMm);
		AddFinite(summary.holdoutP90Mm, q.holdoutP90Mm);
		AddFinite(summary.dynamicLimitMm, q.dynamicLimitMm);
		AddFinite(summary.targetSpanCm, q.targetSpanM * 100.0);
		AddFinite(summary.rotationSpanDeg, q.rotationSpanDeg);
		AddFinite(summary.validRotationPairs, static_cast<double>(q.validRotationPairCount));
		AddFinite(summary.translationCondition, q.translationConditionRatio);
	}
	return summary;
}

void PrintQualitySummary(const std::string& name, std::size_t window, const replay::ReplayResult& result)
{
	const auto summary = SummarizeQualityTrace(result);
	std::cout << "[replay-quality] " << name << " window=" << window << " reports=" << summary.reports
	          << " current=" << summary.currentAccept << "(" << Percent(summary.currentAccept, summary.reports) << "%)"
	          << " legacy=" << summary.legacyPass << "(" << Percent(summary.legacyPass, summary.reports) << "%)"
	          << " geometry=" << summary.geometryPass << "(" << Percent(summary.geometryPass, summary.reports) << "%)"
	          << " robust=" << summary.robustPass << "(" << Percent(summary.robustPass, summary.reports) << "%)"
	          << " holdout=" << summary.holdoutPass << "(" << Percent(summary.holdoutPass, summary.reports) << "%)"
	          << " tracking=" << summary.trackingPass << "(" << Percent(summary.trackingPass, summary.reports) << "%)"
	          << " core_no_holdout=" << summary.coreNoHoldoutAccept << "("
	          << Percent(summary.coreNoHoldoutAccept, summary.reports) << "%)"
	          << " soft_holdout50=" << summary.softHoldout50Accept << "("
	          << Percent(summary.softHoldout50Accept, summary.reports) << "%)"
	          << " soft_holdout75=" << summary.softHoldout75Accept << "("
	          << Percent(summary.softHoldout75Accept, summary.reports) << "%)"
	          << " nova_required=" << summary.novaRequiredAccept << "("
	          << Percent(summary.novaRequiredAccept, summary.reports) << "%)"
	          << " low_residual_geometry_rescue=" << summary.lowResidualGeometryRescue << "("
	          << Percent(summary.lowResidualGeometryRescue, summary.reports) << "%)"
	          << "\n";

	std::cout << "[replay-quantiles] " << name << " window=" << window << " rms_p50=" << Percentile(summary.rmsMm, 0.50)
	          << " rms_p95=" << Percentile(summary.rmsMm, 0.95) << " p95_p50=" << Percentile(summary.p95Mm, 0.50)
	          << " p95_p95=" << Percentile(summary.p95Mm, 0.95)
	          << " holdout_rms_p50=" << Percentile(summary.holdoutRmsMm, 0.50)
	          << " holdout_p90_p50=" << Percentile(summary.holdoutP90Mm, 0.50)
	          << " dynamic_limit_p50=" << Percentile(summary.dynamicLimitMm, 0.50)
	          << " target_span_cm_p50=" << Percentile(summary.targetSpanCm, 0.50)
	          << " rot_span_deg_p50=" << Percentile(summary.rotationSpanDeg, 0.50)
	          << " rot_pairs_p50=" << Percentile(summary.validRotationPairs, 0.50)
	          << " trans_cond_p50=" << Percentile(summary.translationCondition, 0.50) << "\n";

	std::cout << "[replay-reasons] " << name << " window=" << window;
	for (const auto& reason : summary.reasons) {
		std::cout << " " << reason.first << "=" << reason.second;
	}
	std::cout << "\n";
}

// Write a synthetic v4 recording for the locked-snap A/B tests. Models a locked
// continuous session: the head-mount tracker (lighthouse-anchored) never moves,
// while at `flipRow` the HMD's reported world pose teleports `flipDistanceM` --
// the Quest universe-flip signature the snap classifier keys on. Only the
// columns the replay parser reads are emitted (it's column-name based). Pass
// flipRow < 0 for a clean session with no flip.
void WriteLockedSnapV4Recording(const std::filesystem::path& path, int rowCount, int flipRow, double flipDistanceM)
{
	std::ofstream out(path);
	ASSERT_TRUE(out) << path.string();
	out << "# spacecal_log_v4\n";
	out << "Timestamp,ref_tx,ref_ty,ref_tz,ref_qw,ref_qx,ref_qy,ref_qz,"
	       "tgt_tx,tgt_ty,tgt_tz,tgt_qw,tgt_qx,tgt_qy,tgt_qz,tick_phase,"
	       "hmd_tx,hmd_ty,hmd_tz,hmd_qw,hmd_qx,hmd_qy,hmd_qz,"
	       "head_tracker_valid,head_tracker_tx,head_tracker_ty,head_tracker_tz,"
	       "head_tracker_qw,head_tracker_qx,head_tracker_qy,head_tracker_qz,reloc_detected\n";
	out.precision(17);
	for (int i = 0; i < rowCount; ++i) {
		const double t = static_cast<double>(i) / 60.0;
		// Reference + target wander a little so the rows resemble a live session.
		const double refX = 0.01 * (i % 5);
		const double tgtX = refX + 0.15;
		// HMD stationary until the flip, then teleported in X and held there.
		const double hmdX = (flipRow >= 0 && i >= flipRow) ? flipDistanceM : 0.0;
		// Head-mount tracker: lighthouse-tracked physical head never moves.
		const int reloc = (flipRow >= 0 && i == flipRow) ? 1 : 0;
		out << t << "," << refX << ",1.60,0.0,1,0,0,0," << tgtX << ",1.60,0.0,1,0,0,0,Continuous," << hmdX
		    << ",1.60,0.0,1,0,0,0,1,0.0,1.60,-0.1,1,0,0,0," << reloc << "\n";
	}
}

// Write a minimal v3 recording (raw poses + tick_phase, no v4 locked-snap
// columns) so the locked-snap path reports the requires_v4 skip.
void WriteMinimalV3Recording(const std::filesystem::path& path, int rowCount)
{
	std::ofstream out(path);
	ASSERT_TRUE(out) << path.string();
	out << "# spacecal_log_v3\n";
	out << "Timestamp,ref_tx,ref_ty,ref_tz,ref_qw,ref_qx,ref_qy,ref_qz,"
	       "tgt_tx,tgt_ty,tgt_tz,tgt_qw,tgt_qx,tgt_qy,tgt_qz,tick_phase\n";
	out.precision(17);
	for (int i = 0; i < rowCount; ++i) {
		const double t = static_cast<double>(i) / 60.0;
		const double refX = 0.01 * (i % 5);
		out << t << "," << refX << ",1.60,0.0,1,0,0,0," << (refX + 0.15) << ",1.60,0.0,1,0,0,0,Continuous\n";
	}
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

TEST(MotionRecordingReplayTest, LockedSnapReanchorsOnV4UniverseFlip)
{
	const std::filesystem::path path = std::filesystem::temp_directory_path() / "wkopenvr_replay_v4_locked_snap.csv";
	std::filesystem::remove(path);
	WriteLockedSnapV4Recording(path, /*rowCount=*/40, /*flipRow=*/20, /*flipDistanceM=*/0.5);

	const auto recording = replay::LoadRecording(path.string());
	std::filesystem::remove(path);
	ASSERT_TRUE(recording.error.empty()) << recording.error;
	ASSERT_EQ(4, recording.formatVersion);
	ASSERT_TRUE(recording.hasLockedSnapColumns);
	ASSERT_EQ(40u, recording.rows.size());
	// The flip row carries the raw HMD teleport + the (unmoved) head tracker.
	EXPECT_TRUE(recording.rows[20].hasHmdPose);
	EXPECT_TRUE(recording.rows[20].headTrackerValid);
	EXPECT_TRUE(recording.rows[20].relocDetected);
	EXPECT_NEAR(0.5, recording.rows[20].hmd.trans.x(), 1e-9);
	EXPECT_NEAR(0.0, recording.rows[19].hmd.trans.x(), 1e-9);

	replay::ReplayOptions base;
	base.continuous = false;
	base.qualityReportInterval = 0;

	// Toggle ON in a locked style: the corroborated universe flip opens exactly
	// one gentle re-anchor.
	{
		replay::ReplayOptions options = base;
		options.applyLockedSnap = true;
		options.trackingStyle = TrackingStyle::LockedWithRecovery;
		const auto result = replay::RunReplay(recording, options);
		EXPECT_TRUE(result.succeeded) << result.error;
		EXPECT_EQ("applied", result.lockedSnapStatus);
		EXPECT_EQ(1, result.snapReanchors);
	}

	// Toggle OFF: byte-identical legacy replay, no re-anchors counted.
	{
		replay::ReplayOptions options = base;
		options.applyLockedSnap = false;
		const auto result = replay::RunReplay(recording, options);
		EXPECT_TRUE(result.succeeded) << result.error;
		EXPECT_EQ("off", result.lockedSnapStatus);
		EXPECT_EQ(0, result.snapReanchors);
	}

	// Toggle ON but a non-locked style: the gentle path is locked-only, so the
	// snap is classified (v4 columns present -> "applied") yet never re-anchors.
	{
		replay::ReplayOptions options = base;
		options.applyLockedSnap = true;
		options.trackingStyle = TrackingStyle::Continuous;
		const auto result = replay::RunReplay(recording, options);
		EXPECT_TRUE(result.succeeded) << result.error;
		EXPECT_EQ("applied", result.lockedSnapStatus);
		EXPECT_EQ(0, result.snapReanchors);
	}
}

TEST(MotionRecordingReplayTest, LockedSnapNoReanchorWithoutFlip)
{
	const std::filesystem::path path =
	    std::filesystem::temp_directory_path() / "wkopenvr_replay_v4_locked_snap_clean.csv";
	std::filesystem::remove(path);
	WriteLockedSnapV4Recording(path, /*rowCount=*/40, /*flipRow=*/-1, /*flipDistanceM=*/0.0);

	const auto recording = replay::LoadRecording(path.string());
	std::filesystem::remove(path);
	ASSERT_TRUE(recording.error.empty()) << recording.error;
	ASSERT_TRUE(recording.hasLockedSnapColumns);

	replay::ReplayOptions options;
	options.continuous = false;
	options.qualityReportInterval = 0;
	options.applyLockedSnap = true;
	options.trackingStyle = TrackingStyle::LockedWithRecovery;
	const auto result = replay::RunReplay(recording, options);
	EXPECT_TRUE(result.succeeded) << result.error;
	EXPECT_EQ("applied", result.lockedSnapStatus);
	EXPECT_EQ(0, result.snapReanchors);
}

TEST(MotionRecordingReplayTest, LockedSnapRequiresV4OnV3Recording)
{
	const std::filesystem::path path =
	    std::filesystem::temp_directory_path() / "wkopenvr_replay_v3_locked_snap_skip.csv";
	std::filesystem::remove(path);
	WriteMinimalV3Recording(path, /*rowCount=*/40);

	const auto recording = replay::LoadRecording(path.string());
	std::filesystem::remove(path);
	ASSERT_TRUE(recording.error.empty()) << recording.error;
	ASSERT_EQ(3, recording.formatVersion);
	ASSERT_FALSE(recording.hasLockedSnapColumns);

	replay::ReplayOptions options;
	options.continuous = false;
	options.qualityReportInterval = 0;
	options.applyLockedSnap = true;
	options.trackingStyle = TrackingStyle::LockedWithRecovery;
	const auto result = replay::RunReplay(recording, options);
	EXPECT_TRUE(result.succeeded) << result.error;
	EXPECT_EQ("locked_snap_replay_requires_v4", result.lockedSnapStatus);
	EXPECT_EQ(0, result.snapReanchors);
}

TEST(MotionRecordingReplayTest, ReplayLocalRecordingsWhenRequested)
{
	const char* enabled = std::getenv("WKOPENVR_REPLAY_RECORDINGS");
	if (!enabled || std::string(enabled) != "1") {
		GTEST_SKIP() << "Set WKOPENVR_REPLAY_RECORDINGS=1 to replay retained local recordings.";
	}

	const auto inputs = ReplayInputs();
	if (inputs.empty()) {
		GTEST_SKIP() << "No retained spacecal_log recordings found.";
	}

	replay::ReplayOptions baseOptions;
	baseOptions.continuous = true;
	baseOptions.qualityReportInterval = EnvSize("WKOPENVR_REPLAY_QUALITY_INTERVAL", baseOptions.qualityReportInterval);
	baseOptions.includeHoldoutQuality = EnvFlag("WKOPENVR_REPLAY_HOLDOUT", baseOptions.includeHoldoutQuality);
	// Experimental drift-fighting guards (all default off -> legacy replay).
	baseOptions.applyRelocQuarantine = EnvFlag("WKOPENVR_REPLAY_QUARANTINE", baseOptions.applyRelocQuarantine);
	baseOptions.quarantineSec = EnvDouble("WKOPENVR_REPLAY_QUARANTINE_SEC", baseOptions.quarantineSec);
	baseOptions.applyDriftBreaker = EnvFlag("WKOPENVR_REPLAY_DRIFT_BREAKER", baseOptions.applyDriftBreaker);
	baseOptions.driftBreakerMadMult = EnvDouble("WKOPENVR_REPLAY_DRIFT_BREAKER_MULT", baseOptions.driftBreakerMadMult);
	baseOptions.driftBreakerAbsCapMm =
	    EnvDouble("WKOPENVR_REPLAY_DRIFT_BREAKER_CAP_MM", baseOptions.driftBreakerAbsCapMm);
	baseOptions.applyBoundedSolve = EnvFlag("WKOPENVR_REPLAY_BOUNDED_SOLVE", baseOptions.applyBoundedSolve);
	baseOptions.bsPrior = EnvFlag("WKOPENVR_REPLAY_BOUNDED_SOLVE_PRIOR", baseOptions.bsPrior);
	baseOptions.bsPriorLambda = EnvDouble("WKOPENVR_REPLAY_BOUNDED_SOLVE_LAMBDA", baseOptions.bsPriorLambda);
	baseOptions.bsSlew = EnvFlag("WKOPENVR_REPLAY_BOUNDED_SOLVE_SLEW", baseOptions.bsSlew);
	baseOptions.bsMaxStepMm = EnvDouble("WKOPENVR_REPLAY_BOUNDED_SOLVE_STEP_MM", baseOptions.bsMaxStepMm);
	baseOptions.bsCommonMode = EnvFlag("WKOPENVR_REPLAY_BOUNDED_SOLVE_COMMONMODE", baseOptions.bsCommonMode);
	baseOptions.relocProxyJumpM = EnvDouble("WKOPENVR_REPLAY_RELOC_PROXY_M", baseOptions.relocProxyJumpM);
	// Toggle 4 (locked-style snap recovery). Needs a v4 recording; on v3 input the
	// replay reports locked_snap=locked_snap_replay_requires_v4 and counts zero.
	baseOptions.applyLockedSnap = EnvFlag("WKOPENVR_REPLAY_LOCKED_SNAP", baseOptions.applyLockedSnap);
	baseOptions.trackingStyle = EnvTrackingStyle("WKOPENVR_REPLAY_TRACKING_STYLE", baseOptions.trackingStyle);
	const auto sampleWindows = ReplaySampleWindows();

	std::size_t replayed = 0;
	std::size_t skippedEmpty = 0;
	for (const auto& input : inputs) {
		SCOPED_TRACE(input.name);
		const auto recording = replay::LoadRecording(input.path);
		ASSERT_TRUE(recording.error.empty()) << recording.error;
		if (recording.rows.empty()) {
			++skippedEmpty;
			std::cout << "[replay] " << input.name << " skipped=no_replayable_rows\n";
			continue;
		}

		for (std::size_t sampleWindow : sampleWindows) {
			replay::ReplayOptions options = baseOptions;
			options.maxContinuousSamples = sampleWindow;

			const auto result = replay::RunReplay(recording, options);
			EXPECT_TRUE(result.succeeded) << result.error;
			EXPECT_GT(result.rowsReplayed, 0);
			std::cout << "[replay] " << input.name << " format=v" << recording.formatVersion
			          << " window=" << sampleWindow << " holdout=" << (options.includeHoldoutQuality ? 1 : 0)
			          << " quality_interval=" << options.qualityReportInterval << " rows=" << result.rowsReplayed
			          << " accepts=" << result.accepts << " rejects=" << result.rejects
			          << " final_error_mm=" << result.finalErrorMm
			          << " sample_rows_accepted=" << result.sampleRowsAccepted
			          << " sample_rows_rejected=" << result.sampleRowsRejected
			          << " strict_unhealthy=" << result.sampleRowsStrictUnhealthy << " stale=" << result.sampleRowsStale
			          << " unchanged=" << result.sampleRowsUnchanged << " high_motion=" << result.sampleRowsHighMotion
			          << " quality_reports=" << result.qualityReports << " shadow_accepts=" << result.shadowWouldAccept
			          << " shadow_rejects=" << result.shadowWouldReject
			          << " peak_relpose_mad_mm=" << result.peakRelPoseMadMm
			          << " median_relpose_mad_mm=" << result.medianRelPoseMadMm
			          << " final_relpose_mad_mm=" << result.finalRelPoseMadMm
			          << " samples_quarantined=" << result.samplesQuarantined
			          << " freeze_engagements=" << result.freezeEngagements
			          << " snap_reanchors=" << result.snapReanchors << " locked_snap=" << result.lockedSnapStatus
			          << " tracking_style=" << static_cast<int>(options.trackingStyle)
			          << " final_shadow_reason=" << result.finalQuality.shadowRejectReason << "\n";
			PrintQualitySummary(input.name, sampleWindow, result);
			++replayed;
		}
	}

	EXPECT_GT(replayed, 0u) << "No retained recordings contained replayable rows; skipped " << skippedEmpty
	                        << " empty recordings.";
}
