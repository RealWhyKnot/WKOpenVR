#include <gtest/gtest.h>

#include "MotionRecording.h"
#include "WitnessDriftReplay.h"
#include "CalibrationExperimentFlags.h"
#include "Win32Text.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstdio>
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

// WKOPENVR_REPLAY_SEED_PROFILE: "recorded" | "none" | explicit "x,y,z[;rz,ry,rx]"
// (translation cm; ZYX euler degrees, same triple the live profile stores).
// Malformed explicit values leave seeding off so a typo can't silently change
// the baseline being measured.
void ApplySeedEnv(replay::ReplayOptions& options, std::string& seedName)
{
	seedName = "none";
	const char* raw = std::getenv("WKOPENVR_REPLAY_SEED_PROFILE");
	if (!raw) return;
	const std::string value = TrimAscii(raw);
	std::string lower;
	for (char c : value) {
		lower.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
	}
	if (lower.empty() || lower == "none") return;
	if (lower == "recorded") {
		options.seedMode = replay::ReplaySeedMode::Recorded;
		seedName = "recorded";
		return;
	}
	const auto parts = SplitReplayEnvList(value);
	double t[3] = {0.0, 0.0, 0.0};
	double r[3] = {0.0, 0.0, 0.0};
	if (parts.empty() || std::sscanf(parts[0].c_str(), "%lf,%lf,%lf", &t[0], &t[1], &t[2]) != 3) {
		std::cout << "[replay] ignoring malformed WKOPENVR_REPLAY_SEED_PROFILE='" << value << "'\n";
		return;
	}
	if (parts.size() > 1) {
		std::sscanf(parts[1].c_str(), "%lf,%lf,%lf", &r[0], &r[1], &r[2]);
	}
	options.seedMode = replay::ReplaySeedMode::Explicit;
	options.seedTransCm = Eigen::Vector3d(t[0], t[1], t[2]);
	options.seedRotDeg = Eigen::Vector3d(r[0], r[1], r[2]);
	seedName = "explicit";
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

std::string EncodeReplayReasonCounts(const std::vector<replay::ReplayReasonCount>& reasons)
{
	std::string out;
	for (const auto& reason : reasons) {
		if (!out.empty()) out += ";";
		out += reason.reason.empty() ? "unknown" : reason.reason;
		out += ":";
		out += std::to_string(reason.count);
	}
	return out.empty() ? "none" : out;
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

// v5 recording with the annotation stream the live logger writes: a seed
// profile at start, an auto-lock flip mid-file, and a second (mid-session)
// re-seed that the parser must NOT prefer over the first.
void WriteSeedAnnotatedV5Recording(const std::filesystem::path& path, bool seedValid)
{
	std::ofstream out(path);
	ASSERT_TRUE(out) << path.string();
	out << "# spacecal_log_v5\n";
	if (seedValid) {
		out << "# [0.50] StartContinuousCalibration_seed_profile: trans_cm=(23.74,266.35,282.30) mag_cm=388.85 "
		       "rot_deg=(1.500,-0.250,0.125)\n";
	}
	else {
		out << "# [0.50] StartContinuousCalibration_seed_profile: skipped validProfile=0\n";
	}
	out << "Timestamp,ref_tx,ref_ty,ref_tz,ref_qw,ref_qx,ref_qy,ref_qz,"
	       "tgt_tx,tgt_ty,tgt_tz,tgt_qw,tgt_qx,tgt_qy,tgt_qz,tick_phase\n";
	out.precision(17);
	for (int i = 0; i < 8; ++i) {
		const double t = static_cast<double>(i) / 60.0;
		const double refX = 0.01 * static_cast<double>(i);
		out << t << "," << refX << ",1.60,0.0,1,0,0,0," << (refX + 0.15) << ",1.60,0.0,1,0,0,0,Continuous\n";
		if (i == 3) {
			out << "# [0.06] auto_lock_flip: previous=0 now=1 hmdSpeed=0.003mps held_sec=0.00 "
			       "committed_via=stationary_gate\n";
		}
	}
	out << "# [0.20] StartContinuousCalibration_seed_profile: trans_cm=(99.00,0.00,0.00) mag_cm=99.00 "
	       "rot_deg=(0.000,0.000,0.000)\n";
}

// Programmatic recording whose rows all agree on calibration `cTrue` through
// the relpose identity trick (target = cTrue^-1 * ref => the per-sample
// estimate R * I * T^-1 is exactly cTrue). `originDistanceM` + `heightM` set
// the lever arm: 3 m at head height models the head-mount rig far from
// origin. Note even 0 m at head height (y~1.6) carries a ~5 m^2 squared
// lever, which already throttles the fusion gain -- a truly near-origin
// stream needs a small height too.
replay::LoadedRecording MakeRelPoseRecording(const Eigen::AffineCompact3d& cTrue, double originDistanceM,
                                             double heightM, int rowCount)
{
	replay::LoadedRecording rec;
	rec.formatVersion = 5;
	rec.rows.reserve(rowCount);
	for (int i = 0; i < rowCount; ++i) {
		const double yaw = 0.03 * static_cast<double>(i % 40);
		const Eigen::Vector3d trans(originDistanceM + 0.05 * std::sin(0.7 * i), heightM + 0.05 * std::cos(0.5 * i),
		                            0.05 * std::sin(0.3 * i));
		Eigen::AffineCompact3d ref(Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitY())));
		ref.pretranslate(trans);
		const Eigen::AffineCompact3d target = cTrue.inverse() * ref;

		replay::ReplayRow row;
		row.timestamp = static_cast<double>(i) / 90.0;
		row.ref.rot = ref.rotation();
		row.ref.trans = ref.translation();
		row.target.rot = target.rotation();
		row.target.trans = target.translation();
		row.sample = Sample(row.ref, row.target, row.timestamp);
		// Stationary HMD so applied steps count as perceptible shifts.
		row.hasHmdPose = true;
		row.hmd.rot.setIdentity();
		row.hmd.trans = Eigen::Vector3d(0.0, 1.60, 0.0);
		rec.rows.push_back(std::move(row));
	}
	rec.hasLockedSnapColumns = true;
	return rec;
}

void WriteReplayTraceCsv(const std::filesystem::path& path, const replay::ReplayResult& result)
{
	std::ofstream out(path);
	if (!out) {
		std::cout << "[replay-trace] open_failed path=" << path.string() << "\n";
		return;
	}
	out << "timestamp,accepted,reject_reason,current_cal_err_mm,raw_err_mm,applied_cx_cm,applied_cy_cm,applied_cz_cm,"
	       "applied_mag_cm,has_applied,fusion_gain,accum_precision,rel_mad_mm,hmd_stationary,reloc_detected\n";
	out.precision(12);
	for (const auto& t : result.trace) {
		out << t.timestamp << ',' << (t.accepted ? 1 : 0) << ',' << t.rejectReason << ',' << t.currentCalErrMm << ','
		    << t.rawErrMm << ',' << t.appliedCCm.x() << ',' << t.appliedCCm.y() << ',' << t.appliedCCm.z() << ','
		    << t.appliedCCm.norm() << ',' << (t.hasAppliedC ? 1 : 0) << ',' << t.fusionGain << ',' << t.accumPrecision
		    << ',' << t.relMadMm << ',' << (t.hmdStationary ? 1 : 0) << ',' << (t.relocDetected ? 1 : 0) << "\n";
	}
	std::cout << "[replay-trace] rows=" << result.trace.size() << " path=" << path.string() << "\n";
}

void WriteExperimentalFlagsV5Recording(const std::filesystem::path& path)
{
	std::ofstream out(path);
	ASSERT_TRUE(out) << path.string();
	out << "# spacecal_log_v5\n";
	out << "Timestamp,ref_tx,ref_ty,ref_tz,ref_qw,ref_qx,ref_qy,ref_qz,"
	       "tgt_tx,tgt_ty,tgt_tz,tgt_qw,tgt_qx,tgt_qy,tgt_qz,tick_phase,"
	       "hmd_tx,hmd_ty,hmd_tz,hmd_qw,hmd_qx,hmd_qy,hmd_qz,"
	       "head_tracker_valid,head_tracker_tx,head_tracker_ty,head_tracker_tz,"
	       "head_tracker_qw,head_tracker_qx,head_tracker_qy,head_tracker_qz,reloc_detected,"
	       "experimental_flags\n";
	const uint32_t flags = spacecal::calibration_experiments::HeadsetOffsetAutoCorrect;
	out.precision(17);
	for (int i = 0; i < 8; ++i) {
		const double t = static_cast<double>(i) / 60.0;
		const double refX = 0.01 * static_cast<double>(i);
		out << t << "," << refX << ",1.60,0.0,1,0,0,0," << (refX + 0.15)
		    << ",1.60,0.0,1,0,0,0,Continuous,0,1.60,0.0,1,0,0,0,0,0,0,0,1,0,0,0,0," << flags << "\n";
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

TEST(MotionRecordingReplayTest, LoadsV5ExperimentalFlagsAndKeepsV4Compatible)
{
	const std::filesystem::path v5Path = std::filesystem::temp_directory_path() / "wkopenvr_replay_v5_flags.csv";
	std::filesystem::remove(v5Path);
	WriteExperimentalFlagsV5Recording(v5Path);

	const auto v5 = replay::LoadRecording(v5Path.string());
	std::filesystem::remove(v5Path);
	ASSERT_TRUE(v5.error.empty()) << v5.error;
	ASSERT_EQ(5, v5.formatVersion);
	ASSERT_TRUE(v5.hasExperimentalFlagsColumn);
	ASSERT_FALSE(v5.rows.empty());
	EXPECT_TRUE(v5.rows.front().hasExperimentalFlags);
	EXPECT_NE(0u, v5.rows.front().experimentalFlags & spacecal::calibration_experiments::HeadsetOffsetAutoCorrect);

	const std::filesystem::path v4Path = std::filesystem::temp_directory_path() / "wkopenvr_replay_v4_flags_compat.csv";
	std::filesystem::remove(v4Path);
	WriteLockedSnapV4Recording(v4Path, /*rowCount=*/8, /*flipRow=*/-1, /*flipDistanceM=*/0.0);

	const auto v4 = replay::LoadRecording(v4Path.string());
	std::filesystem::remove(v4Path);
	ASSERT_TRUE(v4.error.empty()) << v4.error;
	EXPECT_EQ(4, v4.formatVersion);
	EXPECT_FALSE(v4.hasExperimentalFlagsColumn);
	ASSERT_FALSE(v4.rows.empty());
	EXPECT_FALSE(v4.rows.front().hasExperimentalFlags);
	EXPECT_EQ(0u, v4.rows.front().experimentalFlags);
}

TEST(MotionRecordingReplayTest, FormatsExperimentalToggleAnnotations)
{
	const char* optionNames[] = {
	    "headset_offset_auto_correct",
	};
	for (const char* optionName : optionNames) {
		EXPECT_EQ(std::string(optionName) + "_toggled: source=ui enabled=1",
		          spacecal::calibration_experiments::ToggleAnnotation(optionName, true));
		EXPECT_EQ(std::string(optionName) + "_toggled: source=ui enabled=0",
		          spacecal::calibration_experiments::ToggleAnnotation(optionName, false));
	}
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
	options.includeHoldoutQuality = true;
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
	EXPECT_GE(result.finalQuality.holdoutRmsMm, 0.0);
	EXPECT_GE(result.finalQuality.holdoutP90Mm, 0.0);
	EXPECT_GE(result.finalQuality.holdoutP95Mm, 0.0);
}

TEST(MotionRecordingReplayTest, LoadsSeedProfileAnnotationAndEvents)
{
	const std::filesystem::path path = std::filesystem::temp_directory_path() / "wkopenvr_replay_seed_annotated.csv";
	std::filesystem::remove(path);
	WriteSeedAnnotatedV5Recording(path, /*seedValid=*/true);

	const auto rec = replay::LoadRecording(path.string());
	std::filesystem::remove(path);
	ASSERT_TRUE(rec.error.empty()) << rec.error;

	// First seed annotation wins; the mid-session re-seed (99,0,0) must not
	// replace the session-start profile.
	ASSERT_TRUE(rec.seedProfile.present);
	ASSERT_TRUE(rec.seedProfile.valid);
	EXPECT_NEAR(rec.seedProfile.time, 0.50, 1e-9);
	EXPECT_NEAR(rec.seedProfile.transCm.x(), 23.74, 1e-6);
	EXPECT_NEAR(rec.seedProfile.transCm.y(), 266.35, 1e-6);
	EXPECT_NEAR(rec.seedProfile.transCm.z(), 282.30, 1e-6);
	EXPECT_NEAR(rec.seedProfile.rotDeg.x(), 1.500, 1e-6);
	EXPECT_NEAR(rec.seedProfile.rotDeg.y(), -0.250, 1e-6);
	EXPECT_NEAR(rec.seedProfile.rotDeg.z(), 0.125, 1e-6);

	int seedEvents = 0;
	int flipEvents = 0;
	for (const auto& event : rec.annotations) {
		if (event.key == "StartContinuousCalibration_seed_profile") ++seedEvents;
		if (event.key == "auto_lock_flip") ++flipEvents;
	}
	EXPECT_EQ(2, seedEvents);
	EXPECT_EQ(1, flipEvents);

	// Recorded-mode replay warm-starts from the parsed seed.
	replay::ReplayOptions options;
	options.seedMode = replay::ReplaySeedMode::Recorded;
	const auto result = replay::RunReplay(rec, options);
	EXPECT_TRUE(result.succeeded) << result.error;
	EXPECT_TRUE(result.seedApplied);
	EXPECT_NEAR(result.seedMagCm, 388.85, 0.05);
}

TEST(MotionRecordingReplayTest, SeedProfileSkippedFormDegradesToColdStart)
{
	const std::filesystem::path path = std::filesystem::temp_directory_path() / "wkopenvr_replay_seed_skipped.csv";
	std::filesystem::remove(path);
	WriteSeedAnnotatedV5Recording(path, /*seedValid=*/false);

	const auto rec = replay::LoadRecording(path.string());
	std::filesystem::remove(path);
	ASSERT_TRUE(rec.error.empty()) << rec.error;
	EXPECT_TRUE(rec.seedProfile.present);
	EXPECT_FALSE(rec.seedProfile.valid);

	replay::ReplayOptions options;
	options.seedMode = replay::ReplaySeedMode::Recorded;
	const auto result = replay::RunReplay(rec, options);
	EXPECT_TRUE(result.succeeded) << result.error;
	EXPECT_FALSE(result.seedApplied);
}

// Offline reproduction of the seeded-fusion behavior seen live: with the
// confidence fusion on, a stored profile seed (kSeedPriorPrecision) dominates
// far-from-origin candidates, so a BAD seed is defended instead of corrected
// ("refusing to calibrate"); with fusion off the overwrite path escapes the
// seed on the first accept. A genuinely near-origin stream carries enough
// per-solve precision to correct the same bad seed through the fusion.
TEST(MotionRecordingReplayTest, SeededReplayFusionDefendsBadSeedFarFromOrigin)
{
	Eigen::AffineCompact3d cTrue = Eigen::AffineCompact3d::Identity();
	cTrue.translation() = Eigen::Vector3d(0.20, 0.0, 0.0); // truth: 20 cm
	const Eigen::Vector3d seedTransCm(50.0, 0.0, 0.0);     // stored profile: 30 cm wrong

	replay::ReplayOptions options;
	options.lockRelativePosition = true;
	options.maxContinuousSamples = 25;
	options.qualityReportInterval = 0;
	options.seedMode = replay::ReplaySeedMode::Explicit;
	options.seedTransCm = seedTransCm;

	const auto farRec = MakeRelPoseRecording(cTrue, /*originDistanceM=*/3.0, /*heightM=*/1.60, /*rowCount=*/300);

	// Fusion ON, far from origin: candidates carry almost no geometric
	// precision, so the applied calibration stays pinned to the bad seed.
	options.precisionWeightedRelPose = true;
	const auto farFused = replay::RunReplay(farRec, options);
	ASSERT_TRUE(farFused.succeeded) << farFused.error;
	ASSERT_TRUE(farFused.seedApplied);
	EXPECT_GT(farFused.accepts, 10);
	EXPECT_LT(farFused.appliedMagWanderCm, 5.0)
	    << "fusion should defend the seed far from origin (the live bad-seed regression)";

	// Fusion OFF, same recording: the overwrite path jumps to the solved truth
	// (50 cm seed -> 20 cm truth = one 30 cm applied step). The HMD is
	// stationary in this synthetic session, so that jump is a perceptible
	// shift, and the net drift vector points from seed to truth.
	options.precisionWeightedRelPose = false;
	const auto farUniform = replay::RunReplay(farRec, options);
	ASSERT_TRUE(farUniform.succeeded) << farUniform.error;
	ASSERT_TRUE(farUniform.seedApplied);
	EXPECT_GT(farUniform.accepts, 10);
	EXPECT_GT(farUniform.appliedMagWanderCm, 25.0);
	EXPECT_GT(farUniform.peakAppliedStepCm, 25.0);
	EXPECT_GE(farUniform.largeAppliedSteps, 1);
	EXPECT_GE(farUniform.perceptibleShiftCount, 1);
	EXPECT_GT(farUniform.perceptibleShiftMaxMm, 250.0);
	EXPECT_NEAR(farUniform.netDriftVectorCm.x(), -30.0, 2.0);
	EXPECT_NEAR(farUniform.netDriftVectorCm.y(), 0.0, 2.0);
	EXPECT_NEAR(farUniform.netDriftVectorCm.z(), 0.0, 2.0);

	// Fusion ON, genuinely near origin (small height too): per-solve precision
	// is high, so the same bad seed is corrected within the session. At head
	// height the lever arm already throttles the gain -- that case behaves
	// like `farFused`, which is exactly the live "refusing to calibrate"
	// regression shape.
	options.precisionWeightedRelPose = true;
	const auto nearRec = MakeRelPoseRecording(cTrue, /*originDistanceM=*/0.0, /*heightM=*/0.10, /*rowCount=*/900);
	const auto nearFused = replay::RunReplay(nearRec, options);
	ASSERT_TRUE(nearFused.succeeded) << nearFused.error;
	ASSERT_TRUE(nearFused.seedApplied);
	EXPECT_GT(nearFused.appliedMagWanderCm, 25.0)
	    << "near-origin precision should pull the applied calibration off the bad seed";
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
	// Far-from-origin fix A/B: LOCK_REL exercises the CalibrateByRelPose branch the
	// head-mount rig uses; PRECISION_WEIGHT toggles the geometry-weighted solve.
	baseOptions.lockRelativePosition = EnvFlag("WKOPENVR_REPLAY_LOCK_REL", baseOptions.lockRelativePosition);
	baseOptions.precisionWeightedRelPose =
	    EnvFlag("WKOPENVR_REPLAY_PRECISION_WEIGHT", baseOptions.precisionWeightedRelPose);
	std::string seedName;
	ApplySeedEnv(baseOptions, seedName);
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
			          << " shadow_reject_reasons=" << EncodeReplayReasonCounts(result.shadowRejectReasons)
			          << " holdout_rms_mm=" << result.finalQuality.holdoutRmsMm
			          << " holdout_p90_mm=" << result.finalQuality.holdoutP90Mm
			          << " holdout_p95_mm=" << result.finalQuality.holdoutP95Mm
			          << " holdout_pass=" << (result.finalQuality.holdoutPass ? 1 : 0)
			          << " peak_relpose_mad_mm=" << result.peakRelPoseMadMm
			          << " median_relpose_mad_mm=" << result.medianRelPoseMadMm
			          << " final_relpose_mad_mm=" << result.finalRelPoseMadMm
			          << " lock_rel=" << (options.lockRelativePosition ? 1 : 0)
			          << " precision_weight=" << (options.precisionWeightedRelPose ? 1 : 0) << " seed=" << seedName
			          << " seed_applied=" << (result.seedApplied ? 1 : 0) << " seed_mag_cm=" << result.seedMagCm
			          << " peak_applied_mag_cm=" << result.peakAppliedMagCm
			          << " applied_mag_wander_cm=" << result.appliedMagWanderCm
			          << " peak_applied_step_cm=" << result.peakAppliedStepCm
			          << " large_applied_steps=" << result.largeAppliedSteps
			          << " total_applied_path_cm=" << result.totalAppliedPathCm
			          << " perceptible_shifts=" << result.perceptibleShiftCount
			          << " perceptible_max_mm=" << result.perceptibleShiftMaxMm
			          << " perceptible_sum_mm=" << result.perceptibleShiftSumMm << " net_drift_cm=("
			          << result.netDriftVectorCm.x() << "," << result.netDriftVectorCm.y() << ","
			          << result.netDriftVectorCm.z() << ")"
			          << " net_drift_mag_cm=" << result.netDriftVectorCm.norm()
			          << " solver_sample_rows=" << result.solverSamplesPushed
			          << " solver_sample_ratio=" << result.solverSampleRatio
			          << " final_shadow_reason=" << result.finalQuality.shadowRejectReason << "\n";
			PrintQualitySummary(input.name, sampleWindow, result);
			if (const char* traceDir = std::getenv("WKOPENVR_REPLAY_TRACE_CSV")) {
				const std::filesystem::path dir(TrimAscii(traceDir));
				if (!dir.empty()) {
					std::error_code ec;
					std::filesystem::create_directories(dir, ec);
					WriteReplayTraceCsv(dir / (input.name + ".w" + std::to_string(sampleWindow) + ".trace.csv"),
					                    result);
				}
			}
			++replayed;
		}

		// Witness-drift oracle: offset solve + continuous-correction model on the
		// recorded raw HMD + witness poses (sample-window independent, so once per
		// input). WKOPENVR_REPLAY_CORRECTION=0 measures the uncorrected baseline.
		replay::WitnessDriftOptions driftOpts;
		driftOpts.applyContinuousCorrection = EnvFlag("WKOPENVR_REPLAY_CORRECTION", true);
		driftOpts.correctionSlewMps = EnvDouble("WKOPENVR_REPLAY_SLEW_MPS", driftOpts.correctionSlewMps);
		driftOpts.correctionDeadbandM =
		    EnvDouble("WKOPENVR_REPLAY_DEADBAND_MM", driftOpts.correctionDeadbandM * 1000.0) / 1000.0;
		const auto drift = replay::ComputeWitnessDrift(recording, driftOpts);
		std::cout << "[witness-drift] " << input.name << " correction=" << (driftOpts.applyContinuousCorrection ? 1 : 0)
		          << " slew_mps=" << driftOpts.correctionSlewMps
		          << " deadband_mm=" << (driftOpts.correctionDeadbandM * 1000.0)
		          << " calibrated=" << (drift.calibrated ? 1 : 0) << " note=" << drift.note
		          << " baseline_offset_mm=" << drift.baselineOffsetMm << " baseline_samples=" << drift.baselineSamples
		          << " drift_samples=" << drift.driftSamples << " uncorrected_rms_mm=" << drift.uncorrectedRmsMm
		          << " corrected_rms_mm=" << drift.correctedRmsMm << " reduction_pct=" << drift.reductionPct
		          << " subcap_samples=" << drift.subCapSamples << " subcap_unc_rms_mm=" << drift.subCapUncorrectedRmsMm
		          << " subcap_cor_rms_mm=" << drift.subCapCorrectedRmsMm
		          << " subcap_reduction_pct=" << drift.subCapReductionPct
		          << " uncorrected_p50_mm=" << drift.uncorrectedP50Mm << " corrected_p50_mm=" << drift.correctedP50Mm
		          << " uncorrected_p95_mm=" << drift.uncorrectedP95Mm << " corrected_p95_mm=" << drift.correctedP95Mm
		          << " uncorrected_peak_mm=" << drift.uncorrectedPeakMm
		          << " corrected_peak_mm=" << drift.correctedPeakMm << " reloc_total=" << drift.relocTotal
		          << " reloc_measured=" << drift.relocMeasured << " reloc_flip_like=" << drift.relocFlipLike
		          << " reloc_mean_drift_mm=" << drift.relocMeanDriftMm << "\n";
	}

	EXPECT_GT(replayed, 0u) << "No retained recordings contained replayable rows; skipped " << skippedEmpty
	                        << " empty recordings.";
}
