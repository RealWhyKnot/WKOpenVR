#include "MotionRecording.h"

#include "AutoLockHysteresis.h"        // spacecal::autolock::RobustTranslDeviation -- relative-pose MAD.
#include "ContinuousPrecisionFusion.h" // spacecal::precision -- confidence-weighted fusion (A/B).

#include <windows.h>
#include <shlobj_core.h>

#include <algorithm>
#include <deque>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <unordered_map>
#include <utility>

namespace spacecal::replay {

namespace {

// Strict split-on-comma. Mirrors the live CSV writer (no quoting, no embedded
// newlines). Identical to the parser in tools/replay/main.cpp; deliberately
// duplicated rather than shared so the in-app version can evolve without
// dragging the standalone CLI's link line along.
std::vector<std::string> SplitCsv(const std::string& line)
{
	std::vector<std::string> out;
	std::string cur;
	for (char c : line) {
		if (c == ',') {
			out.push_back(cur);
			cur.clear();
		}
		else {
			cur.push_back(c);
		}
	}
	out.push_back(cur);
	return out;
}

void RTrim(std::string& s)
{
	while (!s.empty() && (s.back() == '\r' || s.back() == '\n' || s.back() == ' ' || s.back() == '\t')) {
		s.pop_back();
	}
}

bool ReadDouble(const std::string& s, double& out)
{
	if (s.empty()) return false;
	char* end = nullptr;
	const double v = std::strtod(s.c_str(), &end);
	if (end == s.c_str()) return false;
	out = v;
	return true;
}

bool ReadInt(const std::string& s, int& out)
{
	if (s.empty()) return false;
	char* end = nullptr;
	const long v = std::strtol(s.c_str(), &end, 10);
	if (end == s.c_str()) return false;
	out = static_cast<int>(v);
	return true;
}

bool ReadBoolToken(const std::string& s, bool& out)
{
	int iv = 0;
	if (ReadInt(s, iv)) {
		out = iv != 0;
		return true;
	}
	if (s == "true" || s == "TRUE" || s == "True") {
		out = true;
		return true;
	}
	if (s == "false" || s == "FALSE" || s == "False") {
		out = false;
		return true;
	}
	return false;
}

int ColIndex(const std::unordered_map<std::string, int>& cols, const char* name)
{
	const auto it = cols.find(name);
	return (it == cols.end()) ? -1 : it->second;
}

bool ReadOptionalBool(const std::vector<std::string>& fields, int idx, bool fallback)
{
	if (idx < 0 || idx >= static_cast<int>(fields.size())) return fallback;
	bool value = fallback;
	return ReadBoolToken(fields[static_cast<size_t>(idx)], value) ? value : fallback;
}

int ReadOptionalInt(const std::vector<std::string>& fields, int idx, int fallback)
{
	if (idx < 0 || idx >= static_cast<int>(fields.size())) return fallback;
	int value = fallback;
	return ReadInt(fields[static_cast<size_t>(idx)], value) ? value : fallback;
}

double ReadOptionalDouble(const std::vector<std::string>& fields, int idx, double fallback)
{
	if (idx < 0 || idx >= static_cast<int>(fields.size())) return fallback;
	double value = fallback;
	return ReadDouble(fields[static_cast<size_t>(idx)], value) ? value : fallback;
}

// Strip a leading "# key=" prefix from an annotation line. Returns the value
// part (post-=) on match, or empty string when the line doesn't have that key.
std::string ParseHeaderKv(const std::string& line, const char* key)
{
	const std::string prefix = std::string("# ") + key + "=";
	if (line.rfind(prefix, 0) != 0) return {};
	return line.substr(prefix.size());
}

// %LocalAppDataLow%\WKOpenVR\Logs -- same path the live logger writes to.
// Was \SpaceCalibrator\Logs\ pre-monorepo; the umbrella consolidation moved
// SC's overlay-side logs to the umbrella's log directory.
std::wstring GetLogsDir()
{
	PWSTR rootPath = nullptr;
	if (SHGetKnownFolderPath(FOLDERID_LocalAppDataLow, 0, nullptr, &rootPath) != S_OK) {
		if (rootPath) CoTaskMemFree(rootPath);
		return {};
	}
	std::wstring p(rootPath);
	CoTaskMemFree(rootPath);
	p += LR"(\WKOpenVR\Logs)";
	return p;
}

ReplayQualitySnapshot SnapshotFromReport(const CalibrationQualityReport& report)
{
	const CalibrationQualityShadowSignals signals = EvaluateCalibrationQualityShadowSignals(report);
	ReplayQualitySnapshot out;
	out.available = true;
	out.shadowWouldAccept = signals.wouldAccept;
	out.shadowRejectReason = signals.firstRejectReason ? signals.firstRejectReason : "unknown";
	out.sampleCount = static_cast<int>(report.sampleCount);
	out.validSampleCount = report.validSampleCount;
	out.strictHealthySampleCount = report.strictHealthySampleCount;
	out.staleSampleCount = report.trackingStaleSampleCount;
	out.jumpSampleCount = report.trackingJumpSampleCount;
	out.zeroPoseSampleCount = report.zeroPoseSampleCount;
	out.unchangedPoseSampleCount = report.unchangedPoseSampleCount;
	out.highMotionSampleCount = report.highMotionSampleCount;
	out.deltaPair23Count = report.deltaPair23DegCount;
	out.rmsMm = report.residuals.rmsM * 1000.0;
	out.p95Mm = report.residuals.p95M * 1000.0;
	out.holdoutRmsMm = report.holdoutResiduals.rmsM * 1000.0;
	out.holdoutP90Mm = report.holdoutResiduals.p90M * 1000.0;
	out.holdoutP95Mm = report.holdoutResiduals.p95M * 1000.0;
	out.targetSpanM = report.targetSpanM;
	out.rotationSpanDeg = report.rotationSpanDeg;
	out.validRotationPairCount = report.validRotationPairCount;
	out.translationRank = report.translationRank;
	out.translationConditionRatio = report.translationConditionRatio;
	out.dynamicLimitMm = report.dynamicLimitM * 1000.0;
	out.outlierFraction = report.residuals.outlierFraction;
	out.maxPoseAgeMs = report.maxPoseAgeMs;
	out.maxPoseGapMs = report.maxPoseGapMs;
	out.maxLinearSpeedMps = report.maxLinearSpeedMps;
	out.maxAngularSpeedDegps = report.maxAngularSpeedDegps;
	out.legacyRmsPass = report.legacyRmsPass;
	out.strictSamplesPass = report.strictSamplesPass;
	out.geometryPass = report.geometryPass;
	out.robustResidualPass = report.robustResidualPass;
	out.holdoutPass = report.holdoutPass;
	out.trackingHealthPass = report.trackingHealthPass;
	out.novaDeltaPairsPass = report.novaDeltaPairsPass;
	return out;
}

void AddReasonCount(std::vector<ReplayReasonCount>& counts, const std::string& reason)
{
	for (auto& entry : counts) {
		if (entry.reason == reason) {
			++entry.count;
			return;
		}
	}
	ReplayReasonCount entry;
	entry.reason = reason;
	entry.count = 1;
	counts.push_back(std::move(entry));
}

// Read a 7-tuple of doubles (tx,ty,tz,qw,qx,qy,qz) from `fields` at the given
// column indexes into `pose` (world space). Returns false if any column is
// missing or unparseable, leaving `pose` untouched. A zero-norm quaternion
// (the "device not tracking" sentinel) is normalised to identity so the
// rotation is still well-formed; only the translation matters for the snap
// displacement deltas the caller computes.
bool ReadPoseColumns(const std::vector<std::string>& fields, int idxTx, int idxTy, int idxTz, int idxQw, int idxQx,
                     int idxQy, int idxQz, Pose& pose)
{
	if (idxTx < 0 || idxTy < 0 || idxTz < 0 || idxQw < 0 || idxQx < 0 || idxQy < 0 || idxQz < 0) return false;
	double t[3], q[4];
	if (!ReadDouble(fields[static_cast<size_t>(idxTx)], t[0]) ||
	    !ReadDouble(fields[static_cast<size_t>(idxTy)], t[1]) ||
	    !ReadDouble(fields[static_cast<size_t>(idxTz)], t[2]) ||
	    !ReadDouble(fields[static_cast<size_t>(idxQw)], q[0]) ||
	    !ReadDouble(fields[static_cast<size_t>(idxQx)], q[1]) ||
	    !ReadDouble(fields[static_cast<size_t>(idxQy)], q[2]) ||
	    !ReadDouble(fields[static_cast<size_t>(idxQz)], q[3])) {
		return false;
	}
	Eigen::Quaterniond rot(q[0], q[1], q[2], q[3]);
	if (rot.norm() >= 1e-9) {
		rot.normalize();
	}
	else {
		rot = Eigen::Quaterniond::Identity();
	}
	pose.rot = rot.toRotationMatrix();
	pose.trans = Eigen::Vector3d(t[0], t[1], t[2]);
	return true;
}

bool ParseAnnotationTimestamp(const std::string& line, const char* marker, double& out)
{
	if (line.rfind("# [", 0) != 0) return false;
	if (line.find(marker) == std::string::npos) return false;
	const std::size_t start = 3;
	const std::size_t end = line.find(']', start);
	if (end == std::string::npos || end <= start) return false;
	const std::string ts = line.substr(start, end - start);
	return ReadDouble(ts, out);
}

// Parse any `# [ts] event ...` line into an AnnotationEvent. The key is the
// first token after the timestamp with a trailing ':' stripped (matching the
// live "event_name: k=v ..." idiom); bracketed tags like "[cal-heartbeat]"
// keep their brackets.
bool ParseAnnotationEvent(const std::string& line, AnnotationEvent& out)
{
	if (line.rfind("# [", 0) != 0) return false;
	const std::size_t tsEnd = line.find(']', 3);
	if (tsEnd == std::string::npos || tsEnd <= 3) return false;
	if (!ReadDouble(line.substr(3, tsEnd - 3), out.time)) return false;
	std::size_t keyStart = tsEnd + 1;
	while (keyStart < line.size() && line[keyStart] == ' ')
		++keyStart;
	std::size_t keyEnd = line.find(' ', keyStart);
	if (keyEnd == std::string::npos) keyEnd = line.size();
	out.key = line.substr(keyStart, keyEnd - keyStart);
	if (!out.key.empty() && out.key.back() == ':') out.key.pop_back();
	out.raw = line;
	return !out.key.empty();
}

// Extract the seeded profile from a StartContinuousCalibration_seed_profile
// annotation. Two live forms:
//   ... trans_cm=(x,y,z) mag_cm=m rot_deg=(z,y,x)
//   ... skipped validProfile=0
bool ParseSeedProfileAnnotation(const AnnotationEvent& event, SeedProfile& out)
{
	if (event.key != "StartContinuousCalibration_seed_profile") return false;
	out.present = true;
	out.time = event.time;
	double t[3], r[3];
	const std::size_t transPos = event.raw.find("trans_cm=(");
	const std::size_t rotPos = event.raw.find("rot_deg=(");
	if (transPos == std::string::npos || rotPos == std::string::npos) {
		out.valid = false; // the "skipped validProfile=0" form
		return true;
	}
	if (sscanf(event.raw.c_str() + transPos, "trans_cm=(%lf,%lf,%lf)", &t[0], &t[1], &t[2]) != 3 ||
	    sscanf(event.raw.c_str() + rotPos, "rot_deg=(%lf,%lf,%lf)", &r[0], &r[1], &r[2]) != 3) {
		out.valid = false;
		return true;
	}
	out.transCm = Eigen::Vector3d(t[0], t[1], t[2]);
	out.rotDeg = Eigen::Vector3d(r[0], r[1], r[2]);
	out.valid = true;
	return true;
}

// ReplayProfileTransform lives in MotionRecording.h so SessionReplay.h can
// seed with the identical mapping.

} // namespace

LoadedRecording LoadRecording(const std::string& path)
{
	LoadedRecording rec;
	rec.sourcePath = path;

	std::ifstream in(path);
	if (!in) {
		rec.error = "Failed to open recording: " + path;
		return rec;
	}

	std::string line;
	int formatVersion = 0;
	std::vector<std::string> header;

	while (std::getline(in, line)) {
		RTrim(line);
		if (line.empty()) continue;
		if (line.rfind('#', 0) == 0) {
			double relocTime = 0.0;
			if (ParseAnnotationTimestamp(line, "hmd_relocalization_detected", relocTime)) {
				rec.relocalizationAnnotationTimes.push_back(relocTime);
			}
			AnnotationEvent event;
			if (ParseAnnotationEvent(line, event)) {
				if (!rec.seedProfile.present) ParseSeedProfileAnnotation(event, rec.seedProfile);
				rec.annotations.push_back(std::move(event));
			}
			if (line.find("spacecal_log_v5") != std::string::npos) {
				formatVersion = 5;
				continue;
			}
			if (line.find("spacecal_log_v4") != std::string::npos) {
				formatVersion = 4;
				continue;
			}
			if (line.find("spacecal_log_v3") != std::string::npos) {
				formatVersion = 3;
				continue;
			}
			if (line.find("spacecal_log_v2") != std::string::npos) {
				formatVersion = 2;
				continue;
			}
			if (line.find("spacecal_log_v") != std::string::npos) {
				rec.error = "Recording is not v2/v3/v4/v5 (saw '" + line + "'). Replay needs v2 or newer.";
				return rec;
			}
			// Pull metadata KVs out of the header annotations the live logger emits.
			std::string v;
			if (!(v = ParseHeaderKv(line, "build_stamp")).empty())
				rec.meta.buildStamp = v;
			else if (!(v = ParseHeaderKv(line, "build_channel")).empty())
				rec.meta.buildChannel = v;
			else if (!(v = ParseHeaderKv(line, "hmd_tracking_system")).empty())
				rec.meta.hmdTrackingSystem = v;
			else if (!(v = ParseHeaderKv(line, "hmd_model")).empty())
				rec.meta.hmdModel = v;
			else if (!(v = ParseHeaderKv(line, "hmd_serial")).empty())
				rec.meta.hmdSerial = v;
			else if (!(v = ParseHeaderKv(line, "steamvr_runtime_path")).empty())
				rec.meta.steamvrRuntime = v;
			else if (!(v = ParseHeaderKv(line, "windows")).empty())
				rec.meta.windowsVersion = v;
			else if (!(v = ParseHeaderKv(line, "logical_processors")).empty()) {
				rec.meta.logicalProcessors = std::atoi(v.c_str());
			}
			continue;
		}
		header = SplitCsv(line);
		break;
	}

	if (header.empty()) {
		rec.error = "Recording has no column header row";
		return rec;
	}
	if (formatVersion < 2) {
		rec.error = "Recording missing '# spacecal_log_v2'/'v3'/'v4'/'v5' banner; v1 logs lack raw poses.";
		return rec;
	}
	rec.formatVersion = formatVersion;

	std::unordered_map<std::string, int> cols;
	for (size_t i = 0; i < header.size(); ++i)
		cols[header[i]] = (int)i;

	const int idxTimestamp = ColIndex(cols, "Timestamp");
	const int idxRefTx = ColIndex(cols, "ref_tx");
	const int idxRefTy = ColIndex(cols, "ref_ty");
	const int idxRefTz = ColIndex(cols, "ref_tz");
	const int idxRefQw = ColIndex(cols, "ref_qw");
	const int idxRefQx = ColIndex(cols, "ref_qx");
	const int idxRefQy = ColIndex(cols, "ref_qy");
	const int idxRefQz = ColIndex(cols, "ref_qz");
	const int idxTgtTx = ColIndex(cols, "tgt_tx");
	const int idxTgtTy = ColIndex(cols, "tgt_ty");
	const int idxTgtTz = ColIndex(cols, "tgt_tz");
	const int idxTgtQw = ColIndex(cols, "tgt_qw");
	const int idxTgtQx = ColIndex(cols, "tgt_qx");
	const int idxTgtQy = ColIndex(cols, "tgt_qy");
	const int idxTgtQz = ColIndex(cols, "tgt_qz");
	const int idxTickPhase = ColIndex(cols, "tick_phase");
	const int idxSampleObserved = ColIndex(cols, "sample_observed");
	const int idxSampleAccepted = ColIndex(cols, "sample_accepted");
	const int idxSamplePairedMotionValid = ColIndex(cols, "sample_paired_motion_valid");
	const int idxSampleRefConnected = ColIndex(cols, "sample_ref_connected");
	const int idxSampleTgtConnected = ColIndex(cols, "sample_tgt_connected");
	const int idxSampleRefPoseValid = ColIndex(cols, "sample_ref_pose_valid");
	const int idxSampleTgtPoseValid = ColIndex(cols, "sample_tgt_pose_valid");
	const int idxSampleRefTrackingResult = ColIndex(cols, "sample_ref_tracking_result");
	const int idxSampleTgtTrackingResult = ColIndex(cols, "sample_tgt_tracking_result");
	const int idxSampleRefAgeMs = ColIndex(cols, "sample_ref_age_ms");
	const int idxSampleTgtAgeMs = ColIndex(cols, "sample_tgt_age_ms");
	const int idxSampleRefGapMs = ColIndex(cols, "sample_ref_gap_ms");
	const int idxSampleTgtGapMs = ColIndex(cols, "sample_tgt_gap_ms");
	const int idxSampleRefSpeedMps = ColIndex(cols, "sample_ref_speed_mps");
	const int idxSampleTgtSpeedMps = ColIndex(cols, "sample_tgt_speed_mps");
	const int idxSampleRefAngSpeedRadps = ColIndex(cols, "sample_ref_ang_speed_radps");
	const int idxSampleTgtAngSpeedRadps = ColIndex(cols, "sample_tgt_ang_speed_radps");
	const int idxSampleRefZeroPose = ColIndex(cols, "sample_ref_zero_pose");
	const int idxSampleTgtZeroPose = ColIndex(cols, "sample_tgt_zero_pose");
	const int idxSampleRefUnchanged = ColIndex(cols, "sample_ref_unchanged");
	const int idxSampleTgtUnchanged = ColIndex(cols, "sample_tgt_unchanged");
	const int idxSampleStale = ColIndex(cols, "sample_stale");
	const int idxSampleJump = ColIndex(cols, "sample_jump");
	const bool hasSampleDiagnostics = idxSampleObserved >= 0 || idxSampleAccepted >= 0;

	// v4 locked-snap columns (all optional; absent in v2/v3 -> idx < 0).
	const int idxHmdTx = ColIndex(cols, "hmd_tx");
	const int idxHmdTy = ColIndex(cols, "hmd_ty");
	const int idxHmdTz = ColIndex(cols, "hmd_tz");
	const int idxHmdQw = ColIndex(cols, "hmd_qw");
	const int idxHmdQx = ColIndex(cols, "hmd_qx");
	const int idxHmdQy = ColIndex(cols, "hmd_qy");
	const int idxHmdQz = ColIndex(cols, "hmd_qz");
	const int idxHeadTrackerValid = ColIndex(cols, "head_tracker_valid");
	const int idxHeadTx = ColIndex(cols, "head_tracker_tx");
	const int idxHeadTy = ColIndex(cols, "head_tracker_ty");
	const int idxHeadTz = ColIndex(cols, "head_tracker_tz");
	const int idxHeadQw = ColIndex(cols, "head_tracker_qw");
	const int idxHeadQx = ColIndex(cols, "head_tracker_qx");
	const int idxHeadQy = ColIndex(cols, "head_tracker_qy");
	const int idxHeadQz = ColIndex(cols, "head_tracker_qz");
	const int idxRelocDetected = ColIndex(cols, "reloc_detected");
	const int idxExperimentalFlags = ColIndex(cols, "experimental_flags");
	rec.hasRelocDetectedColumn = idxRelocDetected >= 0;
	rec.hasExperimentalFlagsColumn = idxExperimentalFlags >= 0;
	// The locked-snap A/B needs both the HMD jump and the head-tracker displacement,
	// so it's only enabled when both pose blocks (and the valid flag) are present.
	const bool hasLockedSnapColumns = idxHmdTx >= 0 && idxHeadTx >= 0 && idxHeadTrackerValid >= 0;
	rec.hasLockedSnapColumns = hasLockedSnapColumns;

	const int required[] = {
	    idxRefTx, idxRefTy, idxRefTz, idxRefQw, idxRefQx, idxRefQy, idxRefQz,
	    idxTgtTx, idxTgtTy, idxTgtTz, idxTgtQw, idxTgtQx, idxTgtQy, idxTgtQz,
	};
	for (int idx : required) {
		if (idx < 0) {
			rec.error =
			    "Recording is missing one of the required raw-pose columns (ref_t{x,y,z}, ref_q{w,x,y,z}, tgt_*).";
			return rec;
		}
	}

	rec.rows.reserve(1024); // avoid early reallocations on typical recordings.

	while (std::getline(in, line)) {
		RTrim(line);
		if (line.empty()) continue;
		if (line.rfind('#', 0) == 0) {
			double relocTime = 0.0;
			if (ParseAnnotationTimestamp(line, "hmd_relocalization_detected", relocTime)) {
				rec.relocalizationAnnotationTimes.push_back(relocTime);
			}
			AnnotationEvent event;
			if (ParseAnnotationEvent(line, event)) {
				if (!rec.seedProfile.present) ParseSeedProfileAnnotation(event, rec.seedProfile);
				rec.annotations.push_back(std::move(event));
			}
			continue;
		}

		const auto fields = SplitCsv(line);
		if ((int)fields.size() < (int)header.size()) {
			// Likely a partial row (log rotated mid-tick). Skip silently — the
			// summary's "rows replayed" already tells the user how many were used.
			continue;
		}

		ReplayRow row;
		double ts = 0.0;
		if (idxTimestamp >= 0) ReadDouble(fields[idxTimestamp], ts);
		row.timestamp = ts;

		double rt[3], rq[4], tt[3], tq[4];
		if (!ReadDouble(fields[idxRefTx], rt[0]) || !ReadDouble(fields[idxRefTy], rt[1]) ||
		    !ReadDouble(fields[idxRefTz], rt[2]) || !ReadDouble(fields[idxRefQw], rq[0]) ||
		    !ReadDouble(fields[idxRefQx], rq[1]) || !ReadDouble(fields[idxRefQy], rq[2]) ||
		    !ReadDouble(fields[idxRefQz], rq[3]) || !ReadDouble(fields[idxTgtTx], tt[0]) ||
		    !ReadDouble(fields[idxTgtTy], tt[1]) || !ReadDouble(fields[idxTgtTz], tt[2]) ||
		    !ReadDouble(fields[idxTgtQw], tq[0]) || !ReadDouble(fields[idxTgtQx], tq[1]) ||
		    !ReadDouble(fields[idxTgtQy], tq[2]) || !ReadDouble(fields[idxTgtQz], tq[3])) {
			continue;
		}

		Eigen::Quaterniond rqQ(rq[0], rq[1], rq[2], rq[3]);
		Eigen::Quaterniond tqQ(tq[0], tq[1], tq[2], tq[3]);
		const bool refQuatValid = rqQ.norm() >= 1e-9;
		const bool targetQuatValid = tqQ.norm() >= 1e-9;
		// Zero-quaternion sentinel = "device wasn't tracking this tick". v2
		// had no separate sample-health columns, so those rows are unreplayable.
		// v3 keeps them as rejected sample evidence when diagnostics are present.
		if (!refQuatValid || !targetQuatValid) {
			if (!hasSampleDiagnostics) continue;
			rqQ = Eigen::Quaterniond::Identity();
			tqQ = Eigen::Quaterniond::Identity();
		}
		else {
			rqQ.normalize();
			tqQ.normalize();
		}

		row.ref.rot = rqQ.toRotationMatrix();
		row.ref.trans = Eigen::Vector3d(rt[0], rt[1], rt[2]);
		row.target.rot = tqQ.toRotationMatrix();
		row.target.trans = Eigen::Vector3d(tt[0], tt[1], tt[2]);
		row.sample = Sample(row.ref, row.target, row.timestamp);

		if (idxTickPhase >= 0) row.tickPhase = fields[idxTickPhase];
		row.hasSampleDiagnostics = hasSampleDiagnostics;
		if (hasSampleDiagnostics) {
			row.sampleObserved = ReadOptionalBool(fields, idxSampleObserved, true);
			row.sampleAccepted = ReadOptionalBool(fields, idxSampleAccepted, row.sampleObserved);
			row.sample.valid = row.sampleObserved && row.sampleAccepted && refQuatValid && targetQuatValid;
			row.sample.pairedMotionValid = ReadOptionalBool(fields, idxSamplePairedMotionValid, true);
			row.sample.refDeviceConnected = ReadOptionalBool(fields, idxSampleRefConnected, true);
			row.sample.targetDeviceConnected = ReadOptionalBool(fields, idxSampleTgtConnected, true);
			row.sample.refPoseValid = ReadOptionalBool(fields, idxSampleRefPoseValid, true);
			row.sample.targetPoseValid = ReadOptionalBool(fields, idxSampleTgtPoseValid, true);
			row.sample.refTrackingResult = ReadOptionalInt(fields, idxSampleRefTrackingResult, 200);
			row.sample.targetTrackingResult = ReadOptionalInt(fields, idxSampleTgtTrackingResult, 200);
			row.sample.refPoseAgeMs = ReadOptionalDouble(fields, idxSampleRefAgeMs, 0.0);
			row.sample.targetPoseAgeMs = ReadOptionalDouble(fields, idxSampleTgtAgeMs, 0.0);
			row.sample.refPoseGapMs = ReadOptionalDouble(fields, idxSampleRefGapMs, 0.0);
			row.sample.targetPoseGapMs = ReadOptionalDouble(fields, idxSampleTgtGapMs, 0.0);
			row.sample.refLinearSpeedMps = ReadOptionalDouble(fields, idxSampleRefSpeedMps, 0.0);
			row.sample.targetLinearSpeedMps = ReadOptionalDouble(fields, idxSampleTgtSpeedMps, 0.0);
			row.sample.refAngularSpeedRadps = ReadOptionalDouble(fields, idxSampleRefAngSpeedRadps, 0.0);
			row.sample.targetAngularSpeedRadps = ReadOptionalDouble(fields, idxSampleTgtAngSpeedRadps, 0.0);
			row.sample.refZeroPose = ReadOptionalBool(fields, idxSampleRefZeroPose, false);
			row.sample.targetZeroPose = ReadOptionalBool(fields, idxSampleTgtZeroPose, false);
			row.sample.refPoseUnchanged = ReadOptionalBool(fields, idxSampleRefUnchanged, false);
			row.sample.targetPoseUnchanged = ReadOptionalBool(fields, idxSampleTgtUnchanged, false);
			row.sample.trackingPoseStale = ReadOptionalBool(fields, idxSampleStale, false);
			row.sample.trackingPoseJump = ReadOptionalBool(fields, idxSampleJump, false);
		}
		else {
			row.sampleObserved = true;
			row.sampleAccepted = true;
			row.sample.valid = true;
		}

		// v4 locked-snap inputs. Optional: absent columns leave hasHmdPose /
		// headTrackerValid false so the locked-snap replay path is skipped. The
		// head-tracker pose is only read when the recorded valid flag is set
		// (mirrors the live detector clearing its cache on tracker invalidity).
		if (hasLockedSnapColumns) {
			row.hasHmdPose =
			    ReadPoseColumns(fields, idxHmdTx, idxHmdTy, idxHmdTz, idxHmdQw, idxHmdQx, idxHmdQy, idxHmdQz, row.hmd);
			if (ReadOptionalBool(fields, idxHeadTrackerValid, false)) {
				row.headTrackerValid = ReadPoseColumns(fields, idxHeadTx, idxHeadTy, idxHeadTz, idxHeadQw, idxHeadQx,
				                                       idxHeadQy, idxHeadQz, row.headTracker);
			}
		}
		row.relocDetected = ReadOptionalBool(fields, idxRelocDetected, false);
		if (row.relocDetected) ++rec.relocDetectedRowCount;
		if (idxExperimentalFlags >= 0) {
			row.hasExperimentalFlags = true;
			row.experimentalFlags = static_cast<uint32_t>(ReadOptionalInt(fields, idxExperimentalFlags, 0));
		}

		rec.rows.push_back(std::move(row));
	}

	return rec;
}

ReplayResult RunReplay(const LoadedRecording& rec, const ReplayOptions& opts)
{
	ReplayResult res;
	if (!rec.error.empty()) {
		res.error = rec.error;
		return res;
	}
	if (rec.rows.empty()) {
		res.error = "Recording has no replayable rows.";
		return res;
	}

	CalibrationCalc calc;
	calc.enableStaticRecalibration = false;
	calc.lockRelativePosition = opts.lockRelativePosition;
	calc.SetPrecisionWeightedRelPose(opts.precisionWeightedRelPose);
	calc.SetGravityConstrainedRelPose(opts.gravityConstrainedRelPose);
	// Relative-pose MAD tracking (the AUTO-lock translMad analog) -- the headline
	// drift signal summarised as peak/median/final.
	std::deque<Eigen::AffineCompact3d> relWindow;
	std::vector<double> allMad;
	// Applied-C trajectory (A/B). appliedC is the worldFromDriver translation the
	// driver would show (cm); its session wander/step is the fly-off signal, which
	// the precision-weighted solve should shrink.
	Eigen::Vector3d appliedC = Eigen::Vector3d::Zero();
	Eigen::Vector3d prevAppliedC = Eigen::Vector3d::Zero();
	bool hasAppliedC = false;
	double minAppliedMag = 0.0, maxAppliedMag = 0.0;
	// Confidence-weighted fusion state (mirrors the live accept path when
	// precisionWeightedRelPose is on): running fused calibration + accumulated
	// precision, so the applied-C trajectory reflects the fusion, not raw solves.
	Eigen::AffineCompact3d appliedTransform = Eigen::AffineCompact3d::Identity();
	double accumPrecision = 0.0;
	int disagreeStreak = 0;

	// Warm start. Mirrors StartContinuousCalibration with a valid profile: the
	// solver estimate is seeded (so the improvement gate compares against it)
	// and the fusion gets the banked-profile prior. The applied-C trajectory
	// starts at the seed, so wander/steps measure movement AWAY from the stored
	// profile -- the signal that distinguishes "fusion defends a bad seed" from
	// "overwrite escapes it".
	{
		bool seed = false;
		Eigen::Vector3d seedTransCm, seedRotDeg;
		if (opts.seedMode == ReplaySeedMode::Recorded && rec.seedProfile.valid) {
			seedTransCm = rec.seedProfile.transCm;
			seedRotDeg = rec.seedProfile.rotDeg;
			seed = true;
		}
		else if (opts.seedMode == ReplaySeedMode::Explicit) {
			seedTransCm = opts.seedTransCm;
			seedRotDeg = opts.seedRotDeg;
			seed = true;
		}
		if (seed) {
			const Eigen::AffineCompact3d seedTransform = ReplayProfileTransform(seedRotDeg, seedTransCm);
			calc.SeedEstimatedTransformation(seedTransform, /*annotate=*/false);
			appliedTransform = seedTransform;
			accumPrecision = spacecal::precision::kSeedPriorPrecision;
			appliedC = seedTransform.translation() * 100.0;
			prevAppliedC = appliedC;
			minAppliedMag = maxAppliedMag = appliedC.norm();
			hasAppliedC = true;
			res.seedApplied = true;
			res.seedMagCm = appliedC.norm();
		}
	}
	// First applied C of the session (seed or first accept); netDriftVectorCm
	// measures where the session ended relative to this.
	Eigen::Vector3d firstAppliedC = appliedC;
	bool haveFirstAppliedC = hasAppliedC;
	// Finite-diff HMD speed for the experience metrics (vecVelocity is not
	// recorded); mirrors the AutoLockSim speed source.
	bool havePrevHmd = false;
	double prevHmdTime = 0.0;
	Eigen::Vector3d prevHmdTrans = Eigen::Vector3d::Zero();
	double lastHmdSpeed = -1.0; // <0 = unknown; unknown never counts as stationary

	const bool boundedContinuous = opts.continuous && opts.maxContinuousSamples > 0;
	const std::size_t continuousWindow = boundedContinuous ? opts.maxContinuousSamples : 0;
	const std::size_t continuousDrop = boundedContinuous ? std::max<std::size_t>(1, continuousWindow / 10) : 0;

	res.trace.reserve(rec.rows.size());
	res.qualityTrace.reserve(opts.qualityReportInterval > 0 ? (rec.rows.size() / opts.qualityReportInterval) + 2 : 1);

	auto captureQuality = [&](ReplayTickResult* tick) {
		if (calc.SampleCount() < 3) return;
		Eigen::AffineCompact3d qualityTransform =
		    calc.isValid() ? calc.Transformation() : Eigen::AffineCompact3d::Identity();
		if (!calc.isValid()) {
			CalibrationCalc probe = calc;
			if (probe.ComputeOneshot(opts.ignoreOutliers)) {
				qualityTransform = probe.Transformation();
			}
		}
		ReplayQualitySnapshot snapshot = SnapshotFromReport(
		    calc.EvaluateCalibrationQuality(qualityTransform, opts.includeHoldoutQuality, opts.ignoreOutliers));
		if (!snapshot.available) return;
		++res.qualityReports;
		if (snapshot.shadowWouldAccept) {
			++res.shadowWouldAccept;
		}
		else {
			++res.shadowWouldReject;
			AddReasonCount(res.shadowRejectReasons, snapshot.shadowRejectReason);
		}
		res.finalQuality = snapshot;
		res.qualityTrace.push_back(snapshot);
		if (tick) {
			tick->rawErrMm = snapshot.rmsMm;
		}
	};

	for (std::size_t rowIndex = 0; rowIndex < rec.rows.size(); ++rowIndex) {
		const auto& row = rec.rows[rowIndex];

		Sample s = row.hasSampleDiagnostics ? row.sample : Sample(row.ref, row.target, row.timestamp);
		if (row.sampleObserved) ++res.sampleRowsObserved;
		if (row.hasSampleDiagnostics) {
			if (row.sampleAccepted && s.valid)
				++res.sampleRowsAccepted;
			else if (row.sampleObserved)
				++res.sampleRowsRejected;
			const bool strictHealthy =
			    s.refDeviceConnected && s.targetDeviceConnected && s.refPoseValid && s.targetPoseValid &&
			    s.refTrackingResult == static_cast<int>(vr::ETrackingResult::TrackingResult_Running_OK) &&
			    s.targetTrackingResult == static_cast<int>(vr::ETrackingResult::TrackingResult_Running_OK);
			if (row.sampleObserved && !strictHealthy) ++res.sampleRowsStrictUnhealthy;
			if (s.trackingPoseStale) ++res.sampleRowsStale;
			if (s.trackingPoseJump) ++res.sampleRowsJump;
			if (row.sampleObserved && !s.pairedMotionValid) ++res.sampleRowsPairedMotionInvalid;
			if (row.sampleObserved && (s.refZeroPose || s.targetZeroPose)) ++res.sampleRowsZeroPose;
			if (row.sampleObserved && (s.refPoseUnchanged || s.targetPoseUnchanged)) ++res.sampleRowsUnchanged;
			if (row.sampleObserved && (std::max(s.refLinearSpeedMps, s.targetLinearSpeedMps) > 1.5 ||
			                           std::max(s.refAngularSpeedRadps, s.targetAngularSpeedRadps) > EIGEN_PI)) {
				++res.sampleRowsHighMotion;
			}
		}
		else {
			++res.sampleRowsAccepted;
		}

		ReplayTickResult tick;
		tick.timestamp = row.timestamp;
		tick.relocDetected = row.relocDetected;
		if (row.hasHmdPose) {
			if (havePrevHmd) {
				const double dt = row.timestamp - prevHmdTime;
				if (dt > 1e-6) lastHmdSpeed = (row.hmd.trans - prevHmdTrans).norm() / dt;
			}
			prevHmdTrans = row.hmd.trans;
			prevHmdTime = row.timestamp;
			havePrevHmd = true;
		}
		tick.hmdStationary = lastHmdSpeed >= 0.0 && spacecal::autolock::HmdIsStationary(lastHmdSpeed);

		if (row.hasSampleDiagnostics && (!row.sampleObserved || !row.sampleAccepted || !s.valid)) {
			tick.rejectReason = row.sampleObserved ? "sample_rejected" : "no_sample";
			res.trace.push_back(std::move(tick));
			continue;
		}

		// Relative pose ref^-1*target -> the AUTO-lock translMad analog below.
		Eigen::AffineCompact3d refW;
		refW.linear() = s.ref.rot;
		refW.translation() = s.ref.trans;
		Eigen::AffineCompact3d tgtW;
		tgtW.linear() = s.target.rot;
		tgtW.translation() = s.target.trans;
		const Eigen::AffineCompact3d rel = refW.inverse() * tgtW;

		calc.PushSample(s);
		++res.solverSamplesPushed;
		if (boundedContinuous) {
			while (calc.SampleCount() > continuousWindow) {
				calc.ShiftSample();
			}
		}
		res.maxSamplesInWindow = std::max(res.maxSamplesInWindow, static_cast<int>(calc.SampleCount()));

		// Relative-pose MAD (the AUTO-lock translMad analog) over a bounded window
		// -- the headline drift signal, summarised as peak/median/final.
		relWindow.push_back(rel);
		while (relWindow.size() > spacecal::autolock::kHistoryMax)
			relWindow.pop_front();
		if (relWindow.size() >= spacecal::autolock::kSamplesNeeded) {
			const double madMm = spacecal::autolock::RobustTranslDeviation(relWindow) * 1000.0;
			allMad.push_back(madMm);
			res.peakRelPoseMadMm = std::max(res.peakRelPoseMadMm, madMm);
			res.finalRelPoseMadMm = madMm;
			tick.relMadMm = madMm;
		}

		if (opts.continuous) {
			if (boundedContinuous && calc.SampleCount() < continuousWindow) {
				tick.rejectReason = "waiting_for_samples";
				if (opts.qualityReportInterval > 0 && ((rowIndex + 1) % opts.qualityReportInterval) == 0) {
					captureQuality(&tick);
				}
				res.trace.push_back(std::move(tick));
				continue;
			}

			bool lerp = false;
			const bool ok = calc.ComputeIncremental(lerp, opts.threshold, opts.maxRelError, opts.ignoreOutliers);
			tick.accepted = ok;
			if (ok) {
				++res.accepts;
				// Applied-C trajectory (the worldFromDriver the driver would show).
				Eigen::Vector3d candidateC;
				if (opts.precisionWeightedRelPose) {
					const double measPrec = spacecal::precision::MeasurementPrecision(calc.MeanSquaredLeverArmM2());
					const double disagreeM =
					    (calc.Transformation().translation() - appliedTransform.translation()).norm();
					if (spacecal::precision::NoteSeedDisagreement(disagreeStreak, disagreeM)) {
						appliedTransform = calc.Transformation();
						accumPrecision = std::min(measPrec, spacecal::precision::kMaxConfidence);
						tick.fusionGain = 1.0;
					}
					else {
						const double gain = spacecal::precision::FusionGain(accumPrecision, measPrec);
						appliedTransform = spacecal::precision::Fuse(appliedTransform, calc.Transformation(), gain);
						accumPrecision = std::min(accumPrecision + measPrec, spacecal::precision::kMaxConfidence);
						tick.fusionGain = gain;
					}
					candidateC = appliedTransform.translation() * 100.0;
				}
				else {
					candidateC = calc.Transformation().translation() * 100.0;
					tick.fusionGain = 1.0;
				}
				appliedC = candidateC;
				const double mag = appliedC.norm();
				if (!hasAppliedC) {
					minAppliedMag = maxAppliedMag = mag;
					hasAppliedC = true;
				}
				else {
					minAppliedMag = std::min(minAppliedMag, mag);
					maxAppliedMag = std::max(maxAppliedMag, mag);
					const double stepCm = (appliedC - prevAppliedC).norm();
					res.peakAppliedStepCm = std::max(res.peakAppliedStepCm, stepCm);
					res.totalAppliedPathCm += stepCm;
					if (stepCm > 20.0) ++res.largeAppliedSteps;
					const double stepMm = stepCm * 10.0;
					if (tick.hmdStationary && stepMm > kPerceptibleShiftMm) {
						++res.perceptibleShiftCount;
						res.perceptibleShiftMaxMm = std::max(res.perceptibleShiftMaxMm, stepMm);
						res.perceptibleShiftSumMm += stepMm;
					}
				}
				if (!haveFirstAppliedC) {
					firstAppliedC = appliedC;
					haveFirstAppliedC = true;
				}
				res.peakAppliedMagCm = maxAppliedMag;
				prevAppliedC = appliedC;
				tick.appliedCCm = appliedC;
				tick.hasAppliedC = true;
				tick.accumPrecision = accumPrecision;
			}
			else
				++res.rejects;
			// CalibrationCalc doesn't expose its post-validate prior error
			// directly, so use the public Validate path against the current
			// estimate — only valid after the first accept, otherwise we
			// leave the error fields at 0.
			if (calc.isValid()) {
				double err = 0.0;
				Eigen::Vector3d offset;
				if (calc.ValidateCalibration(calc.Transformation(), &err, &offset)) {
					tick.currentCalErrMm = err * 1000.0;
				}
			}
			if (!ok) tick.rejectReason = "rejected";
			if (opts.qualityReportInterval > 0 && (ok || ((rowIndex + 1) % opts.qualityReportInterval) == 0)) {
				captureQuality(&tick);
			}

			if (boundedContinuous) {
				for (std::size_t i = 0; i < continuousDrop; ++i) {
					calc.ShiftSample();
				}
			}
		}
		else {
			// Oneshot mode just keeps appending samples. The single Compute below
			// runs after the loop. Per-tick trace stays as samples-only.
			if (opts.qualityReportInterval > 0 && ((rowIndex + 1) % opts.qualityReportInterval) == 0) {
				captureQuality(&tick);
			}
		}

		res.trace.push_back(std::move(tick));
	}

	if (!opts.continuous) {
		const bool ok = calc.ComputeOneshot(opts.ignoreOutliers);
		if (ok)
			++res.accepts;
		else
			++res.rejects;
	}

	captureQuality(nullptr);

	if (!allMad.empty()) {
		std::vector<double> sorted = allMad;
		std::sort(sorted.begin(), sorted.end());
		res.medianRelPoseMadMm = sorted[sorted.size() / 2];
	}
	res.appliedMagWanderCm = hasAppliedC ? (maxAppliedMag - minAppliedMag) : 0.0;
	if (haveFirstAppliedC) {
		res.finalAppliedCCm = appliedC;
		res.netDriftVectorCm = appliedC - firstAppliedC;
	}

	res.rowsReplayed = (int)rec.rows.size();
	const int observedRows = res.sampleRowsObserved > 0 ? res.sampleRowsObserved : res.rowsReplayed;
	if (observedRows > 0) {
		res.solverSampleRatio = static_cast<double>(res.solverSamplesPushed) / static_cast<double>(observedRows);
	}
	res.finalTransformValid = calc.isValid();
	if (calc.isValid()) {
		res.finalTransform = calc.Transformation();
		double err = 0.0;
		Eigen::Vector3d offset;
		if (calc.ValidateCalibration(calc.Transformation(), &err, &offset)) {
			res.finalErrorMm = err * 1000.0;
		}
	}
	res.succeeded = true;
	return res;
}

std::vector<LogFileEntry> ListRecordings()
{
	std::vector<LogFileEntry> out;
	const std::wstring dir = GetLogsDir();
	if (dir.empty()) return out;

	const std::wstring pattern = dir + L"\\spacecal_log.*.txt";
	WIN32_FIND_DATAW find{};
	HANDLE h = FindFirstFileW(pattern.c_str(), &find);
	if (h == INVALID_HANDLE_VALUE) return out;

	do {
		if (find.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) continue;

		LogFileEntry entry;
		// File name only, in UTF-8 for ImGui display.
		const int n = WideCharToMultiByte(CP_UTF8, 0, find.cFileName, -1, nullptr, 0, nullptr, nullptr);
		if (n > 1) {
			entry.name.resize(n - 1);
			WideCharToMultiByte(CP_UTF8, 0, find.cFileName, -1, entry.name.data(), n, nullptr, nullptr);
		}
		entry.fullPath = dir + L"\\" + find.cFileName;
		entry.sizeBytes = ((uint64_t)find.nFileSizeHigh << 32) | (uint64_t)find.nFileSizeLow;
		entry.mtimeFileTime =
		    ((uint64_t)find.ftLastWriteTime.dwHighDateTime << 32) | (uint64_t)find.ftLastWriteTime.dwLowDateTime;
		out.push_back(std::move(entry));
	} while (FindNextFileW(h, &find));
	FindClose(h);

	// Newest-first so the recording the user just made is at the top.
	std::sort(out.begin(), out.end(), [](const LogFileEntry& a, const LogFileEntry& b) {
		if (a.mtimeFileTime == b.mtimeFileTime) return a.name > b.name;
		return a.mtimeFileTime > b.mtimeFileTime;
	});
	return out;
}

RecordingPruneResult PruneRecordings(const RecordingRetentionPolicy& policy)
{
	RecordingPruneResult result;
	const std::vector<LogFileEntry> entries = ListRecordings();
	result.totalFiles = entries.size();
	for (const auto& entry : entries) {
		result.totalBytes = detail::SaturatingAdd(result.totalBytes, entry.sizeBytes);
	}

	const RecordingRetentionPlan plan = PlanRecordingRetention(entries, policy);
	for (std::size_t index : plan.deleteIndexes) {
		if (index >= entries.size()) continue;
		const auto& entry = entries[index];
		if (DeleteFileW(entry.fullPath.c_str())) {
			++result.deletedFiles;
			result.freedBytes = detail::SaturatingAdd(result.freedBytes, entry.sizeBytes);
		}
		else {
			++result.failedDeletes;
		}
	}

	result.keptFiles = result.totalFiles - result.deletedFiles;
	result.keptBytes = (result.totalBytes >= result.freedBytes) ? (result.totalBytes - result.freedBytes) : 0;
	return result;
}

} // namespace spacecal::replay
