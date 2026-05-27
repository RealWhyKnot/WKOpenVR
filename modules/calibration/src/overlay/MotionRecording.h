#pragma once

// Loads a captured "spacecal_log_v2" CSV (the same file format the debug logger
// emits to %LocalAppDataLow%\SpaceCalibrator\Logs\) into memory and replays it
// through a fresh CalibrationCalc. The intent: a user records a problematic
// motion sequence, we ship a fix, the user replays the same recording against
// the new build to see whether the fix changed behavior.
//
// Parsing is column-name based, so adding new columns to the live log is
// backward compatible — only the raw-pose columns are required. The header line
// must begin with `# spacecal_log_v2`; older v1 captures lacked the raw poses
// and can't be replayed.

#include "CalibrationCalc.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace spacecal::replay {

// One replayable tick. Produced by parsing a v2 log row.
struct ReplayRow {
	double timestamp = 0.0;
	Pose ref;
	Pose target;
	std::string tickPhase; // "Continuous", "Begin", etc — informational only.
};

// Parsed file metadata: header annotations the live logger emits up-front
// (build stamp, HMD model, OS version). Useful for the UI to show "this
// recording came from build X on hardware Y".
struct RecordingMeta {
	std::string buildStamp;       // e.g. "2026.4.28.2"
	std::string buildChannel;     // "release" or "dev"
	std::string hmdTrackingSystem;
	std::string hmdModel;
	std::string hmdSerial;
	std::string steamvrRuntime;
	std::string windowsVersion;
	int logicalProcessors = 0;
};

// Result of loading a file. `rows` holds the per-tick raw poses in order;
// `meta` is whatever header info the file embedded; `error` is non-empty on
// any parsing failure (file missing, wrong version banner, missing required
// columns, etc).
struct LoadedRecording {
	RecordingMeta meta;
	std::vector<ReplayRow> rows;
	std::string sourcePath;     // path passed to LoadRecording, for display
	std::string error;          // populated on failure; rows will be empty.
};

// Parse a v2 log file from disk. Returns LoadedRecording with `error` empty
// on success; any failure populates `error` with a human-readable reason.
LoadedRecording LoadRecording(const std::string& path);

// Per-row trace produced during a replay. Lets the UI plot the error/accept
// trajectory and show histograms of rejection reasons. Kept compact — one
// entry per replayed tick — so even a 60s recording at 60Hz is only ~3600
// entries.
struct ReplayTickResult {
	double timestamp = 0.0;
	bool accepted = false;        // ComputeIncremental returned true
	double currentCalErrMm = 0.0; // RMS error against the (running) prior calibration, mm
	double rawErrMm = 0.0;        // RMS error of the candidate computed this tick, mm (NaN if no candidate)
	int    consecutiveRejections = 0;
	std::string rejectReason;     // empty on accept
};

// Replay parameters. Mirror the user-facing knobs in the live CalCtx so
// the user can A/B compare "what would my recording look like with a tighter
// recalibration threshold".
struct ReplayOptions {
	bool   continuous = true;     // false -> single ComputeOneshot at end
	double threshold = 1.5;       // continuousCalibrationThreshold
	double maxRelError = 0.005;
	bool   ignoreOutliers = true;
};

// Result summary. Aggregates whatever is useful at a glance — counts and the
// final transform — plus the full per-tick trace for visualisation.
struct ReplayResult {
	bool   succeeded = false;
	int    rowsReplayed = 0;
	int    accepts = 0;
	int    rejects = 0;
	int    watchdogResets = 0;
	double finalErrorMm = 0.0;    // NaN if calc never produced a valid result
	Eigen::AffineCompact3d finalTransform = Eigen::AffineCompact3d::Identity();
	bool   finalTransformValid = false;
	std::vector<ReplayTickResult> trace;
	std::string error;            // populated on failure
};

// Run the replay synchronously. Cheap enough for in-frame execution: a
// 60-second recording at 60Hz is ~3600 ticks and the math is well under a
// millisecond per tick.
ReplayResult RunReplay(const LoadedRecording& rec, const ReplayOptions& opts);

// List recent v2 log files in the user's logs directory, newest first. Used
// by the UI to populate a "pick a recording" dropdown without forcing the
// user to copy-paste paths.
struct LogFileEntry {
	std::string name;             // file name only
	std::wstring fullPath;        // absolute path for opening
	uint64_t sizeBytes = 0;
	uint64_t mtimeFileTime = 0;   // FILETIME as uint64; for sorting
};

constexpr std::size_t kDevAutoRecordingMaxFiles = 5;
constexpr uint64_t kDevAutoRecordingMaxBytes = 512ull * 1024ull * 1024ull;

struct RecordingRetentionPolicy {
	std::size_t maxFiles = kDevAutoRecordingMaxFiles;
	uint64_t maxTotalBytes = kDevAutoRecordingMaxBytes;
};

struct RecordingRetentionPlan {
	std::vector<std::size_t> deleteIndexes;
	std::size_t keptFiles = 0;
	uint64_t keptBytes = 0;
	uint64_t deletedBytes = 0;
};

namespace detail {
	inline uint64_t SaturatingAdd(uint64_t a, uint64_t b) {
		const uint64_t max = std::numeric_limits<uint64_t>::max();
		return (b > max - a) ? max : (a + b);
	}
}

inline RecordingRetentionPlan PlanRecordingRetention(
	const std::vector<LogFileEntry>& newestFirst,
	const RecordingRetentionPolicy& policy) {
	RecordingRetentionPlan plan;

	for (std::size_t i = 0; i < newestFirst.size(); ++i) {
		const auto& entry = newestFirst[i];

		bool keep = false;
		if (policy.maxFiles > 0 && plan.keptFiles < policy.maxFiles && policy.maxTotalBytes > 0) {
			bool fitsByteBudget = false;
			if (plan.keptBytes <= policy.maxTotalBytes) {
				const uint64_t remainingBytes = policy.maxTotalBytes - plan.keptBytes;
				fitsByteBudget = entry.sizeBytes <= remainingBytes;
			}
			// Keep the newest recording even if a single capture exceeds the byte cap.
			if (!fitsByteBudget && plan.keptFiles == 0) {
				fitsByteBudget = true;
			}
			keep = fitsByteBudget;
		}

		if (keep) {
			++plan.keptFiles;
			plan.keptBytes = detail::SaturatingAdd(plan.keptBytes, entry.sizeBytes);
		} else {
			plan.deleteIndexes.push_back(i);
			plan.deletedBytes = detail::SaturatingAdd(plan.deletedBytes, entry.sizeBytes);
		}
	}

	return plan;
}

struct RecordingPruneResult {
	std::size_t totalFiles = 0;
	uint64_t totalBytes = 0;
	std::size_t deletedFiles = 0;
	uint64_t freedBytes = 0;
	std::size_t failedDeletes = 0;
	std::size_t keptFiles = 0;
	uint64_t keptBytes = 0;
};

std::vector<LogFileEntry> ListRecordings();
RecordingPruneResult PruneRecordings(const RecordingRetentionPolicy& policy = RecordingRetentionPolicy{});

} // namespace spacecal::replay
