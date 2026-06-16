#pragma once

// Loads a captured "spacecal_log_v2/v3/v4" CSV (the same file format the debug logger
// emits to %LocalAppDataLow%\SpaceCalibrator\Logs\) into memory and replays it
// through a fresh CalibrationCalc. The intent: a user records a problematic
// motion sequence, we ship a fix, the user replays the same recording against
// the new build to see whether the fix changed behavior.
//
// Parsing is column-name based, so adding new columns to the live log is
// backward compatible; only the raw-pose columns are required. The header line
// must begin with `# spacecal_log_v2`, `# spacecal_log_v3`, or
// `# spacecal_log_v4`; older v1 captures lacked the raw poses and can't be
// replayed. v3 adds per-row sample-health columns so replay can evaluate
// tracking contamination instead of assuming every restored sample was healthy.
// v4 adds the raw HMD + head-mount tracker poses and a reloc flag so the
// locked-style snap-recovery toggle can be A/B-confirmed offline.

#include "CalibrationCalc.h"
#include "TrackingStyle.h" // TrackingStyle enum for the locked-snap replay option

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace spacecal::replay {

enum class ReplayRelocSource
{
	Auto,
	Annotation,
	Column,
	Proxy,
};

// One replayable tick. Produced by parsing a v2 log row.
struct ReplayRow
{
	double timestamp = 0.0;
	Pose ref;
	Pose target;
	Sample sample;
	bool hasSampleDiagnostics = false;
	bool sampleObserved = true;
	bool sampleAccepted = true;
	std::string tickPhase; // "Continuous", "Begin", etc — informational only.

	// v4 locked-snap inputs (world space). hasHmdPose / headTrackerValid are false
	// for v2/v3 rows; the locked-snap replay path is skipped unless they're present.
	Pose hmd;
	Pose headTracker;
	bool hasHmdPose = false;       // raw HMD pose column present + parsed this row
	bool headTrackerValid = false; // head-mount tracker reported a running pose this row
	bool relocDetected = false;    // HMD relocalization detector fired this row
};

// Parsed file metadata: header annotations the live logger emits up-front
// (build stamp, HMD model, OS version). Useful for the UI to show "this
// recording came from build X on hardware Y".
struct RecordingMeta
{
	std::string buildStamp;   // e.g. "2026.4.28.2"
	std::string buildChannel; // "release" or "dev"
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
struct LoadedRecording
{
	RecordingMeta meta;
	int formatVersion = 0;
	std::vector<ReplayRow> rows;
	std::string sourcePath; // path passed to LoadRecording, for display
	std::string error;      // populated on failure; rows will be empty.
	std::vector<double> relocalizationAnnotationTimes;
	bool hasRelocDetectedColumn = false;
	int relocDetectedRowCount = 0;
	// True when the v4 raw-HMD + head-tracker columns are present, so the
	// locked-snap A/B path can run. v2/v3 recordings leave this false and the
	// replay reports "locked_snap_replay_requires_v4" instead of counting snaps.
	bool hasLockedSnapColumns = false;
};

// Parse a v2 log file from disk. Returns LoadedRecording with `error` empty
// on success; any failure populates `error` with a human-readable reason.
LoadedRecording LoadRecording(const std::string& path);

// Per-row trace produced during a replay. Lets the UI plot the error/accept
// trajectory and show histograms of rejection reasons. Kept compact — one
// entry per replayed tick — so even a 60s recording at 60Hz is only ~3600
// entries.
struct ReplayTickResult
{
	double timestamp = 0.0;
	bool accepted = false;        // ComputeIncremental returned true
	double currentCalErrMm = 0.0; // RMS error against the (running) prior calibration, mm
	double rawErrMm = 0.0;        // RMS error of the candidate computed this tick, mm (NaN if no candidate)
	int consecutiveRejections = 0;
	std::string rejectReason; // empty on accept
};

struct ReplayQualitySnapshot
{
	bool available = false;
	bool shadowWouldAccept = false;
	std::string shadowRejectReason;
	int sampleCount = 0;
	int validSampleCount = 0;
	int strictHealthySampleCount = 0;
	int staleSampleCount = 0;
	int jumpSampleCount = 0;
	int zeroPoseSampleCount = 0;
	int unchangedPoseSampleCount = 0;
	int highMotionSampleCount = 0;
	int deltaPair23Count = 0;
	double rmsMm = 0.0;
	double p95Mm = 0.0;
	double holdoutRmsMm = 0.0;
	double holdoutP90Mm = 0.0;
	double holdoutP95Mm = 0.0;
	double targetSpanM = 0.0;
	double rotationSpanDeg = 0.0;
	int validRotationPairCount = 0;
	int translationRank = 0;
	double translationConditionRatio = 0.0;
	double dynamicLimitMm = 0.0;
	double outlierFraction = 0.0;
	double maxPoseAgeMs = 0.0;
	double maxPoseGapMs = 0.0;
	double maxLinearSpeedMps = 0.0;
	double maxAngularSpeedDegps = 0.0;
	bool legacyRmsPass = false;
	bool strictSamplesPass = false;
	bool geometryPass = false;
	bool robustResidualPass = false;
	bool holdoutPass = false;
	bool trackingHealthPass = false;
	bool novaDeltaPairsPass = false;
};

struct ReplayReasonCount
{
	std::string reason;
	int count = 0;
};

// Replay parameters. Mirror the user-facing knobs in the live CalCtx so
// the user can A/B compare "what would my recording look like with a tighter
// recalibration threshold".
struct ReplayOptions
{
	bool continuous = true; // false -> single ComputeOneshot at end
	double threshold = 1.5; // continuousCalibrationThreshold
	double maxRelError = 0.005;
	bool ignoreOutliers = true;
	std::size_t maxContinuousSamples = 200; // 0 keeps every sample; live continuous mode uses a bounded window.
	std::size_t qualityReportInterval = 50; // 0 disables periodic shadow-quality snapshots.
	bool includeHoldoutQuality = false;

	// --- Experimental drift-fighting guards, applied during replay so a recorded
	// session can be A/B compared with each guard on vs off. All default off ->
	// byte-identical to the legacy replay. See RelocGuard.h / DriftBreaker.h /
	// BoundedSolve.h.
	bool applyRelocQuarantine = false; // toggle 1
	double quarantineSec = 1.0;
	bool applyDriftBreaker = false; // toggle 2
	double driftBreakerMadMult = 8.0;
	double driftBreakerAbsCapMm = 60.0;
	bool applyBoundedSolve = false; // toggle 3 parent
	bool bsPrior = false;
	double bsPriorLambda = 0.2;
	bool bsSlew = false;
	double bsMaxStepMm = 50.0;
	double bsMaxStepDeg = 2.0;
	bool bsCommonMode = false;
	// Relative-pose-jump proxy threshold (m): a jump in ref^-1*target larger than
	// this marks a relocalization for the quarantine/breaker, derived purely from
	// the recorded ref/target poses, so v3 recordings (no raw HMD pose) work too.
	double relocProxyJumpM = 0.05;
	ReplayRelocSource relocSource = ReplayRelocSource::Auto;

	// Toggle 4 (locked-style snap recovery). When on AND the recording is v4 (raw
	// HMD + head-tracker columns present), the replay reproduces the live snap
	// corroboration: a >=30 cm HMD jump confirmed by a <2 cm head-tracker
	// displacement is a Quest universe flip, and in a locked tracking style the
	// experimental toggle opens the gentle re-anchor that the live styleOK gate
	// otherwise skips. Each such row increments ReplayResult::snapReanchors.
	// trackingStyle is the style the A/B assumes; the gentle path only opens for
	// LockedWithRecovery / HardTrackerLock (GentleSnapAllowedInLockedStyle).
	bool applyLockedSnap = false;
	TrackingStyle trackingStyle = TrackingStyle::LockedWithRecovery;
};

// Result summary. Aggregates whatever is useful at a glance — counts and the
// final transform — plus the full per-tick trace for visualisation.
struct ReplayResult
{
	bool succeeded = false;
	int rowsReplayed = 0;
	int accepts = 0;
	int rejects = 0;
	int watchdogResets = 0;
	int maxSamplesInWindow = 0;
	int sampleRowsObserved = 0;
	int sampleRowsAccepted = 0;
	int sampleRowsRejected = 0;
	int sampleRowsStrictUnhealthy = 0;
	int sampleRowsStale = 0;
	int sampleRowsJump = 0;
	int sampleRowsPairedMotionInvalid = 0;
	int sampleRowsZeroPose = 0;
	int sampleRowsUnchanged = 0;
	int sampleRowsHighMotion = 0;
	int qualityReports = 0;
	int shadowWouldAccept = 0;
	int shadowWouldReject = 0;
	double finalErrorMm = 0.0; // NaN if calc never produced a valid result
	// Relative-pose dispersion across the replay (the AUTO-lock translMad analog),
	// in mm -- the headline drift signal for A/B-ing the experimental guards --
	// plus how often each guard engaged. peak/median/final summarise the
	// per-row MAD trajectory over the bounded relative-pose window.
	double peakRelPoseMadMm = 0.0;
	double medianRelPoseMadMm = 0.0;
	double finalRelPoseMadMm = 0.0;
	int samplesQuarantined = 0;
	int freezeEngagements = 0;
	int snapReanchors = 0;
	int relocEvents = 0;
	std::string relocSource = "off";
	// Locked-snap A/B status: "off" (toggle not requested), "applied" (ran on a v4
	// recording), or "locked_snap_replay_requires_v4" (requested but the recording
	// lacks the raw HMD + head-tracker columns, so snapReanchors stays 0).
	std::string lockedSnapStatus = "off";
	Eigen::AffineCompact3d finalTransform = Eigen::AffineCompact3d::Identity();
	bool finalTransformValid = false;
	ReplayQualitySnapshot finalQuality;
	std::vector<ReplayReasonCount> shadowRejectReasons;
	std::vector<ReplayTickResult> trace;
	std::vector<ReplayQualitySnapshot> qualityTrace;
	std::string error; // populated on failure
};

// Run the replay synchronously. Continuous replay uses a bounded sample window
// so long captures exercise the same solver shape as the live refiner instead
// of growing an unbounded history.
ReplayResult RunReplay(const LoadedRecording& rec, const ReplayOptions& opts);

// List recent v2 log files in the user's logs directory, newest first. Used
// by the UI to populate a "pick a recording" dropdown without forcing the
// user to copy-paste paths.
struct LogFileEntry
{
	std::string name;      // file name only
	std::wstring fullPath; // absolute path for opening
	uint64_t sizeBytes = 0;
	uint64_t mtimeFileTime = 0; // FILETIME as uint64; for sorting
};

constexpr std::size_t kDevAutoRecordingMaxFiles = 5;
constexpr uint64_t kDevAutoRecordingMaxBytes = 512ull * 1024ull * 1024ull;

struct RecordingRetentionPolicy
{
	std::size_t maxFiles = kDevAutoRecordingMaxFiles;
	uint64_t maxTotalBytes = kDevAutoRecordingMaxBytes;
};

struct RecordingRetentionPlan
{
	std::vector<std::size_t> deleteIndexes;
	std::size_t keptFiles = 0;
	uint64_t keptBytes = 0;
	uint64_t deletedBytes = 0;
};

namespace detail {
inline uint64_t SaturatingAdd(uint64_t a, uint64_t b)
{
	const uint64_t max = std::numeric_limits<uint64_t>::max();
	return (b > max - a) ? max : (a + b);
}
} // namespace detail

inline RecordingRetentionPlan PlanRecordingRetention(const std::vector<LogFileEntry>& newestFirst,
                                                     const RecordingRetentionPolicy& policy)
{
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
		}
		else {
			plan.deleteIndexes.push_back(i);
			plan.deletedBytes = detail::SaturatingAdd(plan.deletedBytes, entry.sizeBytes);
		}
	}

	return plan;
}

struct RecordingPruneResult
{
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
