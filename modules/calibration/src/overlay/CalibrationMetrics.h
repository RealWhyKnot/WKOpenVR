#pragma once

#include "BuildChannel.h"

#include <cstdint>
#include <deque>
#include <string>
#include <utility>
#include <Eigen/Dense>

namespace Metrics {
extern double TimeSpan, CurrentTime;
extern bool enableLogs;
#if WKOPENVR_BUILD_IS_DEV
extern bool enableReplayCsv;
#endif

double timestamp();
void RecordTimestamp();

template <typename T> class TimeSeries
{
	std::deque<std::pair<double, T>> Data;

public:
	const std::deque<std::pair<double, T>>& data() const { return Data; }

	void Push(const T& data)
	{
		Data.push_back(std::make_pair(CurrentTime, data));

		// Time-based cutoff: drop anything older than `TimeSpan` seconds.
		// Defensive size cap (kMaxPoints): if CurrentTime stalls or rolls
		// backward (clock change, replay tool quirks, NaN propagation) the
		// time-based loop won't bound the deque. 4096 points at 60Hz is
		// ~68 seconds — plenty of headroom over the 30s default TimeSpan
		// without letting growth run away if something goes wrong upstream.
		constexpr size_t kMaxPoints = 4096;
		const double cutoff = CurrentTime - TimeSpan;
		while (!Data.empty() && (Data.front().first < cutoff || Data.size() > kMaxPoints)) {
			Data.pop_front();
		}
	}

	int size() const { return (int)Data.size(); }
	const std::pair<double, T>& operator[](int index) const { return Data[index]; }

	const T& last() const
	{
		static const T fallback{};
		return Data.size() > 0 ? Data.back().second : fallback;
	}

	const double lastTs() const { return Data.size() > 0 ? Data.back().first : 0; }

	// Drop every retained sample. Used at calibration-restart sites so the
	// post-restart rolling window doesn't see values from the previous
	// cycle (which would let pre-restart spikes feed back into spike-vs-
	// median tests on the first few samples of the new cycle).
	void Clear() { Data.clear(); }
};

extern TimeSeries<Eigen::Vector3d> posOffset_rawComputed; // , rotOffset_rawComputed;
extern TimeSeries<Eigen::Vector3d> posOffset_currentCal;  // , rotOffset_currentCal;
extern TimeSeries<Eigen::Vector3d> posOffset_lastSample;  // , rotOffset_lastSample;
extern TimeSeries<Eigen::Vector3d> posOffset_byRelPose;

extern TimeSeries<double> error_rawComputed, error_currentCal, error_byRelPose, error_currentCalRelPose;
extern TimeSeries<double> axisIndependence;
extern TimeSeries<double> computationTime;
extern TimeSeries<double> jitterRef, jitterTarget;

// Smallest/largest singular value ratio of the 2D Kabsch yaw covariance.
// Drops near zero when the user moves on a single axis only.
extern TimeSeries<double> rotationConditionRatio;

// Running count of ComputeIncremental rejections since the last successful
// accept. The stuck-loop watchdog fires when this exceeds its threshold.
extern TimeSeries<double> consecutiveRejections;

// Number of pose samples currently in CalibrationCalc's deque. Helps debug
// the post-watchdog "garbage" failure mode where Clear() drops the buffer
// and a recompute fires before enough fresh samples have accumulated. Also
// lets you correlate "calibration looked good then went off" against a
// specific buffer churn event.
extern TimeSeries<double> samplesInBuffer;

// Motion-coverage scores for the live sample buffer (0..1 each). Surfaced
// in the Calibration Progress popup as live progress bars during one-shot
// collection so the user knows whether their figure-8 wave has produced
// the variety the math needs. Pushed each tick from CollectSample.
extern TimeSeries<double> translationDiversity;
extern TimeSeries<double> rotationDiversity;

// Per-axis bounding-box ranges (centimetres) of the target tracker across
// the live sample buffer. The "Translation coverage" tooltip uses this to
// tell the user which axis (X / Y / Z) is the bottleneck when the bar is
// stuck below 100 %. Whichever component is smallest IS what's pinning
// translationDiversity (= min component / 20 cm). Pushed each tick from
// CollectSample alongside the scalar diversity metrics.
extern TimeSeries<Eigen::Vector3d> translationAxisRangesCm;

// Rolling count of recent samples where the reference and target devices
// moved in disagreement -- one stepped meaningfully while the other was
// effectively stationary. The classic case is the user's headset being
// frozen by a passthrough/desktop overlay while the target tracker keeps
// reporting motion: the math has no usable correspondence in that state,
// but the old diversity bars would still fill from raw target motion.
// The popup reads this each frame and surfaces a warning banner when it
// rises, so the user gets real-time feedback that their data is bad
// rather than discovering it post-solve.
extern TimeSeries<double> pairedMotionWarningCount;

// 1 when the watchdog WANTED to fire (consecutive rejections >= cap and
// m_isValid) but skipped because the prior calibration error is in the
// "healthy" band — i.e. the wedged-at-noise-floor symptom that the
// kHealthyPriorErrorMax constant lets through. Push every tick the
// watchdog evaluates, so a CSV reader can see at a glance whether the
// session is wedged.
extern TimeSeries<double> watchdogHealthySkip;

// The effectivePrior the 1.5× improvement gate compared against this tick
// (millimetres). Equals max(priorCalibrationError*1000, kRejectionFloor*1000)
// — i.e. the value the gate actually used, which can differ from
// error_currentCal by the rejection-floor clamp. Helps explain rejections
// that look puzzling against the raw prior error.
extern TimeSeries<double> effectivePriorMm;

// Adaptive RMS threshold (millimetres) reported by calibration-quality
// diagnostics. Without logging this, "validate_failed" rejections look
// opaque: was the candidate above the measured residual floor, or did a
// geometry term dominate the dynamic limit?
extern TimeSeries<double> validateRmsThresholdMm;

// Cumulative count of stuck-loop watchdog firings. Logged as a per-row
// number so you can grep the CSV for the row index where a fire happened
// even after the # annotation lines are stripped.
extern TimeSeries<double> watchdogResetCount;

// Reason the most recent ComputeIncremental rejected, written as a tag
// string into the "reject_reason" column. Empty when the last call
// accepted. See CalibrationCalc::m_lastRejectReason for the tag set.
extern std::string lastRejectReason;

// Driver-side apply rates (Hz) sampled from the shared-memory telemetry
// counters. Each tick we read the cumulative counter and push the delta
// divided by the elapsed wall time. Lets the overlay confirm that the
// per-tracking-system fallback path is actually firing in the wild.
extern TimeSeries<double> fallbackApplyRate;
extern TimeSeries<double> perIdApplyRate;
extern TimeSeries<double> quashApplyRate;

extern TimeSeries<bool> calibrationApplied;

// Head-mount tracker diagnostics. headMountActive records whether the
// feature is engaged; headMountResidualMm is the per-tick reprojection
// error (mm) between the tracker-derived HMD position and the observed
// HMD pose. questHmdVsProxyDeltaMm tracks disagreement between the Quest
// HMD and the lighthouse-derived proxy -- spikes here indicate a
// worldFromDriver shift that Corroborate mode should suppress.
extern TimeSeries<bool> headMountActive;
extern TimeSeries<double> headMountResidualMm;
extern TimeSeries<double> questHmdVsProxyDeltaMm;
// Snap suppression and driver-synth fallback counts for the head-mount
// Corroborate and DriverSynth paths. Monotonically increasing per session;
// delta between rows gives the per-tick rate.
extern TimeSeries<uint32_t> snapSuppressedCount;
extern TimeSeries<uint32_t> driverSynthFallbackCount;
// Safety boundary state. boundaryActive is true while the simplified
// PC-side boundary is pushed to SteamVR chaperone. chaperoneRePushCount
// counts how many times the boundary was re-pushed after a Quest Guardian
// event clobbered it.
extern TimeSeries<bool> boundaryActive;
extern TimeSeries<uint32_t> chaperoneRePushCount;

struct LogHealth
{
	bool debugEnabled = false;
	bool open = false;
	bool failedToOpen = false;
	bool writeFailed = false;
	bool flushFailed = false;
	bool replayCsvEnabled = false;
	bool devAutoRecording = false;
	std::wstring path;
	long long sizeBytes = -1;
	uint64_t rowsWritten = 0;
	uint64_t annotationsWritten = 0;
	uint64_t openAttempts = 0;
	unsigned long lastErrorCode = 0;
	std::string status;
};

// Phase of the per-tick CalibrationTick state machine at the moment WriteLogEntry()
// is called. Stored in the v2 CSV "tick_phase" column to let the replay harness
// reproduce the same control-flow path that produced each row. Mirrors the
// CalibrationState enum but is held independently so CalibrationMetrics.h doesn't
// have to pull in Calibration.h.
enum class TickPhase
{
	None,
	Begin,
	Rotation,
	Translation,
	Editing,
	Continuous,
	ContinuousStandby,
};

struct ReplaySampleDiagnostics
{
	bool observed = false;
	bool accepted = false;
	bool pairedMotionValid = true;
	bool refDeviceConnected = true;
	bool targetDeviceConnected = true;
	bool refPoseValid = true;
	bool targetPoseValid = true;
	int refTrackingResult = 200;
	int targetTrackingResult = 200;
	double refPoseAgeMs = 0.0;
	double targetPoseAgeMs = 0.0;
	double refPoseGapMs = 0.0;
	double targetPoseGapMs = 0.0;
	double refLinearSpeedMps = 0.0;
	double targetLinearSpeedMps = 0.0;
	double refAngularSpeedRadps = 0.0;
	double targetAngularSpeedRadps = 0.0;
	bool refZeroPose = false;
	bool targetZeroPose = false;
	bool refPoseUnchanged = false;
	bool targetPoseUnchanged = false;
	bool trackingPoseStale = false;
	bool trackingPoseJump = false;
};

// Set the raw reference and target pose (translation + quaternion) and the tick
// phase that will be written by the next WriteLogEntry() call. Caller is expected
// to invoke this once per tick, just before WriteLogEntry(), so the v2 columns
// stay consistent with the metrics already snapshotted into the TimeSeries above.
void SetTickRawPoses(const Eigen::Vector3d& refTrans, const Eigen::Quaterniond& refRot,
                     const Eigen::Vector3d& targetTrans, const Eigen::Quaterniond& targetRot, TickPhase phase);
void SetTickReplaySampleDiagnostics(const ReplaySampleDiagnostics& diagnostics);

void WriteLogAnnotation(const char* s);

// True when diagnostic logging is enabled. Cheap gate (mirrors `enableLogs`, the
// authoritative flag CheckLogOpen uses) so hot per-tick call sites can skip
// building an annotation string that WriteLogAnnotation would otherwise format
// and then drop. Intentionally does NOT also require the file to be open, so the
// logger's lazy open-on-first-write still happens.
bool LoggingEnabled();

// printf-style annotation that formats only when logging is enabled. Equivalent
// to snprintf-into-buffer followed by WriteLogAnnotation, but skips the format
// cost when LoggingEnabled() is false (the release default and dev-toggle-off).
// Use at high-frequency per-tick sites; rare one-shot sites can keep calling
// WriteLogAnnotation directly.
void LogAnnotationf(const char* fmt, ...);

void WriteLogEntry();
bool EnsureLogFileReady(const char* reason = nullptr);
bool FlushLogFile();
LogHealth GetLogHealth();
void WriteLogHealthSnapshot(const char* reason);
void CloseLogFile();
} // namespace Metrics
