#include "CalibrationMetricsSchema.h"

#include "BuildChannel.h"
#include "CalibrationMetrics.h"
#if WKOPENVR_BUILD_IS_DEV
#include "MotionRecording.h"
#endif

#include <ostream>

namespace Metrics {

// v2 wire-format addition: per-tick raw reference and target poses plus the tick
// phase. Filled by SetTickRawPoses() each tick and consumed by the field writers
// below. Defaults are an identity pose with phase=None so that any unexpected
// WriteLogEntry call (i.e. one not preceded by SetTickRawPoses) emits a syntactically
// valid row rather than uninitialized memory. v3 adds sample-health diagnostics
// so offline replays can evaluate the same shadow-quality gates as live sessions.
struct TickRawPoses
{
	Eigen::Vector3d refTrans = Eigen::Vector3d::Zero();
	Eigen::Quaterniond refRot = Eigen::Quaterniond::Identity();
	Eigen::Vector3d targetTrans = Eigen::Vector3d::Zero();
	Eigen::Quaterniond targetRot = Eigen::Quaterniond::Identity();
	TickPhase phase = TickPhase::None;
};
static TickRawPoses g_tickRaw;
static ReplaySampleDiagnostics g_tickSample;
static ReplayLockedSnapInputs g_tickLockedSnap;
static uint32_t g_tickExperimentalFlags = 0;

static const char* TickPhaseName(TickPhase p)
{
	switch (p) {
		case TickPhase::None:
			return "None";
		case TickPhase::Begin:
			return "Begin";
		case TickPhase::Rotation:
			return "Rotation";
		case TickPhase::Translation:
			return "Translation";
		case TickPhase::Editing:
			return "Editing";
		case TickPhase::Continuous:
			return "Continuous";
		case TickPhase::ContinuousStandby:
			return "ContinuousStandby";
	}
	return "None";
}

void SetTickRawPoses(const Eigen::Vector3d& refTrans, const Eigen::Quaterniond& refRot,
                     const Eigen::Vector3d& targetTrans, const Eigen::Quaterniond& targetRot, TickPhase phase)
{
	g_tickRaw.refTrans = refTrans;
	g_tickRaw.refRot = refRot;
	g_tickRaw.targetTrans = targetTrans;
	g_tickRaw.targetRot = targetRot;
	g_tickRaw.phase = phase;
}

void SetTickReplaySampleDiagnostics(const ReplaySampleDiagnostics& diagnostics)
{
	g_tickSample = diagnostics;
}

void SetTickLockedSnapInputs(const ReplayLockedSnapInputs& inputs)
{
	g_tickLockedSnap = inputs;
}

void SetTickExperimentalFlags(uint32_t flags)
{
	g_tickExperimentalFlags = flags;
}

#define TS_FIELD(n) {#n, [](auto& s) { s << n.last(); }}

#define TS_VECTOR_FIELD(n)                                                                                             \
	{#n ".x", [](auto& s) { s << n.last()(0); }}, {#n ".y", [](auto& s) { s << n.last()(1); }},                        \
	{                                                                                                                  \
		#n ".z", [](auto& s) {                                                                                         \
			s << n.last()(2);                                                                                          \
		}                                                                                                              \
	}

#if WKOPENVR_BUILD_IS_DEV
static const CsvField fields[] = {
    {"Timestamp", [](auto& s) { s << CurrentTime; }},

    TS_VECTOR_FIELD(posOffset_rawComputed),
    TS_VECTOR_FIELD(posOffset_currentCal),
    TS_VECTOR_FIELD(posOffset_lastSample),
    TS_VECTOR_FIELD(posOffset_byRelPose),

    TS_FIELD(error_rawComputed),
    TS_FIELD(error_currentCal),
    TS_FIELD(error_byRelPose),
    TS_FIELD(error_currentCalRelPose),
    TS_FIELD(axisIndependence),
    TS_FIELD(rotationConditionRatio),
    TS_FIELD(consecutiveRejections),
    TS_FIELD(samplesInBuffer),
    // Motion-coverage scores for the live sample buffer (0..1 each). Pushed by
    // CollectSample. The Calibration Progress popup reads these for the live
    // "Translation %" / "Rotation %" bars; logging them here lets post-hoc
    // triage see exactly what the bars showed when a one-shot calibration
    // got stuck below the auto-finish threshold.
    TS_FIELD(translationDiversity),
    TS_FIELD(rotationDiversity),
    TS_VECTOR_FIELD(translationAxisRangesCm),
    TS_FIELD(pairedMotionWarningCount),
    // Wedge-detection diagnostics. watchdogHealthySkip flags ticks where the
    // watchdog wanted to clear but couldn't (prior in healthy band).
    // effectivePriorMm is the actual prior the 1.5× gate compared against.
    // validateRmsThresholdMm is the dynamic noise-floor threshold the
    // validate gate used this tick.
    TS_FIELD(watchdogHealthySkip),
    TS_FIELD(effectivePriorMm),
    TS_FIELD(validateRmsThresholdMm),
    TS_FIELD(watchdogResetCount),
    TS_FIELD(computationTime),
    TS_FIELD(jitterRef),
    TS_FIELD(jitterTarget),
    TS_FIELD(fallbackApplyRate),
    TS_FIELD(perIdApplyRate),
    TS_FIELD(quashApplyRate),
    // String-valued reject reason. Empty when the last ComputeIncremental
    // accepted; otherwise one of: "below_floor_or_worse", "axis_variance_low",
    // "rotation_planar", "rotation_no_deltas", "translation_planar",
    // "translation_no_deltas", "validate_failed", "healthy_below_floor".
    {"reject_reason", [](auto& s) { s << lastRejectReason; }},

    {"calibrationApplied",
     [](auto& s) {
	     if (calibrationApplied.lastTs() == CurrentTime) {
		     if (calibrationApplied.last()) {
			     s << "FULL";
		     }
		     else {
			     s << "STATIC";
		     }
	     }
     }},

    // --- v2 columns: raw reference + target poses and tick phase ---------------
    // Translations are in meters, rotations are unit quaternions in (w,x,y,z) order.
    // These are written with full double precision so the replay harness can
    // reconstruct the exact `Sample` values that fed CalibrationCalc::PushSample.
    {"ref_tx",
     [](auto& s) {
	     s.precision(17);
	     s << g_tickRaw.refTrans.x();
     }},
    {"ref_ty",
     [](auto& s) {
	     s.precision(17);
	     s << g_tickRaw.refTrans.y();
     }},
    {"ref_tz",
     [](auto& s) {
	     s.precision(17);
	     s << g_tickRaw.refTrans.z();
     }},
    {"ref_qw",
     [](auto& s) {
	     s.precision(17);
	     s << g_tickRaw.refRot.w();
     }},
    {"ref_qx",
     [](auto& s) {
	     s.precision(17);
	     s << g_tickRaw.refRot.x();
     }},
    {"ref_qy",
     [](auto& s) {
	     s.precision(17);
	     s << g_tickRaw.refRot.y();
     }},
    {"ref_qz",
     [](auto& s) {
	     s.precision(17);
	     s << g_tickRaw.refRot.z();
     }},
    {"tgt_tx",
     [](auto& s) {
	     s.precision(17);
	     s << g_tickRaw.targetTrans.x();
     }},
    {"tgt_ty",
     [](auto& s) {
	     s.precision(17);
	     s << g_tickRaw.targetTrans.y();
     }},
    {"tgt_tz",
     [](auto& s) {
	     s.precision(17);
	     s << g_tickRaw.targetTrans.z();
     }},
    {"tgt_qw",
     [](auto& s) {
	     s.precision(17);
	     s << g_tickRaw.targetRot.w();
     }},
    {"tgt_qx",
     [](auto& s) {
	     s.precision(17);
	     s << g_tickRaw.targetRot.x();
     }},
    {"tgt_qy",
     [](auto& s) {
	     s.precision(17);
	     s << g_tickRaw.targetRot.y();
     }},
    {"tgt_qz",
     [](auto& s) {
	     s.precision(17);
	     s << g_tickRaw.targetRot.z();
     }},
    {"tick_phase", [](auto& s) { s << TickPhaseName(g_tickRaw.phase); }},

    // --- v3 columns: accepted/rejected sample health -------------------------
    {"sample_observed", [](auto& s) { s << (g_tickSample.observed ? 1 : 0); }},
    {"sample_accepted", [](auto& s) { s << (g_tickSample.accepted ? 1 : 0); }},
    {"sample_paired_motion_valid", [](auto& s) { s << (g_tickSample.pairedMotionValid ? 1 : 0); }},
    {"sample_ref_connected", [](auto& s) { s << (g_tickSample.refDeviceConnected ? 1 : 0); }},
    {"sample_tgt_connected", [](auto& s) { s << (g_tickSample.targetDeviceConnected ? 1 : 0); }},
    {"sample_ref_pose_valid", [](auto& s) { s << (g_tickSample.refPoseValid ? 1 : 0); }},
    {"sample_tgt_pose_valid", [](auto& s) { s << (g_tickSample.targetPoseValid ? 1 : 0); }},
    {"sample_ref_tracking_result", [](auto& s) { s << g_tickSample.refTrackingResult; }},
    {"sample_tgt_tracking_result", [](auto& s) { s << g_tickSample.targetTrackingResult; }},
    {"sample_ref_age_ms",
     [](auto& s) {
	     s.precision(17);
	     s << g_tickSample.refPoseAgeMs;
     }},
    {"sample_tgt_age_ms",
     [](auto& s) {
	     s.precision(17);
	     s << g_tickSample.targetPoseAgeMs;
     }},
    {"sample_ref_gap_ms",
     [](auto& s) {
	     s.precision(17);
	     s << g_tickSample.refPoseGapMs;
     }},
    {"sample_tgt_gap_ms",
     [](auto& s) {
	     s.precision(17);
	     s << g_tickSample.targetPoseGapMs;
     }},
    {"sample_ref_speed_mps",
     [](auto& s) {
	     s.precision(17);
	     s << g_tickSample.refLinearSpeedMps;
     }},
    {"sample_tgt_speed_mps",
     [](auto& s) {
	     s.precision(17);
	     s << g_tickSample.targetLinearSpeedMps;
     }},
    {"sample_ref_ang_speed_radps",
     [](auto& s) {
	     s.precision(17);
	     s << g_tickSample.refAngularSpeedRadps;
     }},
    {"sample_tgt_ang_speed_radps",
     [](auto& s) {
	     s.precision(17);
	     s << g_tickSample.targetAngularSpeedRadps;
     }},
    {"sample_ref_zero_pose", [](auto& s) { s << (g_tickSample.refZeroPose ? 1 : 0); }},
    {"sample_tgt_zero_pose", [](auto& s) { s << (g_tickSample.targetZeroPose ? 1 : 0); }},
    {"sample_ref_unchanged", [](auto& s) { s << (g_tickSample.refPoseUnchanged ? 1 : 0); }},
    {"sample_tgt_unchanged", [](auto& s) { s << (g_tickSample.targetPoseUnchanged ? 1 : 0); }},
    {"sample_stale", [](auto& s) { s << (g_tickSample.trackingPoseStale ? 1 : 0); }},
    {"sample_jump", [](auto& s) { s << (g_tickSample.trackingPoseJump ? 1 : 0); }},

    // --- v4 columns: locked-snap corroboration inputs ------------------------
    // Raw HMD pose, head-mount tracker pose (+ valid flag) and a reloc-detected
    // flag. The replay harness computes the per-row HMD jump and head-tracker
    // displacement from these to reproduce the snap classification the
    // locked-style snap-recovery toggle depends on. World space, full precision.
    {"hmd_tx",
     [](auto& s) {
	     s.precision(17);
	     s << g_tickLockedSnap.hmdTrans.x();
     }},
    {"hmd_ty",
     [](auto& s) {
	     s.precision(17);
	     s << g_tickLockedSnap.hmdTrans.y();
     }},
    {"hmd_tz",
     [](auto& s) {
	     s.precision(17);
	     s << g_tickLockedSnap.hmdTrans.z();
     }},
    {"hmd_qw",
     [](auto& s) {
	     s.precision(17);
	     s << g_tickLockedSnap.hmdRot.w();
     }},
    {"hmd_qx",
     [](auto& s) {
	     s.precision(17);
	     s << g_tickLockedSnap.hmdRot.x();
     }},
    {"hmd_qy",
     [](auto& s) {
	     s.precision(17);
	     s << g_tickLockedSnap.hmdRot.y();
     }},
    {"hmd_qz",
     [](auto& s) {
	     s.precision(17);
	     s << g_tickLockedSnap.hmdRot.z();
     }},
    {"head_tracker_valid", [](auto& s) { s << (g_tickLockedSnap.headTrackerValid ? 1 : 0); }},
    {"head_tracker_tx",
     [](auto& s) {
	     s.precision(17);
	     s << g_tickLockedSnap.headTrackerTrans.x();
     }},
    {"head_tracker_ty",
     [](auto& s) {
	     s.precision(17);
	     s << g_tickLockedSnap.headTrackerTrans.y();
     }},
    {"head_tracker_tz",
     [](auto& s) {
	     s.precision(17);
	     s << g_tickLockedSnap.headTrackerTrans.z();
     }},
    {"head_tracker_qw",
     [](auto& s) {
	     s.precision(17);
	     s << g_tickLockedSnap.headTrackerRot.w();
     }},
    {"head_tracker_qx",
     [](auto& s) {
	     s.precision(17);
	     s << g_tickLockedSnap.headTrackerRot.x();
     }},
    {"head_tracker_qy",
     [](auto& s) {
	     s.precision(17);
	     s << g_tickLockedSnap.headTrackerRot.y();
     }},
    {"head_tracker_qz",
     [](auto& s) {
	     s.precision(17);
	     s << g_tickLockedSnap.headTrackerRot.z();
     }},
    {"reloc_detected", [](auto& s) { s << (g_tickLockedSnap.relocDetected ? 1 : 0); }},

    // --- v5 columns: experimental option state -----------------------------
    {"experimental_flags", [](auto& s) { s << g_tickExperimentalFlags; }},
};
#endif

const CsvField* CsvSchemaFields(std::size_t& count)
{
#if WKOPENVR_BUILD_IS_DEV
	count = sizeof fields / sizeof fields[0];
	return fields;
#else
	count = 0;
	return nullptr;
#endif
}

void ResetTickReplaySnapshots()
{
	g_tickSample = ReplaySampleDiagnostics{};
	g_tickLockedSnap = ReplayLockedSnapInputs{};
	g_tickExperimentalFlags = 0;
}

} // namespace Metrics
