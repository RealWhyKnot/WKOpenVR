#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Windows.h>
#include <openvr.h>
#include <vector>
#include <deque>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>

#include "Protocol.h"
#include "HeadMountDriverSynthConfig.h"
// We hold a unique_ptr<CalibrationCalc> in AdditionalCalibration. unique_ptr's
// implicit destructor needs the pointee type complete at the destructor's
// site, including in any TU that destroys an instance (test/replay stubs that
// declare a global CalibrationContext do, transitively). Pulling the full
// header in here -- rather than forward-declaring + defining the destructor
// in Calibration.cpp -- keeps the destructor available everywhere without
// dragging the stubs through extra link steps.
#include "CalibrationCalc.h"

enum class CalibrationState
{
	None,
	Begin,
	Rotation,
	Translation,
	Editing,
	Continuous,
	ContinuousStandby,
};

// Persistent identity for a tracked device. Used to re-resolve the live
// vr device ID after restart / reconnection by matching serial. Defined
// here (above AdditionalCalibration) so the additional-calibration struct
// can hold one without a forward-decl dance.
struct StandbyDevice
{
	std::string trackingSystem;
	std::string model, serial;
};

// One additional calibration entry for the multi-ecosystem case. Each
// AdditionalCalibration aligns one non-HMD tracking system to the HMD's
// tracking system, independently of the "primary" calibration carried in the
// singular fields of CalibrationContext.
//
// Example: a user with a Quest HMD + SlimeVR body trackers + a single Vive
// tracker glued to the headset has the SlimeVR alignment as the primary, and
// the Vive alignment as one entry in additionalCalibrations.
//
// Each entry runs its own continuous-calibration loop with its own sample
// buffer (the unique_ptr<CalibrationCalc> is per-entry). The driver sees them
// via per-tracking-system fallback transforms -- it doesn't care how many
// entries the overlay tracks; it just applies whatever fallbacks arrive over
// IPC, one per tracking system name.
struct AdditionalCalibration
{
	// Identification.
	std::string targetTrackingSystem;

	// Standby record for the target device, used to re-resolve targetID
	// after restart / device reconnection (matched by serial). Reference
	// standby isn't stored here -- the reference is always the HMD, looked
	// up at scan time, same as the primary calibration.
	StandbyDevice targetStandby;

	// Live IDs, refreshed each scan tick.
	int32_t referenceID = -1;
	int32_t targetID = -1;

	// Calibration result, in the same units as the primary.
	Eigen::Vector3d calibratedRotation = Eigen::Vector3d::Zero();
	Eigen::Vector3d calibratedTranslation = Eigen::Vector3d::Zero();
	double calibratedScale = 1.0;
	Eigen::Vector3d continuousCalibrationOffset = Eigen::Vector3d::Zero();

	// Per-extra lock + relative-pose state. Same semantics as the primary's
	// lockRelativePositionMode and friends.
	int lockMode = 0; // 0=OFF, 1=ON, 2=legacy AUTO (treated as OFF). int instead of
	                  // LockMode to keep the forward-decl situation simple here.
	Eigen::AffineCompact3d refToTargetPose = Eigen::AffineCompact3d::Identity();
	bool relativePosCalibrated = false;

	// Resolved (effective) lock state -- ResolveLockMode mirror for extras.
	bool lockRelativePosition = false;

	// Per-extra math state. Pointer (not value) so the type can stay
	// forward-declared in this header. Constructed lazily in Calibration.cpp.
	std::unique_ptr<CalibrationCalc> calc;

	// True once a calibration has been computed for this entry. Until then,
	// no fallback is sent for this tracking system.
	bool valid = false;

	// True when this entry is participating in continuous mode. Set once
	// the wizard finishes calibrating the entry; cleared when the user
	// removes the entry.
	bool enabled = true;

	// Defaulted in-class because CalibrationCalc is a complete type at this
	// point (we included its header above), so unique_ptr's destructor is
	// inlinable. Out-of-line definitions in Calibration.cpp would only show
	// up to TUs that link the overlay -- the test/replay stubs that pull
	// Calibration.h transitively need the destructor available everywhere.
	AdditionalCalibration() : calc(std::make_unique<CalibrationCalc>()) {}
	~AdditionalCalibration() = default;
	AdditionalCalibration(const AdditionalCalibration&) = delete;
	AdditionalCalibration& operator=(const AdditionalCalibration&) = delete;
	AdditionalCalibration(AdditionalCalibration&&) noexcept = default;
	AdditionalCalibration& operator=(AdditionalCalibration&&) noexcept = default;
};

// Operating mode for the head-mounted tracker feature. Off disables the
// entire subsystem. Higher modes are cumulative: DriverSynth implies
// Corroborate which implies AutoPaired.
enum class HeadMountMode : uint8_t
{
	Off = 0,
	AutoPaired = 1,
	Corroborate = 2,
	DriverSynth = 3,
};

enum class HeadMountSampleSource : uint8_t
{
	Unknown = 0,
	PhysicalTracker,
	HeadProxy,
};

enum class TrackingStyle : uint8_t
{
	Manual = 0,
	Continuous = 1,
	LockedWithRecovery = 2,
	HardTrackerLock = 3,
};

// Identity and calibration for a head-mounted tracker (e.g. a Vive tracker
// zip-tied to a Quest headset). headFromTracker is the rigid offset from the
// tracker's local frame to the HMD's local frame, solved by the offset
// calibration wizard.
struct HeadMountConfig
{
	HeadMountMode mode = HeadMountMode::Off;
	std::string trackerSerial;
	std::string trackerModel; // persisted; needed for VRState::FindDevice
	std::string trackerTrackingSystem;
	Eigen::AffineCompact3d headFromTracker = Eigen::AffineCompact3d::Identity();
	bool hideTracker = true;
	bool offsetCalibrated = false;
	// True only when offsetCalibrated was auto-captured rather than set by the
	// manual offset wizard. Kept for persisted-profile provenance; nothing
	// auto-captures anymore, so new profiles only ever write false.
	bool offsetWitnessAutoCaptured = false;
	bool autoCorrectOffset = true;
	bool allowRawHmdFallback = true;
	// Speed-adaptive low-pass on the synthesized HMD pose when locked to the
	// head-mounted tracker (0..100, 0 = off). Tames lighthouse position jitter.
	uint8_t lockedHeadsetSmoothing = 0;
	uint8_t lockedHeadsetRotationSmoothing = 0;
	wkopenvr::headmount::DriverSynthTimingConfig driverSynthTiming;
	// Runtime-resolved OpenVR device ID; not persisted. -1 means unresolved.
	// Set each AssignTargets() call by matching trackerSerial + trackerTrackingSystem.
	int32_t deviceID = -1;
};

struct CalibrationProfileSnapshot
{
	bool captured = false;
	bool enabled = false;
	bool validProfile = false;
	std::string referenceTrackingSystem;
	std::string targetTrackingSystem;
	StandbyDevice referenceStandby;
	StandbyDevice targetStandby;
	Eigen::Vector3d calibratedRotation = Eigen::Vector3d::Zero();
	Eigen::Vector3d calibratedTranslation = Eigen::Vector3d::Zero();
	double calibratedScale = 1.0;
	Eigen::AffineCompact3d refToTargetPose = Eigen::AffineCompact3d::Identity();
	bool relativePosCalibrated = false;
};

struct CalibrationContext
{
	CalibrationState state = CalibrationState::None;
	int32_t referenceID = -1, targetID = -1;

	static const size_t MAX_CONTROLLERS = 8;
	int32_t controllerIDs[MAX_CONTROLLERS];

	StandbyDevice targetStandby, referenceStandby;

	Eigen::Vector3d calibratedRotation;
	Eigen::Vector3d calibratedTranslation;
	double calibratedScale;

	std::string referenceTrackingSystem;
	std::string targetTrackingSystem;

	bool enabled = false;
	bool validProfile = false;
	bool clearOnLog = false;
	// Continuous-mode default ON: the target tracker's pose is suppressed in
	// OpenVR while continuous calibration runs so it doesn't appear as a
	// duplicate of the reference at the wrong location. One-shot is brief and
	// the duplicate isn't disruptive there; the field still gates on
	// state == Continuous in the apply path, so this default only affects
	// continuous behaviour.
	bool quashTargetInContinuous = true;

	// Head-mounted tracker configuration (Quest + lighthouse hybrid).
	TrackingStyle trackingStyle = TrackingStyle::Manual;
	HeadMountConfig headMount;
	uint32_t headMountOffsetVersion = 0;
	HeadMountSampleSource headMountLastSampleSource = HeadMountSampleSource::Unknown;
	HeadMountMode headMountLastSourceMode = HeadMountMode::Off;
	uint32_t headMountLastSourceOffsetVersion = 0;
	int32_t headMountLastSourceDeviceID = -2;
	std::string headMountLastSourceTargetSerial;
	std::string headMountLastSourceTargetSystem;
	bool headMountSourceFingerprintValid = false;
	bool headMountNeedsFreshRelativePose = false;
	double headMountLastSourceResetTime = -1e9;
	uint64_t driverSynthFallbackTotal = 0;
	double timeLastTick = 0, timeLastScan = 0, timeLastAssign = 0;
	// Default ON: drop sample pairs whose rotation axis disagrees with the
	// consensus before the LS solve. Helps with intermittent USB glitches or
	// brief tracking loss. Was OFF historically; flipping it to ON across the
	// board because there is no observed failure mode for clean data (the
	// filter is a no-op when consensus is uniform) and the failure mode for
	// noisy data (one bad sample skewing the fit) is exactly what it prevents.
	bool ignoreOutliers = true;
	double wantedUpdateInterval = 1.0;
	float jitterThreshold = 3.0f;

	bool requireTriggerPressToApply = false;
	bool wasWaitingForTriggers = false;
	bool hasAppliedCalibrationResult = false;

	float xprev, yprev, zprev;
	int consecutiveHmdStalls = 0;

	// Per-CollectSample paired-motion tracking. Used to decide whether the
	// current sample reflects correlated reference+target motion or whether
	// one device moved while the other was frozen (the passthrough/desktop
	// overlay case). Seeded on the first sample of a calibration run and
	// reset to unseeded whenever calibration is restarted (StartCalibration,
	// StartContinuousCalibration, Clear()).
	bool pairedMotionPosSeeded = false;
	Eigen::Vector3d pairedMotionPrevRefPos{0, 0, 0};
	Eigen::Vector3d pairedMotionPrevTgtPos{0, 0, 0};
	// Rolling count of "one moved, other did not" samples in the recent
	// window. Surfaced via Metrics::pairedMotionWarningCount to the popup
	// so the user sees a banner when their headset pose is frozen but the
	// target tracker keeps reporting motion.
	int pairedMotionMismatchCount = 0;

	float continuousCalibrationThreshold;
	float maxRelativeErrorThreshold = 0.005f;
	Eigen::Vector3d continuousCalibrationOffset;

	protocol::AlignmentSpeedParams alignmentSpeedParams;
	bool enableStaticRecalibration;

	// "Lock relative position" -- freezes the relative pose between the
	// reference and target devices once it has been calibrated. When locked,
	// continuous calibration only updates the world anchor frame, not the
	// relationship between the two trackers themselves.  Useful when the
	// target is rigidly attached to the reference (a tracker glued to your
	// HMD, taped to a controller, etc).
	//
	// Legacy-compatible enum:
	//   OFF  -- never lock; continuous calibration is free to re-solve the
	//           relative pose on every cycle.
	//   ON   -- always lock once a relative pose has been recorded.
	//   AUTO -- legacy profile value only. New UI and presets never select it.
	enum class LockMode : int
	{
		OFF = 0,
		ON = 1,
		AUTO = 2
	};
	LockMode lockRelativePositionMode = LockMode::OFF;

	// Resolved/effective lock state -- recomputed each tick from
	// lockRelativePositionMode + the auto-lock detector's verdict.  Existing
	// math code reads this field, so the resolver layer keeps the math
	// untouched while the user-facing knob becomes a tristate.
	bool lockRelativePosition = false;

	// Multi-ecosystem extras: each entry aligns an additional non-HMD tracking
	// system to the HMD's tracking system. Empty for the typical 1-or-2-system
	// case. The wizard appends entries here as it walks the user through each
	// detected non-HMD system. In continuous mode every entry's calibration
	// runs independently in parallel with the primary.
	std::vector<AdditionalCalibration> additionalCalibrations;

	// Wizard state. Persisted as a flag in the profile so we only auto-show
	// the wizard the first time. The user can re-launch from a button in
	// Advanced.
	bool wizardCompleted = false;

	// Resolve `lockRelativePosition` from `lockRelativePositionMode` + the
	// auto-lock detector.  Cheap; safe to call every tick.  Math code reads
	// `lockRelativePosition`, not the mode -- keeping the math layer ignorant
	// of the tristate.
	void ResolveLockMode();

	// "Recalibrate on movement" — gates the driver-side BlendTransform's lerp
	// progress on detected per-frame motion magnitude. With this on, a user who
	// is lying still won't see calibration drift even when the math is updating;
	// the catch-up happens during their next motion, hidden by the natural
	// movement instead of looking like phantom body shifts. Default ON because
	// the failure mode it prevents (visible drift while motionless) is more
	// common in practice than the rare case where you actually want instant
	// updates while stationary.
	bool recalibrateOnMovement = true;

	// UI-only flag toggled by the "Pause updates" button on the Status tab.
	// While true the overlay-side calibration tick is expected to skip the
	// ComputeIncremental call so the current driver-applied offset stays put
	// — useful when something looks momentarily wrong and the user wants to
	// freeze the live view to investigate rather than have it self-correct
	// out from under them. Default false (live updates).
	bool calibrationPaused = false;

	// "Freeze all tracking" time-freeze. When freezeAllTracking is on the driver
	// holds every tracker and controller (and the HMD when freezeIncludeHmd is on)
	// at its last pose until the user turns it off. Toggled from the Advanced tab
	// or the Ctrl+Alt+F hotkey; the overlay resends the state to the driver at
	// ~1 Hz while frozen (a heartbeat) so a dead overlay fails open to live
	// tracking. freezeAllTracking is runtime-only (never persisted, reset to off
	// each session); freezeIncludeHmd is a persisted preference (default off,
	// because a frozen headset locks the rendered view to the head).
	bool freezeAllTracking = false;
	bool freezeIncludeHmd = false;

	// Status-tab UI state: collapses the busier sliders into an "Advanced
	// settings" section. Persisting this is intentional — a user who opened
	// it once probably wants it open next session too.
	bool showAdvancedSettings = false;

	// Native prediction-suppression (see wiki/Prediction-Suppression). Scales
	// velocity/acceleration on per-device pose updates inside our SteamVR
	// driver, which lets the user trade smoothness for raw responsiveness.
	// Per-tracker because not every device wants the same setting -- e.g. a
	// hip tracker that's barely moving wants more smoothing than a wrist
	// tracker that's swinging fast.
	//
	// Per-tracker prediction smoothness, finger-smoothing config, and
	// external-smoothing-tool detection relocated to the Smoothing overlay
	// (Protocol v12 migration, 2026-05-11). SC's calibration context no
	// longer carries that state; it lives in the Smoothing plugin's Config
	// and is pushed over its own IPC pipe.

	Eigen::AffineCompact3d refToTargetPose = Eigen::AffineCompact3d::Identity();
	bool relativePosCalibrated = false;

	enum Speed
	{
		FAST = 0,
		SLOW = 1,
		VERY_SLOW = 2,
	};
	static const char* SpeedName(Speed s)
	{
		switch (s) {
			case FAST:
				return "fast";
			case SLOW:
				return "slow";
			case VERY_SLOW:
				return "very_slow";
		}
		return "?";
	}
	Speed oneShotCalibrationSpeed = FAST;
	Speed continuousCalibrationSpeed = SLOW;

	CalibrationProfileSnapshot continuousStartSnapshot;
	CalibrationProfileSnapshot lastAcceptedContinuousSnapshot;

	// Persistence throttle for continuous-mode offset writes. The in-memory
	// calibration is updated on every accepted candidate, but the registry copy
	// is read only at startup, so we persist on a cadence (see
	// ContinuousPersistDecision.h) instead of every tick. `continuousSaveDirty`
	// marks an in-memory continuous update that has not yet been persisted; it
	// is flushed when continuous mode ends or on shutdown so the latest offset
	// always survives to the next session.
	double lastContinuousSaveTime = -1e9; // Metrics::CurrentTime basis
	Eigen::Vector3d lastPersistedContinuousTranslation = Eigen::Vector3d::Zero();

	// Oversized-persist guard streak (ContinuousPersistDecision.h). Tracks
	// consecutive persist attempts whose translation sits more than
	// kDeferDeltaCm from the last persisted value. firstSeen is tick time
	// (0 = no active streak); the attempt vector judges whether the next
	// oversized attempt agrees with the previous one.
	double anomalousPersistFirstSeen = 0.0;
	int anomalousPersistAgreeCount = 0;
	Eigen::Vector3d anomalousPersistLastAttempt = Eigen::Vector3d::Zero();

	bool continuousSaveDirty = false;

	vr::DriverPose_t devicePoses[vr::k_unMaxTrackedDeviceCount];

	// Per-device shmem-side QPC timestamps captured alongside the most recent pose.
	// Populated by CalibrationTick when ingesting AugmentedPose entries from the
	// driver shared-memory ring.
	LARGE_INTEGER devicePoseSampleTimes[vr::k_unMaxTrackedDeviceCount];

	CalibrationContext()
	{
		calibratedScale = 1.0;
		memset(devicePoses, 0, sizeof(devicePoses));
		memset(devicePoseSampleTimes, 0, sizeof(devicePoseSampleTimes));
		ResetConfig();
	}

	void NoteHeadMountOffsetChanged()
	{
		++headMountOffsetVersion;
		if (headMountOffsetVersion == 0) {
			headMountOffsetVersion = 1;
		}
	}

	void ResetConfig()
	{
		alignmentSpeedParams.thr_rot_tiny = 0.49f * (EIGEN_PI / 180.0f);
		alignmentSpeedParams.thr_rot_small = 0.5f * (EIGEN_PI / 180.0f);
		alignmentSpeedParams.thr_rot_large = 5.0f * (EIGEN_PI / 180.0f);

		alignmentSpeedParams.thr_trans_tiny = 0.98f / 1000.0;  // mm
		alignmentSpeedParams.thr_trans_small = 1.0f / 1000.0;  // mm
		alignmentSpeedParams.thr_trans_large = 20.0f / 1000.0; // mm

		alignmentSpeedParams.align_speed_tiny = 1.0f;
		alignmentSpeedParams.align_speed_small = 1.0f;
		alignmentSpeedParams.align_speed_large = 2.0f;

		continuousCalibrationThreshold = 1.5f;
		maxRelativeErrorThreshold = 0.005f;
		jitterThreshold = 3.0f;

		continuousCalibrationOffset = Eigen::Vector3d::Zero();

		// Static recalibration: when the selected tracking style has locked
		// the relative pose, snap back if the live solver diverges from it.
		// No-op for independent devices (no locked relative pose to snap to),
		// so leaving it on by default is safe and accelerates recovery from
		// brief tracking glitches on rigid setups. The user can still flip it
		// off in Advanced if they want pure incremental behaviour.
		enableStaticRecalibration = true;
	}

	void ClearLogOnMessage() { clearOnLog = true; }

	void Clear()
	{
		calibratedRotation = Eigen::Vector3d();
		calibratedTranslation = Eigen::Vector3d();
		calibratedScale = 1.0;
		referenceTrackingSystem = "";
		targetTrackingSystem = "";
		enabled = false;
		validProfile = false;
		refToTargetPose = Eigen::AffineCompact3d::Identity();

		// Runtime UI state — pausing on an empty profile makes no sense.
		calibrationPaused = false;
		// Freeze is runtime-only; the driver's heartbeat timeout fails it open to
		// live tracking, so it's the safe default when clearing. (freezeIncludeHmd
		// is a persisted preference and is intentionally left alone.)
		freezeAllTracking = false;
		// Default this back ON when clearing — it's the safer setting and
		// matches the construction-time default.
		recalibrateOnMovement = true;
		// The user's lock preference (mode) is intentionally NOT reset -- it's
		// a setting, not calibration data, and a user who deliberately set ON
		// or OFF wants that to persist across profile clears.
		lockRelativePosition = false;
		// Note: showAdvancedSettings is intentionally NOT reset -- it's a
		// user preference that spans profiles.
		// No calibration was performed — relative pose is NOT calibrated. The
		// previous value here was `true`, which left a stale-identity-matrix
		// believed-good and caused StartContinuousCalibration to pass `true` to
		// setRelativeTransformation downstream.
		relativePosCalibrated = false;
		// Continuous-mode runtime offset. ResetConfig() at construction zeroes
		// this; without resetting on Clear() too, a leftover offset from a
		// previous session biased every reference pose in the next continuous
		// calibration ("everything looks consistently 5–10mm off and won't
		// converge" symptom). The offset is a runtime adjustment, not a
		// persistent profile setting — it has no business surviving a Clear().
		continuousCalibrationOffset = Eigen::Vector3d::Zero();
		continuousStartSnapshot = {};
		lastAcceptedContinuousSnapshot = {};
		lastContinuousSaveTime = -1e9;
		lastPersistedContinuousTranslation = Eigen::Vector3d::Zero();
		anomalousPersistFirstSeen = 0.0;
		anomalousPersistAgreeCount = 0;
		anomalousPersistLastAttempt = Eigen::Vector3d::Zero();
		continuousSaveDirty = false;
		headMountSourceFingerprintValid = false;
		headMountLastSampleSource = HeadMountSampleSource::Unknown;
		headMountLastSourceMode = HeadMountMode::Off;
		headMountLastSourceOffsetVersion = headMountOffsetVersion;
		headMountLastSourceDeviceID = -2;
		headMountLastSourceTargetSerial.clear();
		headMountLastSourceTargetSystem.clear();
		headMountNeedsFreshRelativePose = false;
		headMountLastSourceResetTime = -1e9;
		driverSynthFallbackTotal = 0;
	}

	void ClearRuntimeCalibrationForRecovery()
	{
		validProfile = false;
		refToTargetPose = Eigen::AffineCompact3d::Identity();
		relativePosCalibrated = false;
		hasAppliedCalibrationResult = false;
		calibratedTranslation = Eigen::Vector3d::Zero();
		calibratedRotation = Eigen::Vector3d::Zero();
		lastAcceptedContinuousSnapshot = {};
		lastContinuousSaveTime = -1e9;
		lastPersistedContinuousTranslation = Eigen::Vector3d::Zero();
		anomalousPersistFirstSeen = 0.0;
		anomalousPersistAgreeCount = 0;
		anomalousPersistLastAttempt = Eigen::Vector3d::Zero();
		continuousSaveDirty = false;
		headMountNeedsFreshRelativePose = false;
	}

	CalibrationProfileSnapshot CaptureProfileSnapshot() const
	{
		CalibrationProfileSnapshot snap;
		snap.captured = true;
		snap.enabled = enabled;
		snap.validProfile = validProfile;
		snap.referenceTrackingSystem = referenceTrackingSystem;
		snap.targetTrackingSystem = targetTrackingSystem;
		snap.referenceStandby = referenceStandby;
		snap.targetStandby = targetStandby;
		snap.calibratedRotation = calibratedRotation;
		snap.calibratedTranslation = calibratedTranslation;
		snap.calibratedScale = calibratedScale;
		snap.refToTargetPose = refToTargetPose;
		snap.relativePosCalibrated = relativePosCalibrated;
		return snap;
	}

	void RestoreProfileSnapshot(const CalibrationProfileSnapshot& snap)
	{
		if (!snap.captured) return;
		enabled = snap.enabled;
		validProfile = snap.validProfile;
		referenceTrackingSystem = snap.referenceTrackingSystem;
		targetTrackingSystem = snap.targetTrackingSystem;
		referenceStandby = snap.referenceStandby;
		targetStandby = snap.targetStandby;
		calibratedRotation = snap.calibratedRotation;
		calibratedTranslation = snap.calibratedTranslation;
		calibratedScale = snap.calibratedScale;
		refToTargetPose = snap.refToTargetPose;
		relativePosCalibrated = snap.relativePosCalibrated;
	}

	Speed ActiveCalibrationSpeed() const
	{
		return (state == CalibrationState::Continuous || state == CalibrationState::ContinuousStandby)
		           ? continuousCalibrationSpeed
		           : oneShotCalibrationSpeed;
	}

	size_t SampleCount()
	{
		switch (ActiveCalibrationSpeed()) {
			case FAST:
				return 100;
			case SLOW:
				return 250;
			case VERY_SLOW:
				return 500;
			default:
				return 100;
		}
	}

	struct Message
	{
		enum Type
		{
			String,
			Progress
		} type = String;

		Message(Type type) : type(type), progress(0), target(0) {}

		std::string str;
		int progress, target;
	};

	std::deque<Message> messages;

	void Log(const std::string& msg)
	{
		if (clearOnLog) {
			messages.clear();
			clearOnLog = false;
		}

		if (messages.empty() || messages.back().type == Message::Progress) messages.push_back(Message(Message::String));

		OutputDebugStringA(msg.c_str());

		messages.back().str += msg;
		std::cerr << msg;

		while (messages.size() > 15)
			messages.pop_front();
	}

	void Progress(int current, int target)
	{
		if (messages.empty() || messages.back().type == Message::String) messages.push_back(Message(Message::Progress));

		messages.back().progress = current;
		messages.back().target = target;
	}

	bool TargetPoseIsValidSimple() const
	{
		return targetID >= 0 && targetID < (int32_t)vr::k_unMaxTrackedDeviceCount &&
		       devicePoses[targetID].poseIsValid &&
		       devicePoses[targetID].result == vr::ETrackingResult::TrackingResult_Running_OK;
	}

	bool ReferencePoseIsValidSimple() const
	{
		return referenceID >= 0 && referenceID < (int32_t)vr::k_unMaxTrackedDeviceCount &&
		       devicePoses[referenceID].poseIsValid &&
		       devicePoses[referenceID].result == vr::ETrackingResult::TrackingResult_Running_OK;
	}
};

extern CalibrationContext CalCtx;

void InitCalibrator();
void CalibrationTick(double time);

// Persists the latest continuous-mode offset if a throttled update is pending
// (see ContinuousPersistDecision.h). No-op when nothing is pending. Called on
// shutdown so a session that quits mid-continuous-calibration still writes its
// final offset, which the per-tick throttle may otherwise leave up to a couple
// of seconds stale on disk.
void FlushPendingContinuousSave();

// `reason` is a short tag (e.g. "ui_start_button", "continuous_standby") that
// lands in the StartCalibration_state_reset log annotation. Lets a post-session
// grep distinguish the few documented entry points; a default of "unknown"
// catches any caller that hasn't been updated yet so the build stays green.
void StartCalibration(const char* reason = "unknown");
void StartContinuousCalibration(const char* reason = "unknown");
void CancelCalibration(const char* reason = "unknown");
void EndContinuousCalibration();

void ShowCalibrationDebug(int r, int c);
void DebugApplyRandomOffset();

// Re-open the driver pose shared-memory segment. The IPC client invokes this
// after a successful reconnect to vrserver: when vrserver crashes and respawns,
// the named-mapping the overlay had open is destroyed, the mapped view detaches
// silently, and ReadNewPoses() begins yielding zeros. Re-opening picks up the
// new mapping the freshly-respawned driver creates.
void ReopenShmem();
