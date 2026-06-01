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
#include "WarmRestart.h"  // ValidationOutcome enum
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
struct StandbyDevice {
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
struct AdditionalCalibration {
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
	int lockMode = 2; // 0=OFF, 1=ON, 2=AUTO. int instead of LockMode to keep
	                  // the forward-decl situation simple here.
	Eigen::AffineCompact3d refToTargetPose = Eigen::AffineCompact3d::Identity();
	bool relativePosCalibrated = false;
	std::deque<Eigen::AffineCompact3d> autoLockHistory;
	bool autoLockEffectivelyLocked = false;

	// Per-extra pending-flip queue. Mirrors CalibrationContext's primary
	// fields (autoLockHasPendingFlip / autoLockPendingFlipTo /
	// autoLockPendingFlipFirstSeen / autoLockGateHeldWarned); see
	// UpdateAutoLockDetector and CommitPendingAutoLockFlipIfStationary in
	// Calibration.cpp for the rationale.
	bool autoLockHasPendingFlip = false;
	bool autoLockPendingFlipTo = false;
	double autoLockPendingFlipFirstSeen = 0.0;
	bool autoLockGateHeldWarned = false;

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

	// Rolling history of this extra's `priorCalibrationError * 1000` (mm)
	// readings, one push per successful ComputeIncremental. Window is
	// capped at 30 entries to match the primary's geometry-shift
	// rolling-median window. Read at the geometry-shift fire site to
	// compute the extra's spike ratio (latest / median) for the
	// common-mode coherence check; a coherent ratio across all extras
	// implies the primary spike was global (worldFromDriver reanchor,
	// runtime relocalization) rather than pair-local.
	std::deque<double> recentErrorsMm;

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
enum class HeadMountMode : uint8_t {
	Off          = 0,
	AutoPaired   = 1,
	Corroborate  = 2,
	DriverSynth  = 3,
};

enum class HeadMountSampleSource : uint8_t {
	Unknown = 0,
	PhysicalTracker,
	HeadProxy,
};

// Identity and calibration for a head-mounted tracker (e.g. a Vive tracker
// zip-tied to a Quest headset). headFromTracker is the rigid offset from the
// tracker's local frame to the HMD's local frame, solved by the offset
// calibration wizard.
struct HeadMountConfig {
	HeadMountMode mode = HeadMountMode::Off;
	std::string trackerSerial;
	std::string trackerModel;          // persisted; needed for VRState::FindDevice
	std::string trackerTrackingSystem;
	Eigen::AffineCompact3d headFromTracker = Eigen::AffineCompact3d::Identity();
	bool hideTracker = true;
	bool offsetCalibrated = false;
	bool autoCorrectOffset = true;
	wkopenvr::headmount::DriverSynthTimingConfig driverSynthTiming;
	// Runtime-resolved OpenVR device ID; not persisted. -1 means unresolved.
	// Set each AssignTargets() call by matching trackerSerial + trackerTrackingSystem.
	int32_t deviceID = -1;
};

// One vertex of the safety boundary polygon.
struct BoundaryVertex { double x = 0, y = 0, z = 0; };

// Safety boundary for SteamVR chaperone. Target-space boundaries follow the
// calibrated tracker space and transform into standing space before push;
// standing-space boundaries store SteamVR standing-space vertices directly.
// priorChaperone snapshots the user's pre-existing SteamVR chaperone before
// our first push so the "Restore original" action returns the user to where
// they started.
struct BoundaryConfig {
	bool enabled = false;
	std::vector<BoundaryVertex> vertices;
	double floorY = 0.0;
	double ceilingY = 2.5;
	// Boundary geometry lives in SteamVR standing space, independent of the
	// space-calibration transform. Legacy profiles may still load false and
	// are pushed through the transform path until redrawn.
	bool standingSpace = true;
	std::vector<uint8_t> priorChaperone;
	bool priorChaperoneCaptured = false;
};

struct CalibrationProfileSnapshot {
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

	// Cumulative vertical shift (meters) applied to the SteamVR standing-zero pose
	// via "Set floor from controller". Floor height is set by editing the chaperone
	// standing-zero (the runtime-consistent mechanism, like OpenVR Advanced Settings),
	// not by shifting device poses. Tracked only so "Reset floor" can undo exactly
	// what we applied; persisted to know our own contribution across restarts.
	double floorOffsetMetersY = 0.0;
	// True when floorOffsetMetersY is currently applied to the SteamVR
	// standing-zero. Toggling this off lifts the rig back to the headset
	// floor without forgetting the offset, so it can be re-applied later;
	// "Reset floor" clears the offset entirely. Defaults true on load when an
	// offset is present so existing profiles keep their applied floor.
	bool floorEnabled = false;

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
	// Safety boundary: captured chaperone outline + floor/ceiling for re-push.
	BoundaryConfig  boundary;
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

	// One-shot mode: detect SteamVR universe shifts (chaperone reset, seated
	// zero pose reset, etc.) by watching the poses of TrackingReference
	// devices (Lighthouse base stations). When a uniform rigid delta moves
	// every base station in the same tracking system between two consecutive
	// ticks, that's a universe re-origin -- apply the inverse to the stored
	// calibration so body trackers stay aligned with the user's physical
	// position. AUTO (true): runs when ≥2 TrackingReference devices are
	// detected for the relevant system; otherwise no-ops. OFF (false): never
	// runs. Default AUTO. No effect in continuous mode -- continuous already
	// updates each tick and would converge through the shift anyway.
	bool baseStationDriftCorrectionEnabled = true;

	float xprev, yprev, zprev;
	int consecutiveHmdStalls = 0;

	// Per-CollectSample paired-motion tracking. Used to decide whether the
	// current sample reflects correlated reference+target motion or whether
	// one device moved while the other was frozen (the passthrough/desktop
	// overlay case). Seeded on the first sample of a calibration run and
	// reset to unseeded whenever calibration is restarted (StartCalibration,
	// StartContinuousCalibration, Clear()).
	bool        pairedMotionPosSeeded = false;
	Eigen::Vector3d pairedMotionPrevRefPos {0, 0, 0};
	Eigen::Vector3d pairedMotionPrevTgtPos {0, 0, 0};
	// Rolling count of "one moved, other did not" samples in the recent
	// window. Surfaced via Metrics::pairedMotionWarningCount to the popup
	// so the user sees a banner when their headset pose is frozen but the
	// target tracker keeps reporting motion.
	int         pairedMotionMismatchCount = 0;

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
	// Tristate:
	//   OFF  -- never lock; continuous calibration is free to re-solve the
	//           relative pose on every cycle.
	//   ON   -- always lock once a relative pose has been recorded.
	//   AUTO -- (default) detect rigid attachment from observed relative-pose
	//           variance.  Starts unlocked; flips to "effectively locked"
	//           once the relative pose has stayed stable (~5mm translation,
	//           ~1deg rotation) for a sustained window of accepted samples.
	//           Flips back to unlocked if the variance climbs again, e.g.
	//           the user repositioned the tracker.
	enum class LockMode : int { OFF = 0, ON = 1, AUTO = 2 };
	LockMode lockRelativePositionMode = LockMode::AUTO;

	// Resolved/effective lock state -- recomputed each tick from
	// lockRelativePositionMode + the auto-lock detector's verdict.  Existing
	// math code reads this field, so the resolver layer keeps the math
	// untouched while the user-facing knob becomes a tristate.
	bool lockRelativePosition = false;

	// Auto-lock detector state.  Tracks the most recent relative-pose samples
	// (ref^-1 * target) on a sliding window.  When their variance stays below
	// the rigidity thresholds (see AutoLockHysteresis.h) for the full window,
	// autoLockEffectivelyLocked flips true.  Cleared on profile reload /
	// Clear() so a fresh calibration starts unlocked.
	std::deque<Eigen::AffineCompact3d> autoLockHistory;
	bool autoLockEffectivelyLocked = false;

	// Rolling MAD floor for adaptive enter threshold. The detector tracks
	// the minimum robust-translation-deviation reading across a sliding
	// window of recent samples; EnterThresholdFor() scales this up to
	// produce a per-rig enter threshold so a noisy pair (e.g. Quest+
	// Lighthouse cross-system MAD floor ~4 mm) can still engage AUTO Lock
	// instead of waiting forever for MAD to drop below the 3 mm hard floor.
	// History is a deque of (time, mad) pairs trimmed to
	// kFloorWindowSec of age. Floor is recomputed each push as the min of
	// the in-window readings; zero means "no observations yet, use hard
	// floor". Reset on Clear().
	std::deque<std::pair<double, double>> autoLockMadHistory;
	double autoLockMadFloor = 0.0;

	// Wall-clock time of the most recent AUTO Lock flip commit. Used by
	// the heartbeat's `settled=` field via IsSettled, which requires the
	// lock to have held for kSettledMinHoldSec before reporting settled.
	// Zero means no flip has happened this session. Reset on Clear().
	double autoLockLastFlipTime = 0.0;

	// AUTO lock-mode pending-flip queue. The detector produces a target value
	// (`autoLockPendingFlipTo`) when the hysteresis verdict changes; the flip
	// is held until the next CalibrationTick observes the HMD nearly still
	// (kAutoLockStationaryHmdMps). This hides the visible calibration jump
	// that accompanies a locked<->unlocked regime change inside the user's
	// next stillness window. `autoLockHasPendingFlip` is false when the
	// detector and effective state agree.
	bool autoLockHasPendingFlip = false;
	bool autoLockPendingFlipTo = false;

	// Time the current pending AUTO Lock flip was first observed. Used by
	// CommitPendingAutoLockFlipIfStationary to compute how long the commit
	// gate has held a target verdict, so a chronic block becomes visible in
	// the log via [autolock][gate-held]. Zero means no pending flip is
	// currently being tracked. Reset along with the rest of the AUTO Lock
	// state in Clear() and whenever the pending flip is committed or
	// abandoned.
	double autoLockPendingFlipFirstSeen = 0.0;

	// Has the held-too-long warning for the current pending flip already
	// fired? Prevents a per-tick flood while the gate continues to block
	// the same target verdict. Cleared whenever a new pending flip starts.
	bool autoLockGateHeldWarned = false;

	// Recovery-convergence watch: set by RecoverFromWedgedCalibration to the
	// recovery timestamp + recorded hmdDelta. The first post-recovery
	// usingRelPose_fired emits a `[recovery][converged]` line carrying the
	// convergence duration and first relPoseError; this lets a reader tie
	// the physical jump severity to the time it took the cal to recover a
	// usable relative-pose constraint. Both fields zero out after emission.
	double recoveryWaitingSince = 0.0;     // Metrics::CurrentTime basis
	double recoveryHmdDeltaAtStart = 0.0;  // metres of the originating jump

	// Geometry-shift detector grace deadline. Set by StartCalibration to
	// `now + kGeometryShiftGraceSeconds` so the detector is skipped while the
	// cal converges from zero samples after a restart. Without the grace,
	// the first 5-10 samples can spike error_currentCal as the solver
	// settles -- the detector then sees that as a "5x median" excursion and
	// fires another restart, producing the back-to-back-fire pattern that
	// errTail wrap-around already aggravates. Zero means no active grace.
	double geometryShiftGraceUntil = 0.0;

	// Geometry-shift post-fire cooldown deadline. Set whenever a recovery
	// action commits; while CalibrationTick's `time` is less than this,
	// subsequent fires are suppressed and the accumulators reset (so noise
	// from the cooldown window doesn't immediately re-fire when the gate
	// releases). The 2026-05-21 session log had 52 fires in 2.2 h on
	// Quest+Lighthouse cross-system noise -- the cooldown drops the
	// false-fire cadence without affecting response time on a real shift
	// (which fires at full sensitivity once the deadline passes). Zero
	// means no active cooldown. See GeometryShiftDetector.h::
	// kPostFireCooldownSeconds for the duration.
	double geometryShiftCooldownUntil = 0.0;

	// Warm-restart detection. The user takes off the HMD, comes back later,
	// puts it on -- without intervention the solver re-validates the saved
	// profile from an empty sample buffer, which takes 4-7 minutes under
	// any tracking noise (Lighthouse base-station noise, Quest cross-system
	// latency, etc.). During those minutes the user sees their avatar
	// "fly away and fly back" as the candidate solver flips between two
	// minima per tick. The warm-restart snap path detects the HMD's
	// proximity sensor going false -> true after a sustained absence and
	// re-applies the saved profile transform immediately, then tracks a
	// validation grace window. If a real geometry shift fires during the
	// grace window (e.g. the user actually moved a base station while away),
	// grace drops to zero and the normal continuous-cal recovery takes over.
	//
	// `lastUserPresent` defaults to true so a session that starts with
	// the user already wearing the HMD sees no rising edge -- the snap
	// path is mid-session-warm-restart only.
	//
	// `userAwaySince` is the timestamp at which proximity went false;
	// the rising edge only counts as a warm restart if proximity stayed
	// false for at least kWarmRestartMinAwaySeconds (filters
	// proximity-sensor blips that some HMD runtimes emit on radio
	// hiccups).
	//
	// `warmRestartGraceSamples` is a downcounter ticked by
	// CalibrationTick after each ComputeIncremental.
	bool lastUserPresent = true;
	double userAwaySince = 0.0;
	int warmRestartGraceSamples = 0;

	// Session-level continuous-cal tick counter for the cold-start safety
	// gate. Incremented once per CalibrationTick that processes the warm-
	// restart polling block; compared against
	// spacecal::warm_restart::kColdStartGraceTicks to suppress engages
	// during the first ~30 s of a session (when a startup-then-put-on-HMD
	// sequence would otherwise look like a warm restart with no profile to
	// restore from). Reset on Clear() so a fresh profile starts the gate
	// over.
	int warmRestartTickId = 0;

	// HMD world-position captured on the proximity-falling edge (user
	// took the HMD off). On the rising edge, the displacement from this
	// stored position to the current HMD position is the
	// awayPositionDeltaM input to ShouldEngage; large jumps fast-path the
	// engage decision (see WarmRestart.h::kPositionJumpFastPathM) for
	// HMDs whose activity-level signal is unreliable. Zero vector when
	// no falling-edge position has been captured yet.
	Eigen::Vector3d hmdLastKnownPosWhenAway = Eigen::Vector3d::Zero();
	bool hmdLastKnownPosValid = false;

	// MAD-floor reading captured at the moment a warm-restart snap fired.
	// Used by the validation phase as the baseline against which post-snap
	// MAD convergence is judged. NaN until a snap fires.
	double warmRestartMadAtSnap = 0.0;

	// Validation outcome of the most recent warm-restart snap. Set
	// Inconclusive when the snap fires; transitions to Settled or Failed
	// when the validation gate trips (see WarmRestart.h::EvaluateValidation).
	// Reset on Clear().
	spacecal::warm_restart::ValidationOutcome warmRestartValidationState =
		spacecal::warm_restart::ValidationOutcome::Inconclusive;

	// Post-snap retargeting-error accumulator for the warm-restart
	// validation phase. The validator's MAD floor (`autoLockMadFloor`)
	// is a 60 s rolling minimum that includes pre-snap history, so a
	// snap can inherit a quiet pre-snap floor and pass the dispersion
	// check while landing on a stale profile. The bias accumulator only
	// integrates `Metrics::error_currentCal` samples whose push timestamp
	// post-dates `warmRestartLastConsumedErrTs` -- i.e. samples produced
	// strictly after the snap fire. Reset at snap fire and on Clear().
	// `postSnapErrorSumMm` is in millimetres to match the units already
	// stored in `error_currentCal`; the mean is converted to metres at
	// the call site before being passed to EvaluateValidation.
	double postSnapErrorSumMm = 0.0;
	int    postSnapErrorSampleCount = 0;
	double warmRestartLastConsumedErrTs = 0.0;

	// Snap wall-time and source-of-floor tracking. `warmRestartSnapTime`
	// is `Metrics::CurrentTime` at snap fire; `autoLockMadFloorTs` is the
	// timestamp of the sample that produced the current floor value.
	// When `autoLockMadFloorTs < warmRestartSnapTime`, the floor came
	// from pre-snap history; the heartbeat surfaces this distinction via
	// `mad_floor_source` so a "Settled by stale floor" verdict is
	// distinguishable from a "Settled by post-snap convergence" one.
	double warmRestartSnapTime = 0.0;
	double autoLockMadFloorTs = 0.0;

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

	// Push a fresh ref+target world pose into the auto-lock detector.
	// Computes the relative pose (ref^-1 * target), appends it to the
	// history, and re-evaluates rigidity.  Called from the calibration tick
	// each time a new sample is collected.
	void UpdateAutoLockDetector(const Eigen::AffineCompact3d& refWorld,
	                            const Eigen::AffineCompact3d& targetWorld);

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
		// Picks one of the above each tick based on observed reference+target jitter.
		// Lets a casual user not have to think about it: the program watches its own
		// noise floor and slows the calibration down when conditions are bad.
		AUTO = 3,
	};
	// One-shot defaults to FAST. Continuous defaults to AUTO so long-running
	// sessions can adapt buffer size to observed jitter without making one-shot
	// calibration wait on a sticky speed resolver.
	Speed oneShotCalibrationSpeed = FAST;
	Speed continuousCalibrationSpeed = AUTO;

	CalibrationProfileSnapshot continuousStartSnapshot;
	CalibrationProfileSnapshot lastAcceptedContinuousSnapshot;

	vr::DriverPose_t devicePoses[vr::k_unMaxTrackedDeviceCount];

	// Per-device shmem-side QPC timestamps captured alongside the most recent pose.
	// Populated by CalibrationTick when ingesting AugmentedPose entries from the
	// driver shared-memory ring.
	LARGE_INTEGER devicePoseSampleTimes[vr::k_unMaxTrackedDeviceCount];

	CalibrationContext() {
		calibratedScale = 1.0;
		memset(devicePoses, 0, sizeof(devicePoses));
		memset(devicePoseSampleTimes, 0, sizeof(devicePoseSampleTimes));
		ResetConfig();
	}

	void NoteHeadMountOffsetChanged() {
		++headMountOffsetVersion;
		if (headMountOffsetVersion == 0) {
			headMountOffsetVersion = 1;
		}
	}

	void ResetConfig() {
		alignmentSpeedParams.thr_rot_tiny = 0.49f * (EIGEN_PI / 180.0f);
		alignmentSpeedParams.thr_rot_small = 0.5f * (EIGEN_PI / 180.0f);
		alignmentSpeedParams.thr_rot_large = 5.0f * (EIGEN_PI / 180.0f);

		alignmentSpeedParams.thr_trans_tiny = 0.98f / 1000.0; // mm
		alignmentSpeedParams.thr_trans_small = 1.0f / 1000.0; // mm
		alignmentSpeedParams.thr_trans_large = 20.0f / 1000.0; // mm

		alignmentSpeedParams.align_speed_tiny = 1.0f;
		alignmentSpeedParams.align_speed_small = 1.0f;
		alignmentSpeedParams.align_speed_large = 2.0f;

		// Slew-rate cap defaults. Mirror the driver-side Init defaults so a
		// fresh overlay starts pushing the same numbers it would have computed
		// on its own and no first-frame surprise happens before the user opens
		// the Advanced tab. Units match the driver: metres/sec, radians/sec.
		alignmentSpeedParams.slew_stationary_pos_rate = 0.0005;   // 0.5 mm/sec
		alignmentSpeedParams.slew_stationary_rot_rate = 0.000873; // ~0.05 deg/sec
		alignmentSpeedParams.slew_moving_pos_rate     = 0.010;    // 10  mm/sec
		alignmentSpeedParams.slew_moving_rot_rate     = 0.01745;  // ~1.0 deg/sec

		continuousCalibrationThreshold = 1.5f;
		maxRelativeErrorThreshold = 0.005f;
		jitterThreshold = 3.0f;

		continuousCalibrationOffset = Eigen::Vector3d::Zero();

		// Static recalibration: when Lock relative position has identified a
		// rigid attachment (Lock=ON or Lock=AUTO and the auto-detector fired),
		// snap to the locked relative pose if the live solver diverges from it.
		// No-op for independent devices (no locked relative pose to snap to),
		// so leaving it on by default is safe and accelerates recovery from
		// brief tracking glitches on rigid setups. The user can still flip it
		// off in Advanced if they want pure incremental behaviour.
		enableStaticRecalibration = true;

		// Default AUTO. Failure mode is benign: with no base stations
		// detected, the detector simply doesn't fire.
		baseStationDriftCorrectionEnabled = true;
	}

	struct Chaperone
	{
		bool valid = false;
		bool autoApply = true;
		std::vector<vr::HmdQuad_t> geometry;
		vr::HmdMatrix34_t standingCenter = {
			1.0f, 0.0f, 0.0f, 0.0f,
			0.0f, 1.0f, 0.0f, 0.0f,
			0.0f, 0.0f, 1.0f, 0.0f,
		};
		vr::HmdVector2_t playSpaceSize = { 0.0f, 0.0f };
	} chaperone;

	void ClearLogOnMessage() {
		clearOnLog = true;
	}

	void Clear()
	{
		chaperone.geometry.clear();
		chaperone.standingCenter = vr::HmdMatrix34_t();
		chaperone.playSpaceSize = vr::HmdVector2_t();
		chaperone.valid = false;

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
		// Default this back ON when clearing — it's the safer setting and
		// matches the construction-time default.
		recalibrateOnMovement = true;
		// Auto-lock detector resets to "no observations yet". The user's lock
		// preference (mode) is intentionally NOT reset -- it's a setting, not
		// calibration data, and a user who deliberately set ON or OFF wants
		// that to persist across profile clears.
		lockRelativePosition = false;
		autoLockHistory.clear();
		autoLockEffectivelyLocked = false;
		autoLockHasPendingFlip = false;
		autoLockPendingFlipTo = false;
		autoLockPendingFlipFirstSeen = 0.0;
		autoLockGateHeldWarned = false;
		autoLockMadHistory.clear();
		autoLockMadFloor = 0.0;
		autoLockLastFlipTime = 0.0;
		recoveryWaitingSince = 0.0;
		recoveryHmdDeltaAtStart = 0.0;
		geometryShiftGraceUntil = 0.0;
		geometryShiftCooldownUntil = 0.0;
		// Warm-restart state: a Clear means the saved profile is gone, so
		// any pending grace must go too. Default lastUserPresent back to
		// true so the next proximity-false reading produces a clean edge.
		lastUserPresent = true;
		userAwaySince = 0.0;
		warmRestartGraceSamples = 0;
		warmRestartTickId = 0;
		hmdLastKnownPosWhenAway = Eigen::Vector3d::Zero();
		hmdLastKnownPosValid = false;
		warmRestartMadAtSnap = 0.0;
		warmRestartValidationState =
			spacecal::warm_restart::ValidationOutcome::Inconclusive;
		postSnapErrorSumMm = 0.0;
		postSnapErrorSampleCount = 0;
		warmRestartLastConsumedErrTs = 0.0;
		warmRestartSnapTime = 0.0;
		autoLockMadFloorTs = 0.0;
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
		return (state == CalibrationState::Continuous
			|| state == CalibrationState::ContinuousStandby)
			? continuousCalibrationSpeed
			: oneShotCalibrationSpeed;
	}

	// Resolve the user's selected speed to a concrete FAST/SLOW/VERY_SLOW. When
	// the user picks AUTO, use the recent calibration fit RMS as the buffer-size
	// hint:
	//   - clean to typical fits (<5mm RMS) get FAST so calibration converges
	//     quickly. Covers tracker-on-HMD and most lighthouse setups.
	//   - moderately noisy fits (5-10mm RMS) get SLOW
	//   - genuinely noisy / reflective rooms / drifty IMU (>10mm RMS) get VERY_SLOW
	// This is sticky-by-default: we only re-evaluate every few seconds and
	// require the new bucket to have been right for a while before switching,
	// so the buffer size doesn't oscillate during transient noise.
	Speed ResolvedCalibrationSpeed() const;

	size_t SampleCount()
	{
		switch (ResolvedCalibrationSpeed())
		{
		case FAST:
			// 30 samples at ~18 Hz = ~1.7s buffer fill. The direct O(N) translation
			// solve converges with N >= ~20 when the per-axis ranges are >= 10cm;
			// the diversity gate already enforces that, so 30 is comfortably above
			// the math floor.
			return 30;
		case SLOW:
			return 100;
		case VERY_SLOW:
			return 200;
		default:
			return 30;
		}
	}

	struct Message
	{
		enum Type
		{
			String,
			Progress
		} type = String;

		Message(Type type) : type(type), progress(0), target(0) { }

		std::string str;
		int progress, target;
	};

	std::deque<Message> messages;

	void Log(const std::string &msg)
	{
		if (clearOnLog) {
			messages.clear();
			clearOnLog = false;
		}

		if (messages.empty() || messages.back().type == Message::Progress)
			messages.push_back(Message(Message::String));

		OutputDebugStringA(msg.c_str());

		messages.back().str += msg;
		std::cerr << msg;

		while (messages.size() > 15) messages.pop_front();
	}

	void Progress(int current, int target)
	{
		if (messages.empty() || messages.back().type == Message::String)
			messages.push_back(Message(Message::Progress));

		messages.back().progress = current;
		messages.back().target = target;
	}

	bool TargetPoseIsValidSimple() const {
		return targetID >= 0 && targetID < (int32_t)vr::k_unMaxTrackedDeviceCount
			&& devicePoses[targetID].poseIsValid && devicePoses[targetID].result == vr::ETrackingResult::TrackingResult_Running_OK;
	}

	bool ReferencePoseIsValidSimple() const {
		return referenceID >= 0 && referenceID < (int32_t)vr::k_unMaxTrackedDeviceCount
			&& devicePoses[referenceID].poseIsValid && devicePoses[referenceID].result == vr::ETrackingResult::TrackingResult_Running_OK;
	}
};

extern CalibrationContext CalCtx;

// Commits a queued AUTO-Lock-mode flip (held by UpdateAutoLockDetector to
// hide visible jumps) when the HMD is nearly still. No-op when the queue is
// empty or the user is currently moving.
// Called once per CalibrationTick in continuous-cal mode. Public so unit
// tests can drive it directly.
bool CommitPendingAutoLockFlipIfStationary(CalibrationContext& ctx, double hmdSpeedMps, double now);

void InitCalibrator();
void CalibrationTick(double time);

// `reason` is a short tag (e.g. "ui_start_button", "continuous_standby",
// "tracker_liveness_reconnect", "auto_recovery_snap") that lands in the
// StartCalibration_state_reset log annotation. Lets a post-session grep
// distinguish the few documented entry points; a default of "unknown"
// catches any caller that hasn't been updated yet so the build stays green.
void StartCalibration(const char* reason = "unknown");
void StartContinuousCalibration(const char* reason = "unknown");
void CancelCalibration(const char* reason = "unknown");
void EndContinuousCalibration();
void LoadChaperoneBounds();
void ApplyChaperoneBounds();

void ShowCalibrationDebug(int r, int c);
void DebugApplyRandomOffset();

// Dump relocalization state to the structured log. Called from the Logs tab's
// "Dump drift state" button.
void DumpDriftSubsystemState();

// Accessor for the session-counter of stuck-loop watchdog firings. The
// underlying CalibrationCalc instance lives in an anonymous namespace inside
// Calibration.cpp, so we expose this via a free function.
int GetWatchdogResetCount();

// Most recent HMD-relocalization detection event. Returns true if any event
// has been logged this session and populates out parameters with the time
// since the event (seconds), the translation magnitude (meters), and the
// rotation magnitude (degrees). Returns false if the detector hasn't fired
// at all this session.
bool LastDetectedRelocalization(double& outAgeSeconds, double& outDeltaMeters,
                                double& outDeltaDegrees);

// Recent auto-recovery info: returns true if auto-recovery clobbered the
// calibration in the last 60 seconds AND the user hasn't dismissed the
// banner. Populates outAge (seconds since recovery fired) and outDeltaMeters
// (the HMD jump magnitude that triggered the recovery). Used by the UI to
// render a sticky banner with Undo + Dismiss buttons.
bool LastAutoRecoveryActive(double& outAge, double& outDeltaMeters);

// Restore the pre-recovery refToTargetPose / relativePosCalibrated /
// hasAppliedCalibrationResult, taking the user back to the calibration
// state that was in effect before the auto-recovery cleared it. Returns
// true if the snapshot existed and was restored, false if no recovery
// has happened yet or undo was already applied. Idempotent: a second
// click is a no-op.
bool UndoLastAutoRecovery();

// Hide the recovery banner without undoing. The recovered calibration
// continues; only the UI banner disappears.
void DismissAutoRecoveryBanner();

// Re-open the driver pose shared-memory segment. The IPC client invokes this
// after a successful reconnect to vrserver: when vrserver crashes and respawns,
// the named-mapping the overlay had open is destroyed, the mapped view detaches
// silently, and ReadNewPoses() begins yielding zeros. Re-opening picks up the
// new mapping the freshly-respawned driver creates.
void ReopenShmem();
