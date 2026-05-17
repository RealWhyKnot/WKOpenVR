#pragma once

#include <windows.h>
#include <cstdint>
#include <atomic>
#include <stdexcept>
#include <functional>

#ifndef _OPENVR_API
#include <openvr_driver.h>
#endif

// Per-feature pipe names. The driver opens up to three pipes depending on
// which enable_*.flag files are present in its resources directory; each
// consumer overlay connects only to its own pipe. Wire format on all pipes
// is the same protocol::Request/Response struct; the driver routes by
// request type and rejects out-of-feature requests.
#define OPENVR_PAIRDRIVER_CALIBRATION_PIPE_NAME  "\\\\.\\pipe\\WKOpenVR-Calibration"
#define OPENVR_PAIRDRIVER_SMOOTHING_PIPE_NAME    "\\\\.\\pipe\\WKOpenVR-Smoothing"
#define OPENVR_PAIRDRIVER_INPUTHEALTH_PIPE_NAME  "\\\\.\\pipe\\WKOpenVR-InputHealth"
#define OPENVR_PAIRDRIVER_FACETRACKING_PIPE_NAME "\\\\.\\pipe\\WKOpenVR-FaceTracking"
// v16 (2026-05-13): OSC router module. Driver opens this pipe gated on
// enable_oscrouter.flag. Accepts route subscribe/unsubscribe, config push,
// and test-publish requests from the overlay.
#define OPENVR_PAIRDRIVER_OSCROUTER_PIPE_NAME    "\\\\.\\pipe\\WKOpenVR-OscRouter"
// Publish pipe: fire-and-forget from out-of-process sidecars. Wire format:
// 32-byte source-id, 4-byte LE frame length, then `length` bytes of raw OSC.
#define OPENVR_PAIRDRIVER_OSCROUTER_PUB_PIPE_NAME "\\\\.\\pipe\\WKOpenVR-OscRouterPub"
// Captions module. Driver opens this pipe gated on enable_captions.flag.
// Accepts RequestSetCaptionsConfig and RequestCaptionsRestartHost.
#define OPENVR_PAIRDRIVER_CAPTIONS_PIPE_NAME   "\\\\.\\pipe\\WKOpenVR-Captions"
// Phantom Trackers module. Driver opens this pipe gated on
// enable_phantom.flag. Phase 1 accepts RequestSetPhantomConfig (global
// master switch + timeout ladder) and RequestSetPhantomDeviceOptIn (per-
// device dropout bridging toggle). Later phases extend with calibration,
// virtual-tracker, and ML messages without further pipe additions.
#define OPENVR_PAIRDRIVER_PHANTOM_PIPE_NAME    "\\\\.\\pipe\\WKOpenVR-Phantom"

// Pose telemetry shmem segment. Created by the driver only when the calibration
// feature is enabled; the calibration overlay opens it to read driver-side
// pose snapshots and telemetry counters.
// V2 bumps the segment name to WKOpenVR* (V1 was OpenVRPairPoseMemoryV1) so a
// half-updated install -- old driver + new overlay or vice versa -- fails
// loudly on attach instead of silently mapping a foreign segment.
#define OPENVR_PAIRDRIVER_SHMEM_NAME            "WKOpenVRPoseMemoryV2"

// Input-health snapshot shmem segment. Created by the driver only when the
// inputhealth feature is enabled; the InputHealth overlay opens it to read
// per-component statistics published at ~10 Hz from the driver-side worker.
#define OPENVR_PAIRDRIVER_INPUTHEALTH_SHMEM_NAME "WKOpenVRInputHealthMemoryV1"

// Face-tracking per-frame shmem ring. Created by the driver only when the
// facetracking feature is enabled; the C# FaceModuleHost.exe opens it for
// write and publishes per-hardware-frame face/eye samples (~120 Hz) into a
// 32-slot ring. The driver reads the latest frame on its pose-update path
// and applies calibration / eyelid-sync / vergence-lock before publishing to
// SteamVR inputs. Single writer (host) / single reader (driver).
// V2 bumps the segment name to WKOpenVR* (V1 was
// OpenVRPairFaceTrackingFrameRingV1) so a half-updated install fails loudly
// on attach instead of mapping a stale ring with the wrong shape.
#define OPENVR_PAIRDRIVER_FACETRACKING_SHMEM_NAME "WKOpenVRFaceTrackingFrameRingV2"
// OSC router stats shmem. Created by the driver when oscrouter is enabled;
// the overlay reads it at ~10 Hz to show per-route message rates.
#define OPENVR_PAIRDRIVER_OSCROUTER_SHMEM_NAME "WKOpenVROscRouterStatsV1"

// Phantom-tracker per-device state shmem. Created by the driver only when
// the phantom feature is enabled; the Phantom overlay opens it to render
// per-tracker state badges (REAL / RECKON / IK / ML / OUTOFRANGE / LOST)
// and dropout-count counters at ~10 Hz. Single writer (driver) /
// single reader (overlay).
#define OPENVR_PAIRDRIVER_PHANTOM_STATE_SHMEM_NAME "WKOpenVRPhantomStateV1"

// Phantom Phase 3 inference shmems. The driver writes per-frame inputs
// into the IN segment; the sidecar (WKOpenVRPhantomSidecar.exe) reads,
// runs ML inference (passthrough stub today, ONNX SparsePoser in the
// follow-up), and writes per-role completed poses + per-role
// confidence into the OUT segment. The driver consults the OUT
// segment when DropoutState selects SYNTH_ML and falls through to the
// IK fallback when the sidecar is absent / confidence is low.
#define OPENVR_PAIRDRIVER_PHANTOM_INFERENCE_IN_SHMEM_NAME  "WKOpenVRPhantomInferenceInV1"
#define OPENVR_PAIRDRIVER_PHANTOM_INFERENCE_OUT_SHMEM_NAME "WKOpenVRPhantomInferenceOutV1"

#ifdef _OPENVR_API 

namespace vr {
	// We can't include openvr_driver.h as it will result in multiple definition of some structures.
	// However, we need to share driver-specific structures with the client application, so duplicate them
	// here.

	struct DriverPose_t
	{
		/* Time offset of this pose, in seconds from the actual time of the pose,
		 * relative to the time of the PoseUpdated() call made by the driver.
		 */
		double poseTimeOffset;

		/* Generally, the pose maintained by a driver
		 * is in an inertial coordinate system different
		 * from the world system of x+ right, y+ up, z+ back.
		 * Also, the driver is not usually tracking the "head" position,
		 * but instead an internal IMU or another reference point in the HMD.
		 * The following two transforms transform positions and orientations
		 * to app world space from driver world space,
		 * and to HMD head space from driver local body space.
		 *
		 * We maintain the driver pose state in its internal coordinate system,
		 * so we can do the pose prediction math without having to
		 * use angular acceleration.  A driver's angular acceleration is generally not measured,
		 * and is instead calculated from successive samples of angular velocity.
		 * This leads to a noisy angular acceleration values, which are also
		 * lagged due to the filtering required to reduce noise to an acceptable level.
		 */
		vr::HmdQuaternion_t qWorldFromDriverRotation;
		double vecWorldFromDriverTranslation[3];

		vr::HmdQuaternion_t qDriverFromHeadRotation;
		double vecDriverFromHeadTranslation[3];

		/* State of driver pose, in meters and radians. */
		/* Position of the driver tracking reference in driver world space
		* +[0] (x) is right
		* +[1] (y) is up
		* -[2] (z) is forward
		*/
		double vecPosition[3];

		/* Velocity of the pose in meters/second */
		double vecVelocity[3];

		/* Acceleration of the pose in meters/second */
		double vecAcceleration[3];

		/* Orientation of the tracker, represented as a quaternion */
		vr::HmdQuaternion_t qRotation;

		/* Angular velocity of the pose in axis-angle
		* representation. The direction is the angle of
		* rotation and the magnitude is the angle around
		* that axis in radians/second. */
		double vecAngularVelocity[3];

		/* Angular acceleration of the pose in axis-angle
		* representation. The direction is the angle of
		* rotation and the magnitude is the angle around
		* that axis in radians/second^2. */
		double vecAngularAcceleration[3];

		ETrackingResult result;

		bool poseIsValid;
		bool willDriftInYaw;
		bool shouldApplyHeadModel;
		bool deviceIsConnected;
	};

}

#endif

namespace protocol
{
	// v8 (2026-04-28): freezePrediction (bool) -> predictionSmoothness (uint8 0..100).
	// Old field gave a binary on/off; new field is a strength knob. Driver scales
	// velocity / acceleration / poseTimeOffset by (1 - smoothness/100) instead of
	// zeroing them when the bool was true. smoothness=100 reproduces the old freeze
	// behaviour; smoothness=0 leaves the pose untouched (off).
	// v9 (2026-05-02): adds RequestSetFingerSmoothing + FingerSmoothingConfig union
	// member. Driver-side hook on IVRDriverInputInternal::UpdateSkeletonComponent
	// applies per-bone slerp smoothing to Index Knuckles bone arrays before they
	// reach OpenVR consumers (proven 2026-05-02 in VRChat). Default OFF; the union
	// and Request sizeof are unchanged because the new struct is much smaller than
	// SetDeviceTransform -- the bump is to force a paired overlay+driver reinstall
	// so the user sees a clean handshake error instead of a silently-ignored new
	// request type if they upgrade only one half.
	// v10 (2026-05-06): adds the InputHealth subsystem. Driver opens a third pipe
	// (\\.\pipe\WKOpenVR-InputHealth) gated on enable_inputhealth.flag, accepts
	// RequestSetInputHealthConfig + RequestResetInputHealthStats. Wire layout
	// unchanged because both new structs are much smaller than SetDeviceTransform;
	// the bump is again purely to force paired install. The driver-side hooks on
	// UpdateBooleanComponent / UpdateScalarComponent and the snapshot publish
	// path land in subsequent commits behind this protocol baseline.
	// v11 (2026-05-11): adds RequestSetInputHealthCompensation. Snapshot shmem
	// stays unchanged; compensation is overlay -> driver only.
	// v12 (2026-05-11): adds RequestSetDevicePrediction. Splits per-device
	// pose-prediction smoothness off SetDeviceTransform so the Smoothing
	// overlay can own those fields independently of SpaceCalibrator's
	// transform/scale writes. Driver continues to accept the
	// predictionSmoothness / recalibrateOnMovement fields inside
	// SetDeviceTransform for wire compatibility but ignores them; only
	// SetDevicePrediction updates those slots now. Required so SC's
	// per-frame calibration pushes don't clobber the Smoothing plugin's
	// per-tracker prediction settings.
	// v13 (2026-05-11): extends FingerSmoothingConfig with per_finger_smoothness[10]
	// so each finger gets its own strength rather than sharing a single global
	// value. Driver no longer fits the whole config inside a single 8-byte atomic;
	// it splits the per-finger array into a second std::atomic<uint64_t> so the
	// UpdateSkeletonComponent detour reads two atomics per call. The brief skew
	// window between the two acquire loads is bounded to one frame on one finger
	// -- acceptable for a perceptual smoothing knob. Wire struct grows by 12 bytes
	// (10 array + trailing pad); still much smaller than SetDeviceTransform, so
	// sizeof(Request) is unchanged.
	//
	// v14 (2026-05-12): InputHealthConfig shrunk by 2 bytes -- the
	// notification_cooldown_s field was removed from both the overlay-side
	// config object and the wire struct. Field offsets after enable_trigger_remap
	// shift; an old driver paired with a new overlay (or vice versa) would read
	// or write at the wrong offset without this bump. quick.ps1 redeploys the
	// driver and overlay together so end users hit the matching pair, but the
	// version bump prevents a dev-tree mismatch from silently corrupting config.
	//
	// v15 (2026-05-12): adds the FaceTracking subsystem. Driver opens a fourth
	// pipe (\\.\pipe\WKOpenVR-FaceTracking) gated on enable_facetracking.flag,
	// accepts RequestSetFaceTrackingConfig + RequestSetFaceCalibrationCommand +
	// RequestSetFaceActiveModule. A new shmem ring
	// (WKOpenVRFaceTrackingFrameRingV2) carries per-frame face/eye samples
	// from a C# host sidecar (WKOpenVR.FaceModuleHost.exe) into the driver.
	// Wire layout for the existing request types is unchanged; the bump forces
	// paired install so a version skew is rejected at handshake instead of
	// landing as a silently-ignored RequestType.
	//
	// v16 (2026-05-13): adds the OSC Router substrate. Driver opens a fifth
	// pipe (\\.\pipe\WKOpenVR-OscRouter) gated on enable_oscrouter.flag,
	// plus a fire-and-forget publish pipe (\\.\pipe\WKOpenVR-OscRouterPub)
	// for out-of-process sidecars. New request types: RequestOscRouteSubscribe,
	// RequestOscRouteUnsubscribe, RequestOscPublish, RequestOscGetStats.
	// New shmem segment (WKOpenVROscRouterStatsV1) carries per-route counters
	// at ~10 Hz for the overlay's Modules tab display. Also standardises the
	// two legacy pipe-name prefixes from OpenVR-* to WKOpenVR-* for
	// consistency; a full paired reinstall is required regardless due to the
	// version bump.
	//
	// v17 (2026-05-15): CaptionsSupervisorStatus grows last_exit_code
	// (uint32_t) and last_exit_description (char[128]) so the overlay can
	// surface a halted host's exit reason in the Captions tab instead of
	// just showing the halt flag. _pad shrinks from 7 to 3 bytes.
	//
	// v19 (2026-05-16): adds the Phantom Trackers subsystem. Driver opens
	// a new pipe (\\.\pipe\WKOpenVR-Phantom) gated on enable_phantom.flag,
	// accepts RequestSetPhantomConfig (master switch + blend/timeout ladder)
	// and RequestSetPhantomDeviceOptIn (per-serial dropout bridging toggle).
	// New shmem segment (WKOpenVRPhantomStateV1) carries per-device state
	// at ~10 Hz for the overlay's badge display. Both new payload structs
	// are smaller than SetDeviceTransform so the Request union does not
	// grow; static_asserts below pin that. Bump forces paired install so
	// a version skew is rejected at handshake instead of silently dropping
	// the new request types.
	//
	// v20 (2026-05-16): phantom Phase 1.5. Adds RequestSetPhantomDeviceRole
	// (per-serial body-role assignment fed by the T-pose calibration
	// wizard) and RequestSetPhantomTrackerOffset (per-role rigid offset
	// from the HMD captured at T-pose). DropoutState now transitions
	// to SYNTH_IK past the reckon-hold window when any calibration is on
	// file; IkFallback rebuilds a synth pose by applying the rigid offset
	// to the live HMD pose. Both new structs are smaller than
	// SetDeviceTransform; static_asserts pin the union size.
	//
	// v21 (2026-05-16): phantom Phase 2 absent-mode virtual trackers. Adds
	// RequestSetPhantomVirtualEnabled -- a per-role toggle for users who
	// do not physically own a tracker for the role. The driver creates an
	// ITrackedDeviceServerDriver for each enabled role with the published
	// Valve "vive_tracker_<role>" controller type, and pushes IK-derived
	// poses every time the HMD pose updates. Gated on the user having a
	// T-pose calibration for the role.
	const uint32_t Version = 21;

	// Maximum length of a tracking-system-name string (e.g., "lighthouse", "oculus",
	// "Pimax Crystal HMD"). 32 bytes is more than enough for known systems and keeps
	// the IPC payload compact.
	static const size_t MaxTrackingSystemNameLen = 32;

	static const uint32_t INPUTHEALTH_PATH_LEN = 64;

	enum RequestType
	{
		RequestInvalid,
		RequestHandshake,
		RequestSetDeviceTransform,
		RequestSetAlignmentSpeedParams,
		RequestDebugOffset,
		RequestSetTrackingSystemFallback,
		// v9 (2026-05-02): finger-smoothing config push. Driver caches the
		// payload behind a small mutex; the IVRDriverInputInternal hook reads
		// it on every UpdateSkeletonComponent call (~340 Hz/hand).
		RequestSetFingerSmoothing,
		// v10 (2026-05-06): input-health config push. Driver caches the
		// payload as packed atomics; the boolean/scalar input detours read it
		// on every component update. Default-constructed config is "feature
		// off" (master_enabled = false) so the detour fast-paths to passthrough.
		RequestSetInputHealthConfig,
		// v10 (2026-05-06): reset accumulated stats for one device. Used by
		// the wizard's "start fresh" button and by the user's "Don't ask
		// for this device" / "I just got new hardware" flows.
		RequestResetInputHealthStats,
		RequestSetInputHealthCompensation,
		// v12 (2026-05-11): per-device pose-prediction smoothness push from
		// the Smoothing overlay. Driver updates predictionSmoothness +
		// recalibrateOnMovement on the addressed device's transform slot
		// without touching transform / scale / enabled. Lets Smoothing own
		// these fields while SpaceCalibrator keeps owning calibration.
		RequestSetDevicePrediction,
		// v15 (2026-05-12): face-tracking master config (toggles, sync
		// strengths, OSC output endpoint, active module uuid). Driver caches
		// the payload and applies it on its pose-update + frame-publish path.
		RequestSetFaceTrackingConfig,
		// v15: calibration command from the overlay -- begin/end/save/reset
		// learned per-shape envelopes. Driver-side CalibrationEngine owns
		// the persistent state; this is the only mutation path.
		RequestSetFaceCalibrationCommand,
		// v15: pick which hardware module (Quest Pro, Vive FT, ...) the host
		// should load. Driver forwards over its host-side control pipe.
		RequestSetFaceActiveModule,
		// v15 (2026-05-13): signal the driver to terminate and respawn the
		// C# module host. Overlay flushes calibration first, then sends this.
		RequestFaceHostRestart,
		// v16 (2026-05-13): OSC router control. Driver routes these on the
		// \\.\pipe\WKOpenVR-OscRouter pipe, gated on kFeatureOscRouter.
		// Subscribe / Unsubscribe register overlay-side interest in an OSC
		// address pattern and drive per-route counters in the stats shmem.
		// Publish sends a single OSC datagram from the overlay (e.g. test msg).
		// GetStats triggers an immediate stats snapshot response; the normal
		// path is to read the shmem directly at ~10 Hz.
		RequestOscRouteSubscribe,
		RequestOscRouteUnsubscribe,
		RequestOscPublish,
		RequestOscGetStats,
		// Captions module (appended after v16 entries; no version bump).
		// Driver routes these on \\.\pipe\WKOpenVR-Captions, gated on
		// kFeatureCaptions. SetCaptionsConfig pushes mic/language/mode
		// settings; CaptionsRestartHost terminates and respawns the sidecar.
		RequestSetCaptionsConfig,
		RequestCaptionsRestartHost,
		RequestCaptionsGetSupervisorStatus,
		// v19 (2026-05-16): Phantom Trackers module. Driver routes these
		// on \\.\pipe\WKOpenVR-Phantom, gated on kFeaturePhantom.
		// SetPhantomConfig carries the master switch + timeout ladder;
		// SetPhantomDeviceOptIn carries the per-serial dropout bridging
		// toggle (one message per device the user toggles).
		RequestSetPhantomConfig,
		RequestSetPhantomDeviceOptIn,
		// v20 (2026-05-16): phantom Phase 1.5 calibration messages.
		// SetPhantomDeviceRole maps a physical tracker's FNV-1a serial
		// hash to a body role (waist / foot / etc.); SetPhantomTrackerOffset
		// carries the rigid HMD-relative offset captured during the T-pose
		// wizard. Both arrive once per tracker per calibration; the driver
		// keeps the values until cleared.
		RequestSetPhantomDeviceRole,
		RequestSetPhantomTrackerOffset,
		// v21 (2026-05-16): phantom Phase 2 absent-mode toggle. Per-role
		// boolean; setting true causes the driver to create a virtual
		// generic tracker with the published vive_tracker_<role> controller
		// type. Idempotent; flipping to false retracts the virtual device
		// on the next vrserver restart (SteamVR does not allow live
		// TrackedDeviceRemoved without re-Init).
		RequestSetPhantomVirtualEnabled,
	};

	enum ResponseType
	{
		ResponseInvalid,
		ResponseHandshake,
		ResponseSuccess,
		// v16: sent in reply to RequestOscGetStats. Payload is OscRouterStats.
		ResponseOscRouterStats,
		// Captions: sent in reply to RequestCaptionsGetSupervisorStatus.
		ResponseCaptionsSupervisorStatus,
	};

	struct Protocol
	{
		uint32_t version = Version;
	};

	struct AlignmentSpeedParams
	{
		/**
		 * The threshold at which we adjust the alignment speed based on the position offset
		 * between current and target calibrations. Generally, we increase the speed if we go
		 * above small/large, and decrease it only once it's under tiny.
		 * 
		 * These values are expressed as distance squared
		 */
		double thr_trans_tiny, thr_trans_small, thr_trans_large;

		/**
		 * Similar thresholds for rotation offsets, in radians
		 */
		double thr_rot_tiny, thr_rot_small, thr_rot_large;
		
		/**
		 * The speed of alignment, expressed as a lerp/slerp factor. 1 will blend most of the way in <1 second.
		 * (We actually do a lerp(s * delta_t) where s is the speed factor here)
		 */
		double align_speed_tiny, align_speed_small, align_speed_large;
	};

	struct SetDeviceTransform
	{
		uint32_t openVRID;
		bool enabled;
		bool updateTranslation;
		bool updateRotation;
		bool updateScale;
		vr::HmdVector3d_t translation;
		vr::HmdQuaternion_t rotation;
		double scale;
		bool lerp;
		bool quash;

		// Tracking system name of the device with this OpenVR ID, populated by the
		// overlay so the driver can match it against per-system fallbacks without
		// querying VR properties on every pose update. Empty string means "unknown".
		char target_system[MaxTrackingSystemNameLen];

		// Strength of native pose-prediction suppression for this device, on a
		// 0..100 scale. 0 = pose untouched (no suppression). 100 = velocity,
		// acceleration, angular velocity, angular acceleration and poseTimeOffset
		// are all zeroed before the pose ships -- defeating SteamVR's
		// extrapolation entirely (the old binary "freeze" behaviour). Values in
		// between scale velocity / acceleration by (1 - smoothness/100), giving
		// the user a smooth knob from "raw motion" to "fully suppressed".
		//
		// The overlay enforces a hard block on suppressing the calibration
		// reference / target trackers and the HMD: those values are forced to 0
		// regardless of what the user picks in the UI, because suppressing them
		// would corrupt either the calibration math or the user's view.
		uint8_t predictionSmoothness;

		// When true, BlendTransform's lerp toward targetTransform only advances
		// when the device is actually moving — instantaneous per-frame motion
		// gates the blend rate. The result: a stationary user (e.g. lying down)
		// sees no calibration drift even if the math is updating; the offset
		// catches up only when they next move, hidden by the natural motion.
		// Without this, sudden math updates (especially after watchdog clears)
		// cause visible "phantom body movement" while the user is still. Default
		// off in the struct — the overlay toggles it on per-device when the
		// recalibrateOnMovement profile setting is enabled.
		bool recalibrateOnMovement;

		SetDeviceTransform(uint32_t id, bool enabled) :
			openVRID(id), enabled(enabled), updateTranslation(false), updateRotation(false), updateScale(false), translation({}), rotation({1,0,0,0}), scale(1), lerp(false), quash(false), target_system{}, predictionSmoothness(0), recalibrateOnMovement(false) { }

		SetDeviceTransform(uint32_t id, bool enabled, vr::HmdVector3d_t translation) :
			openVRID(id), enabled(enabled), updateTranslation(true), updateRotation(false), updateScale(false), translation(translation), rotation({ 1,0,0,0 }), scale(1), lerp(false), quash(false), target_system{}, predictionSmoothness(0), recalibrateOnMovement(false) { }

		SetDeviceTransform(uint32_t id, bool enabled, vr::HmdQuaternion_t rotation) :
			openVRID(id), enabled(enabled), updateTranslation(false), updateRotation(true), updateScale(false), translation({}), rotation(rotation), scale(1), lerp(false), quash(false), target_system{}, predictionSmoothness(0), recalibrateOnMovement(false) { }

		SetDeviceTransform(uint32_t id, bool enabled, double scale) :
			openVRID(id), enabled(enabled), updateTranslation(false), updateRotation(false), updateScale(true), translation({}), rotation({ 1,0,0,0 }), scale(scale), lerp(false), quash(false), target_system{}, predictionSmoothness(0), recalibrateOnMovement(false) { }

		SetDeviceTransform(uint32_t id, bool enabled, vr::HmdVector3d_t translation, vr::HmdQuaternion_t rotation) :
			openVRID(id), enabled(enabled), updateTranslation(true), updateRotation(true), updateScale(false), translation(translation), rotation(rotation), scale(1), lerp(false), quash(false), target_system{}, predictionSmoothness(0), recalibrateOnMovement(false) { }

		SetDeviceTransform(uint32_t id, bool enabled, vr::HmdVector3d_t translation, vr::HmdQuaternion_t rotation, double scale) :
			openVRID(id), enabled(enabled), updateTranslation(true), updateRotation(true), updateScale(true), translation(translation), rotation(rotation), scale(scale), lerp(false), quash(false), target_system{}, predictionSmoothness(0), recalibrateOnMovement(false) { }
	};

	// Per-finger enable mask layout for FingerSmoothingConfig::finger_mask.
	// Bit (LSB-first):
	//   0  left thumb     5  right thumb
	//   1  left index     6  right index
	//   2  left middle    7  right middle
	//   3  left ring      8  right ring
	//   4  left pinky     9  right pinky
	// All 10 bits set = every finger smoothed (the typical opt-in case). Setting
	// a bit to 0 makes that finger pass through raw, useful when one finger's
	// smoothing produces an artifact and the user wants to isolate it without
	// disabling the whole feature.
	static const uint16_t kAllFingersMask = 0x03FF;

	// POD payload for RequestSetFingerSmoothing. Plain values + flag so the
	// struct is trivially memcpy-safe across the pipe with no marshalling.
	// Default-constructed instance encodes "feature off" (master_enabled = false)
	// — passthrough behaviour. The hook detour does zero work in that state, so
	// shipping with this struct in the union has zero performance cost when the
	// feature is unused.
	struct FingerSmoothingConfig
	{
		// Master kill-switch. DEPRECATED: the overlay always sends 1 here
		// (presence of resources/enable_smoothing.flag is the real master
		// toggle, gated at module load time). Driver-side reseed logic still
		// reads it for the wasOn/isOn diff; layout retained so existing
		// driver builds keep IPC compatibility across this rev. Do not gate
		// new logic on it -- treat it as always-true once the pipe is open.
		bool     master_enabled;

		// Smoothing strength on a 0..100 scale. 0 = pass-through (alpha=1.0,
		// each frame snaps to the incoming bones). 100 = heavy smoothing
		// (alpha clamped near 0, bones lag behind incoming significantly).
		// Linearly mapped to slerp factor by alpha = 1.0 - (smoothness/100)*0.95
		// so 100 still has a tiny per-frame nudge (alpha=0.05) — never fully
		// freezes a finger.
		uint8_t  smoothness;

		// Per-finger enable bits (see kAllFingersMask above). Disabled fingers
		// pass through unsmoothed. Bones outside the 5 finger chains
		// (root, wrist, aux markers) always pass through.
		uint16_t finger_mask;

		// 1 byte of trailing padding here on x64 to round to the natural
		// alignment of the struct. Explicit name so a future reader doesn't
		// mistake it for a meaningful flag — left zero by the overlay; the
		// driver MUST NOT read it.
		uint8_t  _reserved;

		// v13 (2026-05-11): per-finger smoothing strength (0..100), indexed the
		// same way as finger_mask bits 0..9 (left thumb..pinky, right thumb..pinky).
		// Replaces the role of the global `smoothness` field when v13 senders are
		// paired with v13 drivers; the global `smoothness` above is retained as a
		// fallback for clients that haven't been updated to write the array.
		// Per-finger value 0 = the driver falls back to the global `smoothness`
		// for that finger (so an all-zero array reproduces v12 behaviour exactly).
		uint8_t  per_finger_smoothness[10];

		// 2 bytes of trailing padding to round the struct to 8-byte alignment.
		// Left zero by the overlay; the driver MUST NOT read it.
		uint8_t  _reserved2[2];
	};

	// POD payload for RequestSetDevicePrediction. Sent by the Smoothing overlay
	// (v12+) to set per-device pose-prediction suppression without touching the
	// calibration transform that SpaceCalibrator owns. The driver updates
	// transforms[openVRID].predictionSmoothness and nothing else; transform,
	// scale, enabled, quash, target_system, recalibrateOnMovement stay where
	// SC last left them.
	//
	// predictionSmoothness still travels inside SetDeviceTransform for v11
	// wire compatibility, but the driver ignores it there from v12 onward;
	// only this message updates the slot. recalibrateOnMovement is a
	// calibration-blend concept and stays owned by SetDeviceTransform.
	struct SetDevicePrediction
	{
		uint32_t openVRID;
		uint8_t  predictionSmoothness;   // 0..100, see SetDeviceTransform notes
		// Smart smoothing: when non-zero, the driver treats
		// predictionSmoothness as the maximum strength to apply when the
		// device is stationary, and rolls it off toward 0 as the device's
		// linear or angular velocity rises. Stationary jitter is suppressed;
		// real motion (walking, fast aim) is not damped. 0 = static
		// behaviour (predictionSmoothness applied uniformly, prior wire
		// semantics). 1 = adaptive. Old overlays leave this byte zero so
		// they get the prior behaviour automatically.
		uint8_t  smart_enabled;
		uint8_t  _reserved[2];           // pad to 8-byte alignment; must be 0
	};

	// Per-tracking-system fallback transform. Applied to any device whose tracking
	// system matches `system_name` and that doesn't currently have an active per-ID
	// transform. Lets newly connected trackers inherit the calibrated offset
	// immediately, without waiting for the overlay's next scan tick.
	struct SetTrackingSystemFallback
	{
		char system_name[MaxTrackingSystemNameLen];
		bool enabled;
		vr::HmdVector3d_t translation;
		vr::HmdQuaternion_t rotation;
		double scale;
		// Same semantics as SetDeviceTransform::predictionSmoothness (0..100).
		// Applied to every device that picks up this fallback (i.e. every device
		// of the matching tracking system that doesn't have an active per-ID
		// transform). The HMD/ref/target hard-block also applies here.
		uint8_t predictionSmoothness;
		// Same semantics as SetDeviceTransform::recalibrateOnMovement. Applies to
		// every device that picks up this fallback so newly-connected matching-
		// system trackers also get motion-gated blend instead of an instant snap
		// when their first per-ID transform later arrives.
		bool recalibrateOnMovement;
	};

	// POD payload for RequestSetInputHealthConfig. Sized to fit inside an
	// 8-byte atomic so the per-tick read on the driver's input detours can be
	// a single relaxed load (same trick as FingerSmoothingConfig). Default
	// construction encodes "feature off and diagnostics-only" so the detour
	// path stays passthrough until the overlay sends a real config.
	struct InputHealthConfig
	{
		// Master kill-switch. False = the boolean/scalar detours forward
		// component updates untouched and the background worker stays idle.
		bool      master_enabled;

		// When true, the driver records observations and publishes snapshots
		// but never alters component values flowing up to consumers. Useful
		// for triage; the overlay flips this off by default so corrections
		// land as soon as a path crosses its readiness gate.
		bool      diagnostics_only;

		// Per-category compensation toggles. The overlay flips these on by
		// default; have no effect while diagnostics_only is true.
		bool      enable_rest_recenter;
		bool      enable_trigger_remap;

		// Trailing padding. Named so a future reader
		// doesn't mistake it for a meaningful flag; left zero by the overlay.
		uint16_t  _reserved;
	};

	// POD payload for RequestResetInputHealthStats. Identifies the device by
	// hashed serial (8 bytes; client uses FNV-1a or similar deterministic
	// hash on the ETrackedDeviceProperty serial string) and lists which
	// stat categories to wipe.
	struct InputHealthResetStats
	{
		uint64_t device_serial_hash;
		uint8_t  reset_passive;   // wipe Welford / PH / EWMA / polar bins
		uint8_t  reset_active;    // wipe wizard-recorded calibration prior
		uint8_t  reset_curves;    // wipe applied compensation curves
		uint8_t  _reserved[5];
	};

	enum InputHealthCompensationKind : uint8_t
	{
		InputHealthCompScalarSingle = 0,
		InputHealthCompStickX       = 1,
		InputHealthCompStickY       = 2,
		InputHealthCompBoolean      = 3,
	};

	struct InputHealthCompensationEntry
	{
		uint64_t device_serial_hash;
		char     path[INPUTHEALTH_PATH_LEN];
		uint8_t  kind;
		uint8_t  enabled;
		uint8_t  _pad[2];
		float    learned_rest_offset;
		float    learned_trigger_min;
		float    learned_trigger_max;
		float    learned_deadzone_radius;
		uint16_t learned_debounce_us;
		uint16_t _reserved;
	};

	// Maximum length of an OSC host string (dotted-quad or short hostname). 40
	// bytes covers anything reasonable, leaving NUL termination room. Truncation
	// on overflow is the caller's responsibility -- the driver treats the buffer
	// as a NUL-terminated string and stops at the first zero byte.
	static const size_t FACETRACKING_OSC_HOST_LEN = 40;

	// Stable string length for a hardware-module identity. The C# host generates
	// these as RFC 4122 UUID strings; the driver only matches them as opaque
	// NUL-terminated tokens.
	static const size_t FACETRACKING_MODULE_UUID_LEN = 40;

	// v15 (2026-05-12): face-tracking master config.
	//
	// POD-only, fits in the existing Request union -- the static_assert below
	// keeps it under sizeof(SetDeviceTransform). The four-tier toggle / strength
	// design lets the user disable any single feature (eyelid sync, vergence
	// lock, continuous calibration) without rebuilding the pipeline, and
	// supports independent OSC + native output (both can be on; user picks).
	struct FaceTrackingConfig
	{
		// Master kill-switch. False = driver's pose-update path forwards
		// hardware-side face/eye values unmodified (or, if the host isn't
		// running yet, no inputs are published at all). Same fast-path-to-
		// passthrough discipline as the other modules.
		uint8_t master_enabled;

		// Per-feature toggles. Each can be flipped independently; the
		// strength sliders below control how aggressive each is when on.
		uint8_t eyelid_sync_enabled;
		uint8_t eyelid_sync_preserve_winks;
		uint8_t vergence_lock_enabled;

		// 0=off, 1=conservative (slow EMA decay, tight outlier gates),
		// 2=aggressive (faster decay, looser gates). Conservative is the
		// default for cold-start users.
		uint8_t continuous_calib_mode;

		// Output sink toggles. The host process sends OSC to VRChat when
		// output_osc_enabled is set. The native OpenXR eye-gaze path is
		// reserved for Phase 3 -- always zero for now.
		uint8_t output_osc_enabled;
		uint8_t _reserved_native;   // was output_native_enabled; Phase 3
		uint8_t _reserved1;

		// Sync feature strengths on a 0..100 scale, identical semantics to
		// the smoothing module's existing scale. 0=feature observable but no
		// effect, 100=feature fully forces both eyes / eyelids to converge.
		uint8_t eyelid_sync_strength;
		uint8_t vergence_lock_strength;
		uint8_t gaze_smoothing;
		uint8_t openness_smoothing;

		// OSC target. Driver forwards these to the host over the host control
		// pipe; the host owns the UDP socket. Default 127.0.0.1:9000 (VRChat).
		uint16_t osc_port;
		uint16_t _reserved2;
		char     osc_host[FACETRACKING_OSC_HOST_LEN];

		// Active hardware module. Empty string = host picks automatically
		// (first available). Non-empty = host loads the module with the
		// matching uuid and ignores others until the user picks again.
		char     active_module_uuid[FACETRACKING_MODULE_UUID_LEN];
	};

	enum FaceCalibrationOp : uint8_t
	{
		FaceCalibBegin     = 0,
		FaceCalibEnd       = 1,
		FaceCalibSave      = 2,
		FaceCalibResetAll  = 3,
		FaceCalibResetEye  = 4,
		FaceCalibResetExpr = 5,
	};

	// POD payload for RequestSetFaceCalibrationCommand. Carries an op code
	// for the driver-side CalibrationEngine to act on. Per-shape resets and
	// finer-grained controls go through the host control pipe (CBOR), not
	// here -- this is the small, fixed-shape, IPC-fast path.
	struct FaceCalibrationCommand
	{
		uint8_t op;        // see FaceCalibrationOp
		uint8_t _reserved[7];
	};

	// POD payload for RequestSetFaceActiveModule. Picks which hardware
	// module the host should load (Quest Pro, Vive FT, etc.). Driver
	// forwards this to the host process over the internal control pipe
	// the next time the host is reachable.
	struct FaceModuleSelection
	{
		char     uuid[FACETRACKING_MODULE_UUID_LEN];
		uint8_t  _reserved[8];
	};

	// =========================================================================
	// v16: OSC Router protocol additions
	// =========================================================================

	// Opaque token assigned by the overlay subscriber; driver echoes it in
	// per-route stats so the overlay can correlate routes to its own state.
	using OscSubscriberId = uint32_t;

	// Maximum OSC address length stored in the route table. OSC 1.0 allows
	// arbitrary length; 64 bytes covers all known avatar parameter paths.
	// The OOP publish pipe accepts addresses up to 128 bytes (larger buffer).
	static const size_t OSC_ROUTE_ADDR_LEN = 64;

	// POD payload for RequestOscRouteSubscribe. The overlay registers an
	// interest pattern; the driver adds the route and begins counting matches.
	// Pattern follows OSC 1.0 glob: ?, *, [abc], [a-z], {a,b}.
	// Multiple subscribers may overlap on the same pattern -- each is
	// independent with its own counter in the stats shmem slot.
	struct OscRouteSubscribe
	{
		OscSubscriberId subscriber_id;   // assigned by caller; echoed in stats
		uint8_t         enabled;         // 0 = disable (keep slot); 1 = active
		uint8_t         _reserved[3];    // pad to 8-byte boundary; must be 0
		char            pattern[OSC_ROUTE_ADDR_LEN]; // NUL-terminated OSC glob
	};

	// POD payload for RequestOscRouteUnsubscribe.
	struct OscRouteUnsubscribe
	{
		OscSubscriberId subscriber_id;
		uint8_t         _reserved[4];
	};

	// POD payload for RequestOscPublish. Overlay-originated publish (e.g. the
	// "send test message" button in the Modules tab). The driver serialises
	// the address + typetag + arg bytes into a raw OSC packet and sends it to
	// the configured outbound endpoint (default 127.0.0.1:9000).
	// Typetag follows OSC 1.0 (",f", ",i", ",s", ",ff", etc.).
	// arg_bytes must be pre-encoded in OSC 1.0 wire format (big-endian,
	// padded to 4-byte boundary) -- the driver forwards them verbatim after
	// the address and typetag.
	static const size_t OSC_PUBLISH_TYPETAG_LEN = 8;
	// 32 bytes covers 8 floats or 1 int32 + 6 floats etc. Overlay test-publish
	// only needs to send one float (the most common avatar parameter). Larger
	// payloads (chatbox strings) go through the OOP publish pipe instead.
	static const size_t OSC_PUBLISH_ARG_LEN     = 32;

	struct OscPublish
	{
		char    address[OSC_ROUTE_ADDR_LEN];
		char    typetag[OSC_PUBLISH_TYPETAG_LEN];
		uint8_t arg_len;      // actual used bytes in arg_bytes
		uint8_t _reserved[7]; // pad; must be 0
		uint8_t arg_bytes[OSC_PUBLISH_ARG_LEN];
	};

	// POD payload for ResponseOscRouterStats. Carries global totals for the
	// overlay's Modules tab; the per-route breakdown lives in the shmem segment
	// and is read directly. This response is sent on RequestOscGetStats.
	struct OscRouterStats
	{
		uint64_t packets_sent;    // total OSC datagrams sent since driver start
		uint64_t bytes_sent;      // total bytes sent since driver start
		uint64_t packets_dropped; // dropped due to full send queue
		uint32_t active_routes;   // current number of active route table entries
		uint32_t _reserved;
	};

	// =========================================================================
	// Captions module protocol additions (no version bump; appended after v16)
	// =========================================================================

	// Maximum field widths for captions config strings.
	// These are kept small so CaptionsConfig fits inside SetDeviceTransform.
	static const size_t CAPTIONS_LANG_LEN = 16;  // BCP-47 code, e.g. "en", "zh", "auto"
	static const size_t CAPTIONS_ADDR_LEN = 48;  // OSC address, e.g. "/chatbox/input"

	// Captions operating mode.
	enum CaptionsMode : uint8_t
	{
		CaptionsModePushToTalk = 0,
		CaptionsModeAlwaysOn   = 1,
	};

	// POD payload for RequestSetCaptionsConfig. The overlay pushes this
	// whenever the user changes any setting. The driver caches and forwards
	// it to the sidecar over the host control pipe. All string fields are
	// NUL-terminated; the driver truncates silently on overflow.
	//
	// Model paths are not carried over IPC (they are large and change
	// infrequently); the host reads them from a local config JSON file.
	//
	// sizeof(CaptionsConfig) must not exceed sizeof(SetDeviceTransform)
	// -- enforced by the static_assert below.
	struct CaptionsConfig
	{
		// Master enable. 0 = sidecar runs but produces no output (muted).
		uint8_t  master_enabled;

		// Operating mode: PTT or always-on.
		uint8_t  mode;                      // CaptionsMode

		// Notification sound on chatbox send. 0 = silent.
		uint8_t  notify_sound;

		// When nonzero the sidecar writes transcripts to disk.
		uint8_t  transcript_logging;

		// Target chatbox OSC port (default 9000).
		uint16_t chatbox_port;

		// Padding to align the string fields on a natural boundary.
		uint8_t  _pad[2];

		// Source language code ("auto" = whisper auto-detect).
		char     source_lang[CAPTIONS_LANG_LEN];

		// Target language code ("" = transcribe only, no translation).
		char     target_lang[CAPTIONS_LANG_LEN];

		// Chatbox OSC address (default "/chatbox/input").
		char     chatbox_address[CAPTIONS_ADDR_LEN];
	};

	// Response payload for RequestCaptionsGetSupervisorStatus.
	struct CaptionsSupervisorStatus
	{
		// 1 if the circuit breaker has tripped (5 consecutive fast exits);
		// 0 otherwise. When 1, the host will not be respawned until the
		// driver module is restarted or Restart is explicitly requested.
		uint8_t host_halted;
		uint8_t _pad[3];
		uint32_t last_exit_code;
		char last_exit_description[128];
	};

	// POD payload for RequestSetPhantomConfig. Phantom module global config:
	// master switch + the five timeout-ladder values. Per-device dropout
	// opt-in travels in RequestSetPhantomDeviceOptIn (one message per
	// toggle) so this struct stays small and the overlay does not resend
	// the full opt-in map on each slider drag.
	struct PhantomConfig
	{
		// Master switch. False = the phantom hook fast-paths to passthrough
		// (PoseHistory still records so a later master-on toggle has back-
		// history available, but no synthesis is applied and no virtual
		// devices are added).
		uint8_t  master_enabled;
		uint8_t  _pad0[3];

		// Timeout ladder values, all in milliseconds. Defaults track the
		// constants in modules/phantom/src/common/BlendCurves.h:
		//   blend_out_ms  = 80
		//   blend_in_ms   = 150
		//   reckon_hold_ms = 250
		//   synth_hold_ms = 2000  (after this, ETrackingResult flips to
		//                          Running_OutOfRange so consumers gracefully
		//                          drop the tracker)
		//   lost_hold_ms  = 5000  (after this, stop publishing entirely)
		uint32_t blend_out_ms;
		uint32_t blend_in_ms;
		uint32_t reckon_hold_ms;
		uint32_t synth_hold_ms;
		uint32_t lost_hold_ms;
	};

	// POD payload for RequestSetPhantomDeviceOptIn. The overlay sends one of
	// these whenever the user toggles dropout bridging for a specific device
	// in the per-tracker list. Identified by FNV-1a 64-bit hash of the
	// device serial string (matches the InputHealth convention).
	struct PhantomDeviceOptIn
	{
		uint64_t device_serial_hash;
		uint8_t  dropout_enabled;
		uint8_t  _reserved[7];
	};

	// POD payload for RequestSetPhantomDeviceRole. Maps a physical tracker's
	// serial hash to a body role (BodyRole enum; see RoleCatalog.h). The
	// overlay sends one of these per assignment, including a clearing
	// message with body_role = 0 (None) when the user un-assigns. The IK
	// fallback consults the role -> offset table; this message tells the
	// driver which OpenVR device feeds which role.
	struct PhantomDeviceRole
	{
		uint64_t device_serial_hash;
		uint8_t  body_role;      // BodyRole enum value
		uint8_t  _reserved[7];
	};

	// POD payload for RequestSetPhantomTrackerOffset. Carries the rigid
	// HMD-relative offset captured during the T-pose wizard for the
	// addressed body role. calibrated = 0 clears the slot; the driver
	// then falls back to dead reckoning for the role. The rotation is a
	// unit quaternion (w,x,y,z); the position is in metres in the HMD's
	// local frame at the moment of capture.
	struct PhantomTrackerOffset
	{
		uint8_t  body_role;
		uint8_t  calibrated;
		uint8_t  _pad[6];
		double   rel_position[3];
		vr::HmdQuaternion_t rel_rotation;
	};

	// POD payload for RequestSetPhantomVirtualEnabled. Per-role boolean for
	// absent-mode virtual trackers. The driver only honours the request
	// when the role has a T-pose calibration on file (otherwise the
	// virtual tracker would have no pose source). Flipping off removes
	// the virtual device from future creation; the live instance lives
	// out the current vrserver process.
	struct PhantomVirtualEnabled
	{
		uint8_t body_role;
		uint8_t enabled;
		uint8_t _reserved[6];
	};

	struct Request
	{
		RequestType type;

		union {
			SetDeviceTransform setDeviceTransform;
			AlignmentSpeedParams setAlignmentSpeedParams;
			SetTrackingSystemFallback setTrackingSystemFallback;
			// v9: finger smoothing. FingerSmoothingConfig is much smaller than
			// SetDeviceTransform so this addition does not grow the union and
			// therefore does not change sizeof(Request). The Version bump
			// is purely to force paired install -- the wire layout is otherwise
			// backwards-compatible.
			FingerSmoothingConfig setFingerSmoothing;
			// v10: input-health config + stats reset. Both structs are much
			// smaller than SetDeviceTransform so the union does not grow.
			// Same paired-install rationale as v9: bump Version to make a
			// version skew show up at the handshake instead of as a silently
			// ignored RequestType in the dispatcher's default branch.
			InputHealthConfig setInputHealthConfig;
			InputHealthResetStats resetInputHealthStats;
			InputHealthCompensationEntry setInputHealthCompensation;
			// v12: per-device prediction smoothness from the Smoothing overlay.
			// Much smaller than SetDeviceTransform so the union does not grow.
			SetDevicePrediction setDevicePrediction;
			// v15: face-tracking master config + calibration commands + module
			// selection. All three are smaller than SetDeviceTransform so the
			// union does not grow; the static_asserts below enforce that.
			FaceTrackingConfig     setFaceTrackingConfig;
			FaceCalibrationCommand setFaceCalibrationCommand;
			FaceModuleSelection    setFaceActiveModule;
			// v16: OSC router control. All three are smaller than
			// SetDeviceTransform so the union does not grow.
			OscRouteSubscribe   oscRouteSubscribe;
			OscRouteUnsubscribe oscRouteUnsubscribe;
			OscPublish          oscPublish;
			// Captions: config push. Smaller than SetDeviceTransform;
			// static_assert below enforces it.
			CaptionsConfig    setCaptionsConfig;
			// v19: phantom-tracker global config + per-device opt-in. Both
			// are smaller than SetDeviceTransform so the union does not
			// grow; static_asserts below enforce it.
			PhantomConfig     setPhantomConfig;
			PhantomDeviceOptIn setPhantomDeviceOptIn;
			// v20: phantom Phase 1.5 calibration -- per-serial role
			// assignment + per-role rigid offset from HMD. Both smaller
			// than SetDeviceTransform; static_asserts pin the size.
			PhantomDeviceRole     setPhantomDeviceRole;
			PhantomTrackerOffset  setPhantomTrackerOffset;
			// v21: phantom Phase 2 absent-mode virtual-tracker toggle.
			PhantomVirtualEnabled setPhantomVirtualEnabled;
		};

		Request() : type(RequestInvalid), setAlignmentSpeedParams({}) { }
		Request(RequestType type) : type(type), setAlignmentSpeedParams({}) { }
		Request(AlignmentSpeedParams params) : type(RequestType::RequestSetAlignmentSpeedParams), setAlignmentSpeedParams(params) {}
	};

	static_assert(sizeof(InputHealthCompensationEntry) <= sizeof(SetDeviceTransform),
		"InputHealthCompensationEntry must not grow Request");
	static_assert(sizeof(FaceTrackingConfig) <= sizeof(SetDeviceTransform),
		"FaceTrackingConfig must not grow Request");
	static_assert(sizeof(FaceCalibrationCommand) <= sizeof(SetDeviceTransform),
		"FaceCalibrationCommand must not grow Request");
	static_assert(sizeof(FaceModuleSelection) <= sizeof(SetDeviceTransform),
		"FaceModuleSelection must not grow Request");
	static_assert(sizeof(OscRouteSubscribe) <= sizeof(SetDeviceTransform),
		"OscRouteSubscribe must not grow Request");
	static_assert(sizeof(OscRouteUnsubscribe) <= sizeof(SetDeviceTransform),
		"OscRouteUnsubscribe must not grow Request");
	static_assert(sizeof(OscPublish) <= sizeof(SetDeviceTransform),
		"OscPublish must not grow Request");
	static_assert(sizeof(CaptionsConfig) <= sizeof(SetDeviceTransform),
		"CaptionsConfig must not grow Request");
	static_assert(sizeof(PhantomConfig) <= sizeof(SetDeviceTransform),
		"PhantomConfig must not grow Request");
	static_assert(sizeof(PhantomDeviceOptIn) <= sizeof(SetDeviceTransform),
		"PhantomDeviceOptIn must not grow Request");
	static_assert(sizeof(PhantomDeviceRole) <= sizeof(SetDeviceTransform),
		"PhantomDeviceRole must not grow Request");
	static_assert(sizeof(PhantomTrackerOffset) <= sizeof(SetDeviceTransform),
		"PhantomTrackerOffset must not grow Request");
	static_assert(sizeof(PhantomVirtualEnabled) <= sizeof(SetDeviceTransform),
		"PhantomVirtualEnabled must not grow Request");

	struct Response
	{
		ResponseType type;

		union {
			Protocol                  protocol;
			OscRouterStats            oscRouterStats;           // v16: ResponseOscRouterStats
			CaptionsSupervisorStatus captionsSupervisorStatus; // Captions: supervisor state
		};

		Response() : type(ResponseInvalid), protocol({}) {}
		Response(ResponseType type) : type(type), protocol({}) { }
	};

	class DriverPoseShmem {
	public:
		struct AugmentedPose {
			LARGE_INTEGER sample_time;
			int deviceId;
			vr::DriverPose_t pose;
		};

		// Driver-side telemetry counters. Each is a monotonically increasing count of
		// the number of times the driver took the corresponding code path while
		// processing a tracked-device pose update. Relaxed-ordered atomic increments
		// are sufficient because the overlay only consumes deltas — there is no
		// cross-counter consistency requirement.
		struct Telemetry {
			std::atomic<uint64_t> fallback_apply_count;
			std::atomic<uint64_t> per_id_apply_count;
			std::atomic<uint64_t> quash_apply_count;
			std::atomic<uint64_t> reserved[5];
		};

		// Names of the individual telemetry counters, used by IncrementTelemetry to
		// pick which atomic to bump. Kept inside the class so we don't pollute the
		// `protocol` namespace with another enum.
		enum TelemetryField {
			TELEMETRY_FALLBACK_APPLY,
			TELEMETRY_PER_ID_APPLY,
			TELEMETRY_QUASH_APPLY,
		};

		// Sentinel written by the driver into ShmemData::magic. The overlay
		// rejects any segment whose magic doesn't match — guards against
		// reading a stale or wrong-format mapping left behind by a different
		// build.
		static const uint32_t SHMEM_MAGIC = 0xCA11B8A7;

		// Version of the shmem layout. Bump on any field addition / reordering
		// / removal so a driver/overlay version skew is rejected at Open()
		// instead of corrupting poses with mismatched offsets.
		static const uint32_t SHMEM_VERSION = 1;

	private:
		static const uint32_t SYNC_ACTIVE_POSE_B = 0x80000000;
		static const uint32_t BUFFERED_SAMPLES = 64 * 1024;

		struct ShmemData {
			// Magic + version sit first so the overlay can validate a mapping
			// before it touches anything else. Driver Create() writes them; the
			// overlay Open() reads and verifies. A mismatch means the overlay
			// is paired with a different driver build — far better to throw
			// than to silently feed it garbage poses.
			uint32_t magic;
			uint32_t shmem_version;
			// Telemetry sits right after the header so any future growth of the
			// pose ring buffer doesn't shift its address. The shmem layout is
			// regenerated on driver startup and the overlay/driver IPC handshake
			// already gates compatible builds, so a layout change is acceptable
			// — but keeping telemetry pinned to a stable offset keeps debugging
			// dumps readable.
			Telemetry telemetry;
			std::atomic<uint64_t> index;
			AugmentedPose poses[BUFFERED_SAMPLES];
		};

		// Compile-time guard against silent layout drift between driver and
		// overlay builds. If a field is added / reordered / repacked this
		// assertion fires at compile time instead of producing corrupted poses
		// at runtime. Sum of every field's sizeof — the trailing struct ends
		// 8-aligned and every field starts on its natural boundary so the sum
		// matches the struct size exactly. If you intentionally change the
		// layout, bump SHMEM_VERSION and update this assertion.
		static_assert(
			sizeof(ShmemData) ==
				sizeof(uint32_t) + sizeof(uint32_t) +
				sizeof(Telemetry) +
				sizeof(std::atomic<uint64_t>) +
				sizeof(AugmentedPose) * BUFFERED_SAMPLES,
			"DriverPoseShmem::ShmemData layout has drifted; bump SHMEM_VERSION and update the static_assert"
		);
		
	private:
		HANDLE hMapFile;
		ShmemData* pData;
		uint64_t cursor;

		AugmentedPose lastPose[vr::k_unMaxTrackedDeviceCount] = {0};

		std::string LastErrorString(DWORD lastError)
		{
			LPSTR buffer = nullptr;
			size_t size = FormatMessageA(
				FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
				NULL, lastError, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), (LPSTR)&buffer, 0, NULL
			);

			std::string message(buffer, size);
			LocalFree(buffer);
			return message;
		}

	public:
		operator bool() const {
			return pData != nullptr;
		}

		bool operator!() const {
			return pData == nullptr;
		}

		DriverPoseShmem() {
			hMapFile = INVALID_HANDLE_VALUE;
			pData = nullptr;
			cursor = 0;
		}

		~DriverPoseShmem() {
			Close();
		}

		void Close() {
			// Null the members after release so that a second Close() call
			// (the destructor running after Create() already invoked Close()
			// for re-init, then failing partway through) does not double-free
			// stale pointers. The other shmem classes in this file follow
			// the same pattern.
			if (pData) {
				UnmapViewOfFile(pData);
				pData = nullptr;
			}
			if (hMapFile && hMapFile != INVALID_HANDLE_VALUE) {
				CloseHandle(hMapFile);
				hMapFile = INVALID_HANDLE_VALUE;
			}
		}

		bool Create(LPCSTR segment_name) {
			Close();

			hMapFile = CreateFileMappingA(
				INVALID_HANDLE_VALUE,
				NULL,
				PAGE_READWRITE,
				0,
				sizeof(ShmemData),
				segment_name
			);

			if (!hMapFile) return false;

			pData = reinterpret_cast<ShmemData*>(MapViewOfFile(
				hMapFile,
				FILE_MAP_ALL_ACCESS,
				0,
				0,
				sizeof(ShmemData)
			));

			if (!pData) return false;

			// Stamp the header so the overlay can verify it's looking at a
			// driver-compatible mapping. New file mappings are zero-initialised
			// by the OS, so simply writing the magic/version is enough.
			pData->magic = SHMEM_MAGIC;
			pData->shmem_version = SHMEM_VERSION;

			return true;
		}


		void Open(LPCSTR segment_name) {
			Close();

			hMapFile = OpenFileMappingA(
				FILE_MAP_ALL_ACCESS,
				FALSE,
				segment_name
			);

			if (!hMapFile) {
				throw std::runtime_error("Failed to open pose data shared memory segment: " + LastErrorString(GetLastError()));
			}

			pData = reinterpret_cast<ShmemData*>(MapViewOfFile(
				hMapFile,
				FILE_MAP_ALL_ACCESS,
				0,
				0,
				sizeof(ShmemData)
			));

			if (!pData) {
				throw std::runtime_error("Failed to map pose data shared memory segment: " + LastErrorString(GetLastError()));
			}

			// Validate the header. A mismatch means the overlay is paired with a
			// driver build that doesn't share this layout — much safer to throw
			// here than to silently corrupt poses by reading at the wrong offsets.
			if (pData->magic != SHMEM_MAGIC) {
				char buf[128];
				snprintf(buf, sizeof buf,
					"Pose shmem segment magic mismatch: got 0x%08X, expected 0x%08X (driver/overlay version skew?)",
					pData->magic, SHMEM_MAGIC);
				UnmapViewOfFile(pData); pData = nullptr;
				CloseHandle(hMapFile); hMapFile = nullptr;
				throw std::runtime_error(buf);
			}
			if (pData->shmem_version != SHMEM_VERSION) {
				char buf[128];
				snprintf(buf, sizeof buf,
					"Pose shmem segment version mismatch: got %u, expected %u (driver/overlay version skew?)",
					pData->shmem_version, SHMEM_VERSION);
				UnmapViewOfFile(pData); pData = nullptr;
				CloseHandle(hMapFile); hMapFile = nullptr;
				throw std::runtime_error(buf);
			}

			char tmp[256];
			snprintf(tmp, sizeof tmp, "Opened shmem segment: %p\n", pData);
			OutputDebugStringA(tmp);
		}

		void ReadNewPoses(std::function<void(AugmentedPose const&)> cb) {
			if (!pData) throw std::runtime_error("Not open");
			
			uint64_t cur_index = pData->index.load(std::memory_order_acquire);
			if (cur_index < cursor || cur_index - cursor > BUFFERED_SAMPLES / 2) {
				if (cur_index < BUFFERED_SAMPLES / 2)
					cursor = cur_index;
				else
					cursor = cur_index - BUFFERED_SAMPLES / 2;
			}

			while (cursor < cur_index) {
				cb(pData->poses[cursor % BUFFERED_SAMPLES]);
				cursor++;
			}

			std::atomic_thread_fence(std::memory_order_release);
		}

		bool GetPose(int index, vr::DriverPose_t& pose, LARGE_INTEGER *pSampleTime = NULL) {
			ReadNewPoses([this](AugmentedPose const& pose) {
				if (pose.pose.poseIsValid && pose.pose.result == vr::ETrackingResult::TrackingResult_Running_OK) {
					this->lastPose[pose.deviceId] = pose;
				}
			});

			if (index >= 0 && index < vr::k_unMaxTrackedDeviceCount) {
				pose = lastPose[index].pose;
				if (pSampleTime) *pSampleTime = lastPose[index].sample_time;
				return true;
			}
			// Out-of-range index: don't touch `pose`, signal failure. The caller
			// must check the return value (the prior version fell off the end of
			// a non-void function — undefined behaviour, papered over only by the
			// fact that nothing actually called this with an out-of-range index).
			return false;
		}

		void SetPose(int index, const vr::DriverPose_t& pose) {
			if (index >= vr::k_unMaxTrackedDeviceCount) return;
			if (pData == nullptr) return;

			AugmentedPose augPose = {0};
			augPose.deviceId = index;
			augPose.pose = pose;
			QueryPerformanceCounter(&augPose.sample_time);

			uint64_t cur_index = pData->index.load(std::memory_order_relaxed) + 1;
			pData->poses[cur_index % BUFFERED_SAMPLES] = augPose;
			pData->index.store(cur_index, std::memory_order_release);
		}

		// Bump the named telemetry counter by one. Safe to call from the driver's
		// pose-update path — a relaxed atomic increment is sub-ns on x86 and there
		// are no cross-counter ordering requirements.
		void IncrementTelemetry(TelemetryField field) {
			if (pData == nullptr) return;
			switch (field) {
			case TELEMETRY_FALLBACK_APPLY:
				pData->telemetry.fallback_apply_count.fetch_add(1, std::memory_order_relaxed);
				break;
			case TELEMETRY_PER_ID_APPLY:
				pData->telemetry.per_id_apply_count.fetch_add(1, std::memory_order_relaxed);
				break;
			case TELEMETRY_QUASH_APPLY:
				pData->telemetry.quash_apply_count.fetch_add(1, std::memory_order_relaxed);
				break;
			default:
				// Unknown enum value: silently ignore. Explicit default silences
				// implicit-fall-through warnings and documents that no other
				// counters exist today; new fields must be added above.
				break;
			}
		}

		// Snapshot the telemetry counters. Returns true if the shmem segment is open.
		// The values are loaded with relaxed ordering — the overlay only ever
		// computes deltas across snapshots, so torn reads relative to other counters
		// don't matter.
		bool GetTelemetry(uint64_t& fallback_apply, uint64_t& per_id_apply, uint64_t& quash_apply) {
			if (pData == nullptr) return false;
			fallback_apply = pData->telemetry.fallback_apply_count.load(std::memory_order_relaxed);
			per_id_apply = pData->telemetry.per_id_apply_count.load(std::memory_order_relaxed);
			quash_apply = pData->telemetry.quash_apply_count.load(std::memory_order_relaxed);
			return true;
		}
	};

	// =========================================================================
	// InputHealth snapshot shmem.
	//
	// Slot table layout. Each slot holds a per-VRInputComponentHandle_t snapshot
	// of the driver-side stats; the InputHealth overlay reads slots whose
	// `handle` field is non-zero, validates per-slot seqlock, and renders.
	// The driver writer thread runs at ~10 Hz, the slot table is sized to
	// cover any realistic controller topology (256 components).
	// =========================================================================

	// Number of bins in the polar histogram mirror. Matches
	// inputhealth::kBinCount in src/common/inputhealth/PolarHistogram.h. Wire
	// stability is enforced here -- the source-side enum could change
	// independently as long as both sides recompile against this header.
	static const uint32_t INPUTHEALTH_POLAR_BIN_COUNT = 36;

	// Maximum number of component slots in the shmem table. 256 covers any
	// realistic OpenVR topology (Index Knuckles publishes ~50 components per
	// hand; budget hardware is well under).
	static const uint32_t INPUTHEALTH_SLOT_COUNT = 256;

	// Per-slot snapshot body. Plain POD; the writer memcpy's it into the
	// slot's body field under a per-slot seqlock counter (held in the
	// surrounding InputHealthSnapshotSlot, separately from this body).
	// Field order is wire-stable; if a field is added or reordered bump
	// InputHealthSnapshotShmem::SHMEM_VERSION.
	struct InputHealthSnapshotBody
	{
		// Driver-side identity. handle == 0 marks an empty slot the writer
		// has never used or has explicitly retired (e.g. after Shutdown).
		uint64_t handle;
		uint64_t container_handle;
		uint64_t device_serial_hash;
		uint64_t partner_handle;

		// Component metadata.
		char     path[INPUTHEALTH_PATH_LEN];
		uint8_t  is_scalar;
		uint8_t  is_boolean;
		uint8_t  axis_role;       // inputhealth::AxisRole
		uint8_t  ph_initialized;
		uint8_t  ph_triggered;
		uint8_t  ph_triggered_positive;
		uint8_t  rest_min_initialized;
		uint8_t  last_boolean;

		// Welford streaming mean / variance.
		uint64_t welford_count;
		double   welford_mean;
		double   welford_m2;

		// Page-Hinkley accumulator.
		double   ph_mean;
		double   ph_pos;
		double   ph_neg;

		// EWMA-decayed rolling minimum (rest-stuck detection on triggers).
		double   rest_min;

		// Last raw sample on this handle (last_value for scalars, latest
		// boolean state for booleans -- redundant with last_boolean above
		// for booleans, kept consistent for scalars).
		float    last_value;
		uint32_t _pad_lv;

		uint64_t last_update_us;
		uint64_t press_count;

		// Raw scalar range observed since last reset. Cheap O(1) driver-side
		// bookkeeping used by the overlay to sanity-check trigger/stick
		// calibration coverage before making any health claim.
		uint8_t  scalar_range_initialized;
		uint8_t  _pad_scalar_range_flags[3];
		float    observed_min;
		float    observed_max;
		uint32_t _pad_observed_range;

		// Polar histogram bins. Only meaningful for AxisRole::StickX;
		// other roles leave these zero. Per-bin: max_r and count and
		// last_update_us.
		float    polar_max_r[INPUTHEALTH_POLAR_BIN_COUNT];
		uint16_t polar_count[INPUTHEALTH_POLAR_BIN_COUNT];
		uint16_t _pad_polar_count[INPUTHEALTH_POLAR_BIN_COUNT];   // align next field to 8
		uint64_t polar_last_update_us[INPUTHEALTH_POLAR_BIN_COUNT];
		float    polar_global_max_r;
		uint32_t _pad_polar_global;
	};

	// Per-slot record: an atomic seqlock counter followed by the body. The
	// counter is incremented twice per write (odd during, even after) so the
	// reader can detect torn reads. The body is memcpy'd in/out -- never
	// touched field-by-field by the wire layer.
	struct InputHealthSnapshotSlot
	{
		std::atomic<uint64_t>     generation;
		InputHealthSnapshotBody   body;
	};

	// Static asserts to keep wire layout stable across builds. If a field is
	// added or reordered, recompute the expected size and bump SHMEM_VERSION.
	static_assert(std::is_trivially_copyable<InputHealthSnapshotBody>::value,
		"InputHealthSnapshotBody must be trivially copyable for shmem use");
	static_assert(std::is_standard_layout<InputHealthSnapshotBody>::value,
		"InputHealthSnapshotBody must be standard-layout for stable wire format");

	class InputHealthSnapshotShmem
	{
	public:
		// Sentinel + version. Bump version on any field change in
		// InputHealthSnapshotRecord or the header. Mismatched versions are
		// rejected at Open() so the overlay never reads the wrong layout.
		static const uint32_t SHMEM_MAGIC   = 0x494E4848; // 'INHH'
		static const uint32_t SHMEM_VERSION = 2;

	private:
		struct ShmemData
		{
			// Header fields stay at offset 0 so the overlay can validate
			// before touching anything else.
			uint32_t magic;
			uint32_t shmem_version;

			// Number of slots in the table. Pinned to INPUTHEALTH_SLOT_COUNT
			// at create time; written here so a future expansion can be
			// detected without bumping SHMEM_VERSION (the overlay caps its
			// iteration at min(slot_count, INPUTHEALTH_SLOT_COUNT)).
			uint32_t slot_count;

			// Monotonic publish-tick counter. Bumped each time the driver
			// finishes writing a full pass over the slot table. The overlay
			// uses this as a dirty-frame signal so the diagnostics tab can
			// show "live" vs "stale" state.
			std::atomic<uint64_t> publish_tick;

			// Reserved for future header growth without bumping
			// SHMEM_VERSION (so long as the overlay tolerates new flags).
			uint32_t _reserved[6];

			// The slot table. Indexed by [0, slot_count); the writer
			// allocates slots monotonically as new component handles are
			// observed and reuses freed slots after Shutdown.
			InputHealthSnapshotSlot slots[INPUTHEALTH_SLOT_COUNT];
		};

	private:
		HANDLE     hMapFile = INVALID_HANDLE_VALUE;
		ShmemData *pData    = nullptr;

		std::string LastErrorString(DWORD lastError)
		{
			LPSTR buffer = nullptr;
			size_t size = FormatMessageA(
				FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
				NULL, lastError, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), (LPSTR)&buffer, 0, NULL
			);
			std::string message(buffer ? buffer : "", size);
			if (buffer) LocalFree(buffer);
			return message;
		}

	public:
		InputHealthSnapshotShmem() = default;
		~InputHealthSnapshotShmem() { Close(); }

		InputHealthSnapshotShmem(const InputHealthSnapshotShmem &) = delete;
		InputHealthSnapshotShmem &operator=(const InputHealthSnapshotShmem &) = delete;

		operator bool() const { return pData != nullptr; }

		void Close()
		{
			if (pData) { UnmapViewOfFile(pData); pData = nullptr; }
			if (hMapFile && hMapFile != INVALID_HANDLE_VALUE) {
				CloseHandle(hMapFile); hMapFile = INVALID_HANDLE_VALUE;
			}
		}

		// Driver-side: create or re-open the shmem segment, stamp the header,
		// zero the slot table. Idempotent on the same name.
		bool Create(LPCSTR segment_name)
		{
			Close();
			hMapFile = CreateFileMappingA(
				INVALID_HANDLE_VALUE, NULL, PAGE_READWRITE,
				0, sizeof(ShmemData), segment_name);
			if (!hMapFile) return false;

			pData = reinterpret_cast<ShmemData*>(MapViewOfFile(
				hMapFile, FILE_MAP_ALL_ACCESS, 0, 0, sizeof(ShmemData)));
			if (!pData) return false;

			// Fresh mappings are zero-filled by the OS, so only the header
			// fields need an explicit stamp. Existing mappings (driver
			// reload) are left untouched apart from the magic/version
			// re-stamp; the slot generations from the prior incarnation
			// remain valid until the publisher overwrites them.
			pData->magic         = SHMEM_MAGIC;
			pData->shmem_version = SHMEM_VERSION;
			pData->slot_count    = INPUTHEALTH_SLOT_COUNT;
			return true;
		}

		// Overlay-side: open an existing segment. Throws on missing or
		// mismatched magic/version so the caller can surface a paired-install
		// hint to the user instead of silently rendering stale data.
		void Open(LPCSTR segment_name)
		{
			Close();
			hMapFile = OpenFileMappingA(FILE_MAP_READ, FALSE, segment_name);
			if (!hMapFile) {
				throw std::runtime_error(
					"Failed to open InputHealth shmem segment: " +
					LastErrorString(GetLastError()));
			}
			pData = reinterpret_cast<ShmemData*>(MapViewOfFile(
				hMapFile, FILE_MAP_READ, 0, 0, sizeof(ShmemData)));
			if (!pData) {
				DWORD err = GetLastError();
				CloseHandle(hMapFile); hMapFile = INVALID_HANDLE_VALUE;
				throw std::runtime_error(
					"Failed to map InputHealth shmem segment: " +
					LastErrorString(err));
			}
			if (pData->magic != SHMEM_MAGIC) {
				char buf[160];
				snprintf(buf, sizeof buf,
					"InputHealth shmem magic mismatch: got 0x%08X, expected 0x%08X",
					pData->magic, SHMEM_MAGIC);
				Close();
				throw std::runtime_error(buf);
			}
			if (pData->shmem_version != SHMEM_VERSION) {
				char buf[160];
				snprintf(buf, sizeof buf,
					"InputHealth shmem version mismatch: got %u, expected %u",
					pData->shmem_version, SHMEM_VERSION);
				Close();
				throw std::runtime_error(buf);
			}
		}

		// Driver-side: number of slots reserved at create time. Stable for
		// the lifetime of the segment; surfaced so external code can iterate.
		uint32_t SlotCount() const { return pData ? pData->slot_count : 0; }

		// Driver-side: bump the publish-tick counter once per worker pass.
		void BumpPublishTick()
		{
			if (!pData) return;
			pData->publish_tick.fetch_add(1, std::memory_order_release);
		}

		// Overlay-side: most recent publish-tick value. Increases each time
		// the driver finishes a full pass over the slot table.
		uint64_t LoadPublishTick() const
		{
			if (!pData) return 0;
			return pData->publish_tick.load(std::memory_order_acquire);
		}

		// Driver-side: slot accessor. The publisher holds an external
		// handle -> slot map and uses this to obtain the slot pointer for
		// a given index. Returns nullptr if pData is unmapped or index is
		// out of range.
		InputHealthSnapshotSlot *SlotForWrite(uint32_t index)
		{
			if (!pData) return nullptr;
			if (index >= pData->slot_count) return nullptr;
			return &pData->slots[index];
		}

		// Driver-side: write a snapshot body into slot[index] under the
		// per-slot seqlock. Bumps the generation odd before the memcpy and
		// even after, so a concurrent overlay reader can detect torn reads.
		// No-op if the slot pointer is unavailable.
		void WriteSlot(uint32_t index, const InputHealthSnapshotBody &body)
		{
			InputHealthSnapshotSlot *slot = SlotForWrite(index);
			if (!slot) return;
			const uint64_t prev = slot->generation.load(std::memory_order_relaxed);
			// Pre-write fence: mark mid-write before any body byte is touched.
			slot->generation.store(prev + 1, std::memory_order_release);
			std::memcpy(&slot->body, &body, sizeof(InputHealthSnapshotBody));
			// Post-write fence: stamp the new even generation; the reader's
			// acquire load on this counter pairs with this release store and
			// guarantees the body bytes are visible.
			slot->generation.store(prev + 2, std::memory_order_release);
		}

		// Overlay-side: read a snapshot body from slot[index] using the
		// per-slot seqlock. Retries up to `max_retries` times if a torn
		// read is detected. Returns true on success and writes the body to
		// `out`; returns false if the retry budget was exceeded.
		bool TryReadSlot(uint32_t index, InputHealthSnapshotBody &out, int max_retries = 8) const
		{
			if (!pData) return false;
			if (index >= pData->slot_count) return false;
			const InputHealthSnapshotSlot &slot = pData->slots[index];
			for (int attempt = 0; attempt < max_retries; ++attempt) {
				const uint64_t g1 = slot.generation.load(std::memory_order_acquire);
				if ((g1 & 1ULL) != 0ULL) continue; // mid-write
				std::memcpy(&out, &slot.body, sizeof(InputHealthSnapshotBody));
				std::atomic_thread_fence(std::memory_order_acquire);
				const uint64_t g2 = slot.generation.load(std::memory_order_acquire);
				if (g1 == g2) return true;
			}
			return false;
		}
	};

	// =========================================================================
	// FaceTrackingFrame shmem ring.
	//
	// Single-writer (the C# FaceModuleHost.exe sidecar) and single-reader (the
	// driver's pose-update path). Hardware face/eye samples arrive at ~120 Hz;
	// the driver applies calibration / eyelid-sync / vergence-lock on top before
	// publishing to SteamVR inputs and the host's OSC sender. A 32-slot ring
	// gives the driver-side filter chain a few frames of look-back without a
	// separate buffer of its own.
	//
	// Wire layout matches the InputHealthSnapshotShmem discipline: header with
	// magic + version + ring size, atomic publish index, per-slot seqlock
	// generation so a reader can detect torn reads.
	// =========================================================================

	// Number of facial-expression shapes the driver and its consumers (OSC
	// publisher, native SteamVR sink, CalibrationEngine) work with. This is
	// our internal/consumer-facing layout; the wire-side carries upstream's
	// FACETRACKING_UPSTREAM_EXPRESSION_COUNT and the reader remaps before
	// returning a frame to consumers.
	static const uint32_t FACETRACKING_EXPRESSION_COUNT = 63;

	// Number of facial-expression shapes carried on the wire. Matches
	// VRCFaceTracking.Core.Params.Expressions.UnifiedExpressions excluding
	// the Max sentinel. The host writes this many floats dense; the driver
	// remaps to FACETRACKING_EXPRESSION_COUNT via
	// core/src/common/facetracking/UpstreamShapeMap.h before consumption.
	static const uint32_t FACETRACKING_UPSTREAM_EXPRESSION_COUNT = 88;

	struct FaceTrackingFrameBody
	{
		// Sample timestamp as Windows QueryPerformanceCounter ticks at the host's
		// publish time. Both writer and reader resolve QPC against the same
		// performance counter (same boot session), so timestamps are directly
		// comparable without conversion.
		uint64_t qpc_sample_time;

		// FNV-1a-ish 64-bit hash of the source module's stable identity string
		// (e.g. "QuestPro_v2"). Lets the driver detect a hot-swap and reset its
		// calibration state without parsing the full uuid. 0 = unknown / not set.
		uint64_t source_module_uuid_hash;

		// Per-eye origin in HMD space, metres. The host either obtains these
		// from the hardware (Quest Pro reports physical pupil positions) or
		// falls back to half the headset IPD. Used by the driver's vergence-lock
		// math to project gaze rays.
		float    eye_origin_l[3];
		float    eye_origin_r[3];

		// Per-eye gaze direction as a unit vector in HMD space. The OpenVR
		// convention is +X right, +Y up, -Z forward; gaze pointing straight
		// ahead is (0, 0, -1).
		float    eye_gaze_l[3];
		float    eye_gaze_r[3];

		// Per-eye eyelid openness 0..1. 0 = fully closed, 1 = fully open. Linear,
		// not gamma-corrected.
		float    eye_openness_l;
		float    eye_openness_r;

		// Per-eye pupil dilation 0..1. Most hardware exposes this as a relative
		// signal; the driver's continuous calibration normalises across the
		// observed range. 0 = constricted, 1 = dilated.
		float    pupil_dilation_l;
		float    pupil_dilation_r;

		// Per-eye confidence 0..1. If the hardware exposes a confidence signal
		// the host writes it directly; otherwise the host writes a synthesised
		// value (frame age + signal stability). Driver filters use this for
		// weighted blending and eye-dropout fallback.
		float    eye_confidence_l;
		float    eye_confidence_r;

		// Facial expression shapes in Unified Expressions v2 order. Range 0..1
		// per shape unless the SDK documents otherwise (a few directional shapes
		// are signed; the host coerces those into [0,1] before publish so the
		// wire format stays uniform).
		float    expressions[FACETRACKING_EXPRESSION_COUNT];

		// bit 0: eye fields are valid this frame.
		// bit 1: expression fields are valid this frame.
		// Other bits reserved; reader must ignore. Lets a host module that only
		// supports one capability publish frames the driver can still consume.
		uint32_t flags;

		// Head pose in HMD-local space. Yaw/pitch/roll in radians; pos in metres.
		// head_flags bit 0 set: head fields are valid this frame.
		float    head_yaw;
		float    head_pitch;
		float    head_roll;
		float    head_pos_x;
		float    head_pos_y;
		float    head_pos_z;
		uint32_t head_flags;
	};

	static_assert(std::is_trivially_copyable<FaceTrackingFrameBody>::value,
		"FaceTrackingFrameBody must be trivially copyable for shmem use");
	static_assert(std::is_standard_layout<FaceTrackingFrameBody>::value,
		"FaceTrackingFrameBody must be standard-layout for stable wire format");

	// Wire-format body. Same field shape as FaceTrackingFrameBody except the
	// expression array carries FACETRACKING_UPSTREAM_EXPRESSION_COUNT (88)
	// entries in upstream VRCFaceTracking.UnifiedExpressions order rather
	// than FACETRACKING_EXPRESSION_COUNT (63) entries in our internal order.
	// The host writes this struct; FaceTrackingFrameShmem::TryReadLatest
	// translates into FaceTrackingFrameBody before returning to consumers.
	struct FaceTrackingFrameBodyWire
	{
		uint64_t qpc_sample_time;
		uint64_t source_module_uuid_hash;

		float    eye_origin_l[3];
		float    eye_origin_r[3];
		float    eye_gaze_l[3];
		float    eye_gaze_r[3];

		float    eye_openness_l;
		float    eye_openness_r;
		float    pupil_dilation_l;
		float    pupil_dilation_r;
		float    eye_confidence_l;
		float    eye_confidence_r;

		float    expressions[FACETRACKING_UPSTREAM_EXPRESSION_COUNT];

		uint32_t flags;

		float    head_yaw;
		float    head_pitch;
		float    head_roll;
		float    head_pos_x;
		float    head_pos_y;
		float    head_pos_z;
		uint32_t head_flags;
	};

	static_assert(std::is_trivially_copyable<FaceTrackingFrameBodyWire>::value,
		"FaceTrackingFrameBodyWire must be trivially copyable for shmem use");
	static_assert(std::is_standard_layout<FaceTrackingFrameBodyWire>::value,
		"FaceTrackingFrameBodyWire must be standard-layout for stable wire format");

	// Sanity-check that the wire body matches the consumer-facing body
	// everywhere except expressions[]. If a field is added/reordered to
	// FaceTrackingFrameBody, the same change has to land in the wire body
	// or the host writes through the wrong offsets.
	static_assert(
		offsetof(FaceTrackingFrameBodyWire, qpc_sample_time)
		    == offsetof(FaceTrackingFrameBody, qpc_sample_time),
		"qpc_sample_time offset mismatch");
	static_assert(
		offsetof(FaceTrackingFrameBodyWire, expressions)
		    == offsetof(FaceTrackingFrameBody, expressions),
		"expressions offset mismatch");

	struct FaceTrackingFrameSlot
	{
		std::atomic<uint64_t> generation;
		FaceTrackingFrameBodyWire body; // wire format -- 88 upstream shapes
	};

	// host_state byte values written into ShmemData::host_state by the host's
	// FrameWriter. The driver uses this to interpret heartbeat-age semantics:
	// "no publish for 2 s" is fatal when publishing, fine when idle.
	enum HostState : uint32_t
	{
		HostStateLegacy     = 0, // pre-heartbeat host; ignore heartbeat field
		HostStatePublishing = 1, // active module pushing frames at full rate
		HostStateIdle       = 2, // host alive but no module selected / paused
		HostStateDraining   = 3, // host shutting down cleanly; final frames in flight
	};

	class FaceTrackingFrameShmem
	{
	public:
		static const uint32_t SHMEM_MAGIC   = 0x46544652; // 'FTFR'
		static const uint32_t SHMEM_VERSION = 3; // v3: expressions grown to upstream-format 88 slots; driver remaps via UpstreamShapeMap
		static const uint32_t RING_SIZE     = 32;

	private:
		struct ShmemData
		{
			uint32_t magic;            // @  0
			uint32_t shmem_version;    // @  4
			uint32_t ring_size;        // @  8

			// Host-side liveness signals. Hosts that pre-date this field write
			// zero, which we treat as HostStateLegacy and skip the heartbeat
			// check (so the layout change does not require SHMEM_VERSION bump).
			uint32_t              host_state;          // @ 12 -- HostState enum
			std::atomic<uint64_t> host_heartbeat_qpc;  // @ 16 -- QueryPerformanceCounter at last tick

			// Reserved for future header expansion (capability negotiation,
			// per-host stats, etc.). Kept zero by every current writer.
			uint32_t _reserved_header[2];              // @ 24..31

			// Monotonically increasing publish counter. The slot the writer just
			// finished is at index (publish_index - 1) % RING_SIZE. 0 means no
			// frame has been published since shmem creation.
			std::atomic<uint64_t> publish_index;       // @ 32

			FaceTrackingFrameSlot slots[RING_SIZE];    // @ 40
		};
		// Compile-time sanity on the field offsets the C# host depends on.
		// FrameWriter.cs hardcodes 12 / 16 / 32 / 40; if anything shifts here
		// the host writes through the wrong pointer and we get torn / zero
		// data, with no immediate error.
		static_assert(offsetof(ShmemData, host_state)         == 12, "host_state offset");
		static_assert(offsetof(ShmemData, host_heartbeat_qpc) == 16, "host_heartbeat_qpc offset");
		static_assert(offsetof(ShmemData, publish_index)      == 32, "publish_index offset");
		static_assert(offsetof(ShmemData, slots)              == 40, "slots offset");

		HANDLE     hMapFile = INVALID_HANDLE_VALUE;
		ShmemData *pData    = nullptr;

		std::string LastErrorString(DWORD lastError)
		{
			LPSTR buffer = nullptr;
			size_t size = FormatMessageA(
				FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
				NULL, lastError, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), (LPSTR)&buffer, 0, NULL
			);
			std::string message(buffer ? buffer : "", size);
			if (buffer) LocalFree(buffer);
			return message;
		}

	public:
		FaceTrackingFrameShmem() = default;
		~FaceTrackingFrameShmem() { Close(); }

		FaceTrackingFrameShmem(const FaceTrackingFrameShmem &) = delete;
		FaceTrackingFrameShmem &operator=(const FaceTrackingFrameShmem &) = delete;

		operator bool() const { return pData != nullptr; }

		void Close()
		{
			if (pData) { UnmapViewOfFile(pData); pData = nullptr; }
			if (hMapFile && hMapFile != INVALID_HANDLE_VALUE) {
				CloseHandle(hMapFile);
				hMapFile = INVALID_HANDLE_VALUE;
			}
		}

		// Driver-side: create / re-open the shmem segment, stamp the header,
		// zero the ring. Idempotent on the same name. Returns false on failure
		// so the driver can log + run degraded (no face tracking).
		bool Create(LPCSTR segment_name)
		{
			Close();
			hMapFile = CreateFileMappingA(
				INVALID_HANDLE_VALUE, NULL, PAGE_READWRITE,
				0, sizeof(ShmemData), segment_name);
			if (!hMapFile) return false;

			pData = reinterpret_cast<ShmemData*>(MapViewOfFile(
				hMapFile, FILE_MAP_ALL_ACCESS, 0, 0, sizeof(ShmemData)));
			if (!pData) return false;

			pData->magic         = SHMEM_MAGIC;
			pData->shmem_version = SHMEM_VERSION;
			pData->ring_size     = RING_SIZE;
			return true;
		}

		// Host- or reader-side: open an existing segment. Throws on missing or
		// mismatched magic/version so the caller can surface a paired-install
		// hint instead of silently reading stale data. Used by unit tests and
		// the C++ host-supervisor helper if it ever needs to peek.
		void Open(LPCSTR segment_name)
		{
			Close();
			hMapFile = OpenFileMappingA(FILE_MAP_ALL_ACCESS, FALSE, segment_name);
			if (!hMapFile) {
				throw std::runtime_error(
					"Failed to open FaceTracking shmem segment: " +
					LastErrorString(GetLastError()));
			}
			pData = reinterpret_cast<ShmemData*>(MapViewOfFile(
				hMapFile, FILE_MAP_ALL_ACCESS, 0, 0, sizeof(ShmemData)));
			if (!pData) {
				DWORD err = GetLastError();
				CloseHandle(hMapFile);
				hMapFile = INVALID_HANDLE_VALUE;
				throw std::runtime_error(
					"Failed to map FaceTracking shmem segment: " +
					LastErrorString(err));
			}
			if (pData->magic != SHMEM_MAGIC) {
				char buf[160];
				snprintf(buf, sizeof buf,
					"FaceTracking shmem magic mismatch: got 0x%08X, expected 0x%08X",
					pData->magic, SHMEM_MAGIC);
				Close();
				throw std::runtime_error(buf);
			}
			if (pData->shmem_version != SHMEM_VERSION) {
				char buf[160];
				snprintf(buf, sizeof buf,
					"FaceTracking shmem version mismatch: got %u, expected %u",
					pData->shmem_version, SHMEM_VERSION);
				Close();
				throw std::runtime_error(buf);
			}
		}

		uint32_t RingSize() const { return pData ? pData->ring_size : 0; }

		// Reader: latest published index. Increasing means at least one new
		// frame is available since the prior read; equal means no new data.
		uint64_t PublishIndex() const
		{
			return pData ? pData->publish_index.load(std::memory_order_acquire) : 0;
		}

		// Reader: host's last heartbeat timestamp in QueryPerformanceCounter
		// ticks. Zero means the host either has not written a heartbeat yet
		// or is a pre-heartbeat (legacy) build; either way the driver should
		// fall back to handle-only liveness detection.
		uint64_t HostHeartbeatQpc() const
		{
			return pData
				? pData->host_heartbeat_qpc.load(std::memory_order_acquire)
				: 0;
		}

		// Reader: host's self-reported activity state. Used to interpret
		// HostHeartbeatQpc -- a "no heartbeat for 2 s" gap is wedge-level
		// only when state == HostStatePublishing.
		uint32_t HostStateField() const
		{
			// Plain uint32 load is atomic on x64; acquire fence below pairs
			// with the host's release fence in WriteHostState.
			if (!pData) return HostStateLegacy;
			uint32_t v = pData->host_state;
			std::atomic_thread_fence(std::memory_order_acquire);
			return v;
		}

		// Driver (writer-side): reset the heartbeat fields back to "no
		// signal". Called after the supervisor kills a wedged host so the
		// stale heartbeat the dead host left in the segment does not
		// re-trigger the wedge detector before the new host writes its
		// first heartbeat tick.
		void ResetHostLiveness()
		{
			if (!pData) return;
			pData->host_state = HostStateLegacy;
			pData->host_heartbeat_qpc.store(0, std::memory_order_release);
		}

		// Reader: copy the most recently published wire-format frame into
		// `out`. The wire body carries 88 upstream-ordered expression
		// shapes; the caller is expected to remap to our 63-slot ordering
		// via facetracking::RemapUpstreamShapes (UpstreamShapeMap.h) before
		// consumption. FaceFrameReader::TryRead is the production caller
		// and does that remap automatically.
		bool TryReadLatestWire(FaceTrackingFrameBodyWire &out, int max_retries = 8) const
		{
			if (!pData) return false;
			const uint64_t idx = pData->publish_index.load(std::memory_order_acquire);
			if (idx == 0) return false;
			const FaceTrackingFrameSlot &slot = pData->slots[(idx - 1) % RING_SIZE];
			for (int attempt = 0; attempt < max_retries; ++attempt) {
				const uint64_t g1 = slot.generation.load(std::memory_order_acquire);
				if ((g1 & 1ULL) != 0ULL) {
					YieldProcessor();
					continue;
				}
				std::memcpy(&out, &slot.body, sizeof(FaceTrackingFrameBodyWire));
				std::atomic_thread_fence(std::memory_order_acquire);
				const uint64_t g2 = slot.generation.load(std::memory_order_acquire);
				if (g1 == g2) return true;
				YieldProcessor();
			}
			return false;
		}

		// Reader: copy a specific historical wire slot (by absolute publish
		// index). Returns false if the requested index has already been
		// overwritten or if a torn read can't be resolved.
		bool TryReadWireByIndex(uint64_t target_index, FaceTrackingFrameBodyWire &out, int max_retries = 8) const
		{
			if (!pData || target_index == 0) return false;
			const uint64_t now = pData->publish_index.load(std::memory_order_acquire);
			if (target_index > now) return false;
			if (now - target_index >= RING_SIZE) return false;
			const FaceTrackingFrameSlot &slot = pData->slots[(target_index - 1) % RING_SIZE];
			for (int attempt = 0; attempt < max_retries; ++attempt) {
				const uint64_t g1 = slot.generation.load(std::memory_order_acquire);
				if ((g1 & 1ULL) != 0ULL) {
					YieldProcessor();
					continue;
				}
				std::memcpy(&out, &slot.body, sizeof(FaceTrackingFrameBodyWire));
				std::atomic_thread_fence(std::memory_order_acquire);
				const uint64_t g2 = slot.generation.load(std::memory_order_acquire);
				if (g1 == g2) return true;
				YieldProcessor();
			}
			return false;
		}

		// Writer-side helper for unit tests and any future C++ first-party
		// publisher. Accepts the wire body (88 upstream shapes) directly; the
		// host's C# FrameWriter writes this same layout via MemoryMappedFile.
		void Publish(const FaceTrackingFrameBodyWire &body)
		{
			if (!pData) return;
			const uint64_t next = pData->publish_index.load(std::memory_order_relaxed) + 1;
			FaceTrackingFrameSlot &slot = pData->slots[(next - 1) % RING_SIZE];
			const uint64_t prev_gen = slot.generation.load(std::memory_order_relaxed);
			slot.generation.store(prev_gen + 1, std::memory_order_release);
			std::memcpy(&slot.body, &body, sizeof(FaceTrackingFrameBodyWire));
			slot.generation.store(prev_gen + 2, std::memory_order_release);
			pData->publish_index.store(next, std::memory_order_release);
		}
	};

	// =========================================================================
	// OSC Router stats shmem.
	//
	// Written by the OscRouter driver module at ~10 Hz. The overlay reads
	// it directly to populate the Modules tab route list without a round-trip
	// through the IPC pipe. Layout: header (magic + version), global counters,
	// fixed-size route-slot table (32 entries).
	//
	// Writer: driver's OscRouterStatsShmem write side (one worker thread).
	// Readers: overlay's OscRouterStatsReader (any thread via Open()).
	// =========================================================================

	// Max simultaneous route slots in the shmem table. 32 covers the typical
	// use case: one wildcard for face-tracking, several per-feature patterns,
	// and room for overlay test subscriptions.
	static const uint32_t OSC_ROUTER_ROUTE_SLOTS = 32;

	// Per-route stat slot. Written by the driver under a per-slot seqlock.
	// address_pattern and subscriber_id are NUL-terminated, padded to fill.
	struct OscRouterRouteSlot
	{
		std::atomic<uint64_t> generation; // seqlock: odd = mid-write
		char                  address_pattern[OSC_ROUTE_ADDR_LEN];
		char                  subscriber_id[32];
		std::atomic<uint64_t> match_count;    // OSC messages matched by pattern
		std::atomic<uint64_t> drop_count;     // matched but dropped (queue full)
		std::atomic<uint64_t> last_match_tick; // QPC tick of most recent match
		uint8_t               active;         // 1 = slot in use, 0 = empty
		uint8_t               _reserved[7];
	};

	class OscRouterStatsShmem
	{
	public:
		static const uint32_t SHMEM_MAGIC   = 0xC5C7057C;
		static const uint32_t SHMEM_VERSION = 1;

	private:
		struct ShmemData
		{
			uint32_t magic;
			uint32_t shmem_version;
			// Global send counters. Relaxed stores by the send thread;
			// relaxed loads by the overlay -- only deltas matter.
			std::atomic<uint64_t> packets_sent;
			std::atomic<uint64_t> bytes_sent;
			std::atomic<uint64_t> packets_dropped; // send queue overflow
			uint32_t _reserved[4];
			OscRouterRouteSlot routes[OSC_ROUTER_ROUTE_SLOTS];
		};

		HANDLE     hMapFile = INVALID_HANDLE_VALUE;
		ShmemData *pData    = nullptr;

		std::string LastErrorString(DWORD lastError)
		{
			LPSTR buffer = nullptr;
			DWORD size = FormatMessageA(
				FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
				NULL, lastError, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), (LPSTR)&buffer, 0, NULL);
			std::string msg(buffer ? buffer : "", size);
			if (buffer) LocalFree(buffer);
			return msg;
		}

	public:
		OscRouterStatsShmem() = default;
		~OscRouterStatsShmem() { Close(); }

		OscRouterStatsShmem(const OscRouterStatsShmem &) = delete;
		OscRouterStatsShmem &operator=(const OscRouterStatsShmem &) = delete;

		operator bool() const { return pData != nullptr; }

		void Close()
		{
			if (pData) { UnmapViewOfFile(pData); pData = nullptr; }
			if (hMapFile && hMapFile != INVALID_HANDLE_VALUE) {
				CloseHandle(hMapFile); hMapFile = INVALID_HANDLE_VALUE;
			}
		}

		// Driver-side: create or re-open the segment and stamp the header.
		bool Create(LPCSTR segment_name)
		{
			Close();
			hMapFile = CreateFileMappingA(
				INVALID_HANDLE_VALUE, NULL, PAGE_READWRITE,
				0, sizeof(ShmemData), segment_name);
			if (!hMapFile) return false;
			pData = reinterpret_cast<ShmemData*>(MapViewOfFile(
				hMapFile, FILE_MAP_ALL_ACCESS, 0, 0, sizeof(ShmemData)));
			if (!pData) return false;
			pData->magic        = SHMEM_MAGIC;
			pData->shmem_version = SHMEM_VERSION;
			return true;
		}

		// Overlay-side: open an existing segment. Throws on missing or
		// mismatched header so the overlay can surface a paired-install hint.
		void Open(LPCSTR segment_name)
		{
			Close();
			hMapFile = OpenFileMappingA(FILE_MAP_READ, FALSE, segment_name);
			if (!hMapFile) {
				throw std::runtime_error(
					std::string("Failed to open OscRouter shmem: ") +
					LastErrorString(GetLastError()));
			}
			pData = reinterpret_cast<ShmemData*>(MapViewOfFile(
				hMapFile, FILE_MAP_READ, 0, 0, sizeof(ShmemData)));
			if (!pData) {
				DWORD err = GetLastError();
				CloseHandle(hMapFile); hMapFile = INVALID_HANDLE_VALUE;
				throw std::runtime_error(
					std::string("Failed to map OscRouter shmem: ") +
					LastErrorString(err));
			}
			if (pData->magic != SHMEM_MAGIC) {
				char buf[160];
				snprintf(buf, sizeof buf,
					"OscRouter shmem magic mismatch: got 0x%08X, expected 0x%08X",
					pData->magic, SHMEM_MAGIC);
				Close();
				throw std::runtime_error(buf);
			}
			if (pData->shmem_version != SHMEM_VERSION) {
				char buf[160];
				snprintf(buf, sizeof buf,
					"OscRouter shmem version mismatch: got %u, expected %u",
					pData->shmem_version, SHMEM_VERSION);
				Close();
				throw std::runtime_error(buf);
			}
		}

		// Driver-side: increment global send counters from the send thread.
		void AddSent(uint64_t bytes)
		{
			if (!pData) return;
			pData->packets_sent.fetch_add(1, std::memory_order_relaxed);
			pData->bytes_sent.fetch_add(bytes, std::memory_order_relaxed);
		}

		void AddDropped()
		{
			if (!pData) return;
			pData->packets_dropped.fetch_add(1, std::memory_order_relaxed);
		}

		// Driver-side: write one route slot under its seqlock.
		static void CopyStrField(char *dst, size_t dst_size, const char *src)
		{
			if (!src) { dst[0] = '\0'; return; }
			size_t n = 0;
			for (; n < dst_size - 1 && src[n]; ++n) dst[n] = src[n];
			dst[n] = '\0';
		}

		void WriteRoute(uint32_t index,
			const char *pattern, const char *subscriber_id,
			uint64_t match_count, uint64_t drop_count,
			uint64_t last_match_tick, bool active)
		{
			if (!pData || index >= OSC_ROUTER_ROUTE_SLOTS) return;
			OscRouterRouteSlot &s = pData->routes[index];
			const uint64_t prev = s.generation.load(std::memory_order_relaxed);
			s.generation.store(prev + 1, std::memory_order_release);
			CopyStrField(s.address_pattern, sizeof(s.address_pattern), pattern);
			CopyStrField(s.subscriber_id,   sizeof(s.subscriber_id),   subscriber_id);
			s.match_count.store(match_count, std::memory_order_relaxed);
			s.drop_count.store(drop_count, std::memory_order_relaxed);
			s.last_match_tick.store(last_match_tick, std::memory_order_relaxed);
			s.active = active ? 1 : 0;
			s.generation.store(prev + 2, std::memory_order_release);
		}

		// Overlay-side: read global totals into an OscRouterStats struct.
		bool ReadGlobalStats(OscRouterStats &out) const
		{
			if (!pData) return false;
			out.packets_sent    = pData->packets_sent.load(std::memory_order_relaxed);
			out.bytes_sent      = pData->bytes_sent.load(std::memory_order_relaxed);
			out.packets_dropped = pData->packets_dropped.load(std::memory_order_relaxed);
			out.active_routes   = 0;
			for (uint32_t i = 0; i < OSC_ROUTER_ROUTE_SLOTS; ++i)
				if (pData->routes[i].active) ++out.active_routes;
			out._reserved = 0;
			return true;
		}

		// Overlay-side: try reading one route slot under its seqlock.
		// Returns true on a clean read. Caller re-tries on false or skips.
		bool TryReadRoute(uint32_t index, OscRouterRouteSlot &out, int max_retries = 8) const
		{
			if (!pData || index >= OSC_ROUTER_ROUTE_SLOTS) return false;
			const OscRouterRouteSlot &s = pData->routes[index];
			for (int attempt = 0; attempt < max_retries; ++attempt) {
				const uint64_t g1 = s.generation.load(std::memory_order_acquire);
				if ((g1 & 1ULL) != 0ULL) continue;
				std::memcpy(&out, &s, sizeof(OscRouterRouteSlot));
				std::atomic_thread_fence(std::memory_order_acquire);
				const uint64_t g2 = s.generation.load(std::memory_order_acquire);
				if (g1 == g2) return true;
			}
			return false;
		}
	};
}

