#pragma once

// Internal protocol payload definitions. Include through Protocol.h.

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
	// v23 (2026-05-19): gates the driver's write to its stored `tf.quash`.
	// When false, the driver leaves the existing hide state untouched.
	// Lets callers that disable cal for unrelated reasons (e.g.
	// ResetAndDisableOffsets) avoid clobbering a tracker the user has
	// marked as always-hidden. All in-tree constructors below default this
	// to false; sites that intend to set the hide bit write true here AND
	// fill `quash` to the desired value.
	bool updateQuash;

	// Tracking system name of the device with this OpenVR ID, populated by the
	// overlay so the driver can match it against per-system fallbacks without
	// querying VR properties on every pose update. Empty string means "unknown".
	char target_system[MaxTrackingSystemNameLen];

	// Strength of native pose-prediction suppression for this device, on a
	// 0..100 scale. 0 = pose untouched (no suppression). Higher values apply
	// stronger tracker prediction suppression and one-euro position smoothing.
	// 100 is the strongest setting, with more lag allowed than lower values, but
	// it is not a hard pose freeze.
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

	SetDeviceTransform(uint32_t id, bool enabled)
	    : openVRID(id), enabled(enabled), updateTranslation(false), updateRotation(false), updateScale(false),
	      translation({}), rotation({1, 0, 0, 0}), scale(1), lerp(false), quash(false), updateQuash(false),
	      target_system{}, predictionSmoothness(0), recalibrateOnMovement(false)
	{
	}

	SetDeviceTransform(uint32_t id, bool enabled, vr::HmdVector3d_t translation)
	    : openVRID(id), enabled(enabled), updateTranslation(true), updateRotation(false), updateScale(false),
	      translation(translation), rotation({1, 0, 0, 0}), scale(1), lerp(false), quash(false), updateQuash(false),
	      target_system{}, predictionSmoothness(0), recalibrateOnMovement(false)
	{
	}

	SetDeviceTransform(uint32_t id, bool enabled, vr::HmdQuaternion_t rotation)
	    : openVRID(id), enabled(enabled), updateTranslation(false), updateRotation(true), updateScale(false),
	      translation({}), rotation(rotation), scale(1), lerp(false), quash(false), updateQuash(false), target_system{},
	      predictionSmoothness(0), recalibrateOnMovement(false)
	{
	}

	SetDeviceTransform(uint32_t id, bool enabled, double scale)
	    : openVRID(id), enabled(enabled), updateTranslation(false), updateRotation(false), updateScale(true),
	      translation({}), rotation({1, 0, 0, 0}), scale(scale), lerp(false), quash(false), updateQuash(false),
	      target_system{}, predictionSmoothness(0), recalibrateOnMovement(false)
	{
	}

	SetDeviceTransform(uint32_t id, bool enabled, vr::HmdVector3d_t translation, vr::HmdQuaternion_t rotation)
	    : openVRID(id), enabled(enabled), updateTranslation(true), updateRotation(true), updateScale(false),
	      translation(translation), rotation(rotation), scale(1), lerp(false), quash(false), updateQuash(false),
	      target_system{}, predictionSmoothness(0), recalibrateOnMovement(false)
	{
	}

	SetDeviceTransform(uint32_t id, bool enabled, vr::HmdVector3d_t translation, vr::HmdQuaternion_t rotation,
	                   double scale)
	    : openVRID(id), enabled(enabled), updateTranslation(true), updateRotation(true), updateScale(true),
	      translation(translation), rotation(rotation), scale(scale), lerp(false), quash(false), updateQuash(false),
	      target_system{}, predictionSmoothness(0), recalibrateOnMovement(false)
	{
	}
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
	bool master_enabled;

	// Smoothing strength on a 0..100 scale. 0 = pass-through (alpha=1.0,
	// each frame snaps to the incoming bones). 100 = heavy smoothing
	// (alpha clamped near 0, bones lag behind incoming significantly).
	// Linearly mapped to slerp factor by alpha = 1.0 - (smoothness/100)*0.95
	// so 100 still has a tiny per-frame nudge (alpha=0.05) — never fully
	// freezes a finger.
	uint8_t smoothness;

	// Per-finger enable bits (see kAllFingersMask above). Disabled fingers
	// pass through unsmoothed. Bones outside the 5 finger chains
	// (root, wrist, aux markers) always pass through.
	uint16_t finger_mask;

	// 1 byte of trailing padding here on x64 to round to the natural
	// alignment of the struct. Explicit name so a future reader doesn't
	// mistake it for a meaningful flag — left zero by the overlay; the
	// driver MUST NOT read it.
	uint8_t _reserved;

	// v13 (2026-05-11): per-finger smoothing strength (0..100), indexed the
	// same way as finger_mask bits 0..9 (left thumb..pinky, right thumb..pinky).
	// Replaces the role of the global `smoothness` field when v13 senders are
	// paired with v13 drivers; the global `smoothness` above is retained as a
	// fallback for clients that haven't been updated to write the array.
	// Per-finger value 0 = the driver falls back to the global `smoothness`
	// for that finger (so an all-zero array reproduces v12 behaviour exactly).
	uint8_t per_finger_smoothness[10];

	// 2 bytes of trailing padding to round the struct to 8-byte alignment.
	// Left zero by the overlay; the driver MUST NOT read it.
	uint8_t _reserved2[2];
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
	uint8_t predictionSmoothness; // 0..100, see SetDeviceTransform notes
	// Compatibility flag for the former smart-smoothing toggle. Position
	// smoothing now always uses the speed-adaptive one-euro filter for
	// nonzero predictionSmoothness. Dev builds may use this byte to preview
	// rotation filtering; release builds keep raw rotation. Old overlays
	// may leave this byte zero without changing position smoothing.
	uint8_t smart_enabled;
	uint8_t _reserved[2]; // pad to 8-byte alignment; must be 0
};

enum DashboardHandTrackingPrimaryHand : uint8_t
{
	DashboardHandTrackingHandUnknown = 0,
	DashboardHandTrackingHandLeft = 1,
	DashboardHandTrackingHandRight = 2,
};

// v32: SteamVR dashboard hand-tracking state sent by the Dashboard Input overlay.
// update_mono_ms uses a monotonic Windows tick count so the driver can reject
// stale overlay state without trusting wall-clock time.
struct DashboardHandTrackingState
{
	uint8_t enabled;
	uint8_t dashboard_visible;
	uint8_t primary_hand;
	uint8_t _reserved;
	uint64_t update_mono_ms;
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
	bool master_enabled;

	// When true, the driver records observations and publishes snapshots
	// but never alters component values flowing up to consumers. Useful
	// for triage; the overlay flips this off by default so corrections
	// land as soon as a path crosses its readiness gate.
	bool diagnostics_only;

	// Per-category compensation toggles. The overlay flips these on by
	// default; have no effect while diagnostics_only is true.
	bool enable_rest_recenter;
	bool enable_trigger_remap;

	// Trailing padding. Named so a future reader
	// doesn't mistake it for a meaningful flag; left zero by the overlay.
	uint16_t _reserved;
};

// POD payload for RequestResetInputHealthStats. Identifies the device by
// hashed serial (8 bytes; client uses FNV-1a or similar deterministic
// hash on the ETrackedDeviceProperty serial string) and lists which
// stat categories to wipe.
struct InputHealthResetStats
{
	uint64_t device_serial_hash;
	uint8_t reset_passive; // wipe Welford / PH / EWMA / polar bins
	uint8_t reset_active;  // wipe wizard-recorded calibration prior
	uint8_t reset_curves;  // wipe applied compensation curves
	uint8_t _reserved[5];
};

enum InputHealthCompensationKind : uint8_t
{
	InputHealthCompScalarSingle = 0,
	InputHealthCompStickX = 1,
	InputHealthCompStickY = 2,
	InputHealthCompBoolean = 3,
};

struct InputHealthCompensationEntry
{
	uint64_t device_serial_hash;
	char path[INPUTHEALTH_PATH_LEN];
	uint8_t kind;
	uint8_t enabled;
	uint8_t _pad[2];
	float learned_rest_offset;
	float learned_trigger_min;
	float learned_trigger_max;
	float learned_deadzone_radius;
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

// FaceTrackingConfig::expression_correction_flags bit layout. These are
// avatar-expression shaping options, disabled unless explicitly enabled.
static const uint8_t FACETRACKING_EXPR_CORRECT_MOUTH_CLOSE = 0x01;
static const uint8_t FACETRACKING_EXPR_CORRECT_SMILE_OPEN = 0x02;
static const uint8_t FACETRACKING_EXPR_CORRECT_IDLE_CLOSE = 0x04;
static const uint8_t FACETRACKING_EXPR_CORRECT_BROW_SYNC = 0x08;

// FaceTrackingConfig::eyelid_sync_mode values. Eye openness is 0..1 where
// 0 = fully closed and 1 = fully open.
static const uint8_t FACETRACKING_EYELID_SYNC_MOST_CLOSED = 0;
static const uint8_t FACETRACKING_EYELID_SYNC_MOST_OPEN = 1;

// v15 (2026-05-12): face-tracking master config.
//
// POD-only, fits in the existing Request union -- the static_assert below
// keeps it under sizeof(SetDeviceTransform). The four-tier toggle / strength
// design lets the user disable any single feature without rebuilding the
// pipeline, and supports independent OSC + native output (both can be on;
// user picks).
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

	// Retained for profile/wire compatibility. Normal runtime treats this
	// field as off/no-op and overlays should send 0.
	uint8_t continuous_calib_mode;

	// Output sink toggles. The host process sends OSC to VRChat when
	// output_osc_enabled is set. The native OpenXR eye-gaze path is
	// reserved for Phase 3 -- always zero for now.
	uint8_t output_osc_enabled;
	uint8_t _reserved_native; // was output_native_enabled; Phase 3
	uint8_t expression_correction_flags;

	// Sync feature strengths on a 0..100 scale, identical semantics to
	// the smoothing module's existing scale. 0=feature observable but no
	// effect, 100=feature fully forces both eyes / eyelids to converge.
	uint8_t eyelid_sync_strength;
	uint8_t vergence_lock_strength;
	uint8_t gaze_smoothing;
	uint8_t openness_smoothing;
	uint8_t eyelid_sync_mode;
	uint8_t _reserved_face_config;

	// OSC target. Driver forwards these to the host over the host control
	// pipe; the host owns the UDP socket. Default 127.0.0.1:9000 (VRChat).
	uint16_t osc_port;
	// Low byte = mouth shaping strength, high byte = eyelid/brow sync
	// strength. Both are 0..100 and only used when their flag is enabled.
	uint16_t expression_correction_strengths;
	char osc_host[FACETRACKING_OSC_HOST_LEN];

	// Active hardware module. Empty string = host picks automatically
	// (first available). Non-empty = host loads the module with the
	// matching uuid and ignores others until the user picks again.
	char active_module_uuid[FACETRACKING_MODULE_UUID_LEN];
};

enum FaceCalibrationOp : uint8_t
{
	FaceCalibBegin = 0,
	FaceCalibEnd = 1,
	FaceCalibSave = 2,
	FaceCalibResetAll = 3,
	FaceCalibResetEye = 4,
	FaceCalibResetExpr = 5,
};

// POD payload for RequestSetFaceCalibrationCommand. Retained for compatibility;
// normal runtime accepts these commands without applying calibration state.
struct FaceCalibrationCommand
{
	uint8_t op; // see FaceCalibrationOp
	uint8_t _reserved[7];
};

// POD payload for RequestSetFaceActiveModule. Picks which hardware
// module the host should load (Quest Pro, Vive FT, etc.). Driver
// forwards this to the host process over the internal control pipe
// the next time the host is reachable.
struct FaceModuleSelection
{
	char uuid[FACETRACKING_MODULE_UUID_LEN];
	uint8_t _reserved[8];
};

static const uint16_t FACETRACKING_SHAPE_TUNING_RESET_INDEX = 0xFFFFu;
static const uint16_t FACETRACKING_SHAPE_TUNING_DEFAULT_PERCENT = 100u;
static const uint16_t FACETRACKING_SHAPE_TUNING_MAX_PERCENT = 200u;
static const uint16_t FACETRACKING_SHAPE_TUNING_DEFAULT_MIN_PERCENT = 0u;
static const uint16_t FACETRACKING_SHAPE_TUNING_DEFAULT_MAX_PERCENT = 200u;

struct FaceShapeTuningParams
{
	uint16_t scale_percent; // 0..200; 100 = pass through
	uint16_t min_percent;   // 0..200; lower post-scale output limit
	uint16_t max_percent;   // 0..200; upper post-scale output limit
};

// POD payload for RequestSetFaceShapeTuning. index addresses one
// FACETRACKING_EXPRESSION_COUNT slot. index == RESET_INDEX clears the driver's
// cached table back to DEFAULT_PERCENT; overlays send that first before the
// non-default entries for a newly active avatar.
struct FaceShapeTuning
{
	uint16_t index;
	uint16_t scale_percent; // 0..200; 100 = pass through
	uint16_t min_percent;   // 0..200; 0 = default lower bound
	uint16_t max_percent;   // 0..200; 200 = default upper bound
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
	OscSubscriberId subscriber_id;    // assigned by caller; echoed in stats
	uint8_t enabled;                  // 0 = disable (keep slot); 1 = active
	uint8_t _reserved[3];             // pad to 8-byte boundary; must be 0
	char pattern[OSC_ROUTE_ADDR_LEN]; // NUL-terminated OSC glob
};

// POD payload for RequestOscRouteUnsubscribe.
struct OscRouteUnsubscribe
{
	OscSubscriberId subscriber_id;
	uint8_t _reserved[4];
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
static const size_t OSC_PUBLISH_ARG_LEN = 32;

struct OscPublish
{
	char address[OSC_ROUTE_ADDR_LEN];
	char typetag[OSC_PUBLISH_TYPETAG_LEN];
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

// POD payload for RequestSetOscRouterConfig (v22). Live edit of the
// outbound OSC target port from the overlay. The driver applies the new
// port immediately to its UDP send socket; if the IPC isn't connected at
// the moment the user edits the value, the overlay still writes
// profiles/oscrouter.json so the next driver init picks it up. Default
// (and historical) port is 9000.
struct OscRouterConfig
{
	uint16_t send_port;
	uint8_t _reserved[6];
};

// =========================================================================
// Captions module protocol additions (no version bump; appended after v16)
// =========================================================================

// Maximum field widths for captions config strings.
// These are kept small so CaptionsConfig fits inside SetDeviceTransform.
static const size_t CAPTIONS_LANG_LEN = 16; // BCP-47 code, e.g. "en", "zh", "auto"
static const size_t CAPTIONS_ADDR_LEN = 48; // OSC address, e.g. "/chatbox/input"

// Captions operating mode.
enum CaptionsMode : uint8_t
{
	CaptionsModePushToTalk = 0,
	CaptionsModeAlwaysOn = 1,
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
	// Master enable. 0 = driver keeps the captions sidecar stopped.
	uint8_t master_enabled;

	// Operating mode: PTT or always-on.
	uint8_t mode; // CaptionsMode

	// Notification sound on chatbox send. 0 = silent.
	uint8_t notify_sound;

	// When nonzero the sidecar writes transcripts to disk.
	uint8_t transcript_logging;

	// When nonzero the sidecar publishes completed captions to the
	// configured OSC chatbox target. 0 keeps captions local to WKOpenVR.
	uint8_t chatbox_enabled;

	// Captions realtime tuning flags. See CaptionsRealtimeFlags.h.
	uint8_t realtime_flags;

	// Speech model selection. See CaptionsSpeechModels.h.
	uint8_t speech_model;

	uint8_t _pad[1];

	// Target chatbox OSC port (default 9000).
	uint16_t chatbox_port;

	// Minimum gap between queued chatbox sends, used for split long messages.
	uint16_t chatbox_split_delay_ms;

	// Source language code ("auto" = whisper auto-detect).
	char source_lang[CAPTIONS_LANG_LEN];

	// Target language code ("" = transcribe only, no translation).
	char target_lang[CAPTIONS_LANG_LEN];

	// Chatbox OSC address (default "/chatbox/input").
	char chatbox_address[CAPTIONS_ADDR_LEN];
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
	uint8_t master_enabled;
	uint8_t _pad0[3];

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
	uint8_t dropout_enabled;
	uint8_t _reserved[7];
};

// POD payload for RequestSetPhantomDeviceRole. Maps a physical tracker's
// serial hash to a body role (BodyRole enum; see RoleCatalog.h). The
// overlay sends one of these per assignment, including a clearing
// message with body_role = 0 (None) when the user un-assigns. The body
// completion solver treats assigned trackers as measured anchors.
struct PhantomDeviceRole
{
	uint64_t device_serial_hash;
	uint8_t body_role; // BodyRole enum value
	uint8_t _reserved[7];
};

// POD payload for RequestSetPhantomTrackerOffset. Reserved for older
// overlays that sent rigid HMD-relative offsets. Current drivers ignore
// this payload and infer body priors from live poses.
struct PhantomTrackerOffset
{
	uint8_t body_role;
	uint8_t legacy_available;
	uint8_t _pad[6];
	double rel_position[3];
	vr::HmdQuaternion_t rel_rotation;
};

// POD payload for RequestSetPhantomVirtualEnabled. Per-role boolean for
// absent-mode virtual trackers. The driver publishes a role only when
// the in-process completion result passes the configured confidence
// threshold. Flipping off removes the virtual device from future
// creation; the live instance lives out the current vrserver process.
struct PhantomVirtualEnabled
{
	uint8_t body_role;
	uint8_t enabled;
	uint8_t _reserved[6];
};

// POD payload for RequestSetPhantomSolverConfig. Current overlays use this
// only for the minimum confidence threshold. The body-prior dimensions remain
// in the struct for wire compatibility with older overlays, but current
// drivers ignore them and estimate priors from live poses.
struct PhantomSolverConfig
{
	uint8_t legacy_flags;
	uint8_t _reserved[7];
	double floor_y_m;
	double height_m;
	double forward_yaw_rad;
	double stance_width_m;
	double shoulder_width_m;
	double pelvis_width_m;
	double upper_arm_m;
	double lower_arm_m;
	double upper_leg_m;
	double lower_leg_m;
	double virtual_min_confidence;
};

// POD payload for RequestSetHeadMountConfig (v25/v26/v29/v30/v36). The overlay resolves
// trackerSerial to a live deviceId before sending; deviceId == -1 means
// the tracker is not currently present. mode encodes HeadMountMode;
// headFromTrackerTrans and headFromTrackerRot (xyzw) carry the rigid
// offset from the tracker's frame to the HMD's frame.
//
// v26 adds DriverSynth timing values for stale detection and source
// transition smoothing.
// v29 adds allowRawHmdFallback for hard tracker lock.
// v30 adds lockedHeadsetSmoothing for position smoothing.
// v36 adds lockedHeadsetRotationSmoothing.
//
// NOTE: this struct exceeds sizeof(SetDeviceTransform). sizeof(Request)
// grows with this addition. See the v25/v26/v29 version comments in Protocol.h.
struct SetHeadMountConfig
{
	uint32_t mode;    // HeadMountMode value
	int32_t deviceId; // -1 = not resolved
	char trackerSerial[MaxTrackingSystemNameLen];
	char trackerTrackingSystem[MaxTrackingSystemNameLen];
	double headFromTrackerTrans[3]; // metres, tracker local
	double headFromTrackerRot[4];   // quaternion xyzw
	bool hideTracker;
	bool offsetCalibrated;
	bool allowRawHmdFallback;
	uint16_t driverSynthStaleLimitMs;
	uint16_t driverSynthGraceHoldMs;
	uint16_t driverSynthBlendToFallbackMs;
	uint16_t driverSynthStableBeforeSynthMs;
	uint16_t driverSynthBlendToSynthMs;
	uint8_t lockedHeadsetSmoothing;         // 0..100, 0 = off (v30)
	uint8_t lockedHeadsetRotationSmoothing; // 0..100, 0 = off (v36)
};

struct Request
{
	RequestType type;

	union
	{
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
		// v32: dashboard-visible hand-tracking state from the Smoothing overlay.
		// Fits comfortably under SetDeviceTransform; Version bump handles paired
		// install semantics.
		DashboardHandTrackingState setDashboardHandTrackingState;
		// v15: face-tracking master config + compatibility calibration
		// command + module selection + shape tuning. All are smaller than
		// SetDeviceTransform so the union does not grow; the static_asserts
		// below enforce that.
		FaceTrackingConfig setFaceTrackingConfig;
		FaceCalibrationCommand setFaceCalibrationCommand;
		FaceModuleSelection setFaceActiveModule;
		FaceShapeTuning setFaceShapeTuning;
		// v16: OSC router control. All three are smaller than
		// SetDeviceTransform so the union does not grow.
		OscRouteSubscribe oscRouteSubscribe;
		OscRouteUnsubscribe oscRouteUnsubscribe;
		OscPublish oscPublish;
		// Captions: config push. Smaller than SetDeviceTransform;
		// static_assert below enforces it.
		CaptionsConfig setCaptionsConfig;
		// v19: phantom-tracker global config + per-device opt-in. Both
		// are smaller than SetDeviceTransform so the union does not
		// grow; static_asserts below enforce it.
		PhantomConfig setPhantomConfig;
		PhantomDeviceOptIn setPhantomDeviceOptIn;
		// v20: phantom per-serial role assignment + reserved legacy
		// per-role rigid offset from HMD. Both smaller than
		// SetDeviceTransform; static_asserts pin the size.
		PhantomDeviceRole setPhantomDeviceRole;
		PhantomTrackerOffset setPhantomTrackerOffset;
		// v21: phantom Phase 2 absent-mode virtual-tracker toggle.
		PhantomVirtualEnabled setPhantomVirtualEnabled;
		// v28: phantom in-process body completion settings.
		PhantomSolverConfig setPhantomSolverConfig;
		// v22: OSC router live send-port edit. Tiny (8 bytes); does not
		// grow the union.
		OscRouterConfig setOscRouterConfig;
		// v25/v26: head-mounted tracker config. Larger than SetDeviceTransform;
		// sizeof(Request) grows. See the v25/v26 comments in Protocol.h.
		SetHeadMountConfig setHeadMountConfig;
	};

	Request() : type(RequestInvalid), setAlignmentSpeedParams({}) {}
	Request(RequestType type) : type(type), setAlignmentSpeedParams({}) {}
	Request(AlignmentSpeedParams params)
	    : type(RequestType::RequestSetAlignmentSpeedParams), setAlignmentSpeedParams(params)
	{
	}
};

static_assert(sizeof(AlignmentSpeedParams) <= sizeof(SetDeviceTransform), "AlignmentSpeedParams must not grow Request");
static_assert(sizeof(DashboardHandTrackingState) <= sizeof(SetDeviceTransform),
              "DashboardHandTrackingState must not grow Request");
static_assert(sizeof(InputHealthCompensationEntry) <= sizeof(SetDeviceTransform),
              "InputHealthCompensationEntry must not grow Request");
static_assert(sizeof(FaceTrackingConfig) <= sizeof(SetDeviceTransform), "FaceTrackingConfig must not grow Request");
static_assert(sizeof(FaceCalibrationCommand) <= sizeof(SetDeviceTransform),
              "FaceCalibrationCommand must not grow Request");
static_assert(sizeof(FaceModuleSelection) <= sizeof(SetDeviceTransform), "FaceModuleSelection must not grow Request");
static_assert(sizeof(FaceShapeTuning) <= sizeof(SetDeviceTransform), "FaceShapeTuning must not grow Request");
static_assert(sizeof(OscRouteSubscribe) <= sizeof(SetDeviceTransform), "OscRouteSubscribe must not grow Request");
static_assert(sizeof(OscRouteUnsubscribe) <= sizeof(SetDeviceTransform), "OscRouteUnsubscribe must not grow Request");
static_assert(sizeof(OscPublish) <= sizeof(SetDeviceTransform), "OscPublish must not grow Request");
static_assert(sizeof(CaptionsConfig) <= sizeof(SetDeviceTransform), "CaptionsConfig must not grow Request");
static_assert(sizeof(PhantomConfig) <= sizeof(SetDeviceTransform), "PhantomConfig must not grow Request");
static_assert(sizeof(PhantomDeviceOptIn) <= sizeof(SetDeviceTransform), "PhantomDeviceOptIn must not grow Request");
static_assert(sizeof(PhantomDeviceRole) <= sizeof(SetDeviceTransform), "PhantomDeviceRole must not grow Request");
static_assert(sizeof(PhantomTrackerOffset) <= sizeof(SetDeviceTransform), "PhantomTrackerOffset must not grow Request");
static_assert(sizeof(PhantomVirtualEnabled) <= sizeof(SetDeviceTransform),
              "PhantomVirtualEnabled must not grow Request");
static_assert(sizeof(PhantomSolverConfig) <= sizeof(SetDeviceTransform), "PhantomSolverConfig must not grow Request");

struct Response
{
	ResponseType type;

	union
	{
		Protocol protocol;
		OscRouterStats oscRouterStats;                     // v16: ResponseOscRouterStats
		CaptionsSupervisorStatus captionsSupervisorStatus; // Captions: supervisor state
	};

	Response() : type(ResponseInvalid), protocol({}) {}
	Response(ResponseType type) : type(type), protocol({}) {}
};
