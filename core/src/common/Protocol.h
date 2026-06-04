#pragma once

#include <windows.h>
#include <cstdint>
#include <atomic>
#include <stdexcept>
#include <functional>

#ifndef _OPENVR_API
#include <openvr_driver.h>
#endif
#include "ProtocolNames.h"

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

} // namespace vr

#endif

namespace protocol {
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
// for out-of-process sidecars. The publish pipe starts with one 32-byte
// source-id per connection, then repeats 4-byte LE length + raw OSC packet
// frames until disconnect. New request types: RequestOscRouteSubscribe,
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
// Valve "vive_tracker_<role>" controller type, and pushes synthetic
// poses every time the HMD pose updates.
//
// v22 (2026-05-17): OSC router live send-port edit. Adds
// RequestSetOscRouterConfig (OscRouterConfig payload, 8 bytes) so the
// overlay can push a new outbound UDP target port without round-
// tripping through the profile-migration code at next driver init.
// Backwards-compatible at the wire level; drivers that ignore the new
// request continue to read send_port from oscrouter.json at startup.
//
// v23 (2026-05-19): SetDeviceTransform gains an updateQuash flag that
// gates the driver-side write to `tf.quash`. Previously the driver
// unconditionally assigned `tf.quash = newTransform.quash;` on every
// IPC merge -- callers that built a payload via the partial-init
// constructors (e.g. ResetAndDisableOffsets) defaulted quash to false
// and silently wiped a tracker's hide state. New semantics match the
// existing updateTranslation / updateRotation / updateScale pattern:
// the driver only mutates the stored quash when updateQuash is true.
// Layout grows by one byte; padded to natural alignment. Bump forces
// paired install so a stale driver doesn't read updateQuash from a
// garbage byte and flap unpredictably.
//
// v24 (2026-05-24): AlignmentSpeedParams grows four `double` fields
// (slew_stationary_pos_rate, slew_stationary_rot_rate,
// slew_moving_pos_rate, slew_moving_rot_rate) that drive the new
// time-rate cap in BlendTransform when recalibrateOnMovement is on.
// Replaces the regime-based still-floor (10/50/90 percent) that
// previously closed up to ~85 percent of a Large-regime gap per
// second when the user was stationary. AlignmentSpeedParams grows
// by 32 bytes; still smaller than SetDeviceTransform so the Request
// union does not change size. Bump is required because a stale
// driver paired with a new overlay would read garbage from the new
// offsets; we'd rather refuse the handshake.
//
// v25 (2026-05-26): adds RequestSetHeadMountConfig. Payload carries
// the head-mount tracker mode, resolved deviceId, serial + tracking-
// system strings, the rigid headFromTracker offset (translation +
// quaternion), and the hide/offsetCalibrated flags. The payload
// exceeds sizeof(SetDeviceTransform); sizeof(Request) grows. The
// bump forces a paired overlay+driver reinstall so a stale driver
// rejects the handshake rather than dispatching into a mismatched
// union layout.
//
// v26 (2026-05-27): SetHeadMountConfig gains DriverSynth timing
// fields for tracker stale detection, grace hold, fallback blend,
// stable-return hold, and synth blend. The payload grows, so paired
// overlay+driver install is required.
//
// v27 (2026-06-01): AlignmentSpeedParams drops the unused slew-rate
// fields. The driver blend path no longer reads them, and the matching
// overlay controls/config persistence were removed. The payload shrinks,
// so paired overlay+driver install is required.
//
// v28 (2026-06-02): adds RequestSetPhantomSolverConfig for in-process
// Phantom body-completion calibration. The payload is smaller than
// SetDeviceTransform, so sizeof(Request) is unchanged; the bump forces
// paired overlay+driver install.
//
// v29 (2026-06-04): SetHeadMountConfig carries allowRawHmdFallback so
// DriverSynth can distinguish recovery mode from hard tracker lock. The
// field uses existing payload padding, but the semantic change still
// requires paired overlay+driver install.
const uint32_t Version = 29;

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
	// v22 (2026-05-17): OSC router live send-port edit. Lets the overlay
	// push a new outbound port without round-tripping through the
	// profile-migration code at next startup. Driver applies immediately
	// to the UDP send socket; overlay also writes profiles/oscrouter.json
	// so the value survives a restart even if the driver isn't running.
	RequestSetOscRouterConfig,
	// v25 (2026-05-26): head-mounted tracker config push. Overlay
	// resolves the tracker serial to a live deviceId and sends the
	// full HeadMountConfig over the wire so the driver can apply the
	// headFromTracker offset and hide/show the tracker without parsing
	// the profile JSON itself. Driver caches the payload and reads it
	// on the per-tick pose-update path.
	RequestSetHeadMountConfig,
	// v28 (2026-06-02): body-completion solver calibration and
	// confidence threshold for Phantom virtual roles.
	RequestSetPhantomSolverConfig,
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

#include "ProtocolPayloads.h"

class DriverPoseShmem
{
public:
	struct AugmentedPose
	{
		LARGE_INTEGER sample_time;
		int deviceId;
		vr::DriverPose_t pose;
	};

	// Driver-side telemetry counters. Each is a monotonically increasing count of
	// the number of times the driver took the corresponding code path while
	// processing a tracked-device pose update. Relaxed-ordered atomic increments
	// are sufficient because the overlay only consumes deltas — there is no
	// cross-counter consistency requirement.
	struct Telemetry
	{
		std::atomic<uint64_t> fallback_apply_count;
		std::atomic<uint64_t> per_id_apply_count;
		std::atomic<uint64_t> quash_apply_count;
		// Number of times DriverSynth could not use a synthesized HMD pose
		// because the head-mounted tracker was invalid, stale, or unavailable.
		std::atomic<uint64_t> driver_synth_fallback_count;
		std::atomic<uint64_t> reserved[4];
	};

	// Names of the individual telemetry counters, used by IncrementTelemetry to
	// pick which atomic to bump. Kept inside the class so we don't pollute the
	// `protocol` namespace with another enum.
	enum TelemetryField
	{
		TELEMETRY_FALLBACK_APPLY,
		TELEMETRY_PER_ID_APPLY,
		TELEMETRY_QUASH_APPLY,
		TELEMETRY_DRIVER_SYNTH_FALLBACK,
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

	struct ShmemData
	{
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
	static_assert(sizeof(ShmemData) == sizeof(uint32_t) + sizeof(uint32_t) + sizeof(Telemetry) +
	                                       sizeof(std::atomic<uint64_t>) + sizeof(AugmentedPose) * BUFFERED_SAMPLES,
	              "DriverPoseShmem::ShmemData layout has drifted; bump SHMEM_VERSION and update the static_assert");

private:
	HANDLE hMapFile;
	ShmemData* pData;
	uint64_t cursor;

	AugmentedPose lastPose[vr::k_unMaxTrackedDeviceCount] = {0};

	std::string LastErrorString(DWORD lastError)
	{
		LPSTR buffer = nullptr;
		size_t size =
		    FormatMessageA(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
		                   NULL, lastError, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), (LPSTR)&buffer, 0, NULL);

		std::string message(buffer, size);
		LocalFree(buffer);
		return message;
	}

public:
	operator bool() const { return pData != nullptr; }

	bool operator!() const { return pData == nullptr; }

	DriverPoseShmem()
	{
		hMapFile = INVALID_HANDLE_VALUE;
		pData = nullptr;
		cursor = 0;
	}

	~DriverPoseShmem() { Close(); }

	void Close()
	{
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

	bool Create(LPCSTR segment_name)
	{
		Close();

		hMapFile = CreateFileMappingA(INVALID_HANDLE_VALUE, NULL, PAGE_READWRITE, 0, sizeof(ShmemData), segment_name);

		if (!hMapFile) return false;

		pData = reinterpret_cast<ShmemData*>(MapViewOfFile(hMapFile, FILE_MAP_ALL_ACCESS, 0, 0, sizeof(ShmemData)));

		if (!pData) return false;

		// Stamp the header so the overlay can verify it's looking at a
		// driver-compatible mapping. New file mappings are zero-initialised
		// by the OS, so simply writing the magic/version is enough.
		pData->magic = SHMEM_MAGIC;
		pData->shmem_version = SHMEM_VERSION;

		return true;
	}

	void Open(LPCSTR segment_name)
	{
		Close();

		hMapFile = OpenFileMappingA(FILE_MAP_ALL_ACCESS, FALSE, segment_name);

		if (!hMapFile) {
			throw std::runtime_error("Failed to open pose data shared memory segment: " +
			                         LastErrorString(GetLastError()));
		}

		pData = reinterpret_cast<ShmemData*>(MapViewOfFile(hMapFile, FILE_MAP_ALL_ACCESS, 0, 0, sizeof(ShmemData)));

		if (!pData) {
			throw std::runtime_error("Failed to map pose data shared memory segment: " +
			                         LastErrorString(GetLastError()));
		}

		// Validate the header. A mismatch means the overlay is paired with a
		// driver build that doesn't share this layout — much safer to throw
		// here than to silently corrupt poses by reading at the wrong offsets.
		if (pData->magic != SHMEM_MAGIC) {
			char buf[128];
			snprintf(buf, sizeof buf,
			         "Pose shmem segment magic mismatch: got 0x%08X, expected 0x%08X (driver/overlay version skew?)",
			         pData->magic, SHMEM_MAGIC);
			UnmapViewOfFile(pData);
			pData = nullptr;
			CloseHandle(hMapFile);
			hMapFile = nullptr;
			throw std::runtime_error(buf);
		}
		if (pData->shmem_version != SHMEM_VERSION) {
			char buf[128];
			snprintf(buf, sizeof buf,
			         "Pose shmem segment version mismatch: got %u, expected %u (driver/overlay version skew?)",
			         pData->shmem_version, SHMEM_VERSION);
			UnmapViewOfFile(pData);
			pData = nullptr;
			CloseHandle(hMapFile);
			hMapFile = nullptr;
			throw std::runtime_error(buf);
		}

		char tmp[256];
		snprintf(tmp, sizeof tmp, "Opened shmem segment: %p\n", pData);
		OutputDebugStringA(tmp);
	}

	void ReadNewPoses(std::function<void(AugmentedPose const&)> cb)
	{
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

	bool GetPose(int index, vr::DriverPose_t& pose, LARGE_INTEGER* pSampleTime = NULL)
	{
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

	void SetPose(int index, const vr::DriverPose_t& pose)
	{
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
	void IncrementTelemetry(TelemetryField field)
	{
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
			case TELEMETRY_DRIVER_SYNTH_FALLBACK:
				pData->telemetry.driver_synth_fallback_count.fetch_add(1, std::memory_order_relaxed);
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
	bool GetTelemetry(uint64_t& fallback_apply, uint64_t& per_id_apply, uint64_t& quash_apply,
	                  uint64_t& driver_synth_fallback)
	{
		if (pData == nullptr) return false;
		fallback_apply = pData->telemetry.fallback_apply_count.load(std::memory_order_relaxed);
		per_id_apply = pData->telemetry.per_id_apply_count.load(std::memory_order_relaxed);
		quash_apply = pData->telemetry.quash_apply_count.load(std::memory_order_relaxed);
		driver_synth_fallback = pData->telemetry.driver_synth_fallback_count.load(std::memory_order_relaxed);
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
	char path[INPUTHEALTH_PATH_LEN];
	uint8_t is_scalar;
	uint8_t is_boolean;
	uint8_t axis_role;    // inputhealth::AxisRole
	uint8_t scalar_type;  // vr::EVRScalarType
	uint8_t scalar_units; // vr::EVRScalarUnits
	uint8_t ph_initialized;
	uint8_t ph_triggered;
	uint8_t ph_triggered_positive;
	uint8_t rest_min_initialized;
	uint8_t last_boolean;
	uint8_t _pad_component_flags[6];

	// Welford streaming mean / variance.
	uint64_t welford_count;
	double welford_mean;
	double welford_m2;

	// Page-Hinkley accumulator.
	double ph_mean;
	double ph_pos;
	double ph_neg;

	// EWMA-decayed rolling minimum (rest-stuck detection on triggers).
	double rest_min;

	// Last raw sample on this handle (last_value for scalars, latest
	// boolean state for booleans -- redundant with last_boolean above
	// for booleans, kept consistent for scalars).
	float last_value;
	uint32_t _pad_lv;

	uint64_t last_update_us;
	uint64_t press_count;
	uint64_t bounce_transition_count;
	uint32_t bounce_max_interval_us;
	uint32_t _pad_bounce;

	// Raw scalar range observed since last reset. Cheap O(1) driver-side
	// bookkeeping used by the overlay to sanity-check trigger/stick
	// calibration coverage before making any health claim.
	uint8_t scalar_range_initialized;
	uint8_t _pad_scalar_range_flags[3];
	float observed_min;
	float observed_max;
	uint32_t _pad_observed_range;

	// Polar histogram bins. Only meaningful for AxisRole::StickX;
	// other roles leave these zero. Per-bin: max_r and count and
	// last_update_us.
	float polar_max_r[INPUTHEALTH_POLAR_BIN_COUNT];
	uint16_t polar_count[INPUTHEALTH_POLAR_BIN_COUNT];
	uint16_t _pad_polar_count[INPUTHEALTH_POLAR_BIN_COUNT]; // align next field to 8
	uint64_t polar_last_update_us[INPUTHEALTH_POLAR_BIN_COUNT];
	float polar_global_max_r;
	uint32_t _pad_polar_global;
};

// Per-slot record: an atomic seqlock counter followed by the body. The
// counter is incremented twice per write (odd during, even after) so the
// reader can detect torn reads. The body is memcpy'd in/out -- never
// touched field-by-field by the wire layer.
struct InputHealthSnapshotSlot
{
	std::atomic<uint64_t> generation;
	InputHealthSnapshotBody body;
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
	static const uint32_t SHMEM_MAGIC = 0x494E4848; // 'INHH'
	static const uint32_t SHMEM_VERSION = 3;

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
	HANDLE hMapFile = INVALID_HANDLE_VALUE;
	ShmemData* pData = nullptr;

	std::string LastErrorString(DWORD lastError)
	{
		LPSTR buffer = nullptr;
		size_t size =
		    FormatMessageA(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
		                   NULL, lastError, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), (LPSTR)&buffer, 0, NULL);
		std::string message(buffer ? buffer : "", size);
		if (buffer) LocalFree(buffer);
		return message;
	}

public:
	InputHealthSnapshotShmem() = default;
	~InputHealthSnapshotShmem() { Close(); }

	InputHealthSnapshotShmem(const InputHealthSnapshotShmem&) = delete;
	InputHealthSnapshotShmem& operator=(const InputHealthSnapshotShmem&) = delete;

	operator bool() const { return pData != nullptr; }

	void Close()
	{
		if (pData) {
			UnmapViewOfFile(pData);
			pData = nullptr;
		}
		if (hMapFile && hMapFile != INVALID_HANDLE_VALUE) {
			CloseHandle(hMapFile);
			hMapFile = INVALID_HANDLE_VALUE;
		}
	}

	// Driver-side: create or re-open the shmem segment, stamp the header,
	// zero the slot table. Idempotent on the same name.
	bool Create(LPCSTR segment_name)
	{
		Close();
		hMapFile = CreateFileMappingA(INVALID_HANDLE_VALUE, NULL, PAGE_READWRITE, 0, sizeof(ShmemData), segment_name);
		if (!hMapFile) return false;

		pData = reinterpret_cast<ShmemData*>(MapViewOfFile(hMapFile, FILE_MAP_ALL_ACCESS, 0, 0, sizeof(ShmemData)));
		if (!pData) return false;

		// Fresh mappings are zero-filled by the OS, so only the header
		// fields need an explicit stamp. Existing mappings (driver
		// reload) are left untouched apart from the magic/version
		// re-stamp; the slot generations from the prior incarnation
		// remain valid until the publisher overwrites them.
		pData->magic = SHMEM_MAGIC;
		pData->shmem_version = SHMEM_VERSION;
		pData->slot_count = INPUTHEALTH_SLOT_COUNT;
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
			throw std::runtime_error("Failed to open InputHealth shmem segment: " + LastErrorString(GetLastError()));
		}
		pData = reinterpret_cast<ShmemData*>(MapViewOfFile(hMapFile, FILE_MAP_READ, 0, 0, sizeof(ShmemData)));
		if (!pData) {
			DWORD err = GetLastError();
			CloseHandle(hMapFile);
			hMapFile = INVALID_HANDLE_VALUE;
			throw std::runtime_error("Failed to map InputHealth shmem segment: " + LastErrorString(err));
		}
		if (pData->magic != SHMEM_MAGIC) {
			char buf[160];
			snprintf(buf, sizeof buf, "InputHealth shmem magic mismatch: got 0x%08X, expected 0x%08X", pData->magic,
			         SHMEM_MAGIC);
			Close();
			throw std::runtime_error(buf);
		}
		if (pData->shmem_version != SHMEM_VERSION) {
			char buf[160];
			snprintf(buf, sizeof buf, "InputHealth shmem version mismatch: got %u, expected %u", pData->shmem_version,
			         SHMEM_VERSION);
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
	InputHealthSnapshotSlot* SlotForWrite(uint32_t index)
	{
		if (!pData) return nullptr;
		if (index >= pData->slot_count) return nullptr;
		return &pData->slots[index];
	}

	// Driver-side: write a snapshot body into slot[index] under the
	// per-slot seqlock. Bumps the generation odd before the memcpy and
	// even after, so a concurrent overlay reader can detect torn reads.
	// No-op if the slot pointer is unavailable.
	void WriteSlot(uint32_t index, const InputHealthSnapshotBody& body)
	{
		InputHealthSnapshotSlot* slot = SlotForWrite(index);
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
	bool TryReadSlot(uint32_t index, InputHealthSnapshotBody& out, int max_retries = 8) const
	{
		if (!pData) return false;
		if (index >= pData->slot_count) return false;
		const InputHealthSnapshotSlot& slot = pData->slots[index];
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
	float eye_origin_l[3];
	float eye_origin_r[3];

	// Per-eye gaze direction as a unit vector in HMD space. The OpenVR
	// convention is +X right, +Y up, -Z forward; gaze pointing straight
	// ahead is (0, 0, -1).
	float eye_gaze_l[3];
	float eye_gaze_r[3];

	// Per-eye eyelid openness 0..1. 0 = fully closed, 1 = fully open. Linear,
	// not gamma-corrected.
	float eye_openness_l;
	float eye_openness_r;

	// Per-eye pupil dilation 0..1. Most hardware exposes this as a relative
	// signal; the driver's continuous calibration normalises across the
	// observed range. 0 = constricted, 1 = dilated.
	float pupil_dilation_l;
	float pupil_dilation_r;

	// Per-eye confidence 0..1. If the hardware exposes a confidence signal
	// the host writes it directly; otherwise the host writes a synthesised
	// value (frame age + signal stability). Driver filters use this for
	// weighted blending and eye-dropout fallback.
	float eye_confidence_l;
	float eye_confidence_r;

	// Facial expression shapes in Unified Expressions v2 order. Range 0..1
	// per shape unless the SDK documents otherwise (a few directional shapes
	// are signed; the host coerces those into [0,1] before publish so the
	// wire format stays uniform).
	float expressions[FACETRACKING_EXPRESSION_COUNT];

	// bit 0: eye fields are valid this frame.
	// bit 1: expression fields are valid this frame.
	// Other bits reserved; reader must ignore. Lets a host module that only
	// supports one capability publish frames the driver can still consume.
	uint32_t flags;

	// Head pose in HMD-local space. Yaw/pitch/roll in radians; pos in metres.
	// head_flags bit 0 set: head fields are valid this frame.
	float head_yaw;
	float head_pitch;
	float head_roll;
	float head_pos_x;
	float head_pos_y;
	float head_pos_z;
	uint32_t head_flags;

	// Raw upstream VRCFaceTracking UnifiedExpressions slots. This is not part
	// of the shmem wire body; FaceFrameReader copies it from
	// FaceTrackingFrameBodyWire so OSC publishing can mirror VRCFaceTracking's
	// current FT/v2 parameter family without losing shapes during the internal
	// 63-slot remap.
	float upstream_expressions[FACETRACKING_UPSTREAM_EXPRESSION_COUNT];
};

static_assert(std::is_trivially_copyable<FaceTrackingFrameBody>::value,
              "FaceTrackingFrameBody must be trivially copyable for shmem use");
static_assert(std::is_standard_layout<FaceTrackingFrameBody>::value,
              "FaceTrackingFrameBody must be standard-layout for field translation");

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

	float eye_origin_l[3];
	float eye_origin_r[3];
	float eye_gaze_l[3];
	float eye_gaze_r[3];

	float eye_openness_l;
	float eye_openness_r;
	float pupil_dilation_l;
	float pupil_dilation_r;
	float eye_confidence_l;
	float eye_confidence_r;

	float expressions[FACETRACKING_UPSTREAM_EXPRESSION_COUNT];

	uint32_t flags;

	float head_yaw;
	float head_pitch;
	float head_roll;
	float head_pos_x;
	float head_pos_y;
	float head_pos_z;
	uint32_t head_flags;
};

static_assert(std::is_trivially_copyable<FaceTrackingFrameBodyWire>::value,
              "FaceTrackingFrameBodyWire must be trivially copyable for shmem use");
static_assert(std::is_standard_layout<FaceTrackingFrameBodyWire>::value,
              "FaceTrackingFrameBodyWire must be standard-layout for stable wire format");

// Sanity-check that the wire body starts with the same fields as the
// consumer-facing body. FaceTrackingFrameBody may carry non-wire fields at
// the tail, but shared wire fields must stay aligned or the host writes
// through the wrong offsets.
static_assert(offsetof(FaceTrackingFrameBodyWire, qpc_sample_time) == offsetof(FaceTrackingFrameBody, qpc_sample_time),
              "qpc_sample_time offset mismatch");
static_assert(offsetof(FaceTrackingFrameBodyWire, expressions) == offsetof(FaceTrackingFrameBody, expressions),
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
	HostStateLegacy = 0,     // pre-heartbeat host; ignore heartbeat field
	HostStatePublishing = 1, // active module pushing frames at full rate
	HostStateIdle = 2,       // host alive but no module selected / paused
	HostStateDraining = 3,   // host shutting down cleanly; final frames in flight
};

class FaceTrackingFrameShmem
{
public:
	static const uint32_t SHMEM_MAGIC = 0x46544652; // 'FTFR'
	static const uint32_t SHMEM_VERSION =
	    3; // v3: expressions grown to upstream-format 88 slots; driver remaps via UpstreamShapeMap
	static const uint32_t RING_SIZE = 32;

private:
	struct ShmemData
	{
		uint32_t magic;         // @  0
		uint32_t shmem_version; // @  4
		uint32_t ring_size;     // @  8

		// Host-side liveness signals. Hosts that pre-date this field write
		// zero, which we treat as HostStateLegacy and skip the heartbeat
		// check (so the layout change does not require SHMEM_VERSION bump).
		uint32_t host_state;                      // @ 12 -- HostState enum
		std::atomic<uint64_t> host_heartbeat_qpc; // @ 16 -- QueryPerformanceCounter at last tick

		// Reserved for future header expansion (capability negotiation,
		// per-host stats, etc.). Kept zero by every current writer.
		uint32_t _reserved_header[2]; // @ 24..31

		// Monotonically increasing publish counter. The slot the writer just
		// finished is at index (publish_index - 1) % RING_SIZE. 0 means no
		// frame has been published since shmem creation.
		std::atomic<uint64_t> publish_index; // @ 32

		FaceTrackingFrameSlot slots[RING_SIZE]; // @ 40
	};
	// Compile-time sanity on the field offsets the C# host depends on.
	// FrameWriter.cs hardcodes 12 / 16 / 32 / 40; if anything shifts here
	// the host writes through the wrong pointer and we get torn / zero
	// data, with no immediate error.
	static_assert(offsetof(ShmemData, host_state) == 12, "host_state offset");
	static_assert(offsetof(ShmemData, host_heartbeat_qpc) == 16, "host_heartbeat_qpc offset");
	static_assert(offsetof(ShmemData, publish_index) == 32, "publish_index offset");
	static_assert(offsetof(ShmemData, slots) == 40, "slots offset");

	HANDLE hMapFile = INVALID_HANDLE_VALUE;
	ShmemData* pData = nullptr;

	std::string LastErrorString(DWORD lastError)
	{
		LPSTR buffer = nullptr;
		size_t size =
		    FormatMessageA(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
		                   NULL, lastError, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), (LPSTR)&buffer, 0, NULL);
		std::string message(buffer ? buffer : "", size);
		if (buffer) LocalFree(buffer);
		return message;
	}

public:
	FaceTrackingFrameShmem() = default;
	~FaceTrackingFrameShmem() { Close(); }

	FaceTrackingFrameShmem(const FaceTrackingFrameShmem&) = delete;
	FaceTrackingFrameShmem& operator=(const FaceTrackingFrameShmem&) = delete;

	operator bool() const { return pData != nullptr; }

	void Close()
	{
		if (pData) {
			UnmapViewOfFile(pData);
			pData = nullptr;
		}
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
		hMapFile = CreateFileMappingA(INVALID_HANDLE_VALUE, NULL, PAGE_READWRITE, 0, sizeof(ShmemData), segment_name);
		if (!hMapFile) return false;

		pData = reinterpret_cast<ShmemData*>(MapViewOfFile(hMapFile, FILE_MAP_ALL_ACCESS, 0, 0, sizeof(ShmemData)));
		if (!pData) return false;

		pData->magic = SHMEM_MAGIC;
		pData->shmem_version = SHMEM_VERSION;
		pData->ring_size = RING_SIZE;
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
			throw std::runtime_error("Failed to open FaceTracking shmem segment: " + LastErrorString(GetLastError()));
		}
		pData = reinterpret_cast<ShmemData*>(MapViewOfFile(hMapFile, FILE_MAP_ALL_ACCESS, 0, 0, sizeof(ShmemData)));
		if (!pData) {
			DWORD err = GetLastError();
			CloseHandle(hMapFile);
			hMapFile = INVALID_HANDLE_VALUE;
			throw std::runtime_error("Failed to map FaceTracking shmem segment: " + LastErrorString(err));
		}
		if (pData->magic != SHMEM_MAGIC) {
			char buf[160];
			snprintf(buf, sizeof buf, "FaceTracking shmem magic mismatch: got 0x%08X, expected 0x%08X", pData->magic,
			         SHMEM_MAGIC);
			Close();
			throw std::runtime_error(buf);
		}
		if (pData->shmem_version != SHMEM_VERSION) {
			char buf[160];
			snprintf(buf, sizeof buf, "FaceTracking shmem version mismatch: got %u, expected %u", pData->shmem_version,
			         SHMEM_VERSION);
			Close();
			throw std::runtime_error(buf);
		}
	}

	uint32_t RingSize() const { return pData ? pData->ring_size : 0; }

	// Reader: latest published index. Increasing means at least one new
	// frame is available since the prior read; equal means no new data.
	uint64_t PublishIndex() const { return pData ? pData->publish_index.load(std::memory_order_acquire) : 0; }

	// Reader: host's last heartbeat timestamp in QueryPerformanceCounter
	// ticks. Zero means the host either has not written a heartbeat yet
	// or is a pre-heartbeat (legacy) build; either way the driver should
	// fall back to handle-only liveness detection.
	uint64_t HostHeartbeatQpc() const { return pData ? pData->host_heartbeat_qpc.load(std::memory_order_acquire) : 0; }

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
	bool TryReadLatestWire(FaceTrackingFrameBodyWire& out, int max_retries = 8) const
	{
		if (!pData) return false;
		const uint64_t idx = pData->publish_index.load(std::memory_order_acquire);
		if (idx == 0) return false;
		const FaceTrackingFrameSlot& slot = pData->slots[(idx - 1) % RING_SIZE];
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
	bool TryReadWireByIndex(uint64_t target_index, FaceTrackingFrameBodyWire& out, int max_retries = 8) const
	{
		if (!pData || target_index == 0) return false;
		const uint64_t now = pData->publish_index.load(std::memory_order_acquire);
		if (target_index > now) return false;
		if (now - target_index >= RING_SIZE) return false;
		const FaceTrackingFrameSlot& slot = pData->slots[(target_index - 1) % RING_SIZE];
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
	void Publish(const FaceTrackingFrameBodyWire& body)
	{
		if (!pData) return;
		const uint64_t next = pData->publish_index.load(std::memory_order_relaxed) + 1;
		FaceTrackingFrameSlot& slot = pData->slots[(next - 1) % RING_SIZE];
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
	char address_pattern[OSC_ROUTE_ADDR_LEN];
	char subscriber_id[32];
	std::atomic<uint64_t> match_count;     // OSC messages matched by pattern
	std::atomic<uint64_t> drop_count;      // matched but dropped (queue full)
	std::atomic<uint64_t> last_match_tick; // QPC tick of most recent match
	uint8_t active;                        // 1 = slot in use, 0 = empty
	uint8_t _reserved[7];
};

class OscRouterStatsShmem
{
public:
	static const uint32_t SHMEM_MAGIC = 0xC5C7057C;
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

	HANDLE hMapFile = INVALID_HANDLE_VALUE;
	ShmemData* pData = nullptr;

	std::string LastErrorString(DWORD lastError)
	{
		LPSTR buffer = nullptr;
		DWORD size =
		    FormatMessageA(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
		                   NULL, lastError, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), (LPSTR)&buffer, 0, NULL);
		std::string msg(buffer ? buffer : "", size);
		if (buffer) LocalFree(buffer);
		return msg;
	}

public:
	OscRouterStatsShmem() = default;
	~OscRouterStatsShmem() { Close(); }

	OscRouterStatsShmem(const OscRouterStatsShmem&) = delete;
	OscRouterStatsShmem& operator=(const OscRouterStatsShmem&) = delete;

	operator bool() const { return pData != nullptr; }

	void Close()
	{
		if (pData) {
			UnmapViewOfFile(pData);
			pData = nullptr;
		}
		if (hMapFile && hMapFile != INVALID_HANDLE_VALUE) {
			CloseHandle(hMapFile);
			hMapFile = INVALID_HANDLE_VALUE;
		}
	}

	// Driver-side: create or re-open the segment and stamp the header.
	bool Create(LPCSTR segment_name)
	{
		Close();
		hMapFile = CreateFileMappingA(INVALID_HANDLE_VALUE, NULL, PAGE_READWRITE, 0, sizeof(ShmemData), segment_name);
		if (!hMapFile) return false;
		pData = reinterpret_cast<ShmemData*>(MapViewOfFile(hMapFile, FILE_MAP_ALL_ACCESS, 0, 0, sizeof(ShmemData)));
		if (!pData) return false;
		pData->magic = SHMEM_MAGIC;
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
			throw std::runtime_error(std::string("Failed to open OscRouter shmem: ") + LastErrorString(GetLastError()));
		}
		pData = reinterpret_cast<ShmemData*>(MapViewOfFile(hMapFile, FILE_MAP_READ, 0, 0, sizeof(ShmemData)));
		if (!pData) {
			DWORD err = GetLastError();
			CloseHandle(hMapFile);
			hMapFile = INVALID_HANDLE_VALUE;
			throw std::runtime_error(std::string("Failed to map OscRouter shmem: ") + LastErrorString(err));
		}
		if (pData->magic != SHMEM_MAGIC) {
			char buf[160];
			snprintf(buf, sizeof buf, "OscRouter shmem magic mismatch: got 0x%08X, expected 0x%08X", pData->magic,
			         SHMEM_MAGIC);
			Close();
			throw std::runtime_error(buf);
		}
		if (pData->shmem_version != SHMEM_VERSION) {
			char buf[160];
			snprintf(buf, sizeof buf, "OscRouter shmem version mismatch: got %u, expected %u", pData->shmem_version,
			         SHMEM_VERSION);
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
	static void CopyStrField(char* dst, size_t dst_size, const char* src)
	{
		if (!src) {
			dst[0] = '\0';
			return;
		}
		size_t n = 0;
		for (; n < dst_size - 1 && src[n]; ++n)
			dst[n] = src[n];
		dst[n] = '\0';
	}

	void WriteRoute(uint32_t index, const char* pattern, const char* subscriber_id, uint64_t match_count,
	                uint64_t drop_count, uint64_t last_match_tick, bool active)
	{
		if (!pData || index >= OSC_ROUTER_ROUTE_SLOTS) return;
		OscRouterRouteSlot& s = pData->routes[index];
		const uint64_t prev = s.generation.load(std::memory_order_relaxed);
		s.generation.store(prev + 1, std::memory_order_release);
		CopyStrField(s.address_pattern, sizeof(s.address_pattern), pattern);
		CopyStrField(s.subscriber_id, sizeof(s.subscriber_id), subscriber_id);
		s.match_count.store(match_count, std::memory_order_relaxed);
		s.drop_count.store(drop_count, std::memory_order_relaxed);
		s.last_match_tick.store(last_match_tick, std::memory_order_relaxed);
		s.active = active ? 1 : 0;
		s.generation.store(prev + 2, std::memory_order_release);
	}

	// Overlay-side: read global totals into an OscRouterStats struct.
	bool ReadGlobalStats(OscRouterStats& out) const
	{
		if (!pData) return false;
		out.packets_sent = pData->packets_sent.load(std::memory_order_relaxed);
		out.bytes_sent = pData->bytes_sent.load(std::memory_order_relaxed);
		out.packets_dropped = pData->packets_dropped.load(std::memory_order_relaxed);
		out.active_routes = 0;
		for (uint32_t i = 0; i < OSC_ROUTER_ROUTE_SLOTS; ++i)
			if (pData->routes[i].active) ++out.active_routes;
		out._reserved = 0;
		return true;
	}

	// Overlay-side: try reading one route slot under its seqlock.
	// Returns true on a clean read. Caller re-tries on false or skips.
	bool TryReadRoute(uint32_t index, OscRouterRouteSlot& out, int max_retries = 8) const
	{
		if (!pData || index >= OSC_ROUTER_ROUTE_SLOTS) return false;
		const OscRouterRouteSlot& s = pData->routes[index];
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
} // namespace protocol
