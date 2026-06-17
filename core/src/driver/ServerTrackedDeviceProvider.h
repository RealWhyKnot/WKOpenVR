#pragma once

#define EIGEN_MPL2_ONLY

#include "BuildChannel.h"
#include "DriverModule.h"
#include "DriverSynthCompose.h"
#include "IPCServer.h"
#include "ModulePerf.h"
#include "ModuleSafety.h"
#include "Protocol.h"
#include "ServerTrackedDeviceProviderConfigPacking.h"
#include "IsometryTransform.h"
#include "SmartSmoothingShadowMath.h"

#include <Eigen/Dense>

#include <openvr_driver.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <unordered_map>
#include <vector>

class ServerTrackedDeviceProvider : public vr::IServerTrackedDeviceProvider
{
public:
	////// Start vr::IServerTrackedDeviceProvider functions

	/** initializes the driver. This will be called before any other methods are called. */
	virtual vr::EVRInitError Init(vr::IVRDriverContext* pDriverContext) override;

	/** cleans up the driver right before it is unloaded */
	virtual void Cleanup() override;

	/** Returns the version of the ITrackedDeviceServerDriver interface used by this driver */
	virtual const char* const* GetInterfaceVersions() { return vr::k_InterfaceVersions; }

	/** Allows the driver do to some work in the main loop of the server. */
	virtual void RunFrame() override;

	/** Returns true if the driver wants to block Standby mode. */
	virtual bool ShouldBlockStandbyMode() { return false; }

	/** Called when the system is entering Standby mode. The driver should switch itself into whatever sort of low-power
	 * state it has. */
	virtual void EnterStandby() {}

	/** Called when the system is leaving Standby mode. The driver should switch itself back to
	full operation. */
	virtual void LeaveStandby() {}

	////// End vr::IServerTrackedDeviceProvider functions

	ServerTrackedDeviceProvider() = default;
	void SetDeviceTransform(const protocol::SetDeviceTransform& newTransform);
	void SetTrackingSystemFallback(const protocol::SetTrackingSystemFallback& newFallback);
	// v12: per-device pose-prediction push from the Smoothing overlay. Updates
	// the predictionSmoothness + recalibrateOnMovement slots without touching
	// transform/scale/enabled. SetDeviceTransform from SC stopped writing those
	// slots when this message was introduced, so the two overlays no longer
	// clobber each other.
	void SetDevicePrediction(const protocol::SetDevicePrediction& cfg);
	bool HandleDevicePoseUpdated(uint32_t openVRID, vr::DriverPose_t& pose);
	std::vector<std::pair<uint32_t, vr::DriverPose_t>> CollectPhantomSyntheticPoseUpdates(uint32_t triggeringOpenVRID);
	void HandleApplyRandomOffset();
	void HandleSetAlignmentSpeedParams(const protocol::AlignmentSpeedParams params) { alignmentSpeedParams = params; }
	// v25: head-mount tracker config from the overlay. Caches state used by
	// the DriverSynth branch in HandleDevicePoseUpdated.
	void SetHeadMountConfig(const protocol::SetHeadMountConfig& cfg);
	bool HandleIpcRequest(uint32_t featureMask, const protocol::Request& request, protocol::Response& response);
	void OnGetGenericInterface(const char* pchInterface, void* iface);

	// Finger-smoothing config cache. Written by IPCServer when the overlay
	// pushes a new config (rare -- only on UI changes). Read by the
	// IVRDriverInputInternal::UpdateSkeletonComponent detour at hand-update
	// rate (~340 Hz/hand). Held under its OWN mutex distinct from
	// stateMutex so finger updates can never block the pose-update path,
	// and vice-versa.
	void SetFingerSmoothingConfig(const protocol::FingerSmoothingConfig& cfg);
	protocol::FingerSmoothingConfig GetFingerSmoothingConfig() const;
	void SetDashboardHandTrackingState(const protocol::DashboardHandTrackingState& state);
	pairdriver::DashboardHandTrackingSnapshot GetDashboardHandTrackingSnapshot() const;

	// Input-health config cache. Written by IPCServer when the WKOpenVR-InputHealth
	// overlay pushes a new config (rare). Read by the boolean / scalar input
	// detours on every component update once Stage 1B lands (same atomic-pack
	// pattern as fingerCfgPacked so the per-tick read is a single relaxed load).
	void SetInputHealthConfig(const protocol::InputHealthConfig& cfg);
	protocol::InputHealthConfig GetInputHealthConfig() const;
	void SetInputHealthCompensation(const protocol::InputHealthCompensationEntry& entry);
	bool LookupInputHealthCompensation(uint64_t serial_hash, const std::string& path,
	                                   protocol::InputHealthCompensationEntry& out) const;
	void ClearInputHealthCompensation(uint64_t serial_hash);

private:
	struct ActiveDriverModule
	{
		std::unique_ptr<DriverModule> module;
		const openvr_pair::common::module_safety::ModuleSpec* safety = nullptr;
	};

	void DisableActiveModuleAt(size_t index, const char* reason);
	bool DisableActiveModuleByMask(uint32_t featureMask, const char* reason, bool markClean = false);
	void DisableDetachedModule(ActiveDriverModule entry, const char* reason, bool markClean);
	void StopIpcServerForFeatureMask(uint32_t featureMask);
	void ReconcileSidecarFeatureFlags();

	// Per-feature IPC servers, allocated only when the matching enable_*.flag
	// is detected at Init. Any may be null in feature-disabled builds; the
	// pose-update path doesn't touch them.
	std::unique_ptr<IPCServer> calibrationServer;
	std::unique_ptr<IPCServer> smoothingServer;
	std::unique_ptr<IPCServer> dashboardInputServer;
	std::unique_ptr<IPCServer> inputHealthServer;
	std::unique_ptr<IPCServer> faceTrackingServer;
	std::unique_ptr<IPCServer> oscRouterServer;
	std::unique_ptr<IPCServer> captionsServer;
	std::unique_ptr<IPCServer> phantomServer;
	std::mutex activeModulesMutex;
	std::vector<ActiveDriverModule> activeModules;

	// Pose telemetry shmem, only created when calibration is enabled. The
	// calibration overlay opens this segment to read driver-side pose
	// snapshots and telemetry counters.
	protocol::DriverPoseShmem shmem;

	// Always-on perf stats segment: process totals plus per-module CPU and
	// sidecar attribution, published at 1 Hz from RunFrame for the overlay's
	// Modules-tab performance card. Created in Init regardless of feature
	// flags so the overlay can tell "driver not running" from "modules idle".
	protocol::PerfStatsShmem perfStatsShmem;
	openvr_pair::common::moduleperf::Sampler perfSampler{1000};
	// Wall clock of the last [perf] text-line batch; lines keep the old 10 s
	// cadence while the shmem publish runs at the 1 Hz sample rate.
	uint64_t lastPerfLogWallMs = 0;
	uint64_t lastSidecarFlagCheckMs = 0;

	// Bitmask of pairdriver::kFeature* flags detected at Init(). 0 means
	// neither flag was present and the driver is running inert.
	uint32_t featureFlags = 0;

	enum DeltaSize
	{
		TINY,
		SMALL,
		LARGE
	};

#if WKOPENVR_BUILD_IS_DEV
	// Aggregated over a ~5 s window, then logged and reset. The shadow runs a
	// CANDIDATE one-euro (BuildCandidateParams) against the live filter so the
	// log shows the jitter (at rest) vs lag (in motion) tradeoff of a tuning we
	// might promote.
	struct SmartSmoothingShadowStats
	{
		uint64_t samples = 0;
		uint64_t restSamples = 0;
		uint64_t moveSamples = 0;
		uint64_t invalidReportedLinear = 0;
		uint64_t invalidReportedAngular = 0;
		uint64_t reseeds = 0;
		uint64_t gapReseeds = 0;
		uint64_t jumpReseeds = 0;
		uint64_t nonFinitePoseResets = 0;

		// Candidate gate + adaptive cutoff (averaged / peak).
		double sumPosRelease = 0.0;
		double maxPosRelease = 0.0;
		double sumRotRelease = 0.0;
		double maxRotRelease = 0.0;
		double sumPosCutoffHz = 0.0;
		double maxPosCutoffHz = 0.0;
		double sumRotCutoffHz = 0.0;
		double maxRotCutoffHz = 0.0;

		// Rest regime: per-frame output step (jitter). Lower = smoother at rest.
		double sumSqRestRawStepM = 0.0;
		double sumSqRestLiveStepM = 0.0;
		double sumSqRestCandStepM = 0.0;
		double sumSqRestRawRotStepRad = 0.0;
		double sumSqRestLiveRotStepRad = 0.0;
		double sumSqRestCandRotStepRad = 0.0;

		// Move regime: output-vs-raw lag. Lower = more responsive in motion.
		double sumSqMoveLiveLagM = 0.0;
		double maxMoveLiveLagM = 0.0;
		double sumSqMoveCandLagM = 0.0;
		double maxMoveCandLagM = 0.0;
		double sumSqMoveLiveLagRad = 0.0;
		double sumSqMoveCandLagRad = 0.0;

		// Candidate divergence from the live filter and from raw (all samples).
		double sumSqCandLiveErrM = 0.0;
		double maxCandLiveErrM = 0.0;
		double sumSqCandRawErrM = 0.0;
		double sumSqCandLiveErrRad = 0.0;
		double sumSqCandRawErrRad = 0.0;
	};

	struct SmartSmoothingShadowState
	{
		bool previousOutputsInitialized = false;
		LARGE_INTEGER lastSample{};
		LARGE_INTEGER lastLog{};
		LARGE_INTEGER lastCandidateCheck{};

		// The candidate one-euro filter -- its own state, distinct from the live
		// device.smartFilter -- plus the selected variant.
		prediction::smart_shadow::FilterState filter;
		prediction::smart_shadow::CandidateKind candidate = prediction::smart_shadow::CandidateKind::Strong;

		// Previous-frame outputs (raw / live / candidate) for per-frame step.
		double prevRawPos[3] = {0.0, 0.0, 0.0};
		double prevLivePos[3] = {0.0, 0.0, 0.0};
		double prevCandPos[3] = {0.0, 0.0, 0.0};
		double prevRawRot[4] = {1.0, 0.0, 0.0, 0.0};
		double prevLiveRot[4] = {1.0, 0.0, 0.0, 0.0};
		double prevCandRot[4] = {1.0, 0.0, 0.0, 0.0};

		SmartSmoothingShadowStats stats;
	};
#endif

	struct DeviceTransform
	{
		bool enabled = false;
		bool quash = false;
		bool prevQuash = false;
		IsoTransform transform, targetTransform;
		double scale = 1.0;
		LARGE_INTEGER lastPoll{};
		DeltaSize currentRate = DeltaSize::TINY;
		// True when the slot's transform/targetTransform are tracking a tracking-
		// system fallback rather than an overlay-supplied per-ID value. Used to
		// snap on the first activation of fallback.
		bool fallbackActive = false;
		// Prediction-suppression strength on a 0..100 scale. The pose's velocity
		// / acceleration / poseTimeOffset fields and position filter are tuned
		// from this value. 0 = pose untouched; 100 = strongest smoothing with
		// bounded release, not a hard pose freeze. The overlay enforces the hard
		// block on HMD / ref / target, so by the time we see a non-zero value
		// here the sender has already vetted that suppressing this device is safe.
		uint8_t predictionSmoothness = 0;

		// One-euro smoothing. When predictionSmoothness > 0 the device's reported
		// position -- and, in dev builds with smartEnabled on, rotation -- is run
		// through a speed-adaptive low-pass (SmartSmoothingShadowMath.h): heavy
		// smoothing at rest, bounded release in motion, and never frozen. Position
		// smoothing is the baseline for every user; smartEnabled is only the
		// dev-only rotation preview toggle. All fields below are guarded by
		// stateMutex, same as predictionSmoothness.
		bool smartEnabled = false;

		// One-euro filter coefficients (slider-derived via BuildParams) + running
		// state + the QPC timestamp of the previous sample (for per-frame dt and
		// the long-gap reseed). Reset whenever smoothness changes or drops to 0.
		prediction::smart_shadow::Params smartShadowParams;
		prediction::smart_shadow::FilterState smartFilter;
		LARGE_INTEGER smartFilterLastSample{};

		// When true, BlendTransform's lerp toward targetTransform only advances
		// proportional to detected per-frame motion magnitude. A stationary user
		// (lying down, sitting still) sees no calibration drift even when the
		// math has updated -- the catch-up happens during the user's next motion,
		// hidden by the natural movement instead of looking like a phantom body
		// shift. Default false; the overlay enables it per-device when the
		// recalibrateOnMovement profile setting is on.
		bool recalibrateOnMovement = false;

		// Previous-frame world-space pose, captured each call to BlendTransform
		// when recalibrateOnMovement is on. Used to compute per-frame motion
		// magnitude that gates the blend. blendMotionInitialized is reset to
		// false whenever recalibrateOnMovement transitions off so the first
		// frame after re-enable doesn't see a giant stale delta.
		Eigen::Vector3d lastBlendWorldPos = Eigen::Vector3d::Zero();
		Eigen::Quaterniond lastBlendWorldRot = Eigen::Quaterniond::Identity();
		bool blendMotionInitialized = false;

#if WKOPENVR_BUILD_IS_DEV
		SmartSmoothingShadowState smartShadow;
#endif
	};

	struct FallbackTransform
	{
		bool enabled = false;
		IsoTransform transform;
		double scale = 1.0;
		uint8_t predictionSmoothness = 0;
		bool recalibrateOnMovement = false;
	};

	// Tracking-system fallback slot. Stored as a fixed-size flat array indexed
	// by linear scan + memcmp on the system_name buffer (typical case is 2-3
	// systems and we never see more than a handful). Cache-line-friendly and
	// avoids the std::string + std::unordered_map heap traffic that otherwise
	// happens on every pose update for every disabled slot.
	struct FallbackSlot
	{
		// NUL-padded; full buffer is compared so a partial match against a
		// shorter name doesn't false-positive.
		char system_name[protocol::MaxTrackingSystemNameLen];
		FallbackTransform tf;
		bool occupied = false;
	};

	// Lookup state for the lazy VRProperties() tracking-system query in
	// HandleDevicePoseUpdated. The query has to fire from the pose-hook thread
	// because the overlay may not have called SetDeviceTransform for this slot
	// yet; without throttling it gets re-issued on every single pose update for
	// every empty slot up to k_unMaxTrackedDeviceCount.
	enum class LookupState : uint8_t
	{
		NotTried,
		Cached,
		Failed,
	};

	DeviceTransform transforms[vr::k_unMaxTrackedDeviceCount];
	Eigen::Vector3d debugTransform;
	Eigen::Quaterniond debugRotation;

	// Guards transforms[], deviceSystem[], systemFallbacks[] and the lookup
	// state against concurrent access. The IPC server thread mutates these via
	// SetDeviceTransform / SetTrackingSystemFallback while the pose-hook thread
	// reads them in HandleDevicePoseUpdated. std::string assignments + flat-
	// array writes during a concurrent read are UB without synchronisation.
	// Held briefly: hook thread copies the slot's transform + fallback target
	// into stack locals under the lock, then releases it for the math/blend.
	mutable std::mutex stateMutex;

	// Per-ID tracking-system name, populated from SetDeviceTransform messages.
	// Empty if the overlay hasn't told us yet for this slot.
	std::array<std::string, vr::k_unMaxTrackedDeviceCount> deviceSystem;

	// Per-ID state of the lazy VRProperties() tracking-system query. NotTried
	// means we haven't asked yet; Cached means deviceSystem[id] is populated;
	// Failed means the last query came up empty and we should back off.
	LookupState lookupState[vr::k_unMaxTrackedDeviceCount] = {};

	// Timestamp of the last failed VRProperties() query for each slot. Failed
	// queries are retried at most once per second so an unoccupied slot doesn't
	// hammer the property store on every pose update.
	LARGE_INTEGER lastLookupAttempt[vr::k_unMaxTrackedDeviceCount] = {};

	// Fixed-capacity flat list of tracking-system fallbacks. 8 is well above
	// any plausible deployment (HMD vendor + 1-2 controller systems + scratch).
	// Linear scanned in HandleDevicePoseUpdated; replaces the prior
	// std::unordered_map<std::string, FallbackTransform> which heap-allocated
	// on every key insert and required a hash + equality check on every pose
	// update for every disabled slot.
	static const size_t MaxFallbackSlots = 8;
	FallbackSlot systemFallbacks[MaxFallbackSlots] = {};

	// Cached QueryPerformanceFrequency value. QPF is constant for the lifetime
	// of the boot, so we capture it once in Init() rather than re-querying on
	// every BlendTransform call (~50-200 ns saved per pose update).
	LARGE_INTEGER qpcFreq = {};

	DeltaSize currentDeltaSpeed[vr::k_unMaxTrackedDeviceCount];

	protocol::AlignmentSpeedParams alignmentSpeedParams;

	// v25: head-mount tracker driver state. Written by SetHeadMountConfig
	// (IPC thread) and read by the DriverSynth branch in HandleDevicePoseUpdated
	// (pose-hook thread). Guarded by its own mutex so reads and writes don't
	// contend with the main stateMutex. The critical section in the pose hook
	// is a struct copy-out (tiny); the IPC write is rare (UI changes only).
	//
	// HeadMountDriverState mirrors the wire payload but uses fixed-size buffers
	// that are trivially copyable; no std::string so the mutex-guarded copy
	// does not allocate.
	struct HeadMountDriverState
	{
		int mode = 0;          // HeadMountMode value; 3 = DriverSynth
		int32_t deviceId = -1; // -1 = unresolved
		char trackerSerial[protocol::MaxTrackingSystemNameLen] = {};
		char trackerTrackingSystem[protocol::MaxTrackingSystemNameLen] = {};
		double headFromTrackerTrans[3] = {0.0, 0.0, 0.0};
		double headFromTrackerRot[4] = {0.0, 0.0, 0.0, 1.0}; // xyzw
		bool hideTracker = true;
		bool offsetCalibrated = false;
		bool allowRawHmdFallback = true;
		uint8_t lockedHeadsetSmoothing = 0;         // 0..100, 0 = off; synth HMD position smoothing
		uint8_t lockedHeadsetRotationSmoothing = 0; // 0..100, 0 = off; synth HMD rotation smoothing
		wkopenvr::headmount::DriverSynthTimingConfig driverSynthTiming;
	};
	mutable std::mutex m_headMountStateMutex;
	HeadMountDriverState m_headMountState;

	// Per-device tracker pose cache for the DriverSynth path. The pose-hook
	// thread writes the latest tracker pose here when the device's openVRID
	// matches m_headMountState.deviceId. The HMD synthesis branch reads this
	// snapshot (copy-out under lock, then work without lock). Mutex is separate
	// from stateMutex: the tracker snapshot is written on every pose tick for
	// the tracked device, so sharing stateMutex would add contention on every
	// tracker pose update.
	mutable std::mutex m_trackerSnapMutex;
	driver_synth::TrackerSnapshot m_trackerSnap;
	driver_synth::SourceBlendState m_driverSynthBlendState;
	std::atomic<bool> m_driverSynthBlendReset{false};

	// Latched when ApplySmartSmoothing throws in the pose hook. Once set, the
	// pose path skips smoothing (forwards the raw pose) so a faulting math path
	// cannot re-throw on every subsequent frame; the smoothing module is also
	// disabled + fault-marked via DisableActiveModuleByMask so the overlay-side
	// recovery can attribute the fault. Cleared only by a driver reload (fresh
	// object). Mirrors the phantom pose-pipeline guard below.
	std::atomic<bool> m_smoothingPoseFaulted{false};

	// Optional speed-adaptive low-pass for the synthesized (locked) HMD pose.
	// Dedicated filter state so it is independent of any per-device smoothing.
	prediction::smart_shadow::FilterState m_driverSynthHmdFilter;
	LARGE_INTEGER m_driverSynthHmdFilterLastSample{};

	// Finger-smoothing config packed into an atomic uint64_t. Single-writer
	// (IPC thread, on user UI input -- rare) / many-reader (skeletal hook
	// detour, ~340 Hz/hand x 2 hands = 680 Hz). The previous version used
	// a mutex around the 6-byte struct; for a hot read at 680 Hz that's
	// gratuitous contention on a tiny POD that fits in a single cache line.
	// More importantly any future LOG() drift inside the critical section
	// would stall every skeletal update on a disk write.
	//
	// v13 split: FingerSmoothingConfig grew per_finger_smoothness[10] for
	// per-finger strength and no longer fits in a single uint64_t. The pack
	// uses two atomics so the hot path stays lock-free on x64:
	//
	//   fingerCfgPacked            : header (8 bytes) -- master_enabled,
	//                                smoothness (global fallback),
	//                                finger_mask (2 bytes), then per_finger
	//                                bytes [8] and [9] tucked into the
	//                                trailing space so fingers 8 and 9 are
	//                                published atomically with the header.
	//   perFingerSmoothness0to7    : fingers 0..7 packed as 8 raw bytes.
	//
	// The detour does two acquire loads. Skew between the two atomics is
	// bounded to one frame on one finger -- acceptable for a perceptual
	// smoothing knob. SetFingerSmoothingConfig writes both atomics; if both
	// readers race a partial update they may observe one finger value from
	// the old config and another from the new, which is harmless.
	//
	// Default zero = {master_enabled=false, smoothness=0, finger_mask=0,
	// per_finger_smoothness all 0} so the detour fast-paths to passthrough
	// until the overlay has sent a real config.
	mutable std::atomic<uint64_t> fingerCfgPacked{0};
	mutable std::atomic<uint64_t> perFingerSmoothness0to7Packed{0};

	// v32 dashboard hand-tracking state from the Smoothing overlay. Packed into
	// one atomic so the skeletal hook can cheaply tell whether the SteamVR
	// dashboard is currently visible and whether that state is still fresh.
	mutable std::atomic<uint64_t> dashboardHandTrackingPacked{0};
	// Last decoded active state; feeds the staleness hysteresis so a refresh
	// stream hovering near the stale boundary settles instead of flapping.
	mutable std::atomic<bool> dashboardHandTrackingWasActive{false};

	// Input-health config packed into an atomic uint64_t. Same pattern as
	// fingerCfgPacked: single-writer (IPC thread on user UI input), many-
	// reader (Stage 1B+ boolean / scalar input detours, ~hundreds of Hz per
	// component summed across every input on every tracked device). Default
	// zero = {master_enabled=false, diagnostics_only=false, ...} so the
	// detour fast-paths to passthrough until the overlay sends real config.
	// InputHealthConfig is 8 bytes (sized exactly for this pack). The
	// runtime invariant is enforced by a static_assert in the .cpp where the
	// packing happens.
	mutable std::atomic<uint64_t> inputHealthCfgPacked{0};
	mutable std::shared_mutex inputHealthCompMutex;
	// Diagnostic counter: incremented each time a reader had to wait on
	// inputHealthCompMutex (writer held). Not plumbed to the overlay yet;
	// observable under a debugger to quantify lock contention.
	mutable std::atomic<uint64_t> inputHealthCompContentionCount{0};
	std::unordered_map<uint64_t, std::unordered_map<std::string, protocol::InputHealthCompensationEntry>>
	    inputHealthComp;

	// Look up an existing fallback slot by system name (linear scan + memcmp).
	// Returns nullptr if no slot is currently occupied with that name.
	FallbackSlot* FindFallbackSlot(const char* name, size_t len);
	const FallbackSlot* FindFallbackSlot(const char* name, size_t len) const;
	// Find or insert a fallback slot for the given name. Returns nullptr if the
	// flat array is at capacity.
	FallbackSlot* AcquireFallbackSlot(const char* name, size_t len);

	DeltaSize GetTransformDeltaSize(DeltaSize prior_delta, const IsoTransform& deviceWorldPose, const IsoTransform& src,
	                                const IsoTransform& target) const;

	double GetTransformRate(DeltaSize delta) const;

	void BlendTransform(DeviceTransform& device, const IsoTransform& deviceWorldPose) const;
	void ApplyTransform(DeviceTransform& device, vr::DriverPose_t& devicePose) const;

	// Live one-euro smoothing for a tracked device's reported pose. Applies the
	// slider-derived position filter (and, in dev builds with smartEnabled on,
	// the rotation filter) and scales the velocity / acceleration / lookahead
	// fields by a release-modulated factor so extrapolation is suppressed at
	// rest but active in motion. Caller guarantees smoothness > 0; rawPose is the
	// unmodified pose, pose is mutated in place.
	void ApplySmartSmoothing(uint32_t openVRID, DeviceTransform& device, const vr::DriverPose_t& rawPose,
	                         vr::DriverPose_t& pose, uint8_t smoothness) const;

	// Speed-adaptive low-pass for the synthesized (locked) HMD pose. Unlike the
	// per-device smoothing this also filters rotation, since a head-mounted
	// tracker's orientation jitter is the main discomfort when locked; the
	// one-euro release keeps real head motion responsive. smoothness == 0 leaves
	// the pose untouched and reseeds the filter.
	void ApplyLockedHeadsetSmoothing(vr::DriverPose_t& pose, uint8_t positionSmoothness, uint8_t rotationSmoothness);
#if WKOPENVR_BUILD_IS_DEV
	void UpdateSmartSmoothingShadow(uint32_t openVRID, DeviceTransform& device, const vr::DriverPose_t& rawPose,
	                                const vr::DriverPose_t& livePose) const;
#endif
};
