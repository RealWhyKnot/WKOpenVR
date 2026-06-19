#include "SkeletalHookInjector.h"
#include "SkeletalDiagnostics.h"
#include "SkeletalSmoothingMath.h"
#include "FeatureFlags.h"
#include "Hooking.h"
#include "DriverMemoryProbe.h"
#include "InterfaceHookInjector.h" // InterfaceHooks::DetourScope -- bracket
                                   // each detour body so DisableHooks can
                                   // drain in-flight callers before the DLL
                                   // is unmapped on driver unload.
#include "Logging.h"
#include "ModulePerf.h"
#include "ServerTrackedDeviceProvider.h"

#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <exception>
#include <mutex>
#include <shared_mutex>
#include <unordered_map>

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

// =============================================================================
// File-scope state.
// =============================================================================

// Cached driver pointer; set from skeletal::Init(), read on every UpdateSkeleton
// detour call. Matches SC's existing InterfaceHookInjector.cpp pattern (file-
// scope raw pointer, set before any hook can fire). Cleared by Shutdown but
// only after IHook::DestroyAll has removed our hooks, so no in-flight detour
// can race the clear.
static ServerTrackedDeviceProvider* g_driver = nullptr;
static std::atomic<uint32_t> g_skeletalOwners{0};

// Latched when the finger-smoothing detour body throws. Once set, the detour
// passes the incoming bone transforms straight through so a faulting math path
// (slerp / quaternion ops on a bad frame) cannot re-throw at ~680 Hz onto
// SteamVR's input thread, which has no handler -- an escaped throw there
// terminated vrserver and triggered safe mode. We also fault-mark the smoothing
// module so the overlay-side recovery can attribute it and the next driver load
// keeps smoothing disabled. We deliberately do NOT tear the module down from
// this hot input-thread detour (that path is exercised from the pose hook,
// which owns the provider); a marker write is thread-safe and sufficient.
static std::atomic<bool> g_skeletalFaulted{false};

static void SkeletalContainmentFault(const char* what)
{
	// First fault only: subsequent calls fast-path out before reaching here.
	if (g_skeletalFaulted.exchange(true, std::memory_order_relaxed)) return;
	LOG("[skeletal] finger-smoothing detour threw: %s -- passing fingers through unsmoothed and disabling smoothing on "
	    "next driver load",
	    (what && what[0]) ? what : "(unknown exception)");
	auto markOwner = [](uint32_t owners, uint32_t mask, openvr_pair::common::modules::ModuleId id) {
		if ((owners & mask) == 0) return;
		if (const auto* spec = openvr_pair::common::module_safety::FindById(id)) {
			openvr_pair::common::module_safety::MarkFault(*spec, "skeletal_exception");
		}
	};
	const uint32_t owners = g_skeletalOwners.load(std::memory_order_relaxed);
	markOwner(owners, pairdriver::kFeatureSmoothing, openvr_pair::common::modules::ModuleId::Smoothing);
	if (owners == 0) {
		markOwner(pairdriver::kFeatureSmoothing, pairdriver::kFeatureSmoothing,
		          openvr_pair::common::modules::ModuleId::Smoothing);
	}
}

// Per-hand smoothing state. Index 0 = left, 1 = right. Frame data is kept in
// a pure helper state so the per-bone smoothing behavior can be tested without
// a live IVRDriverInput hook.
struct HandState
{
	skeletal::math::FingerFrameState frame;

	// Steady-state motion diagnostic. Track worst per-bone output delta across
	// the 30 s stats window so the next jitter report has a signature in the
	// log without needing a fresh repro. Captures continuous artefacts (stream
	// interleave, slerp amplifying upstream noise, stale prior output causing
	// repeated snap-blend) that the enable-transition diagnostic doesn't see,
	// because that one stops 30 frames after the false->true toggle and the
	// user's jitter typically shows up much later in a session.
	//
	// posDelta is the magnitude of position change between the freshly slerped
	// output and the prior frame's output. quatDot is the absolute dot product
	// of the same two orientations (1 = identical, <0.95 = >18 deg single-frame
	// rotation which finger bones can't physically produce). Reset by
	// MaybeLogStats at each stats line emission.
	float windowMaxPosDelta = 0.0f;
	int windowMaxPosDeltaBone = -1;
	float windowMinQuatDot = 1.0f;
	int windowMinQuatDotBone = -1;
};
static HandState g_handState[2];

// VRInputComponentHandle_t -> handedness (0=left, 1=right). Populated by the
// CreateSkeletonComponent hook by inspecting `pchSkeletonPath` (which is
// "/skeleton/hand/left" or "/skeleton/hand/right" for Index Knuckles). Handles
// not in this map are passed through unsmoothed -- covers any non-Index
// skeletal device whose path doesn't match.
static std::unordered_map<vr::VRInputComponentHandle_t, int> g_handleToHandedness;

// Split locks: handedness is read on every UpdateSkeletonComponent (340 Hz/hand
// = 680 Hz total) and written only on CreateSkeletonComponent (twice per
// session) -- shared_mutex lets the two hot detours run their lookups in
// parallel. HandState is mutated by UpdateSkeletonComponent and reset by the
// rare diagnostic/lifecycle paths, so a plain mutex is enough.
//
// Lock ordering when both are needed: handedness before state.
static std::shared_mutex g_handednessMutex;
static std::mutex g_handStateMutex;

// =============================================================================
// Diagnostic counters. UpdateSkeletonComponent fires ~340 Hz/hand, so a per-
// call LOG would balloon the log file. Instead, we tally outcomes in atomics
// and dump a summary every kStatsLogIntervalSec seconds whenever an
// UpdateSkeleton call lands. The first frame on each hand also gets a one-
// time "first call seen" log so the user can verify the hook is alive end-
// to-end after enabling smoothing in the overlay.
//
// This is the diagnostic surface the user asked for: when finger smoothing
// "doesn't appear to work", these counters answer the three failure modes:
//   1. zero left/right total           -> hook never sees frames
//   2. nonzero total but zero smoothed -> config never reaches driver, or
//                                         master_enabled false / smoothness 0
//   3. nonzero unknown_handle          -> handedness map missed, finger paths
//                                         don't match "/left" or "/right"
// =============================================================================
struct PerHandStats
{
	std::atomic<uint64_t> totalCalls{0};
	std::atomic<uint64_t> smoothedCalls{0};
	std::atomic<uint64_t> passthroughCalls{0};
	std::atomic<bool> firstCallLogged{false};
};
static PerHandStats g_stats[2];
static std::atomic<uint64_t> g_unknownHandleCalls{0};
static std::atomic<uint64_t> g_invalidTransformCalls{0};
static std::atomic<int64_t> g_lastStatsLogQpc{0};
static std::atomic<int64_t> g_lastDeepStateLogQpc{0};
// QPC at subsystem arm; denominates the cumulative per_hand rate. The deep-
// state window is the wrong denominator for cumulative totals (it froze at
// one window length and inflated the printed Hz by total/window).
static std::atomic<int64_t> g_subsystemInitQpc{0};
static LARGE_INTEGER g_qpcFreq{};
static constexpr double kStatsLogIntervalSec = 30.0;
static constexpr double kDeepStateLogIntervalSec = 60.0;

// First-N-calls verbose logging. After install succeeds (or any UpdateSkeleton
// arrives), the first kVerboseFirstCalls calls per hand emit a full parameter
// dump including a sample of the bone array. Beyond that we fall back to the
// throttled stats summary. Catches: incorrect bone-count/layout assumptions,
// detour seeing garbage params (= MinHook patched the wrong target), wrong
// handedness lookup propagating into the smoothing buffer.
static constexpr int kVerboseFirstCalls = 3;
static std::atomic<int> g_verboseCallsRemaining[2] = {{kVerboseFirstCalls}, {kVerboseFirstCalls}};

// Track first-time unknown-handle for an extra-detailed log. After the first
// unknown handle hits, subsequent ones just bump the counter (already done).
// The first one gets a snapshot of the entire g_handleToHandedness map so we
// can see what HANDLES we DO have mapped vs what we're being asked about.
static std::atomic<bool> g_firstUnknownHandleLogged{false};

// Also log the first CreateSkeleton call regardless of whether the path
// matches /left or /right, so we can see if there are paths we're not
// recognising. Beyond that, only matched paths get logged (existing behavior).
static std::atomic<bool> g_firstCreateSkeletonLogged{false};

// Enable-transition diagnostics. Reported finger spasms on toggle are hard
// to root-cause from logs alone; this surface fires one detailed log line on
// every false->true transition of `anySmoothing` per hand, then a delta
// summary every 5 frames for the first 30 frames after the transition. After
// that the diagnostic goes silent. Re-arms whenever anySmoothing flips back
// to false (typically when the user disables smoothing).
//
// What the lines record:
//   - On transition: alpha-per-finger array, plus the first wrist + thumb-meta
//     + index-meta bones so we can compare the seeded frame.previous against
//     the live pTransforms (would surface stale previous-frame data).
//   - Each post-enable sample: max per-bone position delta vs prior frame, and
//     the worst (smallest) quaternion dot product across all 31 bones, which
//     flags spinning / sign-flipped slerp.
static std::atomic<bool> g_lastAnySmoothing[2] = {{false}, {false}};
static std::atomic<int> g_postEnableFramesLeft[2] = {{0}, {0}};
static constexpr int kPostEnableFrames = 30;
static constexpr int kPostEnableLogStride = 5;
// Per-hand snapshot of the bone array as we emitted it on the previous
// smoothed frame. Guarded by g_handStateMutex so accesses serialize with the
// hot path. Filled on each smoothed frame post-transition, read on the next.
struct PostEnableSnapshot
{
	bool valid = false;
	vr::VRBoneTransform_t bones[31] = {};
};
static PostEnableSnapshot g_postEnableSnap[2];

static void MaybeLogStats(const char* callerTag)
{
	if (g_qpcFreq.QuadPart == 0) return;
	LARGE_INTEGER now;
	QueryPerformanceCounter(&now);
	int64_t last = g_lastStatsLogQpc.load(std::memory_order_relaxed);
	if (last == 0) {
		// First call: just seed the timestamp so the next log fires after a
		// full window rather than immediately.
		g_lastStatsLogQpc.compare_exchange_strong(last, now.QuadPart);
		return;
	}
	double elapsedSec = (double)(now.QuadPart - last) / (double)g_qpcFreq.QuadPart;
	if (elapsedSec < kStatsLogIntervalSec) return;
	if (!g_lastStatsLogQpc.compare_exchange_strong(last, now.QuadPart)) return;

	uint64_t l_total = g_stats[0].totalCalls.load();
	uint64_t l_smooth = g_stats[0].smoothedCalls.load();
	uint64_t l_pass = g_stats[0].passthroughCalls.load();
	uint64_t r_total = g_stats[1].totalCalls.load();
	uint64_t r_smooth = g_stats[1].smoothedCalls.load();
	uint64_t r_pass = g_stats[1].passthroughCalls.load();
	uint64_t unknown = g_unknownHandleCalls.load();
	uint64_t invalid = g_invalidTransformCalls.load();

	LOG("[skeletal] stats(%s, %.1fs window) L:%llu(s%llu/p%llu) R:%llu(s%llu/p%llu) unknown_handle=%llu "
	    "invalid_transforms=%llu",
	    callerTag, elapsedSec, (unsigned long long)l_total, (unsigned long long)l_smooth, (unsigned long long)l_pass,
	    (unsigned long long)r_total, (unsigned long long)r_smooth, (unsigned long long)r_pass,
	    (unsigned long long)unknown, (unsigned long long)invalid);

	// Steady-state motion diagnostic. Worst per-bone output deltas over the
	// same window. Healthy slerp running on a still hand: pos delta near zero,
	// quat dot near 1. Interleaved streams or stale prior output: pos delta
	// jumps (centimetres per frame) and quat dot drops (single-frame rotations
	// tens of degrees). Zero/identity values mean smoothing didn't run on any
	// bone in the window (smoothness=0 or finger mask all clear). Reset under
	// g_handStateMutex so the next window starts fresh.
	float l_posDelta, r_posDelta, l_quatDot, r_quatDot;
	int l_posBone, r_posBone, l_quatBone, r_quatBone;
	{
		std::lock_guard<std::mutex> lk(g_handStateMutex);
		l_posDelta = g_handState[0].windowMaxPosDelta;
		l_posBone = g_handState[0].windowMaxPosDeltaBone;
		l_quatDot = g_handState[0].windowMinQuatDot;
		l_quatBone = g_handState[0].windowMinQuatDotBone;
		r_posDelta = g_handState[1].windowMaxPosDelta;
		r_posBone = g_handState[1].windowMaxPosDeltaBone;
		r_quatDot = g_handState[1].windowMinQuatDot;
		r_quatBone = g_handState[1].windowMinQuatDotBone;
		for (int h = 0; h < 2; ++h) {
			g_handState[h].windowMaxPosDelta = 0.0f;
			g_handState[h].windowMaxPosDeltaBone = -1;
			g_handState[h].windowMinQuatDot = 1.0f;
			g_handState[h].windowMinQuatDotBone = -1;
		}
	}
	LOG("[skeletal] motion(%s, %.1fs window) L:maxPosDelta=%.4fm(bone=%d) minQuatDot=%.4f(bone=%d)  "
	    "R:maxPosDelta=%.4fm(bone=%d) minQuatDot=%.4f(bone=%d)",
	    callerTag, elapsedSec, l_posDelta, l_posBone, l_quatDot, l_quatBone, r_posDelta, r_posBone, r_quatDot,
	    r_quatBone);
}

// Forward-declare for use in MaybeLogDeepState below -- defined later in this
// file once we have access to the driver class member.
static void DumpHandleMap(const char* callerTag);

// Comprehensive periodic dump every kDeepStateLogIntervalSec. Includes:
// - Hook registry status (Create/Update hooks present)
// - Current finger config snapshot (driver-side cache)
// - All known handle->handedness mappings
// - Per-hand init state (have we seen a first frame?)
// - Stats snapshot (same numbers as the 30s stats line, with derived rates)
// Fires from the UpdateSkeleton hot path so it only runs when traffic is
// flowing -- silent during driver idle.
static void MaybeLogDeepState(const char* callerTag)
{
	if (g_qpcFreq.QuadPart == 0) return;
	LARGE_INTEGER now;
	QueryPerformanceCounter(&now);
	int64_t last = g_lastDeepStateLogQpc.load(std::memory_order_relaxed);
	if (last == 0) {
		g_lastDeepStateLogQpc.compare_exchange_strong(last, now.QuadPart);
		return;
	}
	double elapsedSec = (double)(now.QuadPart - last) / (double)g_qpcFreq.QuadPart;
	if (elapsedSec < kDeepStateLogIntervalSec) return;
	if (!g_lastDeepStateLogQpc.compare_exchange_strong(last, now.QuadPart)) return;

	uint64_t l_total = g_stats[0].totalCalls.load();
	uint64_t l_smooth = g_stats[0].smoothedCalls.load();
	uint64_t r_total = g_stats[1].totalCalls.load();
	uint64_t r_smooth = g_stats[1].smoothedCalls.load();
	bool l_init, r_init;
	{
		std::lock_guard<std::mutex> lk(g_handStateMutex);
		l_init = g_handState[0].frame.initialized;
		r_init = g_handState[1].frame.initialized;
	}
	const int64_t initQpc = g_subsystemInitQpc.load(std::memory_order_relaxed);
	const double sinceInitSec = initQpc != 0 ? (double)(now.QuadPart - initQpc) / (double)g_qpcFreq.QuadPart : 0.0;
	const double l_hz = skeletal::math::ComputeRateHz(l_total, sinceInitSec);
	const double r_hz = skeletal::math::ComputeRateHz(r_total, sinceInitSec);

	LOG("[skeletal] deep_state(%s, %.1fs window):", callerTag, elapsedSec);
	LOG("[skeletal]   verboseRemaining=L%d/R%d", g_verboseCallsRemaining[0].load(), g_verboseCallsRemaining[1].load());

	// Hook registry snapshot -- confirm both hooks are still present (they
	// shouldn't disappear, but a SteamVR-side reload could in theory drop
	// them; this catches that case). Names are duplicated string literals
	// here so MaybeLogDeepState can be defined above the Hook<> static
	// declarations without forward-declaration gymnastics; a single refactor
	// would have to update both sites if the names ever change.
	LOG("[skeletal]   hooks: createInRegistry=%d updateInRegistry=%d",
	    (int)IHook::Exists("IVRDriverInput::CreateSkeletonComponent"),
	    (int)IHook::Exists("IVRDriverInput::UpdateSkeletonComponent"));

	// Current driver-side finger config snapshot. If this disagrees with what
	// the overlay last sent (visible in the SetFingerSmoothingConfig log),
	// there's an IPC sync bug.
	if (g_driver) {
		auto cfg = g_driver->GetFingerSmoothingConfig();
		LOG("[skeletal]   cfg: enabled=%d smoothness=%u mask=0x%04x", (int)cfg.master_enabled, (unsigned)cfg.smoothness,
		    (unsigned)cfg.finger_mask);
	}
	else {
		LOG("[skeletal]   cfg: g_driver is NULL (subsystem un-Init'd?)");
	}

	LOG("[skeletal]   per_hand: L{init=%d total=%llu(%.1fHz since_init) smoothed=%llu} R{init=%d total=%llu(%.1fHz "
	    "since_init) smoothed=%llu}",
	    (int)l_init, (unsigned long long)l_total, l_hz, (unsigned long long)l_smooth, (int)r_init,
	    (unsigned long long)r_total, r_hz, (unsigned long long)r_smooth);

	DumpHandleMap("deep_state");
}

// Dump the entire handle->handedness map. Defined as a separate function so
// it can be called from multiple diagnostic sites (deep_state, first-unknown-
// handle log).
static void DumpHandleMap(const char* callerTag)
{
	std::shared_lock<std::shared_mutex> lk(g_handednessMutex);
	if (g_handleToHandedness.empty()) {
		LOG("[skeletal]   handle_map(%s): EMPTY -- CreateSkeleton never matched /left or /right", callerTag);
		return;
	}
	LOG("[skeletal]   handle_map(%s): %zu entries:", callerTag, g_handleToHandedness.size());
	for (const auto& kv : g_handleToHandedness) {
		LOG("[skeletal]     handle=%llu -> %s", (unsigned long long)kv.first,
		    kv.second == 0 ? "left" : (kv.second == 1 ? "right" : "?"));
	}
}

// =============================================================================
// Hook<> instances. Names must be unique against everything else registered
// in IHook::hooks (see SC's existing InterfaceHookInjector.cpp). Two slots
// only -- UpdateSkeleton (the smoothing target) and CreateSkeleton (used to
// learn which handle is left vs right). The other 5 IVRDriverInput methods
// are intentionally NOT hooked to keep the surface minimal.
// =============================================================================

static Hook<vr::EVRInputError (*)(vr::IVRDriverInput*, vr::VRInputComponentHandle_t, vr::EVRSkeletalMotionRange,
                                  const vr::VRBoneTransform_t*, uint32_t)>
    PublicUpdateSkeletonHook("IVRDriverInput::UpdateSkeletonComponent");
static Hook<vr::EVRInputError (*)(vr::IVRDriverInput*, vr::PropertyContainerHandle_t, const char*, const char*,
                                  const char*, vr::EVRSkeletalTrackingLevel, const vr::VRBoneTransform_t*, uint32_t,
                                  vr::VRInputComponentHandle_t*)>
    PublicCreateSkeletonHook("IVRDriverInput::CreateSkeletonComponent");

// =============================================================================
// Detours.
// =============================================================================

static vr::EVRInputError
DetourPublicCreateSkeletonComponent(vr::IVRDriverInput* _this, vr::PropertyContainerHandle_t ulContainer,
                                    const char* pchName, const char* pchSkeletonPath, const char* pchBasePosePath,
                                    vr::EVRSkeletalTrackingLevel eSkeletalTrackingLevel,
                                    const vr::VRBoneTransform_t* pGripLimitTransforms,
                                    uint32_t unGripLimitTransformCount, vr::VRInputComponentHandle_t* pHandle)
{
	InterfaceHooks::DetourScope _scope;
	auto result = PublicCreateSkeletonHook.originalFunc(_this, ulContainer, pchName, pchSkeletonPath, pchBasePosePath,
	                                                    eSkeletalTrackingLevel, pGripLimitTransforms,
	                                                    unGripLimitTransformCount, pHandle);

	// First-CreateSkeleton-ever log: regardless of left/right match, dump
	// every parameter. Catches the case where the path uses an unexpected
	// form (not "/left" or "/right" -- maybe "/hand/L", "/skeleton/left_hand",
	// or some other variant we haven't seen) and our handedness map ends up
	// empty as a result.
	bool firstCreateExpected = false;
	if (g_firstCreateSkeletonLogged.compare_exchange_strong(firstCreateExpected, true)) {
		LOG("[skeletal] FIRST CreateSkeleton call: result=%d this=%p container=%llu name='%s' path='%s' "
		    "basePosePath='%s' level=%d gripCount=%u outHandle=%llu",
		    (int)result, (void*)_this, (unsigned long long)ulContainer, pchName ? pchName : "(null)",
		    pchSkeletonPath ? pchSkeletonPath : "(null)", pchBasePosePath ? pchBasePosePath : "(null)",
		    (int)eSkeletalTrackingLevel, unGripLimitTransformCount, pHandle ? (unsigned long long)*pHandle : 0ULL);
	}

	// Only learn handedness when the underlying create succeeded and we got
	// a valid handle + a recognizable left/right path. Anything else is a
	// skeletal device we don't smooth.
	if (result == vr::VRInputError_None && pHandle && *pHandle != vr::k_ulInvalidInputComponentHandle &&
	    pchSkeletonPath) {
		int handedness = -1;
		if (std::strstr(pchSkeletonPath, "/left"))
			handedness = 0;
		else if (std::strstr(pchSkeletonPath, "/right"))
			handedness = 1;
		if (handedness >= 0) {
			// Lock order: handedness before state.
			std::unique_lock<std::shared_mutex> hlk(g_handednessMutex);
			std::lock_guard<std::mutex> slk(g_handStateMutex);

			// Evict any prior handles already mapped to this hand. The
			// lighthouse driver can register a second skeletal source
			// mid-session for a hand whose path was already mapped --
			// observed scenarios: controller reconnect, hot-plugged glove
			// or finger-tracking accessory coming online after the
			// original controller, internal driver-side input-source
			// reset. Without eviction both handles stay in the map, both
			// pass the UpdateSkeleton lookup, and the two bone streams
			// interleave through one g_handState[handedness] EMA filter:
			// the slerp blends frames from stream A against the prior
			// output from stream B, the smoothed output oscillates, and
			// the user sees finger jitter that disappears with
			// smoothness=0 (passthrough does not consult prior output,
			// so there's no interleave to amplify).
			size_t evicted = 0;
			for (auto it = g_handleToHandedness.begin(); it != g_handleToHandedness.end();) {
				if (it->second == handedness && it->first != *pHandle) {
					LOG("[skeletal] CreateSkeleton EVICTED stale handle=%llu (%s hand) -- superseded by handle=%llu",
					    (unsigned long long)it->first, handedness == 0 ? "left" : "right",
					    (unsigned long long)*pHandle);
					it = g_handleToHandedness.erase(it);
					++evicted;
				}
				else {
					++it;
				}
			}

			g_handleToHandedness[*pHandle] = handedness;
			// A re-create for the same hand needs to re-seed from incoming.
			// Mark uninitialised so the first UpdateSkeleton after this snaps
			// to the new pose instead of slerp-blending out of stale state.
			g_handState[handedness].frame.initialized = false;
			// Reset the verbose-call counter for this hand so the user sees
			// detailed dumps after a re-create (driver reload, hand reconnect).
			g_verboseCallsRemaining[handedness].store(kVerboseFirstCalls);
			LOG("[skeletal] CreateSkeleton MAPPED handle=%llu -> %s (path=%s name=%s evicted=%zu map_size_now=%zu)",
			    (unsigned long long)*pHandle, handedness == 0 ? "left" : "right", pchSkeletonPath,
			    pchName ? pchName : "(null)", evicted, g_handleToHandedness.size());
		}
		else {
			// Path didn't match left/right -- log so we can see what other
			// skeletal components are being created (e.g., non-Index trackers
			// with skeletal output, custom drivers).
			LOG("[skeletal] CreateSkeleton SKIPPED (no /left or /right in path): handle=%llu path='%s' name='%s'",
			    (unsigned long long)*pHandle, pchSkeletonPath, pchName ? pchName : "(null)");
		}
	}
	else if (result != vr::VRInputError_None) {
		LOG("[skeletal] CreateSkeleton FAILED: result=%d path='%s'", (int)result,
		    pchSkeletonPath ? pchSkeletonPath : "(null)");
	}
	return result;
}

// Body of the finger-smoothing detour. Wrapped by DetourPublicUpdateSkeletonComponent
// in a try/catch so a throw from the bone math cannot escape onto SteamVR's input
// thread and terminate vrserver. The wrapper owns the DetourScope.
static vr::EVRInputError DetourPublicUpdateSkeletonComponentImpl(vr::IVRDriverInput* _this,
                                                                 vr::VRInputComponentHandle_t ulComponent,
                                                                 vr::EVRSkeletalMotionRange eMotionRange,
                                                                 const vr::VRBoneTransform_t* pTransforms,
                                                                 uint32_t unTransformCount)
{
	// Fast passthrough: feature-off, unrecognised inputs, or no driver pointer.
	// This is the dominant code path when finger smoothing isn't enabled.
	if (!g_driver || !pTransforms || unTransformCount != 31) {
		if (!pTransforms || unTransformCount != 31) {
			g_invalidTransformCalls.fetch_add(1, std::memory_order_relaxed);
		}
		return PublicUpdateSkeletonHook.originalFunc(_this, ulComponent, eMotionRange, pTransforms, unTransformCount);
	}
	auto cfg = g_driver->GetFingerSmoothingConfig();

	// Look up which hand this is. Unknown handles (non-Index skeletal devices)
	// are counted but NOT smoothed -- they fast-path through. We do this lookup
	// before the anySmoothing gate so the per-hand stats counters give a
	// meaningful "frames seen per hand" reading even when smoothness=0,
	// which is the diagnostic the user needs to know whether the hook is
	// actually receiving the lighthouse skeleton stream.
	int handedness = -1;
	{
		// Hot-path lookup. shared_lock lets the two per-hand detours run
		// in parallel; CreateSkeleton (the only writer) takes a unique_lock.
		std::shared_lock<std::shared_mutex> lk(g_handednessMutex);
		auto it = g_handleToHandedness.find(ulComponent);
		if (it != g_handleToHandedness.end()) handedness = it->second;
	}
	if (handedness < 0) {
		g_unknownHandleCalls.fetch_add(1, std::memory_order_relaxed);

		// First-time unknown-handle deep log. Snapshots the entire handle
		// map so we can compare what handle the lighthouse driver is asking
		// about vs what handles we DID see come through CreateSkeleton.
		// Mismatch tells us our hook missed the CreateSkeleton call (install
		// happened too late) or the path didn't match left/right.
		bool expectedFirstUnknown = false;
		if (g_firstUnknownHandleLogged.compare_exchange_strong(expectedFirstUnknown, true)) {
			LOG("[skeletal] FIRST unknown-handle UpdateSkeleton: requested handle=%llu count=%u motionRange=%d (this "
			    "hand was never seen by CreateSkeleton OR path didn't match /left|/right)",
			    (unsigned long long)ulComponent, unTransformCount, (int)eMotionRange);
			DumpHandleMap("first_unknown");
		}

		// Without this, an "all unknown handles" failure (CreateSkeleton hook
		// missed the lighthouse skeleton creation, so the handedness map is
		// empty) would never log stats -- there'd be no other path to MaybeLogStats
		// and the user would see silence in the log instead of the diagnostic
		// that points at the real problem (unknown_handle != 0).
		MaybeLogStats("UpdateSkeleton/unknown");
		return PublicUpdateSkeletonHook.originalFunc(_this, ulComponent, eMotionRange, pTransforms, unTransformCount);
	}

	g_stats[handedness].totalCalls.fetch_add(1, std::memory_order_relaxed);

	// First call per hand: emit a one-time "we're alive" log so the user can
	// confirm in the driver log that the hook is actually receiving frames
	// for each hand. Includes the live config snapshot so the log shows what
	// state the feature is in at the moment the first frame arrived.
	bool expected = false;
	if (g_stats[handedness].firstCallLogged.compare_exchange_strong(expected, true)) {
		LOG("[skeletal] first UpdateSkeleton on %s hand: handle=%llu count=%u motionRange=%d cfg{enabled=%d "
		    "smoothness=%u mask=0x%04x}",
		    handedness == 0 ? "left" : "right", (unsigned long long)ulComponent, unTransformCount, (int)eMotionRange,
		    (int)cfg.master_enabled, (unsigned)cfg.smoothness, (unsigned)cfg.finger_mask);
	}

	// Verbose first-N-calls dump (per hand). After CreateSkeleton mapped this
	// handle (or after install) the counter is set to kVerboseFirstCalls; each
	// call decrements + dumps. After the count exhausts we go silent (the
	// throttled stats line is sufficient for steady-state). Dumps a small
	// sample of bone positions so we can sanity-check the array isn't garbage
	// (root + wrist should be near-identity; thumb metacarpal should be a
	// small offset). Garbage in pTransforms = our detour is patched at the
	// wrong target.
	int verboseRem = g_verboseCallsRemaining[handedness].fetch_sub(1, std::memory_order_relaxed);
	if (verboseRem > 0 && pTransforms) {
		const auto& bone0 = pTransforms[0];
		const auto& bone1 = pTransforms[1];
		const auto& bone2 = pTransforms[2];
		LOG("[skeletal] verbose %s call %d/%d: handle=%llu count=%u motion=%d this=%p "
		    "bones[0..2]={pos=(%.4f,%.4f,%.4f) (%.4f,%.4f,%.4f) (%.4f,%.4f,%.4f) rot[0]=(%.3f,%.3f,%.3f,%.3f)}",
		    handedness == 0 ? "L" : "R", kVerboseFirstCalls - verboseRem + 1, kVerboseFirstCalls,
		    (unsigned long long)ulComponent, unTransformCount, (int)eMotionRange, (void*)_this, bone0.position.v[0],
		    bone0.position.v[1], bone0.position.v[2], bone1.position.v[0], bone1.position.v[1], bone1.position.v[2],
		    bone2.position.v[0], bone2.position.v[1], bone2.position.v[2], bone0.orientation.w, bone0.orientation.x,
		    bone0.orientation.y, bone0.orientation.z);
	}

	// Periodic deep-state dump. Cheap when not firing (atomic load + arithmetic),
	// ~150 chars when it does. Runs from the hot path so it only fires when
	// skeleton traffic is actually flowing -- silent when driver is idle.
	MaybeLogDeepState("UpdateSkeleton");

	// v13: resolve effective smoothness per finger. Per-finger value of 0 means
	// "fall back to the global cfg.smoothness" -- so an all-zero array reproduces
	// v12 behaviour exactly. Precompute the alpha for each of the 5 fingers on
	// this hand once per call so the per-bone inner loop is just a lookup.
	const int handBase = handedness * 5;
	float alphaPerFinger[5];
	bool anySmoothing = false;
	for (int f = 0; f < 5; ++f) {
		uint8_t s = cfg.per_finger_smoothness[handBase + f];
		if (s == 0) s = cfg.smoothness;
		alphaPerFinger[f] = skeletal::math::SmoothnessToAlpha(s);
		const bool fingerEnabled = ((cfg.finger_mask >> (handBase + f)) & 1u) != 0;
		if (s != 0 && fingerEnabled) anySmoothing = true;
	}

	// Detect anySmoothing transitions for enable-diagnostic logging. Cheap atomic
	// exchange; only fires the log path when the value flips.
	const bool prevAnySmoothing = g_lastAnySmoothing[handedness].exchange(anySmoothing, std::memory_order_relaxed);

	if (!anySmoothing) {
		// Re-arm post-enable diagnostic when smoothing turns off, so the next
		// false->true transition will log again.
		if (prevAnySmoothing) {
			g_postEnableFramesLeft[handedness].store(0, std::memory_order_relaxed);
			std::lock_guard<std::mutex> lk(g_handStateMutex);
			g_postEnableSnap[handedness].valid = false;
		}
		g_stats[handedness].passthroughCalls.fetch_add(1, std::memory_order_relaxed);
		MaybeLogStats("UpdateSkeleton");
		return PublicUpdateSkeletonHook.originalFunc(_this, ulComponent, eMotionRange, pTransforms, unTransformCount);
	}

	vr::VRBoneTransform_t smoothed[31];

	{
		std::lock_guard<std::mutex> lk(g_handStateMutex);
		HandState& state = g_handState[handedness];

		if (anySmoothing && !prevAnySmoothing) {
			// False->true transition: log alpha + seeded frame.previous vs the
			// live input so we can see if previous-frame data is stale at the moment
			// smoothing kicks in. Bones 1, 2, 7 = wrist, thumb-meta, index-meta
			// -- three reference points without dumping all 31 bones.
			const auto& p1 = pTransforms[1];
			const auto& p2 = pTransforms[2];
			const auto& p7 = pTransforms[7];
			const auto& s1 = state.frame.previous[1];
			const auto& s2 = state.frame.previous[2];
			const auto& s7 = state.frame.previous[7];
			LOG("[smoothing-diag] enable transition on %s hand: alpha=[%.3f %.3f %.3f %.3f %.3f] "
			    "init=%d prev_bone1=(%.3f,%.3f,%.3f) in_bone1=(%.3f,%.3f,%.3f) "
			    "prev_bone2=(%.3f,%.3f,%.3f) in_bone2=(%.3f,%.3f,%.3f) "
			    "prev_bone7=(%.3f,%.3f,%.3f) in_bone7=(%.3f,%.3f,%.3f)",
			    handedness == 0 ? "left" : "right", alphaPerFinger[0], alphaPerFinger[1], alphaPerFinger[2],
			    alphaPerFinger[3], alphaPerFinger[4], (int)state.frame.initialized, s1.position.v[0], s1.position.v[1],
			    s1.position.v[2], p1.position.v[0], p1.position.v[1], p1.position.v[2], s2.position.v[0],
			    s2.position.v[1], s2.position.v[2], p2.position.v[0], p2.position.v[1], p2.position.v[2],
			    s7.position.v[0], s7.position.v[1], s7.position.v[2], p7.position.v[0], p7.position.v[1],
			    p7.position.v[2]);
			g_postEnableFramesLeft[handedness].store(kPostEnableFrames, std::memory_order_relaxed);
			g_postEnableSnap[handedness].valid = false;
		}

		const auto frameResult = skeletal::math::SmoothFingerFrame(state.frame, pTransforms, unTransformCount, handBase,
		                                                           cfg.finger_mask, alphaPerFinger, smoothed);
		if (frameResult.maxPosDelta > state.windowMaxPosDelta) {
			state.windowMaxPosDelta = frameResult.maxPosDelta;
			state.windowMaxPosDeltaBone = frameResult.maxPosDeltaBone;
		}
		if (frameResult.minQuatDot < state.windowMinQuatDot) {
			state.windowMinQuatDot = frameResult.minQuatDot;
			state.windowMinQuatDotBone = frameResult.minQuatDotBone;
		}

		// Post-enable sample log: every kPostEnableLogStride frames after a
		// false->true transition, log the worst per-bone position delta and
		// worst orientation dot-product against the snapshot of the last
		// smoothed frame. Detects oscillation / sign-flipped slerp / chain
		// incoherence visually as spasm. Silent once the counter hits zero.
		int framesLeft = g_postEnableFramesLeft[handedness].load(std::memory_order_relaxed);
		if (framesLeft > 0) {
			const int frameIdx = kPostEnableFrames - framesLeft + 1; // 1..30
			const bool shouldLog = (frameIdx % kPostEnableLogStride) == 0;
			PostEnableSnapshot& snap = g_postEnableSnap[handedness];
			if (shouldLog && snap.valid) {
				const auto stats = skeletal::diagnostics::MeasureBoneDeltaStats(smoothed, snap.bones, 31);
				LOG("[smoothing-diag] post-enable %s hand frame %d/%d: "
				    "maxPosDelta=%.4fm (bone=%d) minQuatDot=%.4f",
				    handedness == 0 ? "left" : "right", frameIdx, kPostEnableFrames, stats.maxPosDelta,
				    stats.maxPosBone, stats.minQuatDot);
			}
			std::memcpy(snap.bones, smoothed, sizeof(snap.bones));
			snap.valid = true;
			g_postEnableFramesLeft[handedness].store(framesLeft - 1, std::memory_order_relaxed);
		}
	}

	g_stats[handedness].smoothedCalls.fetch_add(1, std::memory_order_relaxed);
	MaybeLogStats("UpdateSkeleton");
	return PublicUpdateSkeletonHook.originalFunc(_this, ulComponent, eMotionRange, smoothed, unTransformCount);
}

// Registered detour. Thin crash-containment wrapper around the body above: keeps
// the DetourScope (so DisableHooks can drain in-flight callers), fast-paths to
// passthrough once smoothing has faulted, and converts any escaped exception
// into an unsmoothed passthrough so vrserver stays alive.
static vr::EVRInputError DetourPublicUpdateSkeletonComponent(vr::IVRDriverInput* _this,
                                                             vr::VRInputComponentHandle_t ulComponent,
                                                             vr::EVRSkeletalMotionRange eMotionRange,
                                                             const vr::VRBoneTransform_t* pTransforms,
                                                             uint32_t unTransformCount)
{
	InterfaceHooks::DetourScope _scope;
	if (g_skeletalFaulted.load(std::memory_order_relaxed)) {
		return PublicUpdateSkeletonHook.originalFunc(_this, ulComponent, eMotionRange, pTransforms, unTransformCount);
	}
	openvr_pair::common::moduleperf::ScopedSection perfSection(openvr_pair::common::modules::ModuleId::Smoothing);
	try {
		return DetourPublicUpdateSkeletonComponentImpl(_this, ulComponent, eMotionRange, pTransforms, unTransformCount);
	}
	catch (const std::exception& ex) {
		SkeletalContainmentFault(ex.what());
	}
	catch (...) {
		SkeletalContainmentFault(nullptr);
	}
	return PublicUpdateSkeletonHook.originalFunc(_this, ulComponent, eMotionRange, pTransforms, unTransformCount);
}

// =============================================================================
// Public API.
// =============================================================================

namespace skeletal {

void Init(ServerTrackedDeviceProvider* driver, uint32_t ownerFeatureMask)
{
	const uint32_t previousOwners = g_skeletalOwners.fetch_or(ownerFeatureMask, std::memory_order_acq_rel);
	g_driver = driver;
	if (previousOwners != 0) {
		LOG("[skeletal] Init: owner attached mask=0x%08x owners=0x%08x", (unsigned)ownerFeatureMask,
		    (unsigned)(previousOwners | ownerFeatureMask));
		return;
	}
	QueryPerformanceFrequency(&g_qpcFreq);
	{
		LARGE_INTEGER initNow;
		QueryPerformanceCounter(&initNow);
		g_subsystemInitQpc.store(initNow.QuadPart);
	}
	g_lastStatsLogQpc.store(0);
	g_lastDeepStateLogQpc.store(0);
	g_unknownHandleCalls.store(0);
	g_invalidTransformCalls.store(0);
	g_firstUnknownHandleLogged.store(false);
	g_firstCreateSkeletonLogged.store(false);
	for (int h = 0; h < 2; ++h) {
		g_stats[h].totalCalls.store(0);
		g_stats[h].smoothedCalls.store(0);
		g_stats[h].passthroughCalls.store(0);
		g_stats[h].firstCallLogged.store(false);
		g_verboseCallsRemaining[h].store(kVerboseFirstCalls);
	}
	{
		// Lock order: handedness before state.
		std::unique_lock<std::shared_mutex> hlk(g_handednessMutex);
		std::lock_guard<std::mutex> slk(g_handStateMutex);
		g_handleToHandedness.clear();
		for (int h = 0; h < 2; ++h) {
			g_handState[h].frame = {};
			g_handState[h].windowMaxPosDelta = 0.0f;
			g_handState[h].windowMaxPosDeltaBone = -1;
			g_handState[h].windowMinQuatDot = 1.0f;
			g_handState[h].windowMinQuatDotBone = -1;
			g_postEnableSnap[h].valid = false;
			g_postEnableFramesLeft[h].store(0, std::memory_order_relaxed);
			g_lastAnySmoothing[h].store(false, std::memory_order_relaxed);
		}
	}
	LOG("[skeletal] Init: subsystem armed (driver=%p owner=0x%08x), awaiting IVRDriverInput interface queries",
	    (void*)driver, (unsigned)ownerFeatureMask);
}

void Shutdown(uint32_t ownerFeatureMask)
{
	const uint32_t previousOwners = g_skeletalOwners.fetch_and(~ownerFeatureMask, std::memory_order_acq_rel);
	const uint32_t remainingOwners = previousOwners & ~ownerFeatureMask;
	if (remainingOwners != 0) {
		LOG("[skeletal] Shutdown: owner detached mask=0x%08x remaining=0x%08x", (unsigned)ownerFeatureMask,
		    (unsigned)remainingOwners);
		return;
	}
	if (previousOwners == 0) {
		LOG("[skeletal] Shutdown: no active owners for mask=0x%08x", (unsigned)ownerFeatureMask);
		return;
	}
	// Called after IHook::DestroyAll + InterfaceHooks::DrainInFlightDetours
	// from the existing DisableHooks(). Our detours are guaranteed to have
	// exited before we get here (drain is the previous step), so no in-flight
	// caller can race anything we do here.
	MaybeLogStats("Shutdown");
	// Force a final deep-state dump regardless of throttle so even a short-
	// lived session leaves us a full snapshot in the log. Bypass the throttle
	// by zeroing the last-log timestamp.
	g_lastDeepStateLogQpc.store(0);
	// Then bump it forward so the elapsed check passes -- we want the dump,
	// not just the seed.
	LARGE_INTEGER fakeOld;
	QueryPerformanceCounter(&fakeOld);
	fakeOld.QuadPart -= (LONGLONG)(kDeepStateLogIntervalSec * (double)g_qpcFreq.QuadPart) + 1;
	g_lastDeepStateLogQpc.store(fakeOld.QuadPart);
	MaybeLogDeepState("Shutdown");
	LOG("[skeletal] Shutdown: subsystem disarmed");

	// Intentionally do NOT clear g_driver. ServerTrackedDeviceProvider
	// outlives this DLL: SteamVR holds the provider object alive across the
	// entire driver session, and Init() will overwrite g_driver on the next
	// load. Nulling it here used to crash vrserver on every reload that
	// coincided with a 340Hz UpdateSkeleton -- the detour reads g_driver
	// after the !g_driver guard and a window between guard and use let the
	// pointer go to NULL mid-call. Item 2's in-flight drain closes that
	// window, but defending the pointer itself keeps the invariant local
	// to this file in case a future detour is added without the scope
	// guard.
	{
		// Lock order: handedness before state.
		std::unique_lock<std::shared_mutex> hlk(g_handednessMutex);
		std::lock_guard<std::mutex> slk(g_handStateMutex);
		g_handleToHandedness.clear();
		for (int h = 0; h < 2; ++h) {
			g_handState[h].frame = {};
			g_postEnableSnap[h].valid = false;
			g_postEnableFramesLeft[h].store(0, std::memory_order_relaxed);
			g_lastAnySmoothing[h].store(false, std::memory_order_relaxed);
		}
	}
}

void MarkFingersNeedReseed(uint16_t fingerBits)
{
	if (fingerBits == 0) return;
	std::lock_guard<std::mutex> lk(g_handStateMutex);
	for (int h = 0; h < 2; ++h) {
		for (int f = 0; f < 5; ++f) {
			if ((fingerBits >> (h * 5 + f)) & 1u) {
				g_handState[h].frame.reseed_pending[f] = true;
			}
		}
	}
}

void TryInstallPublicHooks(void* iface)
{
	if (!iface) return;

	// Idempotent: once both hooks are registered, additional invocations
	// (one per driver context query of IVRDriverInput_*) no-op cheaply.
	// The MinHook target functions are static across all queries returning
	// the same singleton vtable in vrserver, so re-patching would be harmless
	// but we'd rather skip the work + the noisy log lines.
	bool createAlready = IHook::Exists(PublicCreateSkeletonHook.name);
	bool updateAlready = IHook::Exists(PublicUpdateSkeletonHook.name);
	if (createAlready && updateAlready) return;

	LOG("[skeletal] TryInstallPublicHooks invoked: iface=%p createInRegistry=%d updateInRegistry=%d", iface,
	    (int)createAlready, (int)updateAlready);

	// Defensive readability + sanity check. A real C++ object has its vtable
	// pointer at offset 0; the vtable itself is a contiguous array of
	// function pointers in the DLL's .rdata. If the pointer SteamVR returned
	// is something else (e.g. a JsonCpp settings struct, like the
	// IVRDriverInputInternal_XXX chase encountered -- see memory file), the
	// dereferenced "vtable" will not be readable for 7 slots, OR slot 0 and
	// slot 6 will be wildly different addresses (not even in the same
	// module). Both checks must pass before we patch anything.
	if (!openvr_pair::common::IsReadableMemoryRange(iface, sizeof(void*))) {
		LOG("[skeletal] iface %p not readable; aborting install", iface);
		return;
	}
	void** vtable = *((void***)iface);
	if (!openvr_pair::common::IsReadableMemoryRange(vtable, sizeof(void*) * 7)) {
		LOG("[skeletal] vtable %p not readable for 7 slots; aborting install (iface=%p -- likely garbage, e.g. "
		    "settings memory)",
		    (void*)vtable, iface);
		return;
	}
	// BattleAxeVR/PSVR2 shim defensive pattern: real vrserver vtables are a
	// contiguous block of function pointers, all into the same .text section
	// of vrserver.exe. Slot 0 (CreateBoolean) and slot 6 (UpdateSkeleton) are
	// virtual functions of the same C++ class; their addresses are within a
	// few hundred bytes of each other in practice. A spread of more than
	// 64 KB means either (a) the iface pointer is bogus and "vtable" points
	// at random data, or (b) the OpenVR ABI has shifted in a way we don't
	// understand and we should bail rather than patch the wrong target.
	intptr_t spread = (intptr_t)vtable[6] - (intptr_t)vtable[0];
	if (spread < 0) spread = -spread;
	if (spread > 0x10000) {
		LOG("[skeletal] vtable spread |slot6 - slot0| = 0x%llx bytes (>64KB); refusing to install (iface=%p slot0=%p "
		    "slot6=%p -- likely garbage)",
		    (unsigned long long)spread, iface, vtable[0], vtable[6]);
		return;
	}

	// Pre-install snapshot for the post-install diff. The slot values
	// themselves typically don't change with MinHook (it patches the
	// function body, not the pointer in the vtable), so we expect post == pre
	// here. The change should be in originalFunc (which is the MinHook
	// trampoline, distinct from both the original target and our detour).
	void* preCreate = vtable[5];
	void* preUpdate = vtable[6];
	LOG("[skeletal] pre-install snapshot: vtable[5] (Create) = %p, vtable[6] (Update) = %p, spread=0x%llx", preCreate,
	    preUpdate, (unsigned long long)spread);

	if (!createAlready) {
		PublicCreateSkeletonHook.CreateHookInObjectVTable(iface, 5, &DetourPublicCreateSkeletonComponent);
		IHook::Register(&PublicCreateSkeletonHook);
		LOG("[skeletal]   Create hook installed at vtable[5]: originalFunc=%p, detour=%p",
		    (void*)PublicCreateSkeletonHook.originalFunc, (void*)&DetourPublicCreateSkeletonComponent);
	}
	if (!updateAlready) {
		PublicUpdateSkeletonHook.CreateHookInObjectVTable(iface, 6, &DetourPublicUpdateSkeletonComponent);
		IHook::Register(&PublicUpdateSkeletonHook);
		LOG("[skeletal]   Update hook installed at vtable[6]: originalFunc=%p, detour=%p",
		    (void*)PublicUpdateSkeletonHook.originalFunc, (void*)&DetourPublicUpdateSkeletonComponent);
	}

	LOG("[skeletal-probe] %s",
	    openvr_pair::common::DescribeVirtualQueryRegion("public_vtable_slot5", vtable[5]).c_str());
	LOG("[skeletal-probe] %s",
	    openvr_pair::common::DescribeVirtualQueryRegion("public_vtable_slot6", vtable[6]).c_str());

	LOG("[skeletal] installed PUBLIC IVRDriverInput hooks: vtable[5]=Create, vtable[6]=Update -- waiting for first "
	    "calls");
}

} // namespace skeletal
