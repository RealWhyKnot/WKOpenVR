#include "Logging.h"
#include "Hooking.h"
#include "FeatureFlags.h"
#include "InterfaceHookInjector.h"
#include "ServerTrackedDeviceProvider.h"

#include <atomic>
#include <chrono>
#include <cstring>
#include <exception>
#include <thread>
#include <utility>
#include <vector>

static ServerTrackedDeviceProvider* Driver = nullptr;

// Captured at InjectHooks() time and consulted inside DetourGetGenericInterface
// so per-feature inner hooks (IVRServerDriverHost::TrackedDevicePoseUpdated for
// calibration, IVRDriverInput slots 5/6 for skeletal) are only registered when
// their feature flag is set.
static uint32_t s_featureFlags = 0;

// Set to true only when MH_Initialize succeeded and hooks were installed.
// DisableHooks is a no-op when false so we never call MH_Uninitialize on an
// uninitialized library (UB) if InjectHooks returned false.
static bool s_hooksInitialized = false;

// In-flight detour counter. Every detour body in this file AND in
// SkeletalHookInjector.cpp brackets itself with DetourScope which inc/decs
// this counter. DrainInFlightDetours spins until it hits zero so DisableHooks
// can guarantee no detour is mid-execution when it returns.
static std::atomic<int> g_inFlightDetours{0};

namespace InterfaceHooks {

void EnterDetour() noexcept
{
	g_inFlightDetours.fetch_add(1, std::memory_order_acq_rel);
}

void ExitDetour() noexcept
{
	g_inFlightDetours.fetch_sub(1, std::memory_order_acq_rel);
}

void DrainInFlightDetours() noexcept
{
	// Detours are us-scale; in practice this loop exits in a single yield or
	// less. The 500ms cap is a watchdog: if a detour is genuinely stuck (e.g.
	// blocked on a SteamVR mutex held by a thread we're racing) we'd rather
	// log and proceed than hang shutdown forever and force-kill the driver.
	using clock = std::chrono::steady_clock;
	const auto deadline = clock::now() + std::chrono::milliseconds(500);
	while (g_inFlightDetours.load(std::memory_order_acquire) > 0) {
		if (clock::now() > deadline) {
			LOG("DrainInFlightDetours: timeout with %d in-flight callers; proceeding with shutdown anyway",
			    g_inFlightDetours.load(std::memory_order_acquire));
			return;
		}
		std::this_thread::yield();
	}
}

} // namespace InterfaceHooks

// Crash containment for the pose-update detours. A C++ exception thrown by our
// per-frame pose processing (smoothing / driver-synth / calibration math) used
// to propagate out of the detour on SteamVR's pose-reporting thread, which has
// no handler -- std::terminate took vrserver down with it, and SteamVR then
// safe-mode-blocked every external driver on the next launch. We now catch at
// the detour boundary, log (throttled -- a faulting path tends to fault on
// every frame), and let the caller forward the raw pose unchanged. Per-module
// attribution + disable happens deeper in HandleDevicePoseUpdated (the
// smoothing / phantom guards); this is the last-resort net that keeps vrserver
// alive regardless of which path threw.
static void PoseHookContainmentFault(uint32_t deviceId, const char* what) noexcept
{
	static std::atomic<uint64_t> s_count{0};
	const uint64_t n = s_count.fetch_add(1, std::memory_order_relaxed) + 1;
	if (n == 1 || (n % 1000) == 0) {
		LOG("Pose hook caught an exception (device=%u count=%llu): %s -- forwarding raw pose to keep vrserver alive",
		    deviceId, (unsigned long long)n, (what && what[0]) ? what : "(unknown exception)");
	}
}

static void LogFirstPoseHook(uint32_t deviceId) noexcept
{
	static std::atomic<bool> s_logged{false};
	bool expected = false;
	if (s_logged.compare_exchange_strong(expected, true, std::memory_order_relaxed)) {
		LOG("Pose hook first TrackedDevicePoseUpdated device=%u", deviceId);
	}
}

template <typename OriginalFunc>
static void ForwardPhantomSyntheticPoseUpdates(OriginalFunc originalFunc, vr::IVRServerDriverHost* host,
                                               uint32_t triggeringDevice)
{
	if (!Driver || !originalFunc || !host) return;
	try {
		const auto updates = Driver->CollectPhantomSyntheticPoseUpdates(triggeringDevice);
		for (const auto& update : updates) {
			originalFunc(host, update.first, update.second, (uint32_t)sizeof(vr::DriverPose_t));
		}
	}
	catch (const std::exception& ex) {
		PoseHookContainmentFault(triggeringDevice, ex.what());
	}
	catch (...) {
		PoseHookContainmentFault(triggeringDevice, nullptr);
	}
}

static Hook<void* (*)(vr::IVRDriverContext*, const char*, vr::EVRInitError*)>
    GetGenericInterfaceHook("IVRDriverContext::GetGenericInterface");

static Hook<void (*)(vr::IVRServerDriverHost*, uint32_t, const vr::DriverPose_t&, uint32_t)>
    TrackedDevicePoseUpdatedHook005("IVRServerDriverHost005::TrackedDevicePoseUpdated");

static Hook<void (*)(vr::IVRServerDriverHost*, uint32_t, const vr::DriverPose_t&, uint32_t)>
    TrackedDevicePoseUpdatedHook006("IVRServerDriverHost006::TrackedDevicePoseUpdated");

void ForwardPhantomDisconnectPoses(const std::vector<std::pair<uint32_t, vr::DriverPose_t>>& updates)
{
	if (updates.empty()) return;
	vr::IVRServerDriverHost* host = vr::VRServerDriverHost();
	if (!host) return;
	// Use whichever pose-update trampoline is installed so we bypass the detour
	// (no re-entry into HandleDevicePoseUpdated / stateMutex). If neither hook is
	// installed there is no detour to avoid, so the host vtable is safe.
	auto originalFunc = TrackedDevicePoseUpdatedHook006.originalFunc ? TrackedDevicePoseUpdatedHook006.originalFunc
	                                                                 : TrackedDevicePoseUpdatedHook005.originalFunc;
	try {
		for (const auto& update : updates) {
			if (originalFunc) {
				originalFunc(host, update.first, update.second, (uint32_t)sizeof(vr::DriverPose_t));
			}
			else {
				host->TrackedDevicePoseUpdated(update.first, update.second, (uint32_t)sizeof(vr::DriverPose_t));
			}
		}
	}
	catch (const std::exception& ex) {
		PoseHookContainmentFault(0, ex.what());
	}
	catch (...) {
		PoseHookContainmentFault(0, nullptr);
	}
}

static void DetourTrackedDevicePoseUpdated005(vr::IVRServerDriverHost* _this, uint32_t unWhichDevice,
                                              const vr::DriverPose_t& newPose, uint32_t unPoseStructSize)
{
	InterfaceHooks::DetourScope _scope;
	// Inherited from upstream: only run our HandleDevicePoseUpdated when the
	// pose struct size matches the version we expect. If a future SteamVR
	// runtime extends the struct, fall through to the original without
	// touching the pose ourselves rather than risk reading off the end of
	// a smaller-than-expected struct or writing past the end of a larger
	// one. The cleanup vs upstream: dropped a `pNewPose = &newPose; if
	// (pNewPose ...)` indirection that was a dead null-check -- references
	// can't be null in well-formed code, and address-of-reference does
	// nothing the size check doesn't already.
	if (unPoseStructSize == sizeof(vr::DriverPose_t)) {
		LogFirstPoseHook(unWhichDevice);
		auto pose = newPose;
		bool forward = true;
		try {
			forward = Driver->HandleDevicePoseUpdated(unWhichDevice, pose);
		}
		catch (const std::exception& ex) {
			PoseHookContainmentFault(unWhichDevice, ex.what());
			TrackedDevicePoseUpdatedHook005.originalFunc(_this, unWhichDevice, newPose, unPoseStructSize);
			return;
		}
		catch (...) {
			PoseHookContainmentFault(unWhichDevice, nullptr);
			TrackedDevicePoseUpdatedHook005.originalFunc(_this, unWhichDevice, newPose, unPoseStructSize);
			return;
		}
		if (forward) {
			TrackedDevicePoseUpdatedHook005.originalFunc(_this, unWhichDevice, pose, unPoseStructSize);
		}
		ForwardPhantomSyntheticPoseUpdates(TrackedDevicePoseUpdatedHook005.originalFunc, _this, unWhichDevice);
	}
	else {
		TrackedDevicePoseUpdatedHook005.originalFunc(_this, unWhichDevice, newPose, unPoseStructSize);
	}
}

static void DetourTrackedDevicePoseUpdated006(vr::IVRServerDriverHost* _this, uint32_t unWhichDevice,
                                              const vr::DriverPose_t& newPose, uint32_t unPoseStructSize)
{
	InterfaceHooks::DetourScope _scope;
	// See DetourTrackedDevicePoseUpdated005 above for the rationale --
	// same cleanup applied here.
	if (unPoseStructSize == sizeof(vr::DriverPose_t)) {
		LogFirstPoseHook(unWhichDevice);
		auto pose = newPose;
		bool forward = true;
		try {
			forward = Driver->HandleDevicePoseUpdated(unWhichDevice, pose);
		}
		catch (const std::exception& ex) {
			PoseHookContainmentFault(unWhichDevice, ex.what());
			TrackedDevicePoseUpdatedHook006.originalFunc(_this, unWhichDevice, newPose, unPoseStructSize);
			return;
		}
		catch (...) {
			PoseHookContainmentFault(unWhichDevice, nullptr);
			TrackedDevicePoseUpdatedHook006.originalFunc(_this, unWhichDevice, newPose, unPoseStructSize);
			return;
		}
		if (forward) {
			TrackedDevicePoseUpdatedHook006.originalFunc(_this, unWhichDevice, pose, unPoseStructSize);
		}
		ForwardPhantomSyntheticPoseUpdates(TrackedDevicePoseUpdatedHook006.originalFunc, _this, unWhichDevice);
	}
	else {
		TrackedDevicePoseUpdatedHook006.originalFunc(_this, unWhichDevice, newPose, unPoseStructSize);
	}
}

static void* DetourGetGenericInterface(vr::IVRDriverContext* _this, const char* pchInterfaceVersion,
                                       vr::EVRInitError* peError)
{
	InterfaceHooks::DetourScope _scope;
	TRACE("ServerTrackedDeviceProvider::DetourGetGenericInterface(%s)", pchInterfaceVersion);
	auto originalInterface = GetGenericInterfaceHook.originalFunc(_this, pchInterfaceVersion, peError);

	// Installing the inner hooks and notifying modules can touch module code;
	// a throw here must not propagate out of the detour and kill vrserver. On
	// failure we still return the genuine interface SteamVR asked for so the
	// runtime keeps working -- we just skip our own hook wiring for this query.
	try {
		// strcmp avoids a std::string allocation per interface query. This is
		// called for every interface version SteamVR queries during driver
		// init -- not a tight loop, but cumulative startup latency.
		if ((s_featureFlags & pairdriver::kFeatureCalibration) &&
		    std::strcmp(pchInterfaceVersion, "IVRServerDriverHost_005") == 0) {
			if (!IHook::Exists(TrackedDevicePoseUpdatedHook005.name)) {
				TrackedDevicePoseUpdatedHook005.CreateHookInObjectVTable(originalInterface, 1,
				                                                         &DetourTrackedDevicePoseUpdated005);
				IHook::Register(&TrackedDevicePoseUpdatedHook005);
			}
		}
		else if ((s_featureFlags & pairdriver::kFeatureCalibration) &&
		         std::strcmp(pchInterfaceVersion, "IVRServerDriverHost_006") == 0) {
			if (!IHook::Exists(TrackedDevicePoseUpdatedHook006.name)) {
				TrackedDevicePoseUpdatedHook006.CreateHookInObjectVTable(originalInterface, 1,
				                                                         &DetourTrackedDevicePoseUpdated006);
				IHook::Register(&TrackedDevicePoseUpdatedHook006);
			}
		}
		Driver->OnGetGenericInterface(pchInterfaceVersion, originalInterface);
	}
	catch (const std::exception& ex) {
		LOG("GetGenericInterface detour caught exception for '%s': %s",
		    pchInterfaceVersion ? pchInterfaceVersion : "(null)", ex.what());
	}
	catch (...) {
		LOG("GetGenericInterface detour caught an unknown exception for '%s'",
		    pchInterfaceVersion ? pchInterfaceVersion : "(null)");
	}

	return originalInterface;
}

bool InjectHooks(ServerTrackedDeviceProvider* driver, vr::IVRDriverContext* pDriverContext, uint32_t featureFlags)
{
	Driver = driver;
	s_featureFlags = featureFlags;

	if (featureFlags == 0) {
		LOG("InjectHooks: no features enabled; skipping all hook installation");
		// No hooks needed; driver runs inert. Not a failure.
		return true;
	}

	auto err = MH_Initialize();
	if (err != MH_OK) {
		LOG("MH_Initialize error: %s -- driver running without hooks", MH_StatusToString(err));
		return false;
	}

	s_hooksInitialized = true;
	GetGenericInterfaceHook.CreateHookInObjectVTable(pDriverContext, 0, &DetourGetGenericInterface);
	IHook::Register(&GetGenericInterfaceHook);
	return true;
}

void DisableHooks()
{
	// Guard: only tear down if MH_Initialize actually succeeded. Without this,
	// a failed InjectHooks (MH_Initialize error) would call IHook::DestroyAll
	// and MH_Uninitialize on an uninitialized library -- undefined behaviour.
	if (!s_hooksInitialized) {
		return;
	}
	s_hooksInitialized = false;

	// 1. Remove the MinHook patches. After DestroyAll returns, no NEW callers
	//    can enter our detours via the hooked vtable slot (the slot is
	//    restored to point at the original function).
	// 2. Drain in-flight callers. MinHook does NOT wait for already-executing
	//    detour bodies to return -- without this, the pose-update detour
	//    firing at ~kHz across all tracked devices has a window where a
	//    detour body is mid-execution while we tear down state below it. At
	//    SteamVR's driver-unload time that race is fatal: the DLL gets
	//    unmapped while a thread is still inside our code.
	// 3. Drop the skeletal subsystem's cached driver pointer + per-hand
	//    state. Safe now because (1)+(2) guarantee no skeletal detour body
	//    is in flight.
	// 4. Tear down MinHook itself.
	IHook::DestroyAll();
	InterfaceHooks::DrainInFlightDetours();
	MH_Uninitialize();
	s_featureFlags = 0;
}
