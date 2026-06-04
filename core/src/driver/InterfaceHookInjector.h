#pragma once

#include <openvr_driver.h>

#include <cstdint>

class ServerTrackedDeviceProvider;

static void DetourTrackedDevicePoseUpdated(vr::IVRServerDriverHost* _this, uint32_t unWhichDevice,
                                           const vr::DriverPose_t& newPose, uint32_t unPoseStructSize);

// Install the GetGenericInterface detour and register the inner per-interface
// hooks gated by featureFlags (pairdriver::kFeature*). With featureFlags == 0
// the GetGenericInterface detour itself is also skipped and the call is a
// no-op aside from caching the driver pointer.
// Returns true on success. On false the driver should return
// vr::VRInitError_Driver_Failed from Init; DisableHooks becomes a no-op.
bool InjectHooks(ServerTrackedDeviceProvider* driver, vr::IVRDriverContext* pDriverContext, uint32_t featureFlags);

// DisableHooks removes our MinHook patches AND drains in-flight detour callers
// before returning. After it returns no detour can be executing inside our code,
// so the rest of Cleanup (server.Stop, shmem.Close, VR_CLEANUP_SERVER_DRIVER_CONTEXT)
// can tear down state the detours would otherwise have raced against.
void DisableHooks();

namespace InterfaceHooks {

// In-flight detour bookkeeping. Each detour wraps its body with DetourScope so
// EnterDetour/ExitDetour bracket the entire detour body (including the call
// into originalFunc, since MinHook trampolines also live in our DLL's address
// space and can be unmapped on unload). DrainInFlightDetours spin-waits for
// the counter to reach zero — only DisableHooks() should call it directly.
void EnterDetour() noexcept;
void ExitDetour() noexcept;
void DrainInFlightDetours() noexcept;

struct DetourScope
{
	DetourScope() noexcept { EnterDetour(); }
	~DetourScope() noexcept { ExitDetour(); }
	DetourScope(const DetourScope&) = delete;
	DetourScope& operator=(const DetourScope&) = delete;
};

} // namespace InterfaceHooks