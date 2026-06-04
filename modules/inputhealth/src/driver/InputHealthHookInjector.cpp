#include "InputHealthHookInjector.h"

#include "DriverMemoryProbe.h"
#include "Hooking.h"
#include "InputHealthCompensation.h"
#include "InputHealthComponentRegistry.h"
#include "InputHealthObservation.h"
#include "InputHealthState.h"
#include "InterfaceHookInjector.h" // InterfaceHooks::DetourScope
#include "Logging.h"
#include "ServerTrackedDeviceProvider.h"

#include <atomic>
#include <cstdint>
#include <exception>
#include <mutex>
#include <string>

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

// =============================================================================
// File-scope state.
//
// Stage 1D: the per-handle map graduates from {path, flags} to a full
// ComponentStats with Welford / Page-Hinkley / EWMA rolling-min / polar
// histogram. The Update detour bodies feed these on every sample when
// master_enabled is set in the driver-side config; with master_enabled off
// the detours fast-path through unmodified.
//
// Snapshot publishing (driver -> overlay) is NOT in this stage. The
// background worker thread that consumes these stats and emits detection-
// category decisions also lands later. Stage 1D's purpose is to confirm
// the per-tick math is wired up and stays inside the per-call budget
// during real-world testing; Stage 2 builds the overlay client that reads
// the snapshots.
// =============================================================================

std::atomic<ServerTrackedDeviceProvider*> g_driver{nullptr};
std::unordered_map<vr::VRInputComponentHandle_t, inputhealth::ComponentStats> g_componentStats;
std::mutex g_componentMutex;

// One-shot install / probe log markers.
static std::atomic<bool> g_firstCreateBoolLogged{false};
static std::atomic<bool> g_firstCreateScalarLogged{false};
static std::atomic<uint64_t> g_hotPathObservationErrors{0};

// =============================================================================
// Hook<> instances (slots match IVRDriverInput_003 layout in the bundled SDK).
// =============================================================================

static Hook<vr::EVRInputError (*)(vr::IVRDriverInput*, vr::PropertyContainerHandle_t, const char*,
                                  vr::VRInputComponentHandle_t*)>
    CreateBooleanHook("IVRDriverInput::CreateBooleanComponent");
static Hook<vr::EVRInputError (*)(vr::IVRDriverInput*, vr::VRInputComponentHandle_t, bool, double)>
    UpdateBooleanHook("IVRDriverInput::UpdateBooleanComponent");
static Hook<vr::EVRInputError (*)(vr::IVRDriverInput*, vr::PropertyContainerHandle_t, const char*,
                                  vr::VRInputComponentHandle_t*, vr::EVRScalarType, vr::EVRScalarUnits)>
    CreateScalarHook("IVRDriverInput::CreateScalarComponent");
static Hook<vr::EVRInputError (*)(vr::IVRDriverInput*, vr::VRInputComponentHandle_t, float, double)>
    UpdateScalarHook("IVRDriverInput::UpdateScalarComponent");

// =============================================================================
// Helpers.
// =============================================================================

// Wall-clock microseconds since QPC epoch. Cheap and monotonic.
// QPF is constant for the lifetime of the boot; cache it once with the
// thread-safe magic-static guarantee instead of branching on every detour.
static uint64_t QpcMicros()
{
	static const LONGLONG s_freq = []() {
		LARGE_INTEGER f;
		return QueryPerformanceFrequency(&f) ? f.QuadPart : 0;
	}();
	if (s_freq == 0) return 0;
	LARGE_INTEGER t;
	QueryPerformanceCounter(&t);
	return static_cast<uint64_t>(t.QuadPart * 1000000ULL / s_freq);
}

static void LogHotPathObservationError(const char* kind, const char* what)
{
	const uint64_t n = g_hotPathObservationErrors.fetch_add(1, std::memory_order_relaxed) + 1;
	if (n == 1 || n == 100 || (n % 10000) == 0) {
		LOG("[inputhealth] disabled %s observation for this tick after error '%s' (count=%llu); forwarded raw value",
		    kind, what ? what : "unknown", (unsigned long long)n);
	}
}

// =============================================================================
// Detours.
// =============================================================================

static vr::EVRInputError DetourCreateBooleanComponent(vr::IVRDriverInput* _this,
                                                      vr::PropertyContainerHandle_t ulContainer, const char* pchName,
                                                      vr::VRInputComponentHandle_t* pHandle)
{
	InterfaceHooks::DetourScope _scope;
	auto result = CreateBooleanHook.originalFunc(_this, ulContainer, pchName, pHandle);

	bool firstExpected = false;
	if (g_firstCreateBoolLogged.compare_exchange_strong(firstExpected, true)) {
		LOG("[inputhealth] FIRST CreateBooleanComponent: result=%d this=%p container=%llu name='%s' outHandle=%llu",
		    (int)result, (void*)_this, (unsigned long long)ulContainer, pchName ? pchName : "(null)",
		    pHandle ? (unsigned long long)*pHandle : 0ULL);
	}

	if (result == vr::VRInputError_None && pHandle && *pHandle != vr::k_ulInvalidInputComponentHandle && pchName) {
		try {
			inputhealth::RegisterBooleanComponent(*pHandle, ulContainer, pchName);
		}
		catch (const std::exception& e) {
			LogHotPathObservationError("boolean-create", e.what());
		}
		catch (...) {
			LogHotPathObservationError("boolean-create", "non-std exception");
		}
	}
	return result;
}

static vr::EVRInputError DetourUpdateBooleanComponent(vr::IVRDriverInput* _this,
                                                      vr::VRInputComponentHandle_t ulComponent, bool bNewValue,
                                                      double fTimeOffset)
{
	InterfaceHooks::DetourScope _scope;
	bool swallow = false;

	auto* driver = g_driver.load(std::memory_order_acquire);
	if (driver) {
		try {
			const auto cfg = driver->GetInputHealthConfig();
			if (cfg.master_enabled) {
				const uint64_t now_us = QpcMicros();
				// Blocking lock: the publisher's critical sections are O(1)
				// after the StageSnapshots single-pass restructure, so detours
				// block for microseconds at worst. Earlier code used try_lock
				// here and silently dropped observations on contention.
				std::lock_guard<std::mutex> lk(g_componentMutex);
				{
					auto it = g_componentStats.find(ulComponent);
					if (it != g_componentStats.end()) {
						auto& s = it->second;
						inputhealth::ObserveBooleanSample(s, bNewValue, now_us);
						if (!cfg.diagnostics_only) {
							if (s.device_serial_hash == 0 && (now_us - s.last_serial_resolve_attempt_us) > 1000000ULL) {
								s.last_serial_resolve_attempt_us = now_us;
								s.device_serial_hash = inputhealth::ResolveSerialHash(s.container_handle);
								if (s.device_serial_hash != 0 && !s.serial_resolution_logged) {
									s.serial_resolution_logged = true;
									LOG("[inputhealth] resolved late serial for boolean handle=%llu path='%s' -> "
									    "0x%016llx",
									    (unsigned long long)ulComponent, s.path.c_str(),
									    (unsigned long long)s.device_serial_hash);
								}
							}
							if (s.device_serial_hash != 0) {
								protocol::InputHealthCompensationEntry entry{};
								if (driver->LookupInputHealthCompensation(s.device_serial_hash, s.path, entry) &&
								    inputhealth::ShouldSwallowBooleanUpdate(s, entry, bNewValue, now_us)) {
									swallow = true;
								}
							}
						}
						if (!swallow && bNewValue != s.last_boolean) {
							s.last_boolean = bNewValue;
							s.last_committed_us = now_us;
						}
						s.last_update_us = now_us;
						if (!s.first_update_logged) {
							s.first_update_logged = true;
							LOG("[inputhealth] first UpdateBooleanComponent on handle=%llu path='%s' value=%d",
							    (unsigned long long)ulComponent, s.path.c_str(), (int)bNewValue);
						}
					}
				}
			}
		}
		catch (const std::exception& e) {
			swallow = false;
			LogHotPathObservationError("boolean", e.what());
		}
		catch (...) {
			swallow = false;
			LogHotPathObservationError("boolean", "non-std exception");
		}
	}

	if (swallow) return vr::VRInputError_None;
	return UpdateBooleanHook.originalFunc(_this, ulComponent, bNewValue, fTimeOffset);
}

static vr::EVRInputError DetourCreateScalarComponent(vr::IVRDriverInput* _this,
                                                     vr::PropertyContainerHandle_t ulContainer, const char* pchName,
                                                     vr::VRInputComponentHandle_t* pHandle, vr::EVRScalarType eType,
                                                     vr::EVRScalarUnits eUnits)
{
	InterfaceHooks::DetourScope _scope;
	auto result = CreateScalarHook.originalFunc(_this, ulContainer, pchName, pHandle, eType, eUnits);

	bool firstExpected = false;
	if (g_firstCreateScalarLogged.compare_exchange_strong(firstExpected, true)) {
		LOG("[inputhealth] FIRST CreateScalarComponent: result=%d this=%p container=%llu name='%s' type=%d units=%d "
		    "outHandle=%llu",
		    (int)result, (void*)_this, (unsigned long long)ulContainer, pchName ? pchName : "(null)", (int)eType,
		    (int)eUnits, pHandle ? (unsigned long long)*pHandle : 0ULL);
	}

	if (result == vr::VRInputError_None && pHandle && *pHandle != vr::k_ulInvalidInputComponentHandle && pchName) {
		try {
			inputhealth::RegisterScalarComponent(*pHandle, ulContainer, pchName, eType, eUnits);
		}
		catch (const std::exception& e) {
			LogHotPathObservationError("scalar-create", e.what());
		}
		catch (...) {
			LogHotPathObservationError("scalar-create", "non-std exception");
		}
	}
	return result;
}

static vr::EVRInputError DetourUpdateScalarComponent(vr::IVRDriverInput* _this,
                                                     vr::VRInputComponentHandle_t ulComponent, float fNewValue,
                                                     double fTimeOffset)
{
	InterfaceHooks::DetourScope _scope;

	auto* driver = g_driver.load(std::memory_order_acquire);
	if (driver) {
		try {
			const auto cfg = driver->GetInputHealthConfig();
			if (cfg.master_enabled) {
				const uint64_t now_us = QpcMicros();
				// See DetourUpdateBooleanComponent for the rationale on
				// switching from try_lock to a blocking lock.
				std::lock_guard<std::mutex> lk(g_componentMutex);
				{
					auto it = g_componentStats.find(ulComponent);
					if (it != g_componentStats.end()) {
						auto& s = it->second;
						inputhealth::ComponentStats* partnerStats = nullptr;
						auto pit = g_componentStats.find(s.partner_handle);
						if (pit != g_componentStats.end()) partnerStats = &pit->second;

						inputhealth::ObserveScalarSample(s, fNewValue, now_us, partnerStats);

						if (!s.first_update_logged) {
							s.first_update_logged = true;
							LOG("[inputhealth] first UpdateScalarComponent on handle=%llu path='%s' value=%.4f role=%d",
							    (unsigned long long)ulComponent, s.path.c_str(), fNewValue, (int)s.axis_role);
						}

						if (!cfg.diagnostics_only) {
							if (s.device_serial_hash == 0 && (now_us - s.last_serial_resolve_attempt_us) > 1000000ULL) {
								s.last_serial_resolve_attempt_us = now_us;
								s.device_serial_hash = inputhealth::ResolveSerialHash(s.container_handle);
								if (s.device_serial_hash != 0 && !s.serial_resolution_logged) {
									s.serial_resolution_logged = true;
									LOG("[inputhealth] resolved late serial for scalar handle=%llu path='%s' -> "
									    "0x%016llx",
									    (unsigned long long)ulComponent, s.path.c_str(),
									    (unsigned long long)s.device_serial_hash);
								}
							}
						}
						if (!cfg.diagnostics_only && s.device_serial_hash != 0) {
							protocol::InputHealthCompensationEntry entry{};
							if (driver->LookupInputHealthCompensation(s.device_serial_hash, s.path, entry) &&
							    entry.kind != protocol::InputHealthCompBoolean) {
								// Per-feature enable gates. Stick axes require
								// enable_rest_recenter; trigger/single-scalar
								// axes require enable_trigger_remap. Skip
								// compensation if the relevant toggle is off.
								const bool isStick = (s.axis_role == inputhealth::AxisRole::StickX ||
								                      s.axis_role == inputhealth::AxisRole::StickY);
								const bool isTrigger = !isStick; // all non-boolean scalars that aren't sticks
								if (isStick && !cfg.enable_rest_recenter) {
									// rest-recenter disabled; skip
								}
								else if (isTrigger && !cfg.enable_trigger_remap) {
									// trigger remap disabled; skip
								}
								else {
									float partnerValue = 0.0f;
									bool hasPartner = false;
									std::string partnerPath;
									if (partnerStats) {
										partnerValue = partnerStats->last_value;
										hasPartner = true;
										partnerPath = partnerStats->path;
									}
									fNewValue = inputhealth::ApplyScalarCompensation(
									    driver, entry, s, fNewValue, partnerValue, hasPartner, partnerPath);
								}
							}
						}
					}
				}
			}
		}
		catch (const std::exception& e) {
			LogHotPathObservationError("scalar", e.what());
		}
		catch (...) {
			LogHotPathObservationError("scalar", "non-std exception");
		}
	}

	return UpdateScalarHook.originalFunc(_this, ulComponent, fNewValue, fTimeOffset);
}

// =============================================================================
// Public API.
// =============================================================================

namespace inputhealth {

void Init(ServerTrackedDeviceProvider* driver)
{
	g_driver.store(driver, std::memory_order_release);
	g_firstCreateBoolLogged.store(false, std::memory_order_release);
	g_firstCreateScalarLogged.store(false, std::memory_order_release);
	g_hotPathObservationErrors.store(0, std::memory_order_release);
	{
		std::lock_guard<std::mutex> lk(g_componentMutex);
		g_componentStats.clear();
	}
	LOG("[inputhealth] Init: subsystem armed (driver=%p), awaiting IVRDriverInput interface queries", (void*)driver);
}

void Shutdown()
{
	{
		std::lock_guard<std::mutex> lk(g_componentMutex);
		g_componentStats.clear();
	}
	// Detours null-check g_driver via load() before dereferencing, so
	// clearing it here is safe and prevents stale-pointer use after shutdown.
	g_driver.store(nullptr, std::memory_order_release);
	LOG("[inputhealth] Shutdown: subsystem disarmed");
}

void TryInstallScalarBooleanHooks(void* iface)
{
	if (!iface) return;
	if (g_driver.load(std::memory_order_acquire) == nullptr) return;

	bool createBoolAlready = IHook::Exists(CreateBooleanHook.name);
	bool updateBoolAlready = IHook::Exists(UpdateBooleanHook.name);
	bool createScalarAlready = IHook::Exists(CreateScalarHook.name);
	bool updateScalarAlready = IHook::Exists(UpdateScalarHook.name);
	if (createBoolAlready && updateBoolAlready && createScalarAlready && updateScalarAlready) return;

	LOG("[inputhealth] TryInstallScalarBooleanHooks invoked: iface=%p createBool=%d updateBool=%d createScalar=%d "
	    "updateScalar=%d",
	    iface, (int)createBoolAlready, (int)updateBoolAlready, (int)createScalarAlready, (int)updateScalarAlready);

	if (!openvr_pair::common::IsReadableMemoryRange(iface, sizeof(void*))) {
		LOG("[inputhealth] iface %p not readable; aborting install", iface);
		return;
	}
	void** vtable = *((void***)iface);
	if (!openvr_pair::common::IsReadableMemoryRange(vtable, sizeof(void*) * 7)) {
		LOG("[inputhealth] vtable %p not readable for 7 slots; aborting install (iface=%p)", (void*)vtable, iface);
		return;
	}
	intptr_t spread = (intptr_t)vtable[6] - (intptr_t)vtable[0];
	if (spread < 0) spread = -spread;
	if (spread > 0x10000) {
		LOG("[inputhealth] vtable spread |slot6 - slot0| = 0x%llx bytes (>64KB); refusing to install (iface=%p "
		    "slot0=%p slot6=%p)",
		    (unsigned long long)spread, iface, vtable[0], vtable[6]);
		return;
	}

	LOG("[inputhealth] pre-install snapshot: vtable[0]=%p [1]=%p [2]=%p [3]=%p spread=0x%llx", vtable[0], vtable[1],
	    vtable[2], vtable[3], (unsigned long long)spread);

	bool createBoolReady = createBoolAlready;
	bool updateBoolReady = updateBoolAlready;
	bool createScalarReady = createScalarAlready;
	bool updateScalarReady = updateScalarAlready;

	if (!createBoolAlready) {
		createBoolReady = CreateBooleanHook.CreateHookInObjectVTable(iface, 0, &DetourCreateBooleanComponent);
		if (createBoolReady) IHook::Register(&CreateBooleanHook);
	}
	if (!updateBoolAlready) {
		updateBoolReady = UpdateBooleanHook.CreateHookInObjectVTable(iface, 1, &DetourUpdateBooleanComponent);
		if (updateBoolReady) IHook::Register(&UpdateBooleanHook);
	}
	if (!createScalarAlready) {
		createScalarReady = CreateScalarHook.CreateHookInObjectVTable(iface, 2, &DetourCreateScalarComponent);
		if (createScalarReady) IHook::Register(&CreateScalarHook);
	}
	if (!updateScalarAlready) {
		updateScalarReady = UpdateScalarHook.CreateHookInObjectVTable(iface, 3, &DetourUpdateScalarComponent);
		if (updateScalarReady) IHook::Register(&UpdateScalarHook);
	}

	LOG("[inputhealth-probe] %s",
	    openvr_pair::common::DescribeVirtualQueryRegion("public_vtable_slot0", vtable[0]).c_str());
	LOG("[inputhealth-probe] %s",
	    openvr_pair::common::DescribeVirtualQueryRegion("public_vtable_slot1", vtable[1]).c_str());
	LOG("[inputhealth-probe] %s",
	    openvr_pair::common::DescribeVirtualQueryRegion("public_vtable_slot2", vtable[2]).c_str());
	LOG("[inputhealth-probe] %s",
	    openvr_pair::common::DescribeVirtualQueryRegion("public_vtable_slot3", vtable[3]).c_str());

	if (createBoolReady && updateBoolReady && createScalarReady && updateScalarReady) {
		LOG("[inputhealth] installed PUBLIC IVRDriverInput hooks: vtable[0]=CreateBool vtable[1]=UpdateBool "
		    "vtable[2]=CreateScalar vtable[3]=UpdateScalar -- waiting for first calls");
	}
	else {
		LOG("[inputhealth] partial IVRDriverInput hook install; createBool=%d updateBool=%d createScalar=%d "
		    "updateScalar=%d -- missing hooks stay pass-through",
		    (int)createBoolReady, (int)updateBoolReady, (int)createScalarReady, (int)updateScalarReady);
	}
}

} // namespace inputhealth
