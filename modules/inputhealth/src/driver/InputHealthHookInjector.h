#pragma once

#include <openvr_driver.h>

#include "Protocol.h"
#include "InputHealthSnapshotStaging.h"

class ServerTrackedDeviceProvider;

// Public entry points for the InputHealth subsystem.
//
// Lighthouse and other vendor drivers publish per-component input updates by
// calling vr::VRDriverInput()->UpdateBooleanComponent (digital buttons) and
// vr::VRDriverInput()->UpdateScalarComponent (analog axes / triggers / finger
// curls). vr::VRDriverInput() resolves to a pointer obtained through
// IVRDriverContext::GetGenericInterface("IVRDriverInput_003" or "_004") -- the
// same vtable the skeletal subsystem already patches at slots 5 and 6.
//
// The hook installer patches the boolean/scalar create/update slots of that
// public vtable. The hot path records O(1) rolling statistics and applies
// already-learned compensation only when master_enabled is true; otherwise it
// forwards raw values directly.

namespace inputhealth {

// Cache the driver pointer for InputHealth config access from inside future
// detour bodies. Called once from InjectHooks() in InterfaceHookInjector.cpp
// during driver Init when kFeatureInputHealth is set.
void Init(ServerTrackedDeviceProvider* driver);

// Tear down the subsystem. Called from DisableHooks(). The IHook registry
// drops any installed hooks on its own; this hook resets cached driver-
// pointer state so a driver-reload cycle starts clean.
void Shutdown();

// Called from DetourGetGenericInterface when an "IVRDriverInput_*" interface
// (substring; "Internal" excluded) is queried. Stage 1A logs the call so the
// Stage 1B implementer can confirm the same vtable is being seen the
// skeletal subsystem already taps. Stage 1B will replace the body with the
// MinHook installs on slots for UpdateBooleanComponent / UpdateScalarComponent
// (slot indices to be confirmed against headers/openvr_driver.h at the time;
// the public IVRDriverInput_003 layout has them at slots 1 and 3, but
// nothing here depends on the exact indices yet).
void TryInstallScalarBooleanHooks(void* iface);

} // namespace inputhealth
