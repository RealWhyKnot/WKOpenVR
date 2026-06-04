#pragma once

#include <cstdint>
#include <openvr_driver.h>

namespace inputhealth {

void RegisterBooleanComponent(vr::VRInputComponentHandle_t handle, vr::PropertyContainerHandle_t container,
                              const char* path);

void RegisterScalarComponent(vr::VRInputComponentHandle_t handle, vr::PropertyContainerHandle_t container,
                             const char* path, vr::EVRScalarType scalar_type, vr::EVRScalarUnits scalar_units);

// FNV-1a 64-bit hash of the container's Prop_SerialNumber_String, or 0
// when the container is invalid or the property system has not yet
// attached a serial to it. Some drivers publish the serial lazily after
// Create{Boolean,Scalar}Component returns, so callers may need to retry.
uint64_t ResolveSerialHash(vr::PropertyContainerHandle_t container);

} // namespace inputhealth
