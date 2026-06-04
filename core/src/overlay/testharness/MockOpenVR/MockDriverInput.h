#pragma once

#include "BuildChannel.h"

#if WKOPENVR_BUILD_IS_DEV

#include <openvr_driver.h>

#include <atomic>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace openvr_pair::overlay::testharness {

class MockOpenVRRuntime;

// Implements vr::IVRDriverInput (version 003). The InterfaceHookInjector
// installs MinHook detours on slots 0-3 of our vtable (Create/Update
// Boolean/Scalar) so InputHealth observes every component update. The
// smoothing module hooks slots 5-6 for skeletal updates. The mock therefore
// MUST keep these methods virtual and in the same declaration order as the
// pinned openvr_driver.h.
class MockDriverInput : public vr::IVRDriverInput
{
public:
	explicit MockDriverInput(MockOpenVRRuntime& owner);

	// Scenarios use this to drive the InputHealth detour from outside the
	// driver: it calls UpdateScalarComponent through this very mock, which
	// (thanks to InputHealth's MinHook) routes through the driver-side
	// learning logic before returning here.
	vr::EVRInputError SimulateScalarUpdate(const std::string& path, float new_value, double time_offset_sec);
	vr::EVRInputError SimulateBooleanUpdate(const std::string& path, bool new_value, double time_offset_sec);

	// Resolve the handle a driver picked for a given (containerHandle, name).
	// Useful inside scenarios after the driver calls CreateScalarComponent.
	uint64_t FindHandleByPath(uint64_t container, const std::string& name) const;

	// IVRDriverInput
	vr::EVRInputError CreateBooleanComponent(vr::PropertyContainerHandle_t ulContainer, const char* pchName,
	                                         vr::VRInputComponentHandle_t* pHandle) override;
	vr::EVRInputError UpdateBooleanComponent(vr::VRInputComponentHandle_t ulComponent, bool bNewValue,
	                                         double fTimeOffset) override;
	vr::EVRInputError CreateScalarComponent(vr::PropertyContainerHandle_t ulContainer, const char* pchName,
	                                        vr::VRInputComponentHandle_t* pHandle, vr::EVRScalarType eType,
	                                        vr::EVRScalarUnits eUnits) override;
	vr::EVRInputError UpdateScalarComponent(vr::VRInputComponentHandle_t ulComponent, float fNewValue,
	                                        double fTimeOffset) override;
	vr::EVRInputError CreateHapticComponent(vr::PropertyContainerHandle_t ulContainer, const char* pchName,
	                                        vr::VRInputComponentHandle_t* pHandle) override;
	vr::EVRInputError CreateSkeletonComponent(vr::PropertyContainerHandle_t ulContainer, const char* pchName,
	                                          const char* pchSkeletonPath, const char* pchBasePosePath,
	                                          vr::EVRSkeletalTrackingLevel eSkeletalTrackingLevel,
	                                          const vr::VRBoneTransform_t* pGripLimitTransforms,
	                                          uint32_t unGripLimitTransformCount,
	                                          vr::VRInputComponentHandle_t* pHandle) override;
	vr::EVRInputError UpdateSkeletonComponent(vr::VRInputComponentHandle_t ulComponent,
	                                          vr::EVRSkeletalMotionRange eMotionRange,
	                                          const vr::VRBoneTransform_t* pTransforms,
	                                          uint32_t unTransformCount) override;

private:
	struct ComponentInfo
	{
		uint64_t container;
		std::string name;
		enum class Kind
		{
			Boolean,
			Scalar,
			Haptic,
			Skeleton
		};
		Kind kind;
	};

	MockOpenVRRuntime& owner_;
	mutable std::mutex mu_;
	std::atomic<uint64_t> next_handle_{1};
	std::unordered_map<uint64_t, ComponentInfo> by_handle_;
	std::unordered_map<std::string, uint64_t> by_key_; // "<container>:<name>"

	uint64_t AllocateHandle(uint64_t container, std::string name, ComponentInfo::Kind kind);
};

} // namespace openvr_pair::overlay::testharness

#endif // WKOPENVR_BUILD_IS_DEV
