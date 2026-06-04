#include "MockDriverInput.h"

#if WKOPENVR_BUILD_IS_DEV

#include "../HarnessScenario.h"
#include "../MockPoseSource.h"

#include <cstring>

namespace openvr_pair::overlay::testharness {

namespace {

std::string MakeKey(uint64_t container, const std::string& name)
{
	char prefix[32];
	std::snprintf(prefix, sizeof(prefix), "%llu:", (unsigned long long)container);
	return std::string(prefix) + name;
}

MockCall MakeCall(MockCallKind k)
{
	MockCall c;
	c.kind = k;
	return c;
}

} // namespace

MockDriverInput::MockDriverInput(MockOpenVRRuntime& owner) : owner_(owner) {}

uint64_t MockDriverInput::AllocateHandle(uint64_t container, std::string name, ComponentInfo::Kind kind)
{
	const uint64_t h = next_handle_.fetch_add(1, std::memory_order_relaxed);
	const std::string key = MakeKey(container, name);
	{
		std::lock_guard<std::mutex> lock(mu_);
		by_handle_[h] = ComponentInfo{container, std::move(name), kind};
		by_key_[key] = h;
	}
	return h;
}

uint64_t MockDriverInput::FindHandleByPath(uint64_t container, const std::string& name) const
{
	std::lock_guard<std::mutex> lock(mu_);
	auto it = by_key_.find(MakeKey(container, name));
	return it == by_key_.end() ? 0 : it->second;
}

vr::EVRInputError MockDriverInput::CreateBooleanComponent(vr::PropertyContainerHandle_t ulContainer,
                                                          const char* pchName, vr::VRInputComponentHandle_t* pHandle)
{
	if (!pHandle || !pchName) return vr::VRInputError_InvalidHandle;
	const uint64_t h = AllocateHandle(ulContainer, pchName, ComponentInfo::Kind::Boolean);
	*pHandle = (vr::VRInputComponentHandle_t)h;

	MockCall c = MakeCall(MockCallKind::CreateBooleanComponent);
	c.component_handle = h;
	c.property_container = ulContainer;
	c.text = pchName;
	owner_.recorder().Push(std::move(c));
	return vr::VRInputError_None;
}

vr::EVRInputError MockDriverInput::UpdateBooleanComponent(vr::VRInputComponentHandle_t ulComponent, bool bNewValue,
                                                          double fTimeOffset)
{
	MockCall c = MakeCall(MockCallKind::UpdateBooleanComponent);
	c.component_handle = (uint64_t)ulComponent;
	c.b_value = bNewValue;
	c.time_offset_sec = fTimeOffset;
	owner_.recorder().Push(std::move(c));
	return vr::VRInputError_None;
}

vr::EVRInputError MockDriverInput::CreateScalarComponent(vr::PropertyContainerHandle_t ulContainer, const char* pchName,
                                                         vr::VRInputComponentHandle_t* pHandle,
                                                         vr::EVRScalarType /*eType*/, vr::EVRScalarUnits /*eUnits*/)
{
	if (!pHandle || !pchName) return vr::VRInputError_InvalidHandle;
	const uint64_t h = AllocateHandle(ulContainer, pchName, ComponentInfo::Kind::Scalar);
	*pHandle = (vr::VRInputComponentHandle_t)h;

	MockCall c = MakeCall(MockCallKind::CreateScalarComponent);
	c.component_handle = h;
	c.property_container = ulContainer;
	c.text = pchName;
	owner_.recorder().Push(std::move(c));
	return vr::VRInputError_None;
}

vr::EVRInputError MockDriverInput::UpdateScalarComponent(vr::VRInputComponentHandle_t ulComponent, float fNewValue,
                                                         double fTimeOffset)
{
	MockCall c = MakeCall(MockCallKind::UpdateScalarComponent);
	c.component_handle = (uint64_t)ulComponent;
	c.f_value = (double)fNewValue;
	c.time_offset_sec = fTimeOffset;
	owner_.recorder().Push(std::move(c));
	return vr::VRInputError_None;
}

vr::EVRInputError MockDriverInput::CreateHapticComponent(vr::PropertyContainerHandle_t ulContainer, const char* pchName,
                                                         vr::VRInputComponentHandle_t* pHandle)
{
	if (!pHandle || !pchName) return vr::VRInputError_InvalidHandle;
	const uint64_t h = AllocateHandle(ulContainer, pchName, ComponentInfo::Kind::Haptic);
	*pHandle = (vr::VRInputComponentHandle_t)h;
	return vr::VRInputError_None;
}

vr::EVRInputError MockDriverInput::CreateSkeletonComponent(vr::PropertyContainerHandle_t ulContainer,
                                                           const char* pchName, const char* /*pchSkeletonPath*/,
                                                           const char* /*pchBasePosePath*/,
                                                           vr::EVRSkeletalTrackingLevel /*eSkeletalTrackingLevel*/,
                                                           const vr::VRBoneTransform_t* /*pGripLimitTransforms*/,
                                                           uint32_t /*unGripLimitTransformCount*/,
                                                           vr::VRInputComponentHandle_t* pHandle)
{
	if (!pHandle || !pchName) return vr::VRInputError_InvalidHandle;
	const uint64_t h = AllocateHandle(ulContainer, pchName, ComponentInfo::Kind::Skeleton);
	*pHandle = (vr::VRInputComponentHandle_t)h;

	MockCall c = MakeCall(MockCallKind::CreateSkeletonComponent);
	c.component_handle = h;
	c.property_container = ulContainer;
	c.text = pchName;
	owner_.recorder().Push(std::move(c));
	return vr::VRInputError_None;
}

vr::EVRInputError MockDriverInput::UpdateSkeletonComponent(vr::VRInputComponentHandle_t ulComponent,
                                                           vr::EVRSkeletalMotionRange /*eMotionRange*/,
                                                           const vr::VRBoneTransform_t* /*pTransforms*/,
                                                           uint32_t unTransformCount)
{
	MockCall c = MakeCall(MockCallKind::UpdateSkeletonComponent);
	c.component_handle = (uint64_t)ulComponent;
	c.aux_int = unTransformCount;
	owner_.recorder().Push(std::move(c));
	return vr::VRInputError_None;
}

vr::EVRInputError MockDriverInput::SimulateScalarUpdate(const std::string& path, float new_value,
                                                        double time_offset_sec)
{
	// The simulated update walks the same vtable slot as a real driver call.
	// Iterate registered handles to find a matching path (any container).
	uint64_t handle = 0;
	{
		std::lock_guard<std::mutex> lock(mu_);
		for (const auto& kv : by_handle_) {
			if (kv.second.kind != ComponentInfo::Kind::Scalar) continue;
			if (kv.second.name == path) {
				handle = kv.first;
				break;
			}
		}
	}
	if (handle == 0) return vr::VRInputError_InvalidHandle;
	return this->UpdateScalarComponent((vr::VRInputComponentHandle_t)handle, new_value, time_offset_sec);
}

vr::EVRInputError MockDriverInput::SimulateBooleanUpdate(const std::string& path, bool new_value,
                                                         double time_offset_sec)
{
	uint64_t handle = 0;
	{
		std::lock_guard<std::mutex> lock(mu_);
		for (const auto& kv : by_handle_) {
			if (kv.second.kind != ComponentInfo::Kind::Boolean) continue;
			if (kv.second.name == path) {
				handle = kv.first;
				break;
			}
		}
	}
	if (handle == 0) return vr::VRInputError_InvalidHandle;
	return this->UpdateBooleanComponent((vr::VRInputComponentHandle_t)handle, new_value, time_offset_sec);
}

} // namespace openvr_pair::overlay::testharness

#endif // WKOPENVR_BUILD_IS_DEV
