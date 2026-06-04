#include "MockProperties.h"

#if WKOPENVR_BUILD_IS_DEV

#include "../HarnessScenario.h"
#include "../MockPoseSource.h"

#include <cstring>

namespace openvr_pair::overlay::testharness {

namespace {

bool TypeMatches(vr::PropertyTypeTag_t tag, const MockProperties::PropValue& v)
{
	switch (tag) {
		case vr::k_unFloatPropertyTag:
			return std::holds_alternative<float>(v);
		case vr::k_unInt32PropertyTag:
			return std::holds_alternative<int32_t>(v);
		case vr::k_unUint64PropertyTag:
			return std::holds_alternative<uint64_t>(v);
		case vr::k_unBoolPropertyTag:
			return std::holds_alternative<bool>(v);
		case vr::k_unStringPropertyTag:
			return std::holds_alternative<std::string>(v);
		case vr::k_unDoublePropertyTag:
			return std::holds_alternative<double>(v);
		case vr::k_unHmdMatrix34PropertyTag:
			return std::holds_alternative<vr::HmdMatrix34_t>(v);
		default:
			return false;
	}
}

} // namespace

MockProperties::MockProperties(MockOpenVRRuntime& owner) : owner_(owner) {}

uint64_t MockProperties::ContainerForDeviceLocked(uint32_t device_id)
{
	auto it = device_to_container_.find(device_id);
	if (it != device_to_container_.end()) return it->second;
	const uint64_t h = next_container_++;
	device_to_container_[device_id] = h;
	store_[h] = Bucket{};
	return h;
}

void MockProperties::SeedString(uint32_t device_id, vr::ETrackedDeviceProperty prop, std::string value)
{
	std::lock_guard<std::mutex> lock(mu_);
	const uint64_t c = ContainerForDeviceLocked(device_id);
	store_[c].values[(int)prop] = PropValue{std::move(value)};
}

void MockProperties::SeedInt32(uint32_t device_id, vr::ETrackedDeviceProperty prop, int32_t value)
{
	std::lock_guard<std::mutex> lock(mu_);
	const uint64_t c = ContainerForDeviceLocked(device_id);
	store_[c].values[(int)prop] = PropValue{value};
}

void MockProperties::SeedBool(uint32_t device_id, vr::ETrackedDeviceProperty prop, bool value)
{
	std::lock_guard<std::mutex> lock(mu_);
	const uint64_t c = ContainerForDeviceLocked(device_id);
	store_[c].values[(int)prop] = PropValue{value};
}

vr::PropertyContainerHandle_t MockProperties::TrackedDeviceToPropertyContainer(vr::TrackedDeviceIndex_t nDevice)
{
	std::lock_guard<std::mutex> lock(mu_);
	return (vr::PropertyContainerHandle_t)ContainerForDeviceLocked((uint32_t)nDevice);
}

vr::ETrackedPropertyError MockProperties::ReadPropertyBatch(vr::PropertyContainerHandle_t ulContainerHandle,
                                                            vr::PropertyRead_t* pBatch, uint32_t unBatchEntryCount)
{
	if (!pBatch) return vr::TrackedProp_InvalidDevice;
	std::lock_guard<std::mutex> lock(mu_);
	auto it = store_.find((uint64_t)ulContainerHandle);
	for (uint32_t i = 0; i < unBatchEntryCount; ++i) {
		auto& entry = pBatch[i];
		entry.unTag = 0;
		entry.unRequiredBufferSize = 0;
		entry.eError = vr::TrackedProp_UnknownProperty;
		if (it == store_.end()) {
			continue;
		}
		auto valueIt = it->second.values.find((int)entry.prop);
		if (valueIt == it->second.values.end()) {
			continue;
		}
		const auto& v = valueIt->second;
		if (auto p = std::get_if<bool>(&v)) {
			if (entry.unBufferSize >= sizeof(bool)) {
				*(bool*)entry.pvBuffer = *p;
				entry.unRequiredBufferSize = sizeof(bool);
				entry.unTag = vr::k_unBoolPropertyTag;
				entry.eError = vr::TrackedProp_Success;
			}
			else {
				entry.unRequiredBufferSize = sizeof(bool);
				entry.eError = vr::TrackedProp_BufferTooSmall;
			}
		}
		else if (auto p = std::get_if<int32_t>(&v)) {
			if (entry.unBufferSize >= sizeof(int32_t)) {
				*(int32_t*)entry.pvBuffer = *p;
				entry.unRequiredBufferSize = sizeof(int32_t);
				entry.unTag = vr::k_unInt32PropertyTag;
				entry.eError = vr::TrackedProp_Success;
			}
			else {
				entry.unRequiredBufferSize = sizeof(int32_t);
				entry.eError = vr::TrackedProp_BufferTooSmall;
			}
		}
		else if (auto p = std::get_if<uint64_t>(&v)) {
			if (entry.unBufferSize >= sizeof(uint64_t)) {
				*(uint64_t*)entry.pvBuffer = *p;
				entry.unRequiredBufferSize = sizeof(uint64_t);
				entry.unTag = vr::k_unUint64PropertyTag;
				entry.eError = vr::TrackedProp_Success;
			}
			else {
				entry.unRequiredBufferSize = sizeof(uint64_t);
				entry.eError = vr::TrackedProp_BufferTooSmall;
			}
		}
		else if (auto p = std::get_if<float>(&v)) {
			if (entry.unBufferSize >= sizeof(float)) {
				*(float*)entry.pvBuffer = *p;
				entry.unRequiredBufferSize = sizeof(float);
				entry.unTag = vr::k_unFloatPropertyTag;
				entry.eError = vr::TrackedProp_Success;
			}
			else {
				entry.unRequiredBufferSize = sizeof(float);
				entry.eError = vr::TrackedProp_BufferTooSmall;
			}
		}
		else if (auto p = std::get_if<double>(&v)) {
			if (entry.unBufferSize >= sizeof(double)) {
				*(double*)entry.pvBuffer = *p;
				entry.unRequiredBufferSize = sizeof(double);
				entry.unTag = vr::k_unDoublePropertyTag;
				entry.eError = vr::TrackedProp_Success;
			}
			else {
				entry.unRequiredBufferSize = sizeof(double);
				entry.eError = vr::TrackedProp_BufferTooSmall;
			}
		}
		else if (auto p = std::get_if<std::string>(&v)) {
			const uint32_t need = (uint32_t)p->size() + 1;
			if (entry.unBufferSize >= need) {
				std::memcpy(entry.pvBuffer, p->c_str(), need);
				entry.unRequiredBufferSize = need;
				entry.unTag = vr::k_unStringPropertyTag;
				entry.eError = vr::TrackedProp_Success;
			}
			else {
				entry.unRequiredBufferSize = need;
				entry.eError = vr::TrackedProp_BufferTooSmall;
			}
		}
		else if (auto p = std::get_if<vr::HmdMatrix34_t>(&v)) {
			if (entry.unBufferSize >= sizeof(vr::HmdMatrix34_t)) {
				std::memcpy(entry.pvBuffer, p, sizeof(vr::HmdMatrix34_t));
				entry.unRequiredBufferSize = sizeof(vr::HmdMatrix34_t);
				entry.unTag = vr::k_unHmdMatrix34PropertyTag;
				entry.eError = vr::TrackedProp_Success;
			}
			else {
				entry.unRequiredBufferSize = sizeof(vr::HmdMatrix34_t);
				entry.eError = vr::TrackedProp_BufferTooSmall;
			}
		}
	}
	return vr::TrackedProp_Success;
}

vr::ETrackedPropertyError MockProperties::WritePropertyBatch(vr::PropertyContainerHandle_t ulContainerHandle,
                                                             vr::PropertyWrite_t* pBatch, uint32_t unBatchEntryCount)
{
	if (!pBatch) return vr::TrackedProp_InvalidDevice;
	std::lock_guard<std::mutex> lock(mu_);
	auto& bucket = store_[(uint64_t)ulContainerHandle];

	MockCall c;
	c.kind = MockCallKind::WritePropertyBatch;
	c.property_container = (uint64_t)ulContainerHandle;
	c.aux_int = unBatchEntryCount;
	owner_.recorder().Push(std::move(c));

	for (uint32_t i = 0; i < unBatchEntryCount; ++i) {
		auto& entry = pBatch[i];
		entry.eError = vr::TrackedProp_Success;
		if (entry.writeType == vr::PropertyWrite_Erase) {
			bucket.values.erase((int)entry.prop);
			continue;
		}
		switch (entry.unTag) {
			case vr::k_unBoolPropertyTag:
				bucket.values[(int)entry.prop] = PropValue{*(bool*)entry.pvBuffer};
				break;
			case vr::k_unInt32PropertyTag:
				bucket.values[(int)entry.prop] = PropValue{*(int32_t*)entry.pvBuffer};
				break;
			case vr::k_unUint64PropertyTag:
				bucket.values[(int)entry.prop] = PropValue{*(uint64_t*)entry.pvBuffer};
				break;
			case vr::k_unFloatPropertyTag:
				bucket.values[(int)entry.prop] = PropValue{*(float*)entry.pvBuffer};
				break;
			case vr::k_unDoublePropertyTag:
				bucket.values[(int)entry.prop] = PropValue{*(double*)entry.pvBuffer};
				break;
			case vr::k_unStringPropertyTag:
				bucket.values[(int)entry.prop] = PropValue{std::string((const char*)entry.pvBuffer)};
				break;
			case vr::k_unHmdMatrix34PropertyTag:
				bucket.values[(int)entry.prop] = PropValue{*(vr::HmdMatrix34_t*)entry.pvBuffer};
				break;
			default:
				entry.eError = vr::TrackedProp_WrongDataType;
				break;
		}
	}
	return vr::TrackedProp_Success;
}

const char* MockProperties::GetPropErrorNameFromEnum(vr::ETrackedPropertyError error)
{
	switch (error) {
		case vr::TrackedProp_Success:
			return "Success";
		case vr::TrackedProp_WrongDataType:
			return "WrongDataType";
		case vr::TrackedProp_WrongDeviceClass:
			return "WrongDeviceClass";
		case vr::TrackedProp_BufferTooSmall:
			return "BufferTooSmall";
		case vr::TrackedProp_UnknownProperty:
			return "UnknownProperty";
		case vr::TrackedProp_InvalidDevice:
			return "InvalidDevice";
		case vr::TrackedProp_CouldNotContactServer:
			return "CouldNotContactServer";
		case vr::TrackedProp_ValueNotProvidedByDevice:
			return "ValueNotProvidedByDevice";
		case vr::TrackedProp_StringExceedsMaximumLength:
			return "StringExceedsMaximumLength";
		case vr::TrackedProp_NotYetAvailable:
			return "NotYetAvailable";
		case vr::TrackedProp_PermissionDenied:
			return "PermissionDenied";
		case vr::TrackedProp_InvalidOperation:
			return "InvalidOperation";
		default:
			return "Unknown";
	}
}

} // namespace openvr_pair::overlay::testharness

#endif // WKOPENVR_BUILD_IS_DEV
