#pragma once

#include "BuildChannel.h"

#if WKOPENVR_BUILD_IS_DEV

#include <openvr_driver.h>

#include <cstdint>
#include <mutex>
#include <string>
#include <unordered_map>
#include <variant>

namespace openvr_pair::overlay::testharness {

class MockOpenVRRuntime;

// Implements vr::IVRProperties. Backs every container handle with an
// in-memory map of typed property values. Scenarios pre-populate properties
// like Prop_SerialNumber_String to make the driver see realistic devices.
class MockProperties : public vr::IVRProperties
{
public:
	explicit MockProperties(MockOpenVRRuntime& owner);

	using PropValue = std::variant<bool, int32_t, uint64_t, float, double, std::string, vr::HmdMatrix34_t>;

	// Pre-seed properties from a scenario (called BEFORE driver Init when
	// possible; safe to call mid-run too).
	void SeedString(uint32_t device_id, vr::ETrackedDeviceProperty prop, std::string value);
	void SeedInt32(uint32_t device_id, vr::ETrackedDeviceProperty prop, int32_t value);
	void SeedBool(uint32_t device_id, vr::ETrackedDeviceProperty prop, bool value);

	// IVRProperties
	vr::ETrackedPropertyError ReadPropertyBatch(vr::PropertyContainerHandle_t ulContainerHandle,
	                                            vr::PropertyRead_t* pBatch, uint32_t unBatchEntryCount) override;
	vr::ETrackedPropertyError WritePropertyBatch(vr::PropertyContainerHandle_t ulContainerHandle,
	                                             vr::PropertyWrite_t* pBatch, uint32_t unBatchEntryCount) override;
	const char* GetPropErrorNameFromEnum(vr::ETrackedPropertyError error) override;
	vr::PropertyContainerHandle_t TrackedDeviceToPropertyContainer(vr::TrackedDeviceIndex_t nDevice) override;

private:
	struct Bucket
	{
		std::unordered_map<int /*ETrackedDeviceProperty*/, PropValue> values;
	};

	MockOpenVRRuntime& owner_;
	mutable std::mutex mu_;
	std::unordered_map<uint64_t /*container*/, Bucket> store_;
	std::unordered_map<uint32_t /*device*/, uint64_t /*container*/> device_to_container_;
	uint64_t next_container_ = 0x4000000000000000ULL;

	uint64_t ContainerForDeviceLocked(uint32_t device_id);
};

} // namespace openvr_pair::overlay::testharness

#endif // WKOPENVR_BUILD_IS_DEV
