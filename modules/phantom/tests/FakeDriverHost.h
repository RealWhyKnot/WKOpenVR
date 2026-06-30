#pragma once

// Minimal in-process fakes for the OpenVR driver host + properties so
// VirtualTrackerManager / VirtualTrackerDevice can be exercised without a live
// SteamVR runtime. The driver context machinery in openvr_driver.h is header-only
// (vr::InitServerDriverContext sets the module context; vr::VRServerDriverHost() /
// vr::VRProperties() lazily fetch from it), so no extra linking is required.

#include <openvr_driver.h>

#include <cstring>
#include <vector>

namespace phantom_test {

class FakeProperties final : public vr::IVRProperties
{
public:
	vr::ETrackedPropertyError ReadPropertyBatch(vr::PropertyContainerHandle_t, vr::PropertyRead_t*, uint32_t) override
	{
		return vr::TrackedProp_Success;
	}
	vr::ETrackedPropertyError WritePropertyBatch(vr::PropertyContainerHandle_t, vr::PropertyWrite_t*, uint32_t) override
	{
		return vr::TrackedProp_Success;
	}
	const char* GetPropErrorNameFromEnum(vr::ETrackedPropertyError) override { return ""; }
	vr::PropertyContainerHandle_t TrackedDeviceToPropertyContainer(vr::TrackedDeviceIndex_t nDevice) override
	{
		// Any non-invalid handle; Activate only checks it is not k_ulInvalidPropertyContainer.
		return static_cast<vr::PropertyContainerHandle_t>(nDevice) + 1;
	}
};

class FakeServerDriverHost final : public vr::IVRServerDriverHost
{
public:
	// Instrumentation the tests read.
	int add_count = 0;
	int pose_update_count = 0;
	uint32_t last_pose_device = vr::k_unTrackedDeviceIndexInvalid;
	vr::DriverPose_t last_pose{};
	vr::ITrackedDeviceServerDriver* last_driver = nullptr;

	bool TrackedDeviceAdded(const char* /*serial*/, vr::ETrackedDeviceClass /*cls*/,
	                        vr::ITrackedDeviceServerDriver* pDriver) override
	{
		++add_count;
		last_driver = pDriver;
		const vr::TrackedDeviceIndex_t id = next_id_++;
		if (pDriver) pDriver->Activate(id);
		return true;
	}

	void TrackedDevicePoseUpdated(uint32_t unWhichDevice, const vr::DriverPose_t& newPose, uint32_t) override
	{
		++pose_update_count;
		last_pose_device = unWhichDevice;
		last_pose = newPose;
	}

	void VsyncEvent(double) override {}
	void VendorSpecificEvent(uint32_t, vr::EVREventType, const vr::VREvent_Data_t&, double) override {}
	bool IsExiting() override { return false; }
	bool PollNextEvent(vr::VREvent_t*, uint32_t) override { return false; }
	void GetRawTrackedDevicePoses(float, vr::TrackedDevicePose_t*, uint32_t) override {}
	void RequestRestart(const char*, const char*, const char*, const char*) override {}
	uint32_t GetFrameTimings(vr::Compositor_FrameTiming*, uint32_t) override { return 0; }
	void SetDisplayEyeToHead(uint32_t, const vr::HmdMatrix34_t&, const vr::HmdMatrix34_t&) override {}
	void SetDisplayProjectionRaw(uint32_t, const vr::HmdRect2_t&, const vr::HmdRect2_t&) override {}
	void SetRecommendedRenderTargetSize(uint32_t, uint32_t, uint32_t) override {}

private:
	vr::TrackedDeviceIndex_t next_id_ = 1; // device 0 is conventionally the HMD
};

class FakeDriverContext final : public vr::IVRDriverContext
{
public:
	FakeServerDriverHost host;
	FakeProperties props;

	void* GetGenericInterface(const char* pchInterfaceVersion, vr::EVRInitError* peError) override
	{
		if (peError) *peError = vr::VRInitError_None;
		if (pchInterfaceVersion && std::strncmp(pchInterfaceVersion, "IVRServerDriverHost", 19) == 0) {
			return static_cast<vr::IVRServerDriverHost*>(&host);
		}
		if (pchInterfaceVersion && std::strncmp(pchInterfaceVersion, "IVRProperties", 13) == 0) {
			return static_cast<vr::IVRProperties*>(&props);
		}
		if (peError) *peError = vr::VRInitError_Init_InterfaceNotFound;
		return nullptr;
	}

	vr::DriverHandle_t GetDriverHandle() override { return 1; }
};

// Install the fake as the active server-driver context. The InitServer() probe may
// report a missing-interface error (we only fake host + properties), which is
// irrelevant: it still sets the module context so VRServerDriverHost()/VRProperties()
// resolve to the fakes. Call vr::CleanupDriverContext() when done.
inline void InstallFakeDriverContext(FakeDriverContext& ctx)
{
	(void)vr::InitServerDriverContext(&ctx);
}

} // namespace phantom_test
