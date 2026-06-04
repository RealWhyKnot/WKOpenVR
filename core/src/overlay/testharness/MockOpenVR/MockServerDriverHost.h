#pragma once

#include "BuildChannel.h"

#if WKOPENVR_BUILD_IS_DEV

#include <openvr_driver.h>

#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace openvr_pair::overlay::testharness {

class MockOpenVRRuntime;

// Implements vr::IVRServerDriverHost (version 006). The interesting calls
// (TrackedDeviceAdded, TrackedDevicePoseUpdated) push MockCalls into the
// runtime's BarrierQueue so scenarios can assert on emitted state. The
// remaining methods are no-ops or return zero/false.
class MockServerDriverHost : public vr::IVRServerDriverHost
{
public:
	explicit MockServerDriverHost(MockOpenVRRuntime& owner);

	// Sentinel + lookup helpers for scenarios.
	struct DeviceEntry
	{
		uint32_t id;
		std::string serial;
		vr::ETrackedDeviceClass device_class;
		vr::ITrackedDeviceServerDriver* server_driver;
	};
	const std::vector<DeviceEntry>& devices() const noexcept { return devices_; }
	uint32_t FindDeviceBySerial(const std::string& serial) const;

	bool exiting() const noexcept { return exiting_; }
	void set_exiting(bool v) { exiting_ = v; }

	// IVRServerDriverHost
	bool TrackedDeviceAdded(const char* pchDeviceSerialNumber, vr::ETrackedDeviceClass eDeviceClass,
	                        vr::ITrackedDeviceServerDriver* pDriver) override;
	void TrackedDevicePoseUpdated(uint32_t unWhichDevice, const vr::DriverPose_t& newPose,
	                              uint32_t unPoseStructSize) override;
	void VsyncEvent(double vsyncTimeOffsetSeconds) override;
	void VendorSpecificEvent(uint32_t unWhichDevice, vr::EVREventType eventType, const vr::VREvent_Data_t& eventData,
	                         double eventTimeOffset) override;
	bool IsExiting() override;
	bool PollNextEvent(vr::VREvent_t* pEvent, uint32_t uncbVREvent) override;
	void GetRawTrackedDevicePoses(float fPredictedSecondsFromNow, vr::TrackedDevicePose_t* pTrackedDevicePoseArray,
	                              uint32_t unTrackedDevicePoseArrayCount) override;
	void RequestRestart(const char* pchLocalizedReason, const char* pchExecutableToStart, const char* pchArguments,
	                    const char* pchWorkingDirectory) override;
	uint32_t GetFrameTimings(vr::Compositor_FrameTiming* pTiming, uint32_t nFrames) override;
	void SetDisplayEyeToHead(uint32_t unWhichDevice, const vr::HmdMatrix34_t& eyeToHeadLeft,
	                         const vr::HmdMatrix34_t& eyeToHeadRight) override;
	void SetDisplayProjectionRaw(uint32_t unWhichDevice, const vr::HmdRect2_t& eyeLeft,
	                             const vr::HmdRect2_t& eyeRight) override;
	void SetRecommendedRenderTargetSize(uint32_t unWhichDevice, uint32_t nWidth, uint32_t nHeight) override;

private:
	MockOpenVRRuntime& owner_;
	mutable std::mutex mu_;
	std::vector<DeviceEntry> devices_;
	std::unordered_map<std::string, uint32_t> serial_to_id_;
	bool exiting_ = false;
};

} // namespace openvr_pair::overlay::testharness

#endif // WKOPENVR_BUILD_IS_DEV
