#include "MockServerDriverHost.h"

#if WKOPENVR_BUILD_IS_DEV

#include "../HarnessScenario.h"
#include "../MockPoseSource.h" // for forward use; MockOpenVRRuntime::recorder()

#include <cstring>

namespace openvr_pair::overlay::testharness {

namespace {

inline MockCall MakeCall(MockCallKind kind)
{
	MockCall c;
	c.kind = kind;
	return c;
}

} // namespace

MockServerDriverHost::MockServerDriverHost(MockOpenVRRuntime& owner) : owner_(owner) {}

uint32_t MockServerDriverHost::FindDeviceBySerial(const std::string& serial) const
{
	std::lock_guard<std::mutex> lock(mu_);
	auto it = serial_to_id_.find(serial);
	return it == serial_to_id_.end() ? UINT32_MAX : it->second;
}

bool MockServerDriverHost::TrackedDeviceAdded(const char* pchDeviceSerialNumber, vr::ETrackedDeviceClass eDeviceClass,
                                              vr::ITrackedDeviceServerDriver* pDriver)
{
	if (pchDeviceSerialNumber == nullptr) return false;
	const std::string serial(pchDeviceSerialNumber);
	uint32_t id;
	{
		std::lock_guard<std::mutex> lock(mu_);
		auto it = serial_to_id_.find(serial);
		if (it != serial_to_id_.end()) {
			return true; // already added; SteamVR semantics
		}
		id = (uint32_t)devices_.size();
		devices_.push_back(DeviceEntry{id, serial, eDeviceClass, pDriver});
		serial_to_id_.emplace(serial, id);
	}

	MockCall call = MakeCall(MockCallKind::TrackedDeviceAdded);
	call.device_id = id;
	call.aux_int = (uint32_t)eDeviceClass;
	call.text = serial;
	owner_.recorder().Push(std::move(call));
	if (pDriver) {
		const auto err = pDriver->Activate(id);
		if (err != vr::VRInitError_None) {
			MockCall err_call = MakeCall(MockCallKind::LogMessage);
			err_call.device_id = id;
			err_call.aux_int = (uint32_t)err;
			err_call.text = "TrackedDeviceAdded activation failed";
			owner_.recorder().Push(std::move(err_call));
			return false;
		}
	}
	return true;
}

void MockServerDriverHost::TrackedDevicePoseUpdated(uint32_t unWhichDevice, const vr::DriverPose_t& newPose,
                                                    uint32_t /*unPoseStructSize*/)
{
	MockCall call = MakeCall(MockCallKind::TrackedDevicePoseUpdated);
	call.device_id = unWhichDevice;
	call.b_value = newPose.poseIsValid;
	// Stash position[0..2] tightly inside f_value/time_offset/aux_int via reinterpret;
	// scenarios that need full pose snapshots can wire a side-channel later.
	call.f_value = newPose.vecPosition[0];
	call.time_offset_sec = newPose.vecPosition[1];
	call.aux_int = (uint32_t)(int32_t)(newPose.vecPosition[2] * 1000.0); // mm, lossy
	call.has_pose = true;
	call.pose_device_connected = newPose.deviceIsConnected;
	call.pose_tracking_result = static_cast<int32_t>(newPose.result);
	for (int i = 0; i < 3; ++i) {
		call.pose_position[i] = newPose.vecPosition[i];
		call.pose_velocity[i] = newPose.vecVelocity[i];
	}
	call.pose_rotation[0] = newPose.qRotation.w;
	call.pose_rotation[1] = newPose.qRotation.x;
	call.pose_rotation[2] = newPose.qRotation.y;
	call.pose_rotation[3] = newPose.qRotation.z;
	owner_.recorder().Push(std::move(call));
}

void MockServerDriverHost::VsyncEvent(double /*vsyncTimeOffsetSeconds*/) {}

void MockServerDriverHost::VendorSpecificEvent(uint32_t /*unWhichDevice*/, vr::EVREventType /*eventType*/,
                                               const vr::VREvent_Data_t& /*eventData*/, double /*eventTimeOffset*/)
{
}

bool MockServerDriverHost::IsExiting()
{
	return exiting_;
}

bool MockServerDriverHost::PollNextEvent(vr::VREvent_t* /*pEvent*/, uint32_t /*uncbVREvent*/)
{
	return false;
}

void MockServerDriverHost::GetRawTrackedDevicePoses(float /*fPredictedSecondsFromNow*/,
                                                    vr::TrackedDevicePose_t* pTrackedDevicePoseArray,
                                                    uint32_t unTrackedDevicePoseArrayCount)
{
	if (pTrackedDevicePoseArray == nullptr) return;
	std::memset(pTrackedDevicePoseArray, 0, sizeof(vr::TrackedDevicePose_t) * unTrackedDevicePoseArrayCount);
	for (uint32_t i = 0; i < unTrackedDevicePoseArrayCount; ++i) {
		pTrackedDevicePoseArray[i].mDeviceToAbsoluteTracking.m[0][0] = 1.0f;
		pTrackedDevicePoseArray[i].mDeviceToAbsoluteTracking.m[1][1] = 1.0f;
		pTrackedDevicePoseArray[i].mDeviceToAbsoluteTracking.m[2][2] = 1.0f;
	}
}

void MockServerDriverHost::RequestRestart(const char* pchLocalizedReason, const char* /*pchExecutableToStart*/,
                                          const char* /*pchArguments*/, const char* /*pchWorkingDirectory*/)
{
	MockCall call = MakeCall(MockCallKind::RequestRestart);
	if (pchLocalizedReason) call.text = pchLocalizedReason;
	owner_.recorder().Push(std::move(call));
}

uint32_t MockServerDriverHost::GetFrameTimings(vr::Compositor_FrameTiming* /*pTiming*/, uint32_t /*nFrames*/)
{
	return 0;
}

void MockServerDriverHost::SetDisplayEyeToHead(uint32_t /*unWhichDevice*/, const vr::HmdMatrix34_t& /*eyeToHeadLeft*/,
                                               const vr::HmdMatrix34_t& /*eyeToHeadRight*/)
{
}

void MockServerDriverHost::SetDisplayProjectionRaw(uint32_t /*unWhichDevice*/, const vr::HmdRect2_t& /*eyeLeft*/,
                                                   const vr::HmdRect2_t& /*eyeRight*/)
{
}

void MockServerDriverHost::SetRecommendedRenderTargetSize(uint32_t /*unWhichDevice*/, uint32_t /*nWidth*/,
                                                          uint32_t /*nHeight*/)
{
}

} // namespace openvr_pair::overlay::testharness

#endif // WKOPENVR_BUILD_IS_DEV
