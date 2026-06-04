#include "MockPoseSource.h"

#if WKOPENVR_BUILD_IS_DEV

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

#include <cstring>

namespace openvr_pair::overlay::testharness {

MockOpenVRRuntime::MockOpenVRRuntime(std::filesystem::path driver_resources)
    : driver_resources_(std::move(driver_resources))
{
	server_driver_host_ = std::make_unique<MockServerDriverHost>(*this);
	driver_input_ = std::make_unique<MockDriverInput>(*this);
	properties_ = std::make_unique<MockProperties>(*this);
	settings_ = std::make_unique<MockSettings>(*this);
	driver_log_ = std::make_unique<MockDriverLog>(*this);
	resources_ = std::make_unique<MockResources>(*this);
	driver_manager_ = std::make_unique<MockDriverManager>(*this);
	context_ = std::make_unique<MockDriverContext>(*this);

	context_->RegisterInterface(vr::IVRServerDriverHost_Version, server_driver_host_.get());
	context_->RegisterInterface(vr::IVRDriverInput_Version, driver_input_.get());
	context_->RegisterInterface(vr::IVRProperties_Version, properties_.get());
	context_->RegisterInterface(vr::IVRSettings_Version, settings_.get());
	context_->RegisterInterface(vr::IVRDriverLog_Version, driver_log_.get());
	context_->RegisterInterface(vr::IVRResources_Version, resources_.get());
	context_->RegisterInterface(vr::IVRDriverManager_Version, driver_manager_.get());
}

MockPoseSource::MockPoseSource(MockOpenVRRuntime& runtime) : runtime_(runtime) {}

uint32_t MockPoseSource::AddDevice(const std::string& serial, vr::ETrackedDeviceClass device_class)
{
	std::lock_guard<std::mutex> lock(mu_);
	const bool ok = runtime_.server_driver_host().TrackedDeviceAdded(serial.c_str(), device_class, /*pDriver*/ nullptr);
	if (!ok) return UINT32_MAX;
	return runtime_.server_driver_host().FindDeviceBySerial(serial);
}

void MockPoseSource::PushPose(uint32_t device_id, vr::DriverPose_t pose)
{
	// poseTimeOffset is left zero so the driver treats poses as arriving now.
	// Scenarios that care about predicted-time math set it explicitly before
	// calling PushPose.
	runtime_.server_driver_host().TrackedDevicePoseUpdated(device_id, pose, (uint32_t)sizeof(vr::DriverPose_t));
}

vr::DriverPose_t MockPoseSource::MakeIdentityPose(double x, double y, double z)
{
	vr::DriverPose_t p{};
	p.poseIsValid = true;
	p.result = vr::TrackingResult_Running_OK;
	p.deviceIsConnected = true;
	p.willDriftInYaw = false;
	p.shouldApplyHeadModel = false;
	p.qWorldFromDriverRotation.w = 1.0;
	p.qDriverFromHeadRotation.w = 1.0;
	p.qRotation.w = 1.0;
	p.vecPosition[0] = x;
	p.vecPosition[1] = y;
	p.vecPosition[2] = z;
	return p;
}

} // namespace openvr_pair::overlay::testharness

#endif // WKOPENVR_BUILD_IS_DEV
