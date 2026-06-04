#pragma once

#include "Protocol.h"

#include <openvr_driver.h>
#include <atomic>
#include <cstdint>

namespace facetracking {

// ITrackedDeviceServerDriver implementation for the face/eye-tracking sink.
//
// Registered as TrackedDeviceClass_GenericTracker.  Does not provide real
// positional tracking; it exists to host the eye-gaze pose (published via
// VRServerDriverHost()->TrackedDevicePoseUpdated()) and the scalar openness
// and pupil-dilation input components.
//
// PublishFrame() is called from the worker thread after the hot-path filter
// chain has run.  It updates the driver pose and scalar input components so
// OpenVR and the SteamVR OpenXR runtime can forward them.
class FaceTrackingDevice : public vr::ITrackedDeviceServerDriver
{
public:
	FaceTrackingDevice();
	~FaceTrackingDevice() = default;

	// ITrackedDeviceServerDriver
	vr::EVRInitError Activate(uint32_t unObjectId) override;
	void Deactivate() override;
	void EnterStandby() override {}
	void* GetComponent(const char* pchComponentNameAndVersion) override;
	void DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize) override;
	vr::DriverPose_t GetPose() override;

	// Push a processed frame into the registered input components and update
	// the device pose.  Safe to call from the worker thread after Activate().
	void PublishFrame(const protocol::FaceTrackingFrameBody& frame);

	bool IsActive() const { return object_id_ != vr::k_unTrackedDeviceIndexInvalid; }

private:
	uint32_t object_id_;

	// Scalar input component handles for openness and pupil dilation.
	vr::VRInputComponentHandle_t h_open_left_;
	vr::VRInputComponentHandle_t h_open_right_;
	vr::VRInputComponentHandle_t h_pupil_left_;
	vr::VRInputComponentHandle_t h_pupil_right_;

	// Cached pose for GetPose() -- updated by PublishFrame().
	// Written from the worker thread, read by the SteamVR runtime thread
	// (GetPose).  Use relaxed atomic copy via memcpy under a lightweight
	// spinlock (we write infrequently, SteamVR reads at vsync rate).
	// A simple mutex is fine here; the critical section is tiny.
	mutable std::atomic_flag pose_lock_ = ATOMIC_FLAG_INIT;
	vr::DriverPose_t cached_pose_{};

	void SetCachedPose(const vr::DriverPose_t& p);
	vr::DriverPose_t LoadCachedPose() const;

	// Build a driver pose from gaze direction / origin.
	static vr::DriverPose_t BuildGazePose(const float origin[3], const float gaze[3]);
};

} // namespace facetracking
