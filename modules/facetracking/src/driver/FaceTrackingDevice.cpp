#include "FaceTrackingDevice.h"
#include "Logging.h"

#include <openvr_driver.h>

#include <cmath>
#include <cstring>

namespace facetracking {

FaceTrackingDevice::FaceTrackingDevice()
    : object_id_(vr::k_unTrackedDeviceIndexInvalid), h_open_left_(vr::k_ulInvalidInputComponentHandle),
      h_open_right_(vr::k_ulInvalidInputComponentHandle), h_pupil_left_(vr::k_ulInvalidInputComponentHandle),
      h_pupil_right_(vr::k_ulInvalidInputComponentHandle)
{
	// Default pose: valid device, no position, forward-looking orientation.
	cached_pose_.deviceIsConnected = true;
	cached_pose_.poseIsValid = false;
	cached_pose_.result = vr::TrackingResult_Running_OK;
	cached_pose_.qRotation = {1.0, 0.0, 0.0, 0.0};
	cached_pose_.qWorldFromDriverRotation = {1.0, 0.0, 0.0, 0.0};
	cached_pose_.qDriverFromHeadRotation = {1.0, 0.0, 0.0, 0.0};
}

vr::EVRInitError FaceTrackingDevice::Activate(uint32_t unObjectId)
{
	object_id_ = unObjectId;

	vr::PropertyContainerHandle_t container = vr::VRProperties()->TrackedDeviceToPropertyContainer(object_id_);

	vr::VRProperties()->SetStringProperty(container, vr::Prop_SerialNumber_String, "OpenVRPair-FaceTracking-Sink");
	vr::VRProperties()->SetStringProperty(container, vr::Prop_ModelNumber_String, "OpenVRPair FaceTracking");
	vr::VRProperties()->SetStringProperty(container, vr::Prop_ManufacturerName_String, "WhyKnot");
	vr::VRProperties()->SetBoolProperty(container, vr::Prop_DeviceIsWireless_Bool, false);
	vr::VRProperties()->SetBoolProperty(container, vr::Prop_DeviceIsCharging_Bool, false);
	vr::VRProperties()->SetBoolProperty(container, vr::Prop_WillDriftInYaw_Bool, false);
	vr::VRProperties()->SetBoolProperty(container, vr::Prop_DeviceCanPowerOff_Bool, false);

	// Scalar input components for eyelid openness and pupil dilation.
	vr::IVRDriverInput* input = vr::VRDriverInput();
	input->CreateScalarComponent(container, "/input/left/eye/openness", &h_open_left_, vr::VRScalarType_Absolute,
	                             vr::VRScalarUnits_NormalizedOneSided);
	input->CreateScalarComponent(container, "/input/right/eye/openness", &h_open_right_, vr::VRScalarType_Absolute,
	                             vr::VRScalarUnits_NormalizedOneSided);
	input->CreateScalarComponent(container, "/input/left/pupil/dilation", &h_pupil_left_, vr::VRScalarType_Absolute,
	                             vr::VRScalarUnits_NormalizedOneSided);
	input->CreateScalarComponent(container, "/input/right/pupil/dilation", &h_pupil_right_, vr::VRScalarType_Absolute,
	                             vr::VRScalarUnits_NormalizedOneSided);

	FT_LOG_DRV("[device] activated id=%u", unObjectId);
	return vr::VRInitError_None;
}

void FaceTrackingDevice::Deactivate()
{
	object_id_ = vr::k_unTrackedDeviceIndexInvalid;
}

void* FaceTrackingDevice::GetComponent(const char* /*pchComponentNameAndVersion*/)
{
	return nullptr;
}

void FaceTrackingDevice::DebugRequest(const char* /*pchRequest*/, char* pchResponseBuffer,
                                      uint32_t unResponseBufferSize)
{
	if (unResponseBufferSize > 0) pchResponseBuffer[0] = '\0';
}

vr::DriverPose_t FaceTrackingDevice::GetPose()
{
	return LoadCachedPose();
}

// -----------------------------------------------------------------------
// Pose cache (tiny spinlock around a memcpy).
// -----------------------------------------------------------------------

void FaceTrackingDevice::SetCachedPose(const vr::DriverPose_t& p)
{
	while (pose_lock_.test_and_set(std::memory_order_acquire)) {}
	cached_pose_ = p;
	pose_lock_.clear(std::memory_order_release);
}

vr::DriverPose_t FaceTrackingDevice::LoadCachedPose() const
{
	while (pose_lock_.test_and_set(std::memory_order_acquire)) {}
	vr::DriverPose_t copy = cached_pose_;
	pose_lock_.clear(std::memory_order_release);
	return copy;
}

// -----------------------------------------------------------------------
// Gaze-direction -> quaternion.
// Build an orientation that points the device's -Z axis along the gaze
// direction (OpenVR convention: -Z forward, +Y up, +X right).
// -----------------------------------------------------------------------

static vr::HmdQuaternion_t GazeToOrientation(const float g[3])
{
	// Defense-in-depth: the host should already replace non-finite floats
	// with zero, but a path that bypasses FrameWriter::SanitizeNonFinite
	// (manual shmem writes during testing, third-party producers) would
	// otherwise feed NaN through every relational gate below and on into
	// TrackedDevicePoseUpdated as a NaN quaternion.
	if (!std::isfinite(g[0]) || !std::isfinite(g[1]) || !std::isfinite(g[2])) return {1.0, 0.0, 0.0, 0.0};

	float gx = g[0], gy = g[1], gz = g[2];
	float len = std::sqrt(gx * gx + gy * gy + gz * gz);
	if (len < 1e-6f) return {1.0, 0.0, 0.0, 0.0};
	gx /= len;
	gy /= len;
	gz /= len;

	// Right axis = cross((0,1,0), gaze).  If gaze is (near-)vertical use (1,0,0).
	float rx = -gz, ry = 0.f, rz = gx;
	float rlen = std::sqrt(rx * rx + rz * rz);
	if (rlen < 1e-6f) {
		rx = 1.f;
		rz = 0.f;
		rlen = 1.f;
	}
	rx /= rlen;
	rz /= rlen;

	// Up axis = cross(gaze, right).
	float ux = gy * rz - gz * ry;
	float uy = gz * rx - gx * rz;
	float uz = gx * ry - gy * rx;

	// Rotation matrix columns: [right | up | -gaze].
	float m00 = rx, m01 = ux, m02 = -gx;
	float m10 = ry, m11 = uy, m12 = -gy;
	float m20 = rz, m21 = uz, m22 = -gz;

	float trace = m00 + m11 + m22;
	vr::HmdQuaternion_t q{};
	if (trace > 0.f) {
		float s = 0.5f / std::sqrt(trace + 1.f);
		q.w = 0.25f / s;
		q.x = (m21 - m12) * s;
		q.y = (m02 - m20) * s;
		q.z = (m10 - m01) * s;
	}
	else if (m00 > m11 && m00 > m22) {
		float s = 2.f * std::sqrt(1.f + m00 - m11 - m22);
		q.w = (m21 - m12) / s;
		q.x = 0.25f * s;
		q.y = (m01 + m10) / s;
		q.z = (m02 + m20) / s;
	}
	else if (m11 > m22) {
		float s = 2.f * std::sqrt(1.f + m11 - m00 - m22);
		q.w = (m02 - m20) / s;
		q.x = (m01 + m10) / s;
		q.y = 0.25f * s;
		q.z = (m12 + m21) / s;
	}
	else {
		float s = 2.f * std::sqrt(1.f + m22 - m00 - m11);
		q.w = (m10 - m01) / s;
		q.x = (m02 + m20) / s;
		q.y = (m12 + m21) / s;
		q.z = 0.25f * s;
	}
	return q;
}

vr::DriverPose_t FaceTrackingDevice::BuildGazePose(const float origin[3], const float gaze[3])
{
	vr::DriverPose_t p{};
	p.deviceIsConnected = true;
	p.poseIsValid = true;
	p.result = vr::TrackingResult_Running_OK;
	p.vecPosition[0] = origin[0];
	p.vecPosition[1] = origin[1];
	p.vecPosition[2] = origin[2];
	p.qRotation = GazeToOrientation(gaze);
	p.qWorldFromDriverRotation = {1.0, 0.0, 0.0, 0.0};
	p.qDriverFromHeadRotation = {1.0, 0.0, 0.0, 0.0};
	return p;
}

// -----------------------------------------------------------------------
// PublishFrame
// -----------------------------------------------------------------------

void FaceTrackingDevice::PublishFrame(const protocol::FaceTrackingFrameBody& frame)
{
	if (object_id_ == vr::k_unTrackedDeviceIndexInvalid) return;

	auto finite_or_zero = [](float v) {
		return std::isfinite(v) ? v : 0.0f;
	};

	if (frame.flags & 1u) {
		// Publish the left-eye gaze as the device pose (representative single
		// gaze; right-eye data is in the scalar components below). Host-side
		// FrameWriter::SanitizeNonFinite is the primary defense; the local
		// sanitize here covers producers that bypass the C# host (test
		// harnesses, future native publishers).
		const float origin[3] = {
		    finite_or_zero(frame.eye_origin_l[0]),
		    finite_or_zero(frame.eye_origin_l[1]),
		    finite_or_zero(frame.eye_origin_l[2]),
		};
		const float gaze[3] = {
		    finite_or_zero(frame.eye_gaze_l[0]),
		    finite_or_zero(frame.eye_gaze_l[1]),
		    finite_or_zero(frame.eye_gaze_l[2]),
		};
		vr::DriverPose_t pose = BuildGazePose(origin, gaze);
		SetCachedPose(pose);
		vr::VRServerDriverHost()->TrackedDevicePoseUpdated(object_id_, pose, sizeof(pose));

		// Scalar openness and pupil dilation.
		vr::IVRDriverInput* input = vr::VRDriverInput();
		if (h_open_left_ != vr::k_ulInvalidInputComponentHandle)
			input->UpdateScalarComponent(h_open_left_, finite_or_zero(frame.eye_openness_l), 0.0);
		if (h_open_right_ != vr::k_ulInvalidInputComponentHandle)
			input->UpdateScalarComponent(h_open_right_, finite_or_zero(frame.eye_openness_r), 0.0);
		if (h_pupil_left_ != vr::k_ulInvalidInputComponentHandle)
			input->UpdateScalarComponent(h_pupil_left_, finite_or_zero(frame.pupil_dilation_l), 0.0);
		if (h_pupil_right_ != vr::k_ulInvalidInputComponentHandle)
			input->UpdateScalarComponent(h_pupil_right_, finite_or_zero(frame.pupil_dilation_r), 0.0);
	}
}

} // namespace facetracking
