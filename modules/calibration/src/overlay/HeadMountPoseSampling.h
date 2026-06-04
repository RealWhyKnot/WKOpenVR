#pragma once

// Pure helpers for the head-mount tracker pose-sampling path. Extracted so
// unit tests can exercise the composition and resolution logic without a live
// OpenVR runtime.

#include "Calibration.h"
#include "CalibrationCalc.h" // Pose
#include "VRState.h"

#include <Eigen/Geometry>
#include <openvr.h>

#include <cmath>

namespace spacecal::headmount {

struct DriverSynthContinuousStatus
{
	bool ready = false;
	const char* reason = "unknown";
	double hmdProxyDeltaM = -1.0;
};

// Returns true if the head-mount tracker is engaged for sampling:
//   - mode >= AutoPaired
//   - headFromTracker offset calibrated
//   - deviceID resolved (>= 0) and within OpenVR range
//   - the device pose in poseArray is valid and Running_OK
inline bool IsTrackerValidForSampling(const HeadMountConfig& cfg, const vr::DriverPose_t* poseArray,
                                      uint32_t poseArraySize)
{
	if (cfg.mode < HeadMountMode::AutoPaired) return false;
	if (!cfg.offsetCalibrated) return false;
	if (cfg.deviceID < 0 || (uint32_t)cfg.deviceID >= poseArraySize) return false;
	const vr::DriverPose_t& p = poseArray[cfg.deviceID];
	return p.poseIsValid && p.result == vr::ETrackingResult::TrackingResult_Running_OK;
}

// Compute the head-position AffineCompact3d by composing the tracker's world
// pose with headFromTracker. Returns identity when the tracker is invalid.
//
// head_world = tracker_world * headFromTracker
//
// The right-composition maps the offset from the tracker local frame into
// world space: the translation component of headFromTracker is rotated by
// the tracker's world orientation and then added to the tracker world origin.
inline Eigen::AffineCompact3d ComputeHeadWorldPose(const vr::DriverPose_t& trackerRaw,
                                                   const Eigen::AffineCompact3d& headFromTracker)
{
	// Build world pose from DriverPose_t fields.
	// qWorldFromDriverRotation * qRotation gives the device orientation in world space.
	// vecWorldFromDriverTranslation + qWorldFromDriverRotation * vecPosition gives world position.
	Eigen::Quaterniond wfd(trackerRaw.qWorldFromDriverRotation.w, trackerRaw.qWorldFromDriverRotation.x,
	                       trackerRaw.qWorldFromDriverRotation.y, trackerRaw.qWorldFromDriverRotation.z);
	Eigen::Quaterniond localRot(trackerRaw.qRotation.w, trackerRaw.qRotation.x, trackerRaw.qRotation.y,
	                            trackerRaw.qRotation.z);
	Eigen::Quaterniond worldRot = (wfd * localRot).normalized();

	Eigen::Vector3d wfdTrans(trackerRaw.vecWorldFromDriverTranslation[0], trackerRaw.vecWorldFromDriverTranslation[1],
	                         trackerRaw.vecWorldFromDriverTranslation[2]);
	Eigen::Vector3d localPos(trackerRaw.vecPosition[0], trackerRaw.vecPosition[1], trackerRaw.vecPosition[2]);
	Eigen::Vector3d worldTrans = wfdTrans + wfd * localPos;

	Eigen::AffineCompact3d trackerWorld = Eigen::Translation3d(worldTrans) * worldRot;

	return trackerWorld * headFromTracker;
}

inline bool PoseIsRunningOk(const vr::DriverPose_t& pose)
{
	return pose.poseIsValid && pose.result == vr::ETrackingResult::TrackingResult_Running_OK;
}

inline DriverSynthContinuousStatus EvaluateDriverSynthContinuousStatus(const HeadMountConfig& cfg, bool inContinuous,
                                                                       bool targetMatchesHeadMount,
                                                                       const vr::DriverPose_t* poseArray,
                                                                       uint32_t poseArraySize)
{
	DriverSynthContinuousStatus status{};

	if (!inContinuous) {
		status.reason = "not_continuous";
		return status;
	}
	if (cfg.mode != HeadMountMode::DriverSynth) {
		status.reason = "mode_not_driver_synth";
		return status;
	}
	if (!cfg.offsetCalibrated) {
		status.reason = "offset_not_calibrated";
		return status;
	}
	if (!targetMatchesHeadMount) {
		status.reason = "target_mismatch";
		return status;
	}
	if (!poseArray) {
		status.reason = "pose_array_null";
		return status;
	}
	if (cfg.deviceID < 0 || (uint32_t)cfg.deviceID >= poseArraySize) {
		status.reason = "tracker_device_invalid";
		return status;
	}
	if (vr::k_unTrackedDeviceIndex_Hmd >= poseArraySize) {
		status.reason = "hmd_device_invalid";
		return status;
	}

	const vr::DriverPose_t& tracker = poseArray[cfg.deviceID];
	const vr::DriverPose_t& hmd = poseArray[vr::k_unTrackedDeviceIndex_Hmd];
	if (!PoseIsRunningOk(tracker)) {
		status.reason = "tracker_pose_invalid";
		return status;
	}
	if (!PoseIsRunningOk(hmd)) {
		status.reason = "hmd_pose_invalid";
		return status;
	}

	const Eigen::AffineCompact3d proxyHead = ComputeHeadWorldPose(tracker, cfg.headFromTracker);
	const Eigen::AffineCompact3d hmdWorld = ComputeHeadWorldPose(hmd, Eigen::AffineCompact3d::Identity());
	const double deltaM = (proxyHead.translation() - hmdWorld.translation()).norm();
	status.hmdProxyDeltaM = deltaM;
	if (!std::isfinite(deltaM)) {
		status.reason = "delta_non_finite";
		return status;
	}

	status.ready = true;
	status.reason = "driver_synth_ready";
	return status;
}

// Attempt to resolve a head-mount tracker to an OpenVR device ID from a
// VRState snapshot. Both model and serial must be non-empty for FindDevice
// to match a non-HMD device. Returns -1 if not found.
inline int32_t ResolveHeadMountDeviceID(const HeadMountConfig& cfg, const VRState& state)
{
	if (cfg.trackerSerial.empty()) return -1;
	return (int32_t)state.FindDevice(cfg.trackerTrackingSystem, cfg.trackerModel, cfg.trackerSerial);
}

} // namespace spacecal::headmount
