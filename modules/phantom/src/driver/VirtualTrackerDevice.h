#pragma once

#include "RoleCatalog.h"

#include <openvr_driver.h>

#include <atomic>
#include <string>

namespace phantom {

// Per-role virtual tracked device the umbrella driver publishes to SteamVR
// in absent-mode. The pose is sourced from body completion on each HMD-pose
// tick by VirtualTrackerManager; this class is otherwise a thin
// ITrackedDeviceServerDriver shell.
//
// Property values follow the Valve-published vive_tracker_<role> convention
// so VRChat / Resonite / Neos auto-bind the role without the user having
// to touch SteamVR's "Manage Vive Trackers" UI.
class VirtualTrackerDevice final : public vr::ITrackedDeviceServerDriver
{
public:
	explicit VirtualTrackerDevice(BodyRole role);

	// ITrackedDeviceServerDriver
	vr::EVRInitError Activate(vr::TrackedDeviceIndex_t unObjectId) override;
	void Deactivate() override;
	void EnterStandby() override {}
	void* GetComponent(const char* /*pchComponentNameAndVersion*/) override { return nullptr; }
	void DebugRequest(const char* /*pchRequest*/, char* pchResponseBuffer, uint32_t unResponseBufferSize) override;
	vr::DriverPose_t GetPose() override;

	// The serial we registered with SteamVR. Driver hosts identify devices
	// by serial as well as openVRID; expose for log messages.
	const std::string& Serial() const { return serial_; }

	BodyRole Role() const { return role_; }

	// Push a new pose for this device. Cached for any GetPose poll and marked
	// pending. The pose is NOT forwarded here: publishing TrackedDevicePoseUpdated
	// inline would re-enter the umbrella's pose hook (which holds its state mutex)
	// and self-deadlock. VirtualTrackerManager::CollectPoseUpdates drains the
	// pending pose so the umbrella forwards it outside the hook lock.
	void Publish(const vr::DriverPose_t& pose);

	// If a pose has been published since the last drain, copy it out and clear
	// the pending flag. Returns false when nothing new is pending. Same-thread
	// as Publish (the pose-hook thread); no locking needed.
	bool TakePendingPose(vr::DriverPose_t& out);

	bool Activated() const { return object_id_ != vr::k_unTrackedDeviceIndexInvalid; }
	vr::TrackedDeviceIndex_t ObjectId() const { return object_id_; }

private:
	BodyRole role_;
	std::string serial_;
	vr::TrackedDeviceIndex_t object_id_ = vr::k_unTrackedDeviceIndexInvalid;
	vr::PropertyContainerHandle_t prop_container_ = vr::k_ulInvalidPropertyContainer;

	// Last-published pose. Returned from GetPose if SteamVR polls.
	// Atomic-via-mutex-free via std::atomic on a POD-like struct: in
	// practice GetPose is only called by SteamVR's worker thread and
	// Publish is called by our hot path, so a single-writer-single-reader
	// pattern with a guard byte is enough.
	vr::DriverPose_t last_pose_{};
	std::atomic<uint32_t> pose_epoch_{0};

	// Set by Publish, cleared by TakePendingPose. Both run on the pose-hook
	// thread, so a plain bool is sufficient.
	bool pending_ = false;
};

} // namespace phantom
