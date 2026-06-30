#pragma once

#include "RoleCatalog.h"
#include "TrackerModelCatalog.h"

#include <openvr_driver.h>

#include <atomic>
#include <string>

namespace phantom {

// A pose that tells SteamVR the device is present but no longer tracking, so it
// hides the tracker promptly instead of floating its last valid pose until the
// runtime's own stale timeout (10-30 s). Identity transforms; position is
// irrelevant once poseIsValid / deviceIsConnected are false.
vr::DriverPose_t MakeDisconnectedVirtualPose();

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
	VirtualTrackerDevice(BodyRole role, TrackerModel model);

	// ITrackedDeviceServerDriver
	vr::EVRInitError Activate(vr::TrackedDeviceIndex_t unObjectId) override;
	void Deactivate() override;
	// Treated as a disconnect: report not-connected until a fresh Publish reopens
	// the gate, so a tracker does not float while the system is in standby.
	void EnterStandby() override;
	void* GetComponent(const char* /*pchComponentNameAndVersion*/) override { return nullptr; }
	void DebugRequest(const char* /*pchRequest*/, char* pchResponseBuffer, uint32_t unResponseBufferSize) override;
	vr::DriverPose_t GetPose() override;

	// The serial we registered with SteamVR. Driver hosts identify devices
	// by serial as well as openVRID; expose for log messages.
	const std::string& Serial() const { return serial_; }

	BodyRole Role() const { return role_; }

	// Change the SteamVR render model. Re-applies Prop_RenderModelName_String if the
	// device is already activated (best-effort live update); the change is guaranteed
	// on the next vrserver restart. Called from the IPC thread when the user picks a
	// different model in the overlay.
	void SetModel(TrackerModel model);

	// Push a new pose for this device. Cached for any GetPose poll and marked
	// pending. The pose is NOT forwarded here: publishing TrackedDevicePoseUpdated
	// inline would re-enter the umbrella's pose hook (which holds its state mutex)
	// and self-deadlock. VirtualTrackerManager::CollectPoseUpdates drains the
	// pending pose so the umbrella forwards it outside the hook lock.
	void Publish(const vr::DriverPose_t& pose);

	// Cache a disconnected pose and mark it pending so the next drain forwards it
	// through the same out-of-lock channel as Publish. Pose-hook thread.
	void PublishDisconnect();

	// If a pose has been published since the last drain, copy it out and clear
	// the pending flag. Returns false when nothing new is pending. Same-thread
	// as Publish (the pose-hook thread); no locking needed.
	bool TakePendingPose(vr::DriverPose_t& out);

	// Connection gate consulted by GetPose. Atomic so the IPC thread can close it
	// the instant a role is disabled/blocked (GetPose then reports disconnected
	// without waiting for the next tick's pushed pose). Publish reopens it.
	void SetReportConnected(bool connected);
	bool ReportsConnected() const { return report_connected_.load(std::memory_order_acquire); }

	// Whether the last pose pushed through the drain was a connected one. Pose-hook
	// thread only; used by the manager to push exactly one disconnected pose when a
	// role is retracted (independent of the GetPose gate above).
	bool PushedConnected() const { return pushed_connected_; }

	bool Activated() const { return object_id_ != vr::k_unTrackedDeviceIndexInvalid; }
	vr::TrackedDeviceIndex_t ObjectId() const { return object_id_; }

private:
	BodyRole role_;
	// Render model presented to SteamVR. Set at construction, read in Activate, and
	// updated live by SetModel. Cosmetic only (controller type drives role binding).
	TrackerModel model_;
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

	// True while the device should report its tracked pose; false makes GetPose
	// return a disconnected pose. Written from both the pose-hook thread (Publish/
	// PublishDisconnect) and the IPC thread (SetEnabled/SetRoleBlocked), so it is
	// atomic; last_pose_ stays single-writer on the pose-hook thread.
	std::atomic<bool> report_connected_{true};

	// Set by Publish, cleared by TakePendingPose. Both run on the pose-hook
	// thread, so a plain bool is sufficient.
	bool pending_ = false;

	// True after a connected pose was pushed, false after a disconnected one.
	// Pose-hook thread only; drives the one-shot retract in the manager.
	bool pushed_connected_ = false;
};

} // namespace phantom
