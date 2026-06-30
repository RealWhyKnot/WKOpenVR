#include "VirtualTrackerDevice.h"

#include "Logging.h"

#include <cstring>

namespace phantom {

namespace {

// FNV-1a 64-bit over a string; used to derive a stable serial suffix.
uint64_t Fnv1a64(const char* s)
{
	uint64_t h = 0xcbf29ce484222325ull;
	for (; s && *s; ++s) {
		h ^= static_cast<unsigned char>(*s);
		h *= 0x100000001b3ull;
	}
	return h;
}

} // namespace

VirtualTrackerDevice::VirtualTrackerDevice(BodyRole role) : role_(role)
{
	char buf[96];
	const char* roleKey = BodyRoleToKey(role);
	std::snprintf(buf, sizeof(buf), "WKOPENVR-%s-%016llx", roleKey, static_cast<unsigned long long>(Fnv1a64(roleKey)));
	serial_ = buf;
	// Identity rotation, zero translation; safe default until the first
	// Publish overwrites with a solver-derived pose.
	last_pose_.qWorldFromDriverRotation = {1, 0, 0, 0};
	last_pose_.qDriverFromHeadRotation = {1, 0, 0, 0};
	last_pose_.qRotation = {1, 0, 0, 0};
	last_pose_.result = vr::TrackingResult_Uninitialized;
}

vr::EVRInitError VirtualTrackerDevice::Activate(vr::TrackedDeviceIndex_t unObjectId)
{
	object_id_ = unObjectId;
	prop_container_ = vr::VRProperties()->TrackedDeviceToPropertyContainer(unObjectId);
	if (prop_container_ == vr::k_ulInvalidPropertyContainer) {
		LOG("[phantom] VirtualTrackerDevice(%s) Activate: no property container", serial_.c_str());
		return vr::VRInitError_Driver_Failed;
	}

	auto* props = vr::VRProperties();
	const char* controllerType = BodyRoleToControllerType(role_);
	const char* roleKey = BodyRoleToKey(role_);

	props->SetStringProperty(prop_container_, vr::Prop_TrackingSystemName_String, "wkopenvr");
	props->SetStringProperty(prop_container_, vr::Prop_ModelNumber_String, "WKOpenVR Virtual Tracker");
	props->SetStringProperty(prop_container_, vr::Prop_SerialNumber_String, serial_.c_str());
	props->SetStringProperty(prop_container_, vr::Prop_RenderModelName_String, "{htc}vr_tracker_vive_1_0");
	if (controllerType) {
		props->SetStringProperty(prop_container_, vr::Prop_ControllerType_String, controllerType);
	}
	char profilePath[160];
	bool hasProfile = false;
	if (BodyRoleInputProfilePath(role_, profilePath, sizeof(profilePath))) {
		props->SetStringProperty(prop_container_, vr::Prop_InputProfilePath_String, profilePath);
		hasProfile = true;
	}

	props->SetInt32Property(prop_container_, vr::Prop_DeviceClass_Int32, vr::TrackedDeviceClass_GenericTracker);
	props->SetInt32Property(prop_container_, vr::Prop_ControllerRoleHint_Int32, vr::TrackedControllerRole_OptOut);

	props->SetBoolProperty(prop_container_, vr::Prop_NeverTracked_Bool, false);
	props->SetBoolProperty(prop_container_, vr::Prop_Identifiable_Bool, false);

	LOG("[phantom] VirtualTrackerDevice activated: role=%s serial=%s objectId=%u controller_type=%s profile=%s",
	    roleKey, serial_.c_str(), (unsigned)unObjectId, controllerType ? controllerType : "(none)",
	    hasProfile ? profilePath : "(none)");
	return vr::VRInitError_None;
}

void VirtualTrackerDevice::Deactivate()
{
	object_id_ = vr::k_unTrackedDeviceIndexInvalid;
	prop_container_ = vr::k_ulInvalidPropertyContainer;
}

void VirtualTrackerDevice::DebugRequest(const char* /*pchRequest*/, char* pchResponseBuffer,
                                        uint32_t unResponseBufferSize)
{
	if (pchResponseBuffer && unResponseBufferSize > 0) {
		pchResponseBuffer[0] = '\0';
	}
}

vr::DriverPose_t VirtualTrackerDevice::GetPose()
{
	// Single-reader path off SteamVR's worker. The acquire on pose_epoch_
	// is paired with the release in Publish; a torn read returns a stale
	// pose that is still valid (last-known) rather than a half-written
	// mess.
	(void)pose_epoch_.load(std::memory_order_acquire);
	return last_pose_;
}

void VirtualTrackerDevice::Publish(const vr::DriverPose_t& pose)
{
	last_pose_ = pose;
	pose_epoch_.fetch_add(1, std::memory_order_release);
	// Do NOT call TrackedDevicePoseUpdated here: this runs inside the umbrella's
	// pose hook (holding its state mutex), and TrackedDevicePoseUpdated re-enters
	// that hook on the same thread -> "resource deadlock would occur". Mark the
	// pose pending; the manager drains it through CollectSilentPoseUpdates, which
	// the umbrella forwards after releasing the lock.
	pending_ = true;
}

bool VirtualTrackerDevice::TakePendingPose(vr::DriverPose_t& out)
{
	if (!pending_) return false;
	pending_ = false;
	out = last_pose_;
	return true;
}

} // namespace phantom
