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

VirtualTrackerDevice::VirtualTrackerDevice(BodyRole role)
    : role_(role)
{
    char buf[96];
    const char* roleKey = BodyRoleToKey(role);
    std::snprintf(buf, sizeof(buf),
        "WKOPENVR-%s-%016llx",
        roleKey,
        static_cast<unsigned long long>(Fnv1a64(roleKey)));
    serial_ = buf;
    // Identity rotation, zero translation; safe default until the first
    // Publish overwrites with an IK-derived pose.
    last_pose_.qWorldFromDriverRotation = {1, 0, 0, 0};
    last_pose_.qDriverFromHeadRotation  = {1, 0, 0, 0};
    last_pose_.qRotation = {1, 0, 0, 0};
    last_pose_.result = vr::TrackingResult_Uninitialized;
}

vr::EVRInitError VirtualTrackerDevice::Activate(vr::TrackedDeviceIndex_t unObjectId)
{
    object_id_ = unObjectId;
    prop_container_ = vr::VRProperties()->TrackedDeviceToPropertyContainer(unObjectId);
    if (prop_container_ == vr::k_ulInvalidPropertyContainer) {
        LOG("[phantom] VirtualTrackerDevice(%s) Activate: no property container",
            serial_.c_str());
        return vr::VRInitError_Driver_Failed;
    }

    auto* props = vr::VRProperties();
    const char* controllerType = BodyRoleToControllerType(role_);
    const char* roleKey = BodyRoleToKey(role_);

    props->SetStringProperty(prop_container_, vr::Prop_TrackingSystemName_String, "wkopenvr");
    props->SetStringProperty(prop_container_, vr::Prop_ModelNumber_String, "WKOpenVR Virtual Tracker");
    props->SetStringProperty(prop_container_, vr::Prop_SerialNumber_String, serial_.c_str());
    props->SetStringProperty(prop_container_, vr::Prop_RenderModelName_String,
        "{htc}vr_tracker_vive_1_0");
    if (controllerType) {
        props->SetStringProperty(prop_container_, vr::Prop_ControllerType_String, controllerType);
    }
    // {wkopenvr} resource root resolves to drivers/01wkopenvr/resources/.
    char profilePath[160];
    std::snprintf(profilePath, sizeof(profilePath),
        "{wkopenvr}/input/vive_tracker_%s_profile.json", roleKey);
    props->SetStringProperty(prop_container_, vr::Prop_InputProfilePath_String, profilePath);

    props->SetInt32Property(prop_container_, vr::Prop_DeviceClass_Int32,
        vr::TrackedDeviceClass_GenericTracker);
    props->SetInt32Property(prop_container_, vr::Prop_ControllerRoleHint_Int32,
        vr::TrackedControllerRole_OptOut);

    props->SetBoolProperty(prop_container_, vr::Prop_NeverTracked_Bool, false);
    props->SetBoolProperty(prop_container_, vr::Prop_Identifiable_Bool, false);

    LOG("[phantom] VirtualTrackerDevice activated: role=%s serial=%s objectId=%u",
        roleKey, serial_.c_str(), (unsigned)unObjectId);
    return vr::VRInitError_None;
}

void VirtualTrackerDevice::Deactivate()
{
    object_id_ = vr::k_unTrackedDeviceIndexInvalid;
    prop_container_ = vr::k_ulInvalidPropertyContainer;
}

void VirtualTrackerDevice::DebugRequest(const char* /*pchRequest*/,
                                         char* pchResponseBuffer,
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

    if (object_id_ != vr::k_unTrackedDeviceIndexInvalid && vr::VRServerDriverHost()) {
        vr::VRServerDriverHost()->TrackedDevicePoseUpdated(
            object_id_, last_pose_, sizeof(last_pose_));
    }
}

} // namespace phantom
