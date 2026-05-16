#include "VirtualTrackerManager.h"

#include "Logging.h"

#include <cstddef>

namespace phantom {

VirtualTrackerManager::VirtualTrackerManager() = default;

void VirtualTrackerManager::OnDriverInit()
{
    init_time_ = std::chrono::steady_clock::now();
}

void VirtualTrackerManager::SetEnabled(BodyRole role, bool enabled)
{
    const auto idx = static_cast<size_t>(role);
    if (idx >= enabled_.size()) return;
    enabled_[idx] = enabled;
    if (!enabled && devices_[idx]) {
        // SteamVR does not honour TrackedDeviceRemoved live for generic
        // trackers; the slot stays activated for the rest of the session.
        // Mark it disabled so future Tick() calls stop publishing poses
        // and the device goes "stale" (SteamVR drops it after its own
        // timeout). Next vrserver restart will not re-activate.
        LOG("[phantom] virtual role %s disabled; pose publishing halted "
            "(virtual device stays until vrserver restart)",
            BodyRoleToKey(role));
    }
}

bool VirtualTrackerManager::IsEnabled(BodyRole role) const
{
    const auto idx = static_cast<size_t>(role);
    if (idx >= enabled_.size()) return false;
    return enabled_[idx];
}

bool VirtualTrackerManager::IsCalibrated(BodyRole role, const IkFallback& ik) const
{
    return ik.HasOffset(role);
}

void VirtualTrackerManager::MaybeActivate(BodyRole role, const IkFallback& ik)
{
    const auto idx = static_cast<size_t>(role);
    if (idx >= devices_.size()) return;
    if (devices_[idx]) return;                   // already activated
    if (!enabled_[idx]) return;
    if (!ik.HasOffset(role)) return;             // calibration prerequisite
    if (!hmd_pose_seen_.load(std::memory_order_acquire)) return;

    // openvr#1536 mitigation: wait at least kInitSettleDelay past driver
    // init before any TrackedDeviceAdded call so SteamVR has finished
    // enumerating real devices.
    if (std::chrono::steady_clock::now() - init_time_ < kInitSettleDelay) {
        return;
    }

    auto device = std::make_unique<VirtualTrackerDevice>(role);
    const std::string serial = device->Serial();
    if (!vr::VRServerDriverHost()) {
        LOG("[phantom] cannot activate virtual %s: VRServerDriverHost null",
            BodyRoleToKey(role));
        return;
    }
    const bool ok = vr::VRServerDriverHost()->TrackedDeviceAdded(
        serial.c_str(),
        vr::TrackedDeviceClass_GenericTracker,
        device.get());
    if (!ok) {
        LOG("[phantom] TrackedDeviceAdded failed for virtual %s (serial=%s)",
            BodyRoleToKey(role), serial.c_str());
        return;
    }
    devices_[idx] = std::move(device);
    LOG("[phantom] virtual %s activated", BodyRoleToKey(role));
}

void VirtualTrackerManager::Tick(const vr::DriverPose_t& hmd_pose,
                                 const IkFallback& ik)
{
    if (hmd_pose.poseIsValid
        && hmd_pose.deviceIsConnected
        && hmd_pose.result == vr::TrackingResult_Running_OK) {
        hmd_pose_seen_.store(true, std::memory_order_release);
    }

    for (uint8_t i = 0; i < kBodyRoleCount; ++i) {
        const auto role = static_cast<BodyRole>(i);
        if (BodyRoleToControllerType(role) == nullptr) continue; // HMD / hand roles not publishable
        MaybeActivate(role, ik);

        auto& dev = devices_[i];
        if (!dev || !dev->Activated()) continue;
        if (!enabled_[i]) continue;

        vr::DriverPose_t synth{};
        if (ik.Solve(role, hmd_pose, synth)) {
            dev->Publish(synth);
        }
    }
}

int VirtualTrackerManager::ActiveCount() const
{
    int n = 0;
    for (const auto& d : devices_) if (d && d->Activated()) ++n;
    return n;
}

} // namespace phantom
