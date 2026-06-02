#pragma once

#include "BodyCompletionSolver.h"
#include "RoleCatalog.h"
#include "VirtualTrackerDevice.h"

#include <openvr_driver.h>

#include <array>
#include <atomic>
#include <chrono>
#include <memory>

namespace phantom {

// Owns up to one VirtualTrackerDevice per BodyRole. The overlay flips
// per-role enables via RequestSetPhantomVirtualEnabled; this class decides
// when it is safe to call TrackedDeviceAdded (HMD pose seen + post-Init
// settle delay; deferred-add gate mitigates openvr#1536-class regressions
// where adding a virtual device before physical tracker enumeration
// stabilises suppresses pose acquisition on later devices) and drives
// per-tick pose publishing from BodyCompletionSolver output.
//
// Phase 2 limitation: SteamVR does not support TrackedDeviceRemoved
// live for generic trackers; flipping a role's enable to false retracts
// the virtual device on the next vrserver restart, not in-process.
class VirtualTrackerManager
{
public:
    VirtualTrackerManager();

    // Called once when the driver module activates. Captures the init
    // timestamp used by the deferred-add gate.
    void OnDriverInit();

    // Overlay-driven per-role toggle. Stored locally; the actual
    // TrackedDeviceAdded happens lazily in MaybeActivate when the gate
    // conditions are met.
    void SetEnabled(BodyRole role, bool enabled);
    bool IsEnabled(BodyRole role) const;

    // Called every tick by PhantomModule (specifically when the HMD pose
    // updates). Lazily activates pending virtual devices and pushes
    // solver-derived poses on every already-activated device.
    void Tick(const vr::DriverPose_t& hmd_pose,
              const BodyCompletionResult& body,
              double min_confidence);

    // Diagnostic: how many virtual devices are currently activated.
    int ActiveCount() const;

private:
    void MaybeActivate(BodyRole role);

    std::array<std::unique_ptr<VirtualTrackerDevice>, kBodyRoleCount> devices_{};
    std::array<bool, kBodyRoleCount> enabled_{};

    std::atomic<bool> hmd_pose_seen_{false};
    std::chrono::steady_clock::time_point init_time_{};

    // Defer TrackedDeviceAdded for this long after driver init so SteamVR
    // has time to enumerate real devices first. The exact duration is a
    // tradeoff: too short and openvr#1536 regressions surface, too long
    // and the user waits visibly for body trackers after launch. 3 s
    // matches what SlimeVR and Amethyst settled on.
    static constexpr std::chrono::milliseconds kInitSettleDelay{3000};
};

} // namespace phantom
