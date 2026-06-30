#pragma once

#include "BodyCompletionSolver.h"
#include "RoleCatalog.h"
#include "VirtualTrackerDevice.h"

#include <openvr_driver.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

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
	int EnabledCount() const;

	// Blocks virtual publishing for roles already claimed by a physical
	// tracker. This protects VRChat from duplicate body-role inputs.
	void SetRoleBlocked(BodyRole role, bool blocked);
	bool IsRoleBlocked(BodyRole role) const;

	void SetMasterEnabled(bool enabled);
	bool MasterEnabled() const;

	// Called every tick by PhantomModule (specifically when the HMD pose
	// updates). Lazily activates pending virtual devices and pushes
	// solver-derived poses on every already-activated device.
	void Tick(const vr::DriverPose_t& hmd_pose, const BodyCompletionResult& body, double min_confidence);

	// Drain the latest pose for each active virtual device into `out` as
	// (objectId, pose) pairs, so the umbrella can forward them via the original
	// TrackedDevicePoseUpdated outside the pose-hook lock. Publishing inline from
	// Tick re-enters the hook and self-deadlocks; this is the safe channel that
	// dropout bridging already uses. Call on the pose-hook thread after Tick.
	void CollectPoseUpdates(std::vector<std::pair<uint32_t, vr::DriverPose_t>>& out);

	// Diagnostic: how many virtual devices are currently activated.
	int ActiveCount() const;

private:
	void MaybeActivate(BodyRole role);

	std::array<std::unique_ptr<VirtualTrackerDevice>, kBodyRoleCount> devices_{};
	std::array<bool, kBodyRoleCount> enabled_{};
	std::array<bool, kBodyRoleCount> blocked_{};

	std::atomic<bool> master_enabled_{false};
	std::atomic<bool> hmd_pose_seen_{false};
	std::chrono::steady_clock::time_point init_time_{};
	std::chrono::steady_clock::time_point last_diag_log_{};
	uint64_t diag_ticks_ = 0;
	uint64_t diag_published_ = 0;
	uint64_t diag_skip_invalid_ = 0;
	uint64_t diag_skip_confidence_ = 0;

	// Defer TrackedDeviceAdded for this long after driver init so SteamVR
	// has time to enumerate real devices first. The exact duration is a
	// tradeoff: too short and openvr#1536 regressions surface, too long
	// and the user waits visibly for body trackers after launch. 3 s
	// matches what SlimeVR and Amethyst settled on.
	static constexpr std::chrono::milliseconds kInitSettleDelay{3000};
};

} // namespace phantom
