#pragma once

#include "RoleCatalog.h"

#include <openvr_driver.h>

#include <array>
#include <cstdint>

namespace phantom {

// Legacy per-role rigid offset from the HMD frame (translation + rotation).
// The automatic body-prior path no longer writes these offsets, but the
// fallback stays available so older payloads remain harmless.
//
// This is intentionally simpler than full-chain IK: it produces a
// plausible-looking foot/waist/chest pose as long as the user's body
// orientation roughly tracks their head orientation (which is true for
// the vast majority of social-VR posture). Full chain IK (knees /
// elbows that bend independently of head yaw) is a Phase 2 follow-up;
// the IkFallback class fronts both and the upgrade does not require
// touching DropoutState or PhantomDriverModule.
struct TrackerOffset
{
	BodyRole role = BodyRole::None;

	// Offset of this tracker's position relative to the HMD's pose.
	double rel_position[3] = {0, 0, 0};

	// Rotation of this tracker relative to the HMD. Quaternion (w,x,y,z).
	// Identity = aligned with HMD.
	vr::HmdQuaternion_t rel_rotation = {1, 0, 0, 0};

	bool available = false;
};

// Legacy solver-side IK fallback. Consulted by PhantomDriverModule only if a
// compatible older payload populated an offset.
class IkFallback
{
public:
	IkFallback() = default;

	// Mutators called from the IPC dispatcher when a legacy offset arrives.
	// Thread-safe at the per-role granularity:
	// the table is small enough that copying the whole struct on read is
	// cheaper than a per-entry mutex.
	void SetOffset(BodyRole role, const double rel_pos[3], const vr::HmdQuaternion_t& rel_rot);
	void ClearOffset(BodyRole role);
	void ClearAll();

	bool HasOffset(BodyRole role) const;

	// True if at least one tracker has an offset on file. The
	// DropoutState ladder only transitions to SYNTH_IK when this is true;
	// otherwise it stays in SYNTH_RECKON (dead reckoning) and the
	// damping carries the avatar to rest.
	bool AnyOffsetAvailable() const;

	// Project the rigid offset for `role` onto the live HMD pose to
	// produce a synth tracker pose. Returns false if no offset is on file
	// for the role (caller falls back to dead reckoning).
	// hmd_pose is the pose the umbrella driver most recently observed
	// for the HMD device (after smoothing / calibration transforms).
	bool Solve(BodyRole role, const vr::DriverPose_t& hmd_pose, vr::DriverPose_t& out_pose) const;

private:
	std::array<TrackerOffset, kBodyRoleCount> offsets_{};
};

} // namespace phantom
