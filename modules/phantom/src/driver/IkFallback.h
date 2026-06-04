#pragma once

#include "RoleCatalog.h"

#include <openvr_driver.h>

#include <array>
#include <cstdint>

namespace phantom {

// Per-(physical-tracker) calibration captured during the T-pose wizard.
// Stores the rigid offset from the HMD frame (translation + rotation) at
// the moment the user held T-pose. When the tracker drops out the IK
// fallback recomposes a synth pose by applying this offset to the
// HMD's current pose -- the "rigid attachment to head" approximation.
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

	// T-pose-captured offset of this tracker's position relative to the
	// HMD's pose at the same instant. Stored in HMD-local coordinates
	// (HMD-yaw-aligned, gravity-up frame).
	double rel_position[3] = {0, 0, 0};

	// T-pose-captured rotation of this tracker relative to the HMD's
	// rotation. Quaternion (w,x,y,z). Identity = aligned with HMD.
	vr::HmdQuaternion_t rel_rotation = {1, 0, 0, 0};

	// True once a T-pose capture has populated this slot. False slots are
	// ignored by the solver (it leaves the dead-reckoned pose in place).
	bool calibrated = false;
};

// Solver-side IK fallback. Configured by the overlay (T-pose wizard) via
// IPC; consulted by PhantomDriverModule when a tracker enters SYNTH_IK.
class IkFallback
{
public:
	IkFallback() = default;

	// Mutators called from the IPC dispatcher when the overlay pushes
	// a calibration update. Thread-safe at the per-role granularity:
	// the table is small enough that copying the whole struct on read is
	// cheaper than a per-entry mutex.
	void SetOffset(BodyRole role, const double rel_pos[3], const vr::HmdQuaternion_t& rel_rot);
	void ClearOffset(BodyRole role);
	void ClearAll();

	bool HasOffset(BodyRole role) const;

	// True if at least one tracker has a calibration on file. The
	// DropoutState ladder only transitions to SYNTH_IK when this is true;
	// otherwise it stays in SYNTH_RECKON (dead reckoning) and the
	// damping carries the avatar to rest.
	bool AnyCalibrated() const;

	// Project the rigid offset for `role` onto the live HMD pose to
	// produce a synth tracker pose. Returns false if no calibration is
	// on file for the role (caller falls back to dead reckoning).
	// hmd_pose is the pose the umbrella driver most recently observed
	// for the HMD device (after smoothing / calibration transforms).
	bool Solve(BodyRole role, const vr::DriverPose_t& hmd_pose, vr::DriverPose_t& out_pose) const;

private:
	std::array<TrackerOffset, kBodyRoleCount> offsets_{};
};

} // namespace phantom
