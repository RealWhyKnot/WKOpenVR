#pragma once

#include "PhantomInferenceShmem.h"
#include "RoleCatalog.h"

#include <openvr_driver.h>

namespace phantom {

// Driver-side view of the sidecar's output shmem. Opens lazily on first
// successful access, reads the latest output frame with a seqlock-style
// epoch check, and exposes per-role poses + confidence to the
// DropoutState SYNTH_ML branch.
//
// The bridge tolerates a missing / stale OUT segment: if the sidecar
// is absent, IsReady() returns false and the driver falls through to
// the IK fallback. The bridge does not write into the OUT segment.
class SidecarBridge
{
public:
	SidecarBridge() = default;

	// Idempotent. Tries to open the OUT shmem; returns true once opened.
	bool TryOpen();
	void Close();
	bool IsReady() const { return ready_; }

	// Latest global confidence reported by the sidecar (0..1). Returns 0
	// if the bridge is not ready or no output has arrived yet.
	float GlobalConfidence() const;

	// Per-role pose. Returns false if (a) the bridge is not ready,
	// (b) the role's output slot is not flagged valid, or (c) the
	// global confidence is below `min_global`. The driver passes
	// `min_global` from PhantomConfig so the threshold is user-tunable.
	bool FetchPose(BodyRole role, float min_global, vr::DriverPose_t& out_pose) const;

private:
	bool ready_ = false;
	mutable PhantomInferenceOutShmem shmem_;
};

} // namespace phantom
