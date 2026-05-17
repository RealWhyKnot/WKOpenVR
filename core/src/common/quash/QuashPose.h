#pragma once

#include <openvr_driver.h>

namespace openvr_pair::common::quash {

// Mutate `pose` for a tracker hidden by SpaceCalibrator's "Hide tracker
// (during cal)" toggle. Keeps the device connected to SteamVR (no
// disconnect storm), freezes the visible pose at the last calibrated
// value (so the user never sees raw uncalibrated vendor pose leaking
// through as a ghost device at the wrong position), and marks the
// result as Calibrating_OutOfRange. SteamVR's standard behavior for
// OutOfRange dims the device in its UI without removing it, and apps
// like VRChat keep their bindings.
//
// When `lastGoodValid` is false (first frame after a device appears
// in a quashed state, with no calibrated pose cached yet), the
// incoming pose flows through unchanged except for the flag adjustments
// below. Subsequent quashed frames then freeze on the cached value as
// soon as one non-quashed frame populates it.
inline void ApplyQuashToPose(vr::DriverPose_t& pose,
                             const vr::DriverPose_t& lastGood,
                             bool lastGoodValid)
{
    if (lastGoodValid) {
        pose = lastGood;
    }
    pose.deviceIsConnected = true;
    pose.poseIsValid       = true;
    pose.result            = vr::TrackingResult_Calibrating_OutOfRange;
}

} // namespace openvr_pair::common::quash
