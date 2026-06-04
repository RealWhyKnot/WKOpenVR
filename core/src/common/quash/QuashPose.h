#pragma once

#include <openvr_driver.h>

namespace openvr_pair::common::quash {

// Hide-offset magnitudes (meters). The tracker's real motion is preserved;
// only its world position is translated by this vector before publishing to
// SteamVR. Picked far enough from any conceivable play space that the
// rendered model can never overlap the user. Tracker stays Running_OK so
// downstream consumers (VRChat, phantom dropout bridging, TrackerLiveness)
// see continuous valid motion -- no timeout, no liveness fire.
constexpr double kQuashOffsetX = 10000.0;
constexpr double kQuashOffsetZ = 10000.0;
constexpr double kQuashOffsetY = 0.0;

// Translate a tracker's pose by a fixed vector so the rendered model sits
// well outside the user's play space, while leaving everything else (rotation,
// velocity, validity, tracking result) intact. Cheaper than the prior
// "replace pose with frozen sentinel at Y=-1000" design and avoids tripping
// downstream timeout / dropout gates that watch for stalled motion.
inline void ApplyQuashToPose(vr::DriverPose_t& pose)
{
	pose.vecPosition[0] += kQuashOffsetX;
	pose.vecPosition[1] += kQuashOffsetY;
	pose.vecPosition[2] += kQuashOffsetZ;
}

} // namespace openvr_pair::common::quash
