#pragma once

#include <cstdint>

// Runtime feature detection. The driver looks for marker files in its own
// resources directory at Init() and only wires up the matching subsystems.
// Each consumer overlay's installer drops the appropriate flag:
//
//   resources/enable_calibration.flag    -- WKOpenVR-SpaceCalibrator
//   resources/enable_smoothing.flag      -- WKOpenVR-Smoothing
//   resources/enable_inputhealth.flag    -- WKOpenVR-InputHealth
//   resources/enable_facetracking.flag   -- WKOpenVR-VRCFT
//   resources/enable_oscrouter.flag      -- WKOpenVR-OSCRouter
//   resources/enable_captions.flag       -- WKOpenVR-Captions
//   resources/enable_phantom.flag        -- WKOpenVR-Phantom
//
// Any subset (including the empty subset) may be present. With no flags the
// driver loads but stays inert (no hooks installed, no pipes opened, no
// shmem segment) so SteamVR's auto-load doesn't break for users who
// installed the shared driver without any consumer.
//
// Facetracking and captions always imply OSC Router at runtime because both
// publish through the centralized OSC path.

namespace pairdriver {

constexpr uint32_t kFeatureCalibration  = 1u << 0;
constexpr uint32_t kFeatureSmoothing    = 1u << 1;
constexpr uint32_t kFeatureInputHealth  = 1u << 2;
constexpr uint32_t kFeatureFaceTracking = 1u << 3;
constexpr uint32_t kFeatureOscRouter    = 1u << 4;
constexpr uint32_t kFeatureCaptions     = 1u << 5;
constexpr uint32_t kFeaturePhantom      = 1u << 6;

constexpr uint32_t ComposeFeatureFlags(
	bool calibration,
	bool smoothing,
	bool inputHealth,
	bool faceTracking,
	bool oscRouter,
	bool captions,
	bool phantom)
{
	uint32_t flags = 0;
	if (calibration)  flags |= kFeatureCalibration;
	if (smoothing)    flags |= kFeatureSmoothing;
	if (inputHealth)  flags |= kFeatureInputHealth;
	if (faceTracking) flags |= kFeatureFaceTracking;
	if (oscRouter || faceTracking || captions) flags |= kFeatureOscRouter;
	if (captions)     flags |= kFeatureCaptions;
	if (phantom)      flags |= kFeaturePhantom;
	return flags;
}

// Returns the bitwise OR of detected feature flags. Logs the path it scanned
// and the result to the driver log so install issues are easy to diagnose.
uint32_t DetectFeatureFlags();

} // namespace pairdriver
