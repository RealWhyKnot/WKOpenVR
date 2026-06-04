#pragma once

#include <algorithm>
#include <cstdint>

namespace phantom {

// Default timing constants for the dropout / synthesis / hand-back pipeline.
// The overlay surfaces these as advanced sliders; the driver loads the user's
// values via PhantomConfig and falls back to these on first launch or parse
// failure. Names match the PhantomConfig wire field names exactly so a
// reader chasing one source of truth to the other has no surprises.
struct DefaultTimings
{
	// Real pose silent for at least this long flips the tracker into the
	// dropout ladder. Below this, a one-frame skip is treated as normal jitter.
	static constexpr uint32_t kDropoutSilenceMs = 40;

	// Real -> synth fade duration. Short because the real signal is still
	// recent and we do not want to advertise the dropout into the avatar.
	static constexpr uint32_t kBlendOutMs = 80;

	// Synth -> real fade duration on recovery. Longer because the synthesised
	// pose may have drifted; an instant snap would be visible.
	static constexpr uint32_t kBlendInMs = 150;

	// Total time after dropout start during which dead reckoning is the
	// synthesis source. Past this, the ladder escalates to IK / ML / hold.
	static constexpr uint32_t kReckonHoldMs = 250;

	// Total time after dropout start during which any synthesis source is
	// published with ETrackingResult = Running_OK. Past this, the published
	// result flips to Running_OutOfRange so downstream consumers (VRChat,
	// Resonite) stop using the tracker in their IK chain.
	static constexpr uint32_t kSynthHoldMs = 2000;

	// Total time after dropout start before the phantom module stops
	// publishing entirely on the device. SteamVR treats absence as
	// disconnect after its own short timeout.
	static constexpr uint32_t kLostHoldMs = 5000;
};

// Smooth ease-in-out curve, clamped. Used for both real -> synth and
// synth -> real blends. weight is the elapsed fraction (0..1) through the
// blend window; returns the weight to assign to the *target* pose. Real ->
// synth uses target = synth; synth -> real uses target = real. Same curve
// keeps the perceptual feel symmetric.
inline double SmoothBlendWeight(double t)
{
	t = std::clamp(t, 0.0, 1.0);
	// smoothstep: 3t^2 - 2t^3. Cheap, monotonic, zero-derivative endpoints.
	return t * t * (3.0 - 2.0 * t);
}

} // namespace phantom
