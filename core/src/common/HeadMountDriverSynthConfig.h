#pragma once

#include <algorithm>

namespace wkopenvr::headmount {

constexpr int kDriverSynthStaleLimitMsDefault = 80;
// Hold the last good tracker-synth pose for this long before fading to the raw
// headset. Longer is gentler: the raw headset moves with inside-out
// relocalization, so a longer hold keeps a brief tracker dropout from snapping
// the locked view. The per-profile slider can raise this further.
constexpr int kDriverSynthGraceHoldMsDefault = 1000;
constexpr int kDriverSynthBlendToFallbackMsDefault = 500;
constexpr int kDriverSynthStableBeforeSynthMsDefault = 350;
constexpr int kDriverSynthBlendToSynthMsDefault = 500;

constexpr int kDriverSynthStaleLimitMsMin = 10;
constexpr int kDriverSynthStaleLimitMsMax = 250;
constexpr int kDriverSynthTransitionMsMin = 0;
constexpr int kDriverSynthTransitionMsMax = 5000;

struct DriverSynthTimingConfig
{
	int staleLimitMs = kDriverSynthStaleLimitMsDefault;
	int graceHoldMs = kDriverSynthGraceHoldMsDefault;
	int blendToFallbackMs = kDriverSynthBlendToFallbackMsDefault;
	int stableBeforeSynthMs = kDriverSynthStableBeforeSynthMsDefault;
	int blendToSynthMs = kDriverSynthBlendToSynthMsDefault;
};

inline int ClampDriverSynthStaleLimitMs(int value)
{
	return std::clamp(value, kDriverSynthStaleLimitMsMin, kDriverSynthStaleLimitMsMax);
}

inline int ClampDriverSynthTransitionMs(int value)
{
	return std::clamp(value, kDriverSynthTransitionMsMin, kDriverSynthTransitionMsMax);
}

inline DriverSynthTimingConfig ClampDriverSynthTimingConfig(DriverSynthTimingConfig cfg)
{
	cfg.staleLimitMs = ClampDriverSynthStaleLimitMs(cfg.staleLimitMs);
	cfg.graceHoldMs = ClampDriverSynthTransitionMs(cfg.graceHoldMs);
	cfg.blendToFallbackMs = ClampDriverSynthTransitionMs(cfg.blendToFallbackMs);
	cfg.stableBeforeSynthMs = ClampDriverSynthTransitionMs(cfg.stableBeforeSynthMs);
	cfg.blendToSynthMs = ClampDriverSynthTransitionMs(cfg.blendToSynthMs);
	return cfg;
}

inline bool DriverSynthTimingIsDefault(const DriverSynthTimingConfig& cfg)
{
	const DriverSynthTimingConfig def{};
	return cfg.staleLimitMs == def.staleLimitMs && cfg.graceHoldMs == def.graceHoldMs &&
	       cfg.blendToFallbackMs == def.blendToFallbackMs && cfg.stableBeforeSynthMs == def.stableBeforeSynthMs &&
	       cfg.blendToSynthMs == def.blendToSynthMs;
}

} // namespace wkopenvr::headmount
