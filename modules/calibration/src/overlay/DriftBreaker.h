#pragma once

// Pure helpers for the drift circuit-breaker (auto-freeze on MAD spike) toggle.
//
// In a bad Continuous session the relative-pose translation MAD climbed to
// 360-466 mm (normal steady-state is ~5-6 mm) as repeated Quest relocalizations
// dragged the solve off. The user's only recourse was to manually turn on
// headset locking, which froze the relative pose and stopped the solver chasing
// the garbage. This helper automates that intervention: when the live MAD blows
// past a multiple of the recent floor (or an absolute mm cap), the caller
// engages the same effective relative-pose lock the user would have, and
// releases it once the MAD restabilizes well under the trip level.
//
// The breaker is a one-way override layered on top of the existing AUTO-lock
// state machine -- it only forces lock ON, never writes the AUTO pending-flip
// queue, and releases through its own hysteresis so it cannot flap against the
// detector. All functions are pure: the caller passes the live MAD and floor in
// millimetres and the configured multiplier / cap.

namespace spacecal::drift_breaker {

// Default trip multiple over the recent MAD floor. The AUTO-lock leave
// threshold is ~3x the floor and panic-unlock is ~40 mm; 8x sits well above
// normal lock churn so the breaker only engages on genuine runaway drift.
constexpr double kDefaultMadMult = 8.0;

// Default absolute MAD ceiling in millimetres. The bad session sat at 360+ mm;
// 60 mm is far above any healthy steady-state yet well below the runaway range,
// and it still trips when the floor estimate itself is unreliable (the floor
// climbed to 100+ mm in the bad session, which would defeat a floor-only gate).
constexpr double kDefaultAbsCapMm = 60.0;

// Release happens at half the active trip level, giving a 2:1 hysteresis
// band so a MAD hovering near the trip point does not flap the freeze on and off.
constexpr double kReleaseHysteresisMult = 0.5;

// True when the live relative-pose MAD warrants freezing the calibration.
//
// Trips if either:
//   - floor gate: madFloorMm > 0 and kMult > 0 and translMadMm >= kMult*madFloorMm
//   - cap gate:   absCapMm   > 0 and translMadMm >= absCapMm
//
// With kMult <= 0 the floor gate is disabled; with absCapMm <= 0 the cap gate is
// disabled; with both disabled the breaker never freezes (a safe "off" config).
inline bool ShouldFreeze(double translMadMm, double madFloorMm, double kMult, double absCapMm)
{
	const bool floorTrip = (madFloorMm > 0.0 && kMult > 0.0) && (translMadMm >= kMult * madFloorMm);
	const bool capTrip = (absCapMm > 0.0) && (translMadMm >= absCapMm);
	return floorTrip || capTrip;
}

// True when an engaged freeze should be released because the MAD has dropped
// back under every enabled gate's hysteresis release level. When both gates are
// disabled, release immediately so stale runtime state cannot keep forcing lock.
inline bool ShouldRelease(double translMadMm, double madFloorMm, double kMult, double absCapMm)
{
	const bool floorGateEnabled = madFloorMm > 0.0 && kMult > 0.0;
	const bool capGateEnabled = absCapMm > 0.0;
	if (!floorGateEnabled && !capGateEnabled) return true;

	const bool belowFloorRelease = !floorGateEnabled || translMadMm < kReleaseHysteresisMult * kMult * madFloorMm;
	const bool belowCapRelease = !capGateEnabled || translMadMm < kReleaseHysteresisMult * absCapMm;
	return belowFloorRelease && belowCapRelease;
}

} // namespace spacecal::drift_breaker
