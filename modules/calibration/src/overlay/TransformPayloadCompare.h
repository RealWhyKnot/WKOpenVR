#pragma once

#include "Protocol.h"

#include <cmath>
#include <cstring>

// Equality test for the per-device transform payload the overlay sends to the
// driver, with a sub-perceptual deadband on the transform itself. Pure and
// header-only so it can be unit-tested (tests/test_transform_payload_compare.cpp)
// without the overlay's OpenVR/IPC dependencies.
//
// Why a deadband: continuous calibration updates its offset on every accepted
// candidate, so the exact byte-compare this replaced differed essentially every
// tick and forced a blocking IPC round-trip per device per tick (~13 sends/sec
// in a live session, almost all of it sub-millimetre solver jitter). The cache
// behind this compare stores the LAST SENT payload, so drift accumulates against
// a fixed anchor: the worst-case staleness of the driver's offset is one
// deadband, well below tracker noise, and a steadily drifting offset still
// reaches the driver within a deadband of the live value.
//
// Every non-transform field is compared EXACTLY -- enable, the update bits, the
// motion-gate `lerp` flag, the head-mount `quash`/`updateQuash` flags,
// prediction, recalibrateOnMovement, scale and the target system string -- so any
// meaningful state change propagates immediately; only the translation/rotation
// numbers have a tolerance.

namespace spacecal::apply {

// Translation tolerance, metres. 0.1 mm is below the noise floor of lighthouse
// / inside-out tracking, so a suppressed change is imperceptible by definition.
inline constexpr double kTransformNearEqualMeters = 1e-4;

// Rotation tolerance as 1 - |dot| of the two unit quaternions. Using |dot| makes
// q and -q (the same orientation) compare equal. 1 - |dot| ~= (theta/2)^2 / 2,
// so 2e-8 maps to ~0.02 deg.
inline constexpr double kTransformNearEqualRotDotEps = 2e-8;

inline bool TransformPayloadNearEqual(const protocol::SetDeviceTransform& a, const protocol::SetDeviceTransform& b)
{
	// Exact on everything except the transform components.
	if (a.openVRID != b.openVRID) return false;
	if (a.enabled != b.enabled) return false;
	if (a.updateTranslation != b.updateTranslation) return false;
	if (a.updateRotation != b.updateRotation) return false;
	if (a.updateScale != b.updateScale) return false;
	if (a.lerp != b.lerp) return false;
	if (a.quash != b.quash) return false;
	if (a.updateQuash != b.updateQuash) return false;
	if (a.predictionSmoothness != b.predictionSmoothness) return false;
	if (a.recalibrateOnMovement != b.recalibrateOnMovement) return false;
	if (a.scale != b.scale) return false;
	if (std::memcmp(a.target_system, b.target_system, sizeof a.target_system) != 0) return false;

	// Translation deadband (squared Euclidean distance vs squared tolerance).
	const double dx = a.translation.v[0] - b.translation.v[0];
	const double dy = a.translation.v[1] - b.translation.v[1];
	const double dz = a.translation.v[2] - b.translation.v[2];
	if ((dx * dx + dy * dy + dz * dz) > (kTransformNearEqualMeters * kTransformNearEqualMeters)) {
		return false;
	}

	// Rotation deadband.
	const double dot = a.rotation.w * b.rotation.w + a.rotation.x * b.rotation.x + a.rotation.y * b.rotation.y +
	                   a.rotation.z * b.rotation.z;
	if ((1.0 - std::fabs(dot)) > kTransformNearEqualRotDotEps) {
		return false;
	}

	return true;
}

} // namespace spacecal::apply
