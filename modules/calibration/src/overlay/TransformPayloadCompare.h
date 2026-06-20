#pragma once

#include "Protocol.h"

#include <cmath>
#include <cstring>

// Equality tests for transform payloads the overlay sends to the driver, with
// a sub-perceptual deadband on the transform itself. Pure and header-only so
// it can be unit-tested (tests/test_transform_payload_compare.cpp) without the
// overlay's OpenVR/IPC dependencies.
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

inline bool TranslationNearEqual(const vr::HmdVector3d_t& a, const vr::HmdVector3d_t& b)
{
	const double dx = a.v[0] - b.v[0];
	const double dy = a.v[1] - b.v[1];
	const double dz = a.v[2] - b.v[2];
	return (dx * dx + dy * dy + dz * dz) <= (kTransformNearEqualMeters * kTransformNearEqualMeters);
}

inline bool RotationNearEqual(const vr::HmdQuaternion_t& a, const vr::HmdQuaternion_t& b)
{
	const double dot = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
	return (1.0 - std::fabs(dot)) <= kTransformNearEqualRotDotEps;
}

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
	if (!TranslationNearEqual(a.translation, b.translation)) return false;

	// Rotation deadband.
	if (!RotationNearEqual(a.rotation, b.rotation)) return false;

	return true;
}

inline bool FallbackPayloadNearEqual(const protocol::SetTrackingSystemFallback& a,
                                     const protocol::SetTrackingSystemFallback& b)
{
	// Exact on routing and policy fields; deadband only the transform.
	if (std::memcmp(a.system_name, b.system_name, sizeof a.system_name) != 0) return false;
	if (a.enabled != b.enabled) return false;
	if (a.predictionSmoothness != b.predictionSmoothness) return false;
	if (a.recalibrateOnMovement != b.recalibrateOnMovement) return false;
	if (a.scale != b.scale) return false;
	if (!TranslationNearEqual(a.translation, b.translation)) return false;
	if (!RotationNearEqual(a.rotation, b.rotation)) return false;

	return true;
}

} // namespace spacecal::apply
