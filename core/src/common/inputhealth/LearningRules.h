#pragma once

#include "Protocol.h"
#include "inputhealth/PathClassifier.h"
#include "inputhealth/PathPolicy.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>

namespace inputhealth {

// OpenVR EVRScalarType / EVRScalarUnits values, mirrored here to keep these
// helpers free of an openvr_driver.h dependency.
constexpr uint8_t kOpenVrScalarTypeAbsolute = 0;
constexpr uint8_t kOpenVrScalarTypeRelative = 1;
constexpr uint8_t kOpenVrScalarUnitsNormalizedOneSided = 0;
constexpr uint8_t kOpenVrScalarUnitsNormalizedTwoSided = 1;

constexpr float kStrictRestThreshold = 0.05f;
constexpr float kStableStickRestRadiusCap = 0.15f;
constexpr float kStableTriggerRestCap = 0.10f;
constexpr float kStableRestSpanMax = 0.010f;
constexpr uint64_t kStableRestWindowUs = 1000000ULL;

constexpr uint32_t kButtonBounceMinIntervalUs = 200;
constexpr uint32_t kButtonBounceMaxIntervalUs = 20000;
constexpr uint64_t kButtonBounceReadyTransitions = 3;

inline bool IsAbsoluteScalar(uint8_t scalar_type)
{
	return scalar_type == kOpenVrScalarTypeAbsolute;
}

inline bool IsTwoSidedScalar(uint8_t scalar_units)
{
	return scalar_units == kOpenVrScalarUnitsNormalizedTwoSided;
}

inline bool IsOneSidedScalar(uint8_t scalar_units)
{
	return scalar_units == kOpenVrScalarUnitsNormalizedOneSided;
}

inline bool ScalarMetadataAllowsCompensation(uint8_t kind, const std::string& path, uint8_t scalar_type,
                                             uint8_t scalar_units)
{
	if (!IsAbsoluteScalar(scalar_type)) return false;

	const PathFamily family = ClassifyPathFamily(path);
	if (IsTriggerRemapFamily(family) || IsIdleFloorFamily(family)) {
		return kind == protocol::InputHealthCompScalarSingle && IsOneSidedScalar(scalar_units);
	}

	if (IsThumbstickAxisFamily(family)) {
		return (kind == protocol::InputHealthCompStickX || kind == protocol::InputHealthCompStickY) &&
		       IsTwoSidedScalar(scalar_units);
	}

	return false;
}

inline bool ScalarMetadataAllowsLearning(PathFamily family, const std::string& path, uint8_t kind, uint8_t scalar_type,
                                         uint8_t scalar_units)
{
	if (!AllowsPersistentScalarLearning(family)) return false;
	return ScalarMetadataAllowsCompensation(kind, path, scalar_type, scalar_units);
}

inline bool ScalarMetadataAllowsLearning(PathClass path_class, const std::string& path, uint8_t kind,
                                         uint8_t scalar_type, uint8_t scalar_units)
{
	if (!IsCompensationPath(path_class)) return false;
	return ScalarMetadataAllowsLearning(ClassifyPathFamily(path), path, kind, scalar_type, scalar_units);
}

inline bool IsStrictStickRest(float x, float y, bool buttons_quiet)
{
	return buttons_quiet && std::fabs(x) < kStrictRestThreshold && std::fabs(y) < kStrictRestThreshold;
}

inline bool IsStableStickRestCandidate(float x, float y, bool buttons_quiet)
{
	return buttons_quiet &&
	       std::hypot(static_cast<double>(x), static_cast<double>(y)) <= static_cast<double>(kStableStickRestRadiusCap);
}

inline bool IsStrictTriggerRest(float value)
{
	return value < kStrictRestThreshold;
}

inline bool IsStableTriggerRestCandidate(float value, bool buttons_quiet)
{
	return buttons_quiet && value >= 0.0f && value <= kStableTriggerRestCap;
}

struct StableRestWindow
{
	bool active = false;
	uint64_t since_us = 0;
	float min_primary = 0.0f;
	float max_primary = 0.0f;
	float min_partner = 0.0f;
	float max_partner = 0.0f;
};

inline void ResetStableRestWindow(StableRestWindow& w)
{
	w = StableRestWindow{};
}

inline bool UpdateStableRestWindow(StableRestWindow& w, bool candidate, float primary, float partner, uint64_t now_us,
                                   float max_span = kStableRestSpanMax, uint64_t min_window_us = kStableRestWindowUs)
{
	if (!candidate) {
		ResetStableRestWindow(w);
		return false;
	}

	if (!w.active) {
		w.active = true;
		w.since_us = now_us;
		w.min_primary = w.max_primary = primary;
		w.min_partner = w.max_partner = partner;
		return false;
	}

	w.min_primary = std::min(w.min_primary, primary);
	w.max_primary = std::max(w.max_primary, primary);
	w.min_partner = std::min(w.min_partner, partner);
	w.max_partner = std::max(w.max_partner, partner);

	const float primary_span = w.max_primary - w.min_primary;
	const float partner_span = w.max_partner - w.min_partner;
	if (primary_span > max_span || partner_span > max_span) {
		w.active = true;
		w.since_us = now_us;
		w.min_primary = w.max_primary = primary;
		w.min_partner = w.max_partner = partner;
		return false;
	}

	return now_us >= w.since_us && now_us - w.since_us >= min_window_us;
}

inline bool IsLikelyButtonBounceInterval(uint64_t interval_us)
{
	return interval_us >= kButtonBounceMinIntervalUs && interval_us <= kButtonBounceMaxIntervalUs;
}

inline uint32_t DebounceFromBounceInterval(uint32_t interval_us)
{
	const uint32_t padded = interval_us + 1000U;
	return std::max<uint32_t>(1000U, std::min<uint32_t>(kButtonBounceMaxIntervalUs, padded));
}

inline bool IsSystemButtonPath(const std::string& path)
{
	return path.find("/input/system/") != std::string::npos;
}

} // namespace inputhealth
