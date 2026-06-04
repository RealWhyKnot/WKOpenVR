#pragma once

#include "Protocol.h"
#include "inputhealth/LearningRules.h"
#include "inputhealth/PathPolicy.h"

#include <algorithm>
#include <cmath>
#include <string>

namespace inputhealth {

inline float ClampScalar(float value, float lo, float hi)
{
	return std::max(lo, std::min(value, hi));
}

inline float ApplyScalarCompensationValue(uint8_t kind, const std::string& path, uint8_t scalar_type,
                                          uint8_t scalar_units, float raw_value, float learned_rest_offset,
                                          float learned_trigger_min, float learned_trigger_max,
                                          float learned_deadzone_radius, float partner_value, bool has_partner,
                                          float partner_rest_offset = 0.0f, bool has_partner_rest_offset = false)
{
	const PathFamily family = ClassifyPathFamily(path);
	if (!AllowsDriverCompensation(family) || !ScalarMetadataAllowsCompensation(kind, path, scalar_type, scalar_units)) {
		return raw_value;
	}

	if (IsTriggerRemapFamily(family)) {
		if (learned_trigger_min > 0.0f || learned_trigger_max > 0.0f) {
			const float max_value = learned_trigger_max > 0.0f ? learned_trigger_max : 1.0f;
			const float range = std::max(0.001f, max_value - learned_trigger_min);
			return ClampScalar((raw_value - learned_trigger_min) / range, 0.0f, 1.0f);
		}
		return ClampScalar(raw_value, 0.0f, 1.0f);
	}

	if (IsIdleFloorFamily(family)) {
		const float floor = ClampScalar(learned_rest_offset, 0.0f, 0.05f);
		return ClampScalar(raw_value - floor, 0.0f, 1.0f);
	}

	if (IsThumbstickAxisFamily(family)) {
		float value = raw_value - learned_rest_offset;
		if (learned_deadzone_radius > 0.0f) {
			float radial_partner = partner_value;
			if (has_partner && has_partner_rest_offset) {
				radial_partner = partner_value - partner_rest_offset;
			}
			const float radius =
			    has_partner ? std::sqrt(value * value + radial_partner * radial_partner) : std::fabs(value);
			if (radius < learned_deadzone_radius) value = 0.0f;
		}
		return ClampScalar(value, -1.0f, 1.0f);
	}

	return raw_value;
}

} // namespace inputhealth
