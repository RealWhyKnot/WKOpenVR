#pragma once

#include <cstring>

namespace spacecal::reject_reason {

inline bool Equals(const char* value, const char* expected)
{
	return value && expected && std::strcmp(value, expected) == 0;
}

inline bool NeedsMoreRotation(const char* reason)
{
	return Equals(reason, "rotation_no_deltas") || Equals(reason, "rotation_planar");
}

inline bool NeedsMoreTranslation(const char* reason)
{
	return Equals(reason, "translation_no_deltas") || Equals(reason, "translation_planar") ||
	       Equals(reason, "axis_variance_low");
}

inline bool IsMotionQualityGate(const char* reason)
{
	return NeedsMoreRotation(reason) || NeedsMoreTranslation(reason);
}

inline const char* UserHint(const char* reason)
{
	if (NeedsMoreRotation(reason)) {
		return "Move with more varied rotation before the next update can be trusted.";
	}
	if (NeedsMoreTranslation(reason)) {
		return "Move through more varied directions before the next update can be trusted.";
	}
	return "Continuous calibration is running and waiting for a usable update.";
}

} // namespace spacecal::reject_reason
