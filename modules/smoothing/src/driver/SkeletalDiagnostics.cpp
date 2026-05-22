#include "SkeletalDiagnostics.h"

#include <cmath>

namespace skeletal::diagnostics {

BoneDeltaStats MeasureBoneDeltaStats(
	const vr::VRBoneTransform_t* current,
	const vr::VRBoneTransform_t* previous,
	uint32_t count)
{
	BoneDeltaStats stats{};
	if (!current || !previous) return stats;

	for (uint32_t i = 0; i < count; ++i) {
		const auto& a = current[i];
		const auto& b = previous[i];
		const float dx = a.position.v[0] - b.position.v[0];
		const float dy = a.position.v[1] - b.position.v[1];
		const float dz = a.position.v[2] - b.position.v[2];
		const float d  = std::sqrt(dx*dx + dy*dy + dz*dz);
		if (d > stats.maxPosDelta) {
			stats.maxPosDelta = d;
			stats.maxPosBone = static_cast<int>(i);
		}
		const float dot = a.orientation.w * b.orientation.w
			+ a.orientation.x * b.orientation.x
			+ a.orientation.y * b.orientation.y
			+ a.orientation.z * b.orientation.z;
		const float absDot = dot < 0.0f ? -dot : dot;
		if (absDot < stats.minQuatDot) stats.minQuatDot = absDot;
	}

	return stats;
}

} // namespace skeletal::diagnostics
