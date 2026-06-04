#pragma once

#include <cstdint>

#include <openvr_driver.h>

namespace skeletal::diagnostics {

struct BoneDeltaStats
{
	float maxPosDelta = 0.0f;
	int maxPosBone = -1;
	float minQuatDot = 1.0f;
};

BoneDeltaStats MeasureBoneDeltaStats(const vr::VRBoneTransform_t* current, const vr::VRBoneTransform_t* previous,
                                     uint32_t count);

} // namespace skeletal::diagnostics
