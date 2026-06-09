#pragma once

#include "Protocol.h"

#include <cstdint>

namespace facetracking {

inline constexpr const char* kExpressionNames[protocol::FACETRACKING_EXPRESSION_COUNT] = {
    "EyeLookOutLeft",
    "EyeLookInLeft",
    "EyeLookUpLeft",
    "EyeLookDownLeft",
    "EyeLookOutRight",
    "EyeLookInRight",
    "EyeLookUpRight",
    "EyeLookDownRight",
    "EyeWideLeft",
    "EyeWideRight",
    "EyeSquintLeft",
    "EyeSquintRight",
    "BrowLowererLeft",
    "BrowLowererRight",
    "BrowInnerUpLeft",
    "BrowInnerUpRight",
    "BrowOuterUpLeft",
    "BrowOuterUpRight",
    "BrowPinchLeft",
    "BrowPinchRight",
    "CheekPuffLeft",
    "CheekPuffRight",
    "CheekSuckLeft",
    "CheekSuckRight",
    "NoseSneerLeft",
    "NoseSneerRight",
    "JawOpen",
    "JawForward",
    "JawLeft",
    "JawRight",
    "LipSuckUpperLeft",
    "LipSuckUpperRight",
    "LipSuckLowerLeft",
    "LipSuckLowerRight",
    "LipFunnelUpperLeft",
    "LipFunnelUpperRight",
    "LipFunnelLowerLeft",
    "LipFunnelLowerRight",
    "LipPuckerUpperLeft",
    "LipPuckerUpperRight",
    "MouthClose",
    "MouthUpperLeft",
    "MouthUpperRight",
    "MouthLowerLeft",
    "MouthLowerRight",
    "MouthSmileLeft",
    "MouthSmileRight",
    "MouthSadLeft",
    "MouthSadRight",
    "MouthStretchLeft",
    "MouthStretchRight",
    "MouthDimpleLeft",
    "MouthDimpleRight",
    "MouthRaiserUpper",
    "MouthRaiserLower",
    "MouthPressLeft",
    "MouthPressRight",
    "MouthTightenerLeft",
    "MouthTightenerRight",
    "TongueOut",
    "TongueUp",
    "TongueDown",
    "TongueLeft",
};

static_assert(sizeof(kExpressionNames) / sizeof(kExpressionNames[0]) == protocol::FACETRACKING_EXPRESSION_COUNT,
              "kExpressionNames length must match FACETRACKING_EXPRESSION_COUNT");

inline const char* ExpressionName(uint32_t index)
{
	if (index >= protocol::FACETRACKING_EXPRESSION_COUNT) return "";
	return kExpressionNames[index];
}

} // namespace facetracking
