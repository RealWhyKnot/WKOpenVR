#pragma once

#include <cstdint>

#include <openvr_driver.h>

namespace skeletal::math {

float Lerpf(float a, float b, float t);
vr::HmdQuaternionf_t SlerpQuat(const vr::HmdQuaternionf_t& a, vr::HmdQuaternionf_t b, float t);
int FingerIndexForBone(uint32_t bone);
float SmoothnessToAlpha(uint8_t smoothness);

} // namespace skeletal::math
