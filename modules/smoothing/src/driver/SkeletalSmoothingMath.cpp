#include "SkeletalSmoothingMath.h"

#include <cmath>

namespace skeletal::math {

float Lerpf(float a, float b, float t)
{
	return a + (b - a) * t;
}

vr::HmdQuaternionf_t SlerpQuat(const vr::HmdQuaternionf_t& a, vr::HmdQuaternionf_t b, float t)
{
	float dot = a.x*b.x + a.y*b.y + a.z*b.z + a.w*b.w;
	if (dot < 0.0f) {
		b.x = -b.x; b.y = -b.y; b.z = -b.z; b.w = -b.w;
		dot = -dot;
	}

	if (dot > 0.9995f) {
		vr::HmdQuaternionf_t r{
			Lerpf(a.w, b.w, t),
			Lerpf(a.x, b.x, t),
			Lerpf(a.y, b.y, t),
			Lerpf(a.z, b.z, t)
		};
		float len = std::sqrt(r.w*r.w + r.x*r.x + r.y*r.y + r.z*r.z);
		if (len > 0.0f) {
			float inv = 1.0f / len;
			r.w *= inv; r.x *= inv; r.y *= inv; r.z *= inv;
		}
		return r;
	}

	float theta_0     = std::acos(dot);
	float sin_theta_0 = std::sin(theta_0);
	float theta       = theta_0 * t;
	float sin_theta   = std::sin(theta);
	float s1 = std::cos(theta) - dot * sin_theta / sin_theta_0;
	float s2 = sin_theta / sin_theta_0;
	return vr::HmdQuaternionf_t{
		s1*a.w + s2*b.w,
		s1*a.x + s2*b.x,
		s1*a.y + s2*b.y,
		s1*a.z + s2*b.z
	};
}

int FingerIndexForBone(uint32_t bone)
{
	if (bone >= 2  && bone <= 5)  return 0;
	if (bone >= 6  && bone <= 10) return 1;
	if (bone >= 11 && bone <= 15) return 2;
	if (bone >= 16 && bone <= 20) return 3;
	if (bone >= 21 && bone <= 25) return 4;
	return -1;
}

float SmoothnessToAlpha(uint8_t smoothness)
{
	float s = static_cast<float>(smoothness);
	if (s < 0.0f)   s = 0.0f;
	if (s > 100.0f) s = 100.0f;
	return 1.0f - (s / 100.0f) * 0.95f;
}

} // namespace skeletal::math
