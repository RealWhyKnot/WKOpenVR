#include "BlendController.h"

#include "BlendCurves.h"

#include <algorithm>
#include <cmath>

namespace phantom {

double BlendController::QuatDot(const vr::HmdQuaternion_t& a, const vr::HmdQuaternion_t& b)
{
	return a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
}

namespace {

void Slerp(const vr::HmdQuaternion_t& a, const vr::HmdQuaternion_t& b, double alpha, vr::HmdQuaternion_t& out)
{
	double dot = BlendController::QuatDot(a, b);
	// Take the shorter arc by flipping b if the dot is negative.
	vr::HmdQuaternion_t bAdj = b;
	if (dot < 0.0) {
		bAdj.w = -b.w;
		bAdj.x = -b.x;
		bAdj.y = -b.y;
		bAdj.z = -b.z;
		dot = -dot;
	}
	// Near-parallel: fall back to linear interpolation + renormalise to
	// avoid divide-by-near-zero in sin(theta).
	if (dot > 0.9995) {
		out.w = a.w + alpha * (bAdj.w - a.w);
		out.x = a.x + alpha * (bAdj.x - a.x);
		out.y = a.y + alpha * (bAdj.y - a.y);
		out.z = a.z + alpha * (bAdj.z - a.z);
	}
	else {
		const double theta = std::acos(std::clamp(dot, -1.0, 1.0));
		const double sinT = std::sin(theta);
		const double wa = std::sin((1.0 - alpha) * theta) / sinT;
		const double wb = std::sin(alpha * theta) / sinT;
		out.w = wa * a.w + wb * bAdj.w;
		out.x = wa * a.x + wb * bAdj.x;
		out.y = wa * a.y + wb * bAdj.y;
		out.z = wa * a.z + wb * bAdj.z;
	}
	const double n = std::sqrt(out.w * out.w + out.x * out.x + out.y * out.y + out.z * out.z);
	if (n > 1e-12) {
		out.w /= n;
		out.x /= n;
		out.y /= n;
		out.z /= n;
	}
	else {
		out.w = 1.0;
		out.x = out.y = out.z = 0.0;
	}
}

} // namespace

void BlendController::Lerp(const vr::DriverPose_t& a, const vr::DriverPose_t& b, double alpha, vr::DriverPose_t& out)
{
	alpha = std::clamp(alpha, 0.0, 1.0);
	const double s = SmoothBlendWeight(alpha);

	out = a;
	for (int i = 0; i < 3; ++i) {
		out.vecPosition[i] = a.vecPosition[i] * (1.0 - s) + b.vecPosition[i] * s;
		out.vecVelocity[i] = a.vecVelocity[i] * (1.0 - s) + b.vecVelocity[i] * s;
		out.vecAcceleration[i] = a.vecAcceleration[i] * (1.0 - s) + b.vecAcceleration[i] * s;
		out.vecAngularVelocity[i] = a.vecAngularVelocity[i] * (1.0 - s) + b.vecAngularVelocity[i] * s;
		out.vecAngularAcceleration[i] = a.vecAngularAcceleration[i] * (1.0 - s) + b.vecAngularAcceleration[i] * s;
	}
	Slerp(a.qRotation, b.qRotation, s, out.qRotation);
	out.poseTimeOffset = a.poseTimeOffset * (1.0 - s) + b.poseTimeOffset * s;
}

} // namespace phantom
