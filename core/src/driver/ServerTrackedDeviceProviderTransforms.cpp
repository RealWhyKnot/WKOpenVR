#include "ServerTrackedDeviceProvider.h"
#include "IsometryTransform.h"
#include "Logging.h"

#include <algorithm>
#include <cmath>
#include <cstring>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace {

vr::HmdQuaternion_t convert(const Eigen::Quaterniond& q)
{
	vr::HmdQuaternion_t result;
	result.w = q.w();
	result.x = q.x();
	result.y = q.y();
	result.z = q.z();
	return result;
}

IsoTransform toIsoWorldTransform(const vr::DriverPose_t& pose)
{
	Eigen::Quaterniond rot(pose.qWorldFromDriverRotation.w, pose.qWorldFromDriverRotation.x,
	                       pose.qWorldFromDriverRotation.y, pose.qWorldFromDriverRotation.z);
	Eigen::Vector3d trans(pose.vecWorldFromDriverTranslation[0], pose.vecWorldFromDriverTranslation[1],
	                      pose.vecWorldFromDriverTranslation[2]);
	return IsoTransform(rot, trans);
}

} // namespace
/**
 * This function heuristically evaluates the amount of drift between the src and target playspace transforms,
 * evaluated centered on the `pose` device transform. This is then used to control the speed of realignment.
 */
ServerTrackedDeviceProvider::DeltaSize
ServerTrackedDeviceProvider::GetTransformDeltaSize(DeltaSize prior_delta, const IsoTransform& deviceWorldPose,
                                                   const IsoTransform& src, const IsoTransform& target) const
{
	const auto src_pose = src * deviceWorldPose;
	const auto target_pose = target * deviceWorldPose;

	// Use .norm() (linear metres) here, NOT .squaredNorm(). The thresholds
	// (alignmentSpeedParams.thr_trans_*) are populated everywhere -- driver
	// Init(), overlay defaults, the user-tunable UI sliders -- as linear
	// distances in metres. Comparing squaredNorm against linear thresholds
	// silently squared the gate: a 20 mm threshold required a 141 mm offset
	// to trip, so currentRate was permanently TINY (= 0.05 lerp) for any
	// realistic continuous-cal correction. The runtime cost of the sqrt is
	// well under 1 us per call; this hot path runs at ~kHz, the math budget
	// is microseconds. .angularDistance() is already linear (radians).
	const auto trans_delta = (src_pose.translation - target_pose.translation).norm();
	const auto rot_delta = src_pose.rotation.angularDistance(target_pose.rotation);

	DeltaSize trans_level, rot_level;

	if (trans_delta > alignmentSpeedParams.thr_trans_large)
		trans_level = DeltaSize::LARGE;
	else if (trans_delta > alignmentSpeedParams.thr_trans_small)
		trans_level = DeltaSize::SMALL;
	else
		trans_level = DeltaSize::TINY;

	if (rot_delta > alignmentSpeedParams.thr_rot_large)
		rot_level = DeltaSize::LARGE;
	else if (rot_delta > alignmentSpeedParams.thr_rot_small)
		rot_level = DeltaSize::SMALL;
	else
		rot_level = DeltaSize::TINY;

	if (trans_level == DeltaSize::TINY && rot_level == DeltaSize::TINY)
		return DeltaSize::TINY;
	else
		return std::max(prior_delta, std::max(trans_level, rot_level));
}

double ServerTrackedDeviceProvider::GetTransformRate(DeltaSize delta) const
{
	switch (delta) {
		case DeltaSize::TINY:
			return alignmentSpeedParams.align_speed_tiny;
		case DeltaSize::SMALL:
			return alignmentSpeedParams.align_speed_small;
		default:
			return alignmentSpeedParams.align_speed_large;
	}
}

/**
 * Smoothly interpolates the device active transform toward the target transform
 * at a time-based rate (GetTransformRate by delta size), clamped to [0,1]. Real
 * tracking discontinuities (chi-square reanchor, Quest re-localization) still snap
 * via the direct-assign paths in SetDeviceTransform / first-fallback-activation.
 */
void ServerTrackedDeviceProvider::BlendTransform(DeviceTransform& device, const IsoTransform& deviceWorldPose) const
{
	LARGE_INTEGER timestamp;
	QueryPerformanceCounter(&timestamp);

	// qpcFreq captured once in Init(); QPF is constant per boot so re-querying
	// here would be wasted work on the pose-update hot path. dt is wall-clock
	// seconds since the prior call.
	const double dt = (timestamp.QuadPart - device.lastPoll.QuadPart) / (double)qpcFreq.QuadPart;
	device.lastPoll = timestamp;

	double lerp = dt * GetTransformRate(device.currentRate);

	if (lerp > 1.0) lerp = 1.0;
	if (lerp < 0 || isnan(lerp)) lerp = 0;

	device.transform = device.transform.interpolateAround(lerp, device.targetTransform, deviceWorldPose.translation);
}

void ServerTrackedDeviceProvider::ApplyTransform(DeviceTransform& device, vr::DriverPose_t& devicePose) const
{
	auto deviceWorldTransform = toIsoWorldTransform(devicePose);
	deviceWorldTransform = device.transform * deviceWorldTransform;
	devicePose.vecWorldFromDriverTranslation[0] = deviceWorldTransform.translation(0);
	devicePose.vecWorldFromDriverTranslation[1] = deviceWorldTransform.translation(1);
	devicePose.vecWorldFromDriverTranslation[2] = deviceWorldTransform.translation(2);
	devicePose.qWorldFromDriverRotation = convert(deviceWorldTransform.rotation);
}

ServerTrackedDeviceProvider::FallbackSlot* ServerTrackedDeviceProvider::FindFallbackSlot(const char* name, size_t len)
{
	if (len == 0 || len > protocol::MaxTrackingSystemNameLen) return nullptr;
	for (size_t i = 0; i < MaxFallbackSlots; ++i) {
		if (!systemFallbacks[i].occupied) continue;
		// Compare the full buffer length so a shorter prefix can't accidentally
		// match a longer occupant. The buffer is NUL-padded after assignment so
		// `len` bytes followed by a sentinel NUL is sufficient to distinguish.
		if (memcmp(systemFallbacks[i].system_name, name, len) == 0 &&
		    (len == protocol::MaxTrackingSystemNameLen || systemFallbacks[i].system_name[len] == '\0')) {
			return &systemFallbacks[i];
		}
	}
	return nullptr;
}

const ServerTrackedDeviceProvider::FallbackSlot* ServerTrackedDeviceProvider::FindFallbackSlot(const char* name,
                                                                                               size_t len) const
{
	return const_cast<ServerTrackedDeviceProvider*>(this)->FindFallbackSlot(name, len);
}

ServerTrackedDeviceProvider::FallbackSlot* ServerTrackedDeviceProvider::AcquireFallbackSlot(const char* name,
                                                                                            size_t len)
{
	if (len == 0 || len > protocol::MaxTrackingSystemNameLen) return nullptr;
	if (auto* existing = FindFallbackSlot(name, len)) return existing;
	for (size_t i = 0; i < MaxFallbackSlots; ++i) {
		if (!systemFallbacks[i].occupied) {
			memset(systemFallbacks[i].system_name, 0, sizeof systemFallbacks[i].system_name);
			memcpy(systemFallbacks[i].system_name, name, len);
			systemFallbacks[i].occupied = true;
			systemFallbacks[i].tf = FallbackTransform{};
			return &systemFallbacks[i];
		}
	}
	return nullptr;
}
