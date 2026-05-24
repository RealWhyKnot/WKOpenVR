#include "ServerTrackedDeviceProvider.h"
#include "IsometryTransform.h"
#include "Logging.h"
#include "SlewRateCap.h"

#include <algorithm>
#include <cmath>
#include <cstring>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace {

vr::HmdQuaternion_t convert(const Eigen::Quaterniond& q) {
	vr::HmdQuaternion_t result;
	result.w = q.w();
	result.x = q.x();
	result.y = q.y();
	result.z = q.z();
	return result;
}

IsoTransform toIsoWorldTransform(const vr::DriverPose_t& pose) {
	Eigen::Quaterniond rot(pose.qWorldFromDriverRotation.w, pose.qWorldFromDriverRotation.x, pose.qWorldFromDriverRotation.y, pose.qWorldFromDriverRotation.z);
	Eigen::Vector3d trans(pose.vecWorldFromDriverTranslation[0], pose.vecWorldFromDriverTranslation[1], pose.vecWorldFromDriverTranslation[2]);
	return IsoTransform(rot, trans);
}

} // namespace
/**
 * This function heuristically evaluates the amount of drift between the src and target playspace transforms,
 * evaluated centered on the `pose` device transform. This is then used to control the speed of realignment.
 */
ServerTrackedDeviceProvider::DeltaSize ServerTrackedDeviceProvider::GetTransformDeltaSize(
	DeltaSize prior_delta,
	const IsoTransform& deviceWorldPose,
	const IsoTransform& src,
	const IsoTransform& target
) const {
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

	if (trans_delta > alignmentSpeedParams.thr_trans_large) trans_level = DeltaSize::LARGE;
	else if (trans_delta > alignmentSpeedParams.thr_trans_small) trans_level = DeltaSize::SMALL;
	else trans_level = DeltaSize::TINY;

	if (rot_delta > alignmentSpeedParams.thr_rot_large) rot_level = DeltaSize::LARGE;
	else if (rot_delta > alignmentSpeedParams.thr_rot_small) rot_level = DeltaSize::SMALL;
	else rot_level = DeltaSize::TINY;

	if (trans_level == DeltaSize::TINY && rot_level == DeltaSize::TINY) return DeltaSize::TINY;
	else return std::max(prior_delta, std::max(trans_level, rot_level));
}

double ServerTrackedDeviceProvider::GetTransformRate(DeltaSize delta) const {
	switch (delta) {
	case DeltaSize::TINY: return alignmentSpeedParams.align_speed_tiny;
	case DeltaSize::SMALL: return alignmentSpeedParams.align_speed_small;
	default: return alignmentSpeedParams.align_speed_large;
	}
}

/**
 * Smoothly interpolates the device active transform towards the target transform.
 * When recalibrateOnMovement is enabled on the slot, the lerp progress is bounded
 * by an absolute mm/sec and deg/sec cap (alignmentSpeedParams.slew_* fields).
 * Stationary users see corrections converge at the imperceptible-drift threshold
 * (~0.5 mm/sec by default); when moving, the cap blends up to the much higher
 * moving rate so catch-up of accumulated drift happens quickly without visible
 * snap.
 *
 * 2026-05-24: replaced the prior regime-based still-floor (10/50/90 percent
 * by Tiny/Normal/Large classification) -- the 90 percent Large floor closed
 * ~85 percent of the gap per second when stationary, producing visible jumps
 * the moment the user moved after a long still stretch (user report:
 * "I have to wave a limb to get it to calibrate" gave way to "still very
 * snappy when I have it on"). The cap math lives in SlewRateCap.h; the
 * snap-bypass paths in SetDeviceTransform / first-fallback-activation that
 * directly assign `tf.transform = tf.targetTransform` are untouched, so
 * real tracking discontinuities (chi-square reanchor, Quest re-localization)
 * still snap as today.
 */
void ServerTrackedDeviceProvider::BlendTransform(DeviceTransform& device, const IsoTransform &deviceWorldPose) const {
	LARGE_INTEGER timestamp;
	QueryPerformanceCounter(&timestamp);

	// qpcFreq captured once in Init(); QPF is constant per boot so re-querying
	// here would be wasted work on the pose-update hot path. dt is the wall-
	// clock seconds since the prior call -- the slew cap below needs it
	// independently of the lerp factor so it can express its bound in physical
	// per-second units.
	const double dt = (timestamp.QuadPart - device.lastPoll.QuadPart) / (double)qpcFreq.QuadPart;
	device.lastPoll = timestamp;

	double lerp = dt * GetTransformRate(device.currentRate);

	if (device.recalibrateOnMovement) {
		if (!device.blendMotionInitialized) {
			// First frame since the flag was enabled: capture the reference pose
			// and skip blend progress this tick -- there's nothing to compute a
			// meaningful delta against yet, and we don't want a stale prior pose
			// (from before re-enable) to produce a giant phantom motion gate.
			device.lastBlendWorldPos = deviceWorldPose.translation;
			device.lastBlendWorldRot = deviceWorldPose.rotation;
			device.blendMotionInitialized = true;
			lerp = 0.0;
		} else {
			// Per-frame device motion in normalized units. kPosFullScale and
			// kRotFullScale are the per-frame deltas at which the gate is fully
			// open -- small typical-jitter motions produce partial gate, sustained
			// natural motion produces gate=1.
			constexpr double kPosFullScale = 0.005;   // 5 mm
			constexpr double kRotFullScale = 0.0175;  // ~1 deg in radians
			const double devPosDelta = (deviceWorldPose.translation - device.lastBlendWorldPos).norm();
			const double devRotDelta = deviceWorldPose.rotation.angularDistance(device.lastBlendWorldRot);
			const double motionGate = std::min(1.0,
				std::max(devPosDelta / kPosFullScale, devRotDelta / kRotFullScale));

			// Pending correction magnitude in metres / radians. The cap is
			// expressed in the same units so the divide produces a clean scale
			// factor; the prior code converted to mm + degrees only because the
			// regime classifier wanted those units, which is no longer relevant.
			const double pendingPosM =
				(device.targetTransform.translation - device.transform.translation).norm();
			const double pendingRotRad =
				device.targetTransform.rotation.angularDistance(device.transform.rotation);

			// Blend the stationary and moving caps by motionGate, then apply.
			// Motion (motionGate -> 1) raises the cap so accumulated drift catches
			// up quickly while the user moves; standing still drops the cap to
			// the stationary rate so the transform converges imperceptibly.
			const double capPos = spacecal::slew::BlendRate(
				alignmentSpeedParams.slew_stationary_pos_rate,
				alignmentSpeedParams.slew_moving_pos_rate,
				motionGate);
			const double capRot = spacecal::slew::BlendRate(
				alignmentSpeedParams.slew_stationary_rot_rate,
				alignmentSpeedParams.slew_moving_rot_rate,
				motionGate);

			lerp = spacecal::slew::ApplyCap(
				lerp, pendingPosM, pendingRotRad, capPos, capRot, dt);

			// Throttled heartbeat: only when actively slewing (above the
			// converged thresholds) AND at most once per ~5 s. Lets a session log
			// confirm the cap is binding without flooding when convergence has
			// finished and the slot is idle.
			constexpr double kSlewLogThresholdPosM = 0.001;     // 1 mm
			constexpr double kSlewLogThresholdRotRad = 0.001745; // ~0.1 deg
			if (pendingPosM > kSlewLogThresholdPosM || pendingRotRad > kSlewLogThresholdRotRad) {
				const double secSinceLastLog = device.lastSlewLogTime.QuadPart > 0
					? (timestamp.QuadPart - device.lastSlewLogTime.QuadPart) / (double)qpcFreq.QuadPart
					: 1e9;
				if (secSinceLastLog >= 5.0) {
					LOG("[blend][slew] pendPosMm=%.2f pendRotDeg=%.3f gate=%.2f capPosMmS=%.2f capRotDegS=%.3f effLerp=%.4f",
						pendingPosM * 1000.0,
						pendingRotRad * (180.0 / M_PI),
						motionGate,
						capPos * 1000.0,
						capRot * (180.0 / M_PI),
						lerp);
					device.lastSlewLogTime = timestamp;
				}
			}

			device.lastBlendWorldPos = deviceWorldPose.translation;
			device.lastBlendWorldRot = deviceWorldPose.rotation;
		}
	} else if (device.blendMotionInitialized) {
		// Flag was on but is now off -- reset so re-enabling later doesn't see a
		// stale prior pose.
		device.blendMotionInitialized = false;
	}

	if (lerp > 1.0)
		lerp = 1.0;
	if (lerp < 0 || isnan(lerp))
		lerp = 0;

	device.transform = device.transform.interpolateAround(lerp, device.targetTransform, deviceWorldPose.translation);
}

void ServerTrackedDeviceProvider::ApplyTransform(DeviceTransform& device, vr::DriverPose_t& devicePose) const {
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
		if (memcmp(systemFallbacks[i].system_name, name, len) == 0
			&& (len == protocol::MaxTrackingSystemNameLen || systemFallbacks[i].system_name[len] == '\0')) {
			return &systemFallbacks[i];
		}
	}
	return nullptr;
}

const ServerTrackedDeviceProvider::FallbackSlot* ServerTrackedDeviceProvider::FindFallbackSlot(const char* name, size_t len) const
{
	return const_cast<ServerTrackedDeviceProvider*>(this)->FindFallbackSlot(name, len);
}

ServerTrackedDeviceProvider::FallbackSlot* ServerTrackedDeviceProvider::AcquireFallbackSlot(const char* name, size_t len)
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
