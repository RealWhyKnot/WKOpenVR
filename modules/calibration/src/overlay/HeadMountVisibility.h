#pragma once

#include "Calibration.h"

#include <cstdint>
#include <string>

namespace wkopenvr::headmount {

inline bool ShouldHideHeadMountTracker(const CalibrationContext& ctx,
	uint32_t openVrId,
	const std::string& serial,
	const std::string& trackingSystem)
{
	if (openVrId == vr::k_unTrackedDeviceIndex_Hmd) return false;

	const HeadMountConfig& hm = ctx.headMount;
	if (!hm.hideTracker) return false;
	if (hm.trackerSerial.empty() || serial.empty()) return false;
	if (serial != hm.trackerSerial) return false;
	if (!hm.trackerTrackingSystem.empty()
		&& !trackingSystem.empty()
		&& trackingSystem != hm.trackerTrackingSystem) {
		return false;
	}

	return true;
}

inline bool ShouldHideContinuousTarget(const CalibrationContext& ctx,
	uint32_t openVrId)
{
	if (openVrId == vr::k_unTrackedDeviceIndex_Hmd) return false;
	return ctx.state == CalibrationState::Continuous
		&& static_cast<int32_t>(openVrId) == ctx.targetID
		&& ctx.quashTargetInContinuous;
}

inline bool ShouldQuashPublishedTrackerPose(const CalibrationContext& ctx,
	uint32_t openVrId,
	const std::string& serial,
	const std::string& trackingSystem)
{
	return ShouldHideHeadMountTracker(ctx, openVrId, serial, trackingSystem)
		|| ShouldHideContinuousTarget(ctx, openVrId);
}

} // namespace wkopenvr::headmount
