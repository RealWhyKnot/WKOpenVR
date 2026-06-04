#pragma once

#include "Calibration.h"
#include "HeadMountTargetBinding.h"

namespace wkopenvr::headmount {

struct OffsetCalibrationPreflight
{
	bool ready = false;
	const char* reason = "unknown";
	const char* message = "Offset calibration is not ready.";
};

inline const char* HeadMountModeLabel(HeadMountMode mode)
{
	switch (mode) {
		case HeadMountMode::Off:
			return "off";
		case HeadMountMode::AutoPaired:
			return "stabilize";
		case HeadMountMode::Corroborate:
			return "block_jumps";
		case HeadMountMode::DriverSynth:
			return "driver_synth";
		default:
			return "unknown";
	}
}

inline bool HeadMountModeUsesOffsetInContinuous(HeadMountMode mode)
{
	return mode >= HeadMountMode::AutoPaired;
}

inline OffsetCalibrationPreflight EvaluateOffsetCalibrationPreflight(const CalibrationContext& ctx)
{
	const HeadMountConfig& hm = ctx.headMount;

	if (!HasContinuousTargetIdentity(ctx) || hm.trackerSerial.empty()) {
		return {false, "no_continuous_target",
		        "Start continuous calibration with the headset-mounted tracker as the target first."};
	}
	if (!HeadMountMatchesContinuousTarget(ctx)) {
		return {false, "target_mismatch",
		        "The selected continuous target changed. Restart continuous calibration with the headset tracker."};
	}
	if (hm.deviceID < 0) {
		return {false, "tracker_not_resolved", "Waiting for the headset tracker to appear in SteamVR."};
	}
	if (ctx.state != CalibrationState::Continuous) {
		return {false, "continuous_not_running",
		        "Start continuous calibration and let it settle before solving the offset."};
	}
	if (!ctx.validProfile) {
		return {false, "profile_not_ready", "Waiting for a valid continuous calibration profile."};
	}
	if (!ctx.relativePosCalibrated) {
		return {false, "relative_pose_not_ready", "Waiting for continuous calibration to lock the relative pose."};
	}
	if (HeadMountModeUsesOffsetInContinuous(hm.mode)) {
		return {false, "head_mount_mode_active", "Turn the head-mounted tracker mode Off before solving this offset."};
	}

	return {true, "ready", "Ready to solve the headset tracker offset."};
}

} // namespace wkopenvr::headmount
