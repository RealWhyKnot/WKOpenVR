#pragma once

#include "Calibration.h"
#include "SnapSuppression.h" // EffectiveHeadMountMode

namespace wkopenvr::headmount {

// A witness puck is "present" when the head-mount tracker is bound to a
// resolved OpenVR device. Per-tick pose validity is checked separately at each
// corroboration read site; presence alone is enough to enable the passive
// Corroborate role (an invalid pose this tick yields headTrackerDelta < 0,
// which the downstream classifiers treat as "no corroboration", never as a
// destructive trigger).
inline bool WitnessPresent(const CalibrationContext& ctx)
{
	return ctx.headMount.deviceID >= 0 && (uint32_t)ctx.headMount.deviceID < vr::k_unMaxTrackedDeviceCount;
}

// Effective head-mount mode for corroboration/recovery decisions: promotes to
// at least Corroborate when a witness puck is present, without mutating the
// persisted (style-derived) config mode. Use this -- not ctx.headMount.mode --
// anywhere corroboration, the AUTO Lock witness gate, or snap classification is
// decided, so the witness works in Continuous/Manual styles too.
inline HeadMountMode EffectiveHeadMountMode(const CalibrationContext& ctx)
{
	return spacecal::snap_suppression::EffectiveHeadMountMode(ctx.headMount.mode, WitnessPresent(ctx));
}

inline bool IsContinuousHeadMountBindingState(CalibrationState state)
{
	return state == CalibrationState::Continuous || state == CalibrationState::ContinuousStandby;
}

inline bool HasContinuousTargetIdentity(const CalibrationContext& ctx)
{
	return !ctx.targetStandby.serial.empty() && !ctx.targetStandby.trackingSystem.empty();
}

inline bool HeadMountMatchesContinuousTarget(const CalibrationContext& ctx)
{
	return HasContinuousTargetIdentity(ctx) && ctx.headMount.trackerSerial == ctx.targetStandby.serial &&
	       ctx.headMount.trackerTrackingSystem == ctx.targetStandby.trackingSystem;
}

inline bool BindHeadMountToContinuousTarget(CalibrationContext& ctx)
{
	// Run in every calibration state. Mirroring targetStandby + targetID into
	// the head-mount config is pure data motion; gating it on Continuous*
	// left hm.deviceID stuck at -1 while the user sat on the Headset tab in
	// state=None, which is why the status line read "Tracker not reporting a
	// valid pose." even when SteamVR was tracking the device fine.
	if (!HasContinuousTargetIdentity(ctx)) {
		return false;
	}

	bool changed = false;
	HeadMountConfig& hm = ctx.headMount;
	const bool identityChanged =
	    hm.trackerSerial != ctx.targetStandby.serial || hm.trackerTrackingSystem != ctx.targetStandby.trackingSystem;

	if (hm.trackerSerial != ctx.targetStandby.serial) {
		hm.trackerSerial = ctx.targetStandby.serial;
		changed = true;
	}
	if (hm.trackerModel != ctx.targetStandby.model) {
		hm.trackerModel = ctx.targetStandby.model;
		changed = true;
	}
	if (hm.trackerTrackingSystem != ctx.targetStandby.trackingSystem) {
		hm.trackerTrackingSystem = ctx.targetStandby.trackingSystem;
		changed = true;
	}

	const int32_t targetDeviceId =
	    (ctx.targetID >= 0 && ctx.targetID < (int32_t)vr::k_unMaxTrackedDeviceCount) ? ctx.targetID : -1;
	if (hm.deviceID != targetDeviceId) {
		hm.deviceID = targetDeviceId;
		changed = true;
	}

	if (identityChanged) {
		if (hm.offsetCalibrated) {
			hm.offsetCalibrated = false;
			changed = true;
		}
		hm.offsetWitnessAutoCaptured = false;
		hm.headFromTracker = Eigen::AffineCompact3d::Identity();
		ctx.NoteHeadMountOffsetChanged();
	}

	return changed;
}

} // namespace wkopenvr::headmount
