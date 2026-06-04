#pragma once

#include "Calibration.h"

inline const char* TrackingStyleLabel(TrackingStyle style)
{
	switch (style) {
		case TrackingStyle::Manual:
			return "Manual calibration";
		case TrackingStyle::Continuous:
			return "Continuous calibration";
		case TrackingStyle::LockedWithRecovery:
			return "Locked headset with recovery";
		case TrackingStyle::HardTrackerLock:
			return "Hard tracker lock";
	}
	return "Manual calibration";
}

inline const char* TrackingStyleSummary(TrackingStyle style)
{
	switch (style) {
		case TrackingStyle::Manual:
			return "No headset tracker. Calibrate once when tracking drifts.";
		case TrackingStyle::Continuous:
			return "Tracker assists calibration, but headset remains raw.";
		case TrackingStyle::LockedWithRecovery:
			return "Stable headset tracker drives the headset, with recovery enabled.";
		case TrackingStyle::HardTrackerLock:
			return "Stable headset tracker drives the headset with no raw fallback.";
	}
	return "";
}

inline bool TrackingStyleUsesHeadTracker(TrackingStyle style)
{
	return style != TrackingStyle::Manual;
}

inline bool TrackingStyleUsesHeadsetSynthesis(TrackingStyle style)
{
	return style == TrackingStyle::LockedWithRecovery || style == TrackingStyle::HardTrackerLock;
}

inline bool TrackingStyleRunsContinuous(TrackingStyle style)
{
	return style == TrackingStyle::Continuous || style == TrackingStyle::LockedWithRecovery;
}

inline bool TrackingStyleShowsBoundarySetup(TrackingStyle style)
{
	return style == TrackingStyle::LockedWithRecovery;
}

inline TrackingStyle TrackingStyleFromRaw(int raw, TrackingStyle fallback = TrackingStyle::Manual)
{
	if (raw < (int)TrackingStyle::Manual || raw > (int)TrackingStyle::HardTrackerLock) {
		return fallback;
	}
	return (TrackingStyle)raw;
}

inline CalibrationContext::LockMode ExplicitLockModeFromRaw(int raw)
{
	return raw == (int)CalibrationContext::LockMode::ON ? CalibrationContext::LockMode::ON
	                                                    : CalibrationContext::LockMode::OFF;
}

inline TrackingStyle InferTrackingStyleFromConfig(const CalibrationContext& ctx)
{
	if (ctx.headMount.mode == HeadMountMode::DriverSynth) {
		return ctx.headMount.allowRawHmdFallback ? TrackingStyle::LockedWithRecovery : TrackingStyle::HardTrackerLock;
	}
	if (ctx.state == CalibrationState::Continuous || ctx.state == CalibrationState::ContinuousStandby) {
		return TrackingStyle::Continuous;
	}
	return TrackingStyle::Manual;
}

inline void ApplyTrackingStylePreset(CalibrationContext& ctx, TrackingStyle style)
{
	ctx.trackingStyle = style;
	ctx.quashTargetInContinuous = true;
	ctx.headMount.hideTracker = true;

	switch (style) {
		case TrackingStyle::Manual:
			ctx.headMount.mode = HeadMountMode::Off;
			ctx.headMount.allowRawHmdFallback = true;
			ctx.lockRelativePositionMode = CalibrationContext::LockMode::OFF;
			break;
		case TrackingStyle::Continuous:
			ctx.headMount.mode = HeadMountMode::Off;
			ctx.headMount.allowRawHmdFallback = true;
			ctx.lockRelativePositionMode = CalibrationContext::LockMode::OFF;
			break;
		case TrackingStyle::LockedWithRecovery:
			ctx.headMount.mode = HeadMountMode::DriverSynth;
			ctx.headMount.allowRawHmdFallback = true;
			ctx.lockRelativePositionMode = CalibrationContext::LockMode::ON;
			break;
		case TrackingStyle::HardTrackerLock:
			ctx.headMount.mode = HeadMountMode::DriverSynth;
			ctx.headMount.allowRawHmdFallback = false;
			ctx.lockRelativePositionMode = CalibrationContext::LockMode::ON;
			break;
	}
}
