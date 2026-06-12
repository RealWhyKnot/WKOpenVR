#pragma once

#include "Calibration.h"
#include "TrackingStyle.h"

#include <string>

namespace wkopenvr::tracking_style_ui {

struct ActionInputs
{
	TrackingStyle style = TrackingStyle::Manual;
	CalibrationState state = CalibrationState::None;
	bool vrReady = true;
	bool validProfile = false;
	bool offsetCalibrated = false;
	bool relativePosCalibrated = false;
	bool offsetPreflightReady = true;
	bool targetMatches = true;
	std::string vrBlockedMessage;
	std::string offsetPreflightMessage;
};

inline bool ContinuousCalibrationIsRunning(CalibrationState state)
{
	return state == CalibrationState::Continuous || state == CalibrationState::ContinuousStandby;
}

inline bool HeadsetTrackerLockReady(const ActionInputs& in)
{
	return in.offsetCalibrated && in.relativePosCalibrated;
}

inline const char* PrimaryActionLabel(const ActionInputs& in)
{
	const bool continuousRunning = ContinuousCalibrationIsRunning(in.state);
	switch (in.style) {
		case TrackingStyle::Manual:
			return "Calibrate now";
		case TrackingStyle::Continuous:
			return continuousRunning ? "Restart continuous calibration" : "Start continuous calibration";
		case TrackingStyle::LockedWithRecovery:
			if (!continuousRunning) return "Start setup";
			if (!in.offsetCalibrated) return "Calibrate headset tracker";
			if (!in.relativePosCalibrated) return "Save relative pose";
			return "Re-calibrate headset tracker";
		case TrackingStyle::HardTrackerLock:
			if (!continuousRunning) {
				return HeadsetTrackerLockReady(in) ? "Hard tracker lock active" : "Start setup";
			}
			if (!in.offsetCalibrated) return "Calibrate headset tracker";
			if (!in.relativePosCalibrated) return "Save relative pose";
			return "Finish setup";
	}
	return "Calibrate now";
}

inline bool PrimaryActionEnabled(const ActionInputs& in, std::string* reason = nullptr)
{
	if (!in.vrReady) {
		if (reason) *reason = in.vrBlockedMessage;
		return false;
	}

	const bool continuousRunning = ContinuousCalibrationIsRunning(in.state);
	const bool headsetStyle = TrackingStyleUsesHeadsetSynthesis(in.style);
	if (headsetStyle && continuousRunning && !in.offsetCalibrated && !in.offsetPreflightReady) {
		if (reason) *reason = in.offsetPreflightMessage;
		return false;
	}
	if (headsetStyle && continuousRunning && in.offsetCalibrated && !in.relativePosCalibrated) {
		if (!in.validProfile) {
			if (reason) *reason = "Save or create a calibration profile before locking the headset tracker.";
			return false;
		}
		if (!in.targetMatches) {
			if (reason) *reason = "Restart setup with the selected headset tracker.";
			return false;
		}
	}
	if (in.style == TrackingStyle::HardTrackerLock && !continuousRunning && HeadsetTrackerLockReady(in)) {
		if (reason) *reason = "Hard tracker lock is active. Select another tracking style to change modes.";
		return false;
	}

	return true;
}

} // namespace wkopenvr::tracking_style_ui
