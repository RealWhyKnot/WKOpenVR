#pragma once

#include "Calibration.h"
#include "VRState.h"

#include <openvr.h>

namespace wkopenvr::spacecal::selection {

inline bool HasStandbyIdentity(const StandbyDevice& device)
{
	return !device.trackingSystem.empty() && !device.serial.empty();
}

inline const VRDevice* FindDeviceById(const VRState& state, int32_t id)
{
	if (id < 0) return nullptr;
	for (const auto& device : state.devices) {
		if (device.id == id) return &device;
	}
	return nullptr;
}

inline void SelectDevice(CalibrationContext& ctx, bool reference, const VRDevice& device)
{
	int32_t& id = reference ? ctx.referenceID : ctx.targetID;
	std::string& trackingSystem = reference
		? ctx.referenceTrackingSystem
		: ctx.targetTrackingSystem;
	StandbyDevice& standby = reference
		? ctx.referenceStandby
		: ctx.targetStandby;

	id = device.id;
	trackingSystem = device.trackingSystem;
	standby.trackingSystem = device.trackingSystem;
	standby.model = device.model;
	standby.serial = device.serial;
}

inline const VRDevice* FindHmdInSystem(const VRState& state, const std::string& trackingSystem)
{
	for (const auto& device : state.devices) {
		if (device.trackingSystem == trackingSystem
			&& device.deviceClass == vr::TrackedDeviceClass_HMD) {
			return &device;
		}
	}
	return nullptr;
}

inline const VRDevice* FindPreferredHeadsetTracker(
	const VRState& state,
	const std::string& referenceTrackingSystem,
	const std::string& targetTrackingSystem,
	const HeadMountConfig& headMount)
{
	const VRDevice* configured = nullptr;
	const VRDevice* singleCandidate = nullptr;
	int candidateCount = 0;

	for (const auto& device : state.devices) {
		if (device.trackingSystem != targetTrackingSystem) continue;
		if (device.trackingSystem == referenceTrackingSystem) continue;
		if (device.deviceClass != vr::TrackedDeviceClass_GenericTracker) continue;

		if (!headMount.trackerSerial.empty()
			&& device.serial == headMount.trackerSerial
			&& (headMount.trackerTrackingSystem.empty()
				|| device.trackingSystem == headMount.trackerTrackingSystem)) {
			configured = &device;
		}

		singleCandidate = &device;
		++candidateCount;
	}

	if (configured) return configured;
	return candidateCount == 1 ? singleCandidate : nullptr;
}

inline bool AutoSelectHeadsetTrackerPair(CalibrationContext& ctx, const VRState& state)
{
	if (ctx.state != CalibrationState::None) return false;
	if (ctx.referenceTrackingSystem.empty() || ctx.targetTrackingSystem.empty()) return false;
	if (ctx.referenceTrackingSystem == ctx.targetTrackingSystem) return false;

	bool changed = false;

	const VRDevice* selectedReference = FindDeviceById(state, ctx.referenceID);
	const bool referenceNeedsFreshPick =
		!HasStandbyIdentity(ctx.referenceStandby)
		&& (!selectedReference
			|| selectedReference->deviceClass != vr::TrackedDeviceClass_HMD);
	if (referenceNeedsFreshPick) {
		if (const VRDevice* hmd = FindHmdInSystem(state, ctx.referenceTrackingSystem)) {
			SelectDevice(ctx, true, *hmd);
			changed = true;
		}
	}

	selectedReference = FindDeviceById(state, ctx.referenceID);
	if (!selectedReference
		|| selectedReference->deviceClass != vr::TrackedDeviceClass_HMD) {
		return changed;
	}

	const VRDevice* selectedTarget = FindDeviceById(state, ctx.targetID);
	const bool targetNeedsFreshPick =
		!HasStandbyIdentity(ctx.targetStandby)
		&& (!selectedTarget
			|| selectedTarget->deviceClass != vr::TrackedDeviceClass_GenericTracker);
	if (targetNeedsFreshPick) {
		if (const VRDevice* tracker = FindPreferredHeadsetTracker(
				state,
				ctx.referenceTrackingSystem,
				ctx.targetTrackingSystem,
				ctx.headMount)) {
			SelectDevice(ctx, false, *tracker);
			changed = true;
		}
	}

	return changed;
}

} // namespace wkopenvr::spacecal::selection
