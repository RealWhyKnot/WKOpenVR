#pragma once

#include "BlendCurves.h"
#include "PhantomTypes.h"
#include "RoleCatalog.h"

#include <cstdint>

namespace phantom::ui {

inline bool ShouldAttemptDriverConnection(bool vrConnected)
{
	return vrConnected;
}

inline bool ShouldShowDriverError(bool vrConnected, bool hasDriverError)
{
	return vrConnected && hasDriverError;
}

// Display tone for diagnostics cells. Kept ImGui-free (no StatusTone / Theme
// dependency) so this header stays usable from the unit-test target, which puts
// modules/phantom/src/overlay on the include path but not core/src/overlay. The
// overlay draw code maps PhantomTone onto the shared ui::StatusTone + palette.
enum class PhantomTone
{
	Ok,
	Pending,
	Warn,
	Error,
	Info,
	Idle,
};

// Per-tracker dropout-ladder state -> display tone. The progression walks green
// (real) -> amber (a blend in flight) -> orange (sustained synthetic or flagged
// out-of-range) -> red (no longer publishing), so a row's color tracks how
// degraded its pose source is.
inline PhantomTone TrackerStateTone(phantom::TrackerState s)
{
	switch (s) {
		case phantom::TrackerState::REAL:
			return PhantomTone::Ok;
		case phantom::TrackerState::BLEND_OUT:
		case phantom::TrackerState::BLEND_IN:
			return PhantomTone::Pending;
		case phantom::TrackerState::SYNTH_RECKON:
		case phantom::TrackerState::SYNTH_IK:
		case phantom::TrackerState::SYNTH_ML:
		case phantom::TrackerState::OUT_OF_RANGE:
			return PhantomTone::Warn;
		case phantom::TrackerState::LOST:
			return PhantomTone::Error;
	}
	return PhantomTone::Idle;
}

// Body-completion solver mode -> display tone. Mode values mirror the raw
// uint8_t the driver publishes (see SolverModeLabel in PhantomPlugin.cpp); the
// BodyCompletionMode enum itself lives in a driver header off this include path,
// so we switch the raw code: 0 none -> Idle, 1 measured -> Ok, 2..5
// inferred/contact -> Warn, 6 low_confidence -> Error.
inline PhantomTone SolverModeTone(uint8_t mode)
{
	switch (mode) {
		case 0:
			return PhantomTone::Idle;
		case 1:
			return PhantomTone::Ok;
		case 2:
		case 3:
		case 4:
		case 5:
			return PhantomTone::Warn;
		case 6:
			return PhantomTone::Error;
		default:
			return PhantomTone::Idle;
	}
}

enum class VirtualRoleTier
{
	Safer,
	Beta,
	Experimental,
};

inline VirtualRoleTier GetVirtualRoleTier(phantom::BodyRole role)
{
	switch (role) {
		case phantom::BodyRole::Waist:
		case phantom::BodyRole::Chest:
			return VirtualRoleTier::Safer;
		case phantom::BodyRole::LeftKnee:
		case phantom::BodyRole::RightKnee:
		case phantom::BodyRole::LeftElbow:
		case phantom::BodyRole::RightElbow:
			return VirtualRoleTier::Beta;
		case phantom::BodyRole::LeftFoot:
		case phantom::BodyRole::RightFoot:
			return VirtualRoleTier::Experimental;
		default:
			return VirtualRoleTier::Experimental;
	}
}

inline const char* VirtualRoleTierLabel(VirtualRoleTier tier)
{
	switch (tier) {
		case VirtualRoleTier::Safer:
			return "Safer";
		case VirtualRoleTier::Beta:
			return "Beta";
		case VirtualRoleTier::Experimental:
			return "Experimental";
	}
	return "Experimental";
}

inline const char* VirtualRoleTierHelp(VirtualRoleTier tier)
{
	switch (tier) {
		case VirtualRoleTier::Safer:
			return "Usually safer than legs, but still requires calibration and confidence gates.";
		case VirtualRoleTier::Beta:
			return "Best with real waist/feet/controller anchors. Disable it if avatar IK gets worse.";
		case VirtualRoleTier::Experimental:
			return "High visual risk when wrong. Use only after previewing calibration and confidence.";
	}
	return "Use only after previewing calibration and confidence.";
}

struct VirtualRoleReadiness
{
	bool canEnable = false;
	const char* reason = nullptr;
};

inline VirtualRoleReadiness EvaluateVirtualRoleReadiness(bool solverCalibrated, bool physicalRoleAssigned)
{
	if (physicalRoleAssigned) {
		return {false, "A physical tracker is already assigned to this role."};
	}
	if (!solverCalibrated) {
		return {false, "Capture neutral standing before enabling estimated trackers."};
	}
	return {true, nullptr};
}

enum class DropoutTimingPreset
{
	Conservative,
	Balanced,
	Extended,
	Custom,
};

struct DropoutTimingValues
{
	uint32_t blend_out_ms = 0;
	uint32_t blend_in_ms = 0;
	uint32_t reckon_hold_ms = 0;
	uint32_t synth_hold_ms = 0;
	uint32_t lost_hold_ms = 0;
};

inline DropoutTimingValues ValuesForDropoutTimingPreset(DropoutTimingPreset preset)
{
	switch (preset) {
		case DropoutTimingPreset::Conservative:
			return {/*blend_out_ms=*/60,
			        /*blend_in_ms=*/120,
			        /*reckon_hold_ms=*/150,
			        /*synth_hold_ms=*/750,
			        /*lost_hold_ms=*/1500};
		case DropoutTimingPreset::Balanced:
			return {/*blend_out_ms=*/phantom::DefaultTimings::kBlendOutMs,
			        /*blend_in_ms=*/phantom::DefaultTimings::kBlendInMs,
			        /*reckon_hold_ms=*/phantom::DefaultTimings::kReckonHoldMs,
			        /*synth_hold_ms=*/phantom::DefaultTimings::kSynthHoldMs,
			        /*lost_hold_ms=*/phantom::DefaultTimings::kLostHoldMs};
		case DropoutTimingPreset::Extended:
			return {/*blend_out_ms=*/120,
			        /*blend_in_ms=*/250,
			        /*reckon_hold_ms=*/350,
			        /*synth_hold_ms=*/3500,
			        /*lost_hold_ms=*/8000};
		case DropoutTimingPreset::Custom:
			break;
	}
	return ValuesForDropoutTimingPreset(DropoutTimingPreset::Balanced);
}

inline bool DropoutTimingEquals(const DropoutTimingValues& lhs, const DropoutTimingValues& rhs)
{
	return lhs.blend_out_ms == rhs.blend_out_ms && lhs.blend_in_ms == rhs.blend_in_ms &&
	       lhs.reckon_hold_ms == rhs.reckon_hold_ms && lhs.synth_hold_ms == rhs.synth_hold_ms &&
	       lhs.lost_hold_ms == rhs.lost_hold_ms;
}

inline DropoutTimingPreset ClassifyDropoutTiming(const DropoutTimingValues& values)
{
	const DropoutTimingPreset presets[] = {
	    DropoutTimingPreset::Conservative,
	    DropoutTimingPreset::Balanced,
	    DropoutTimingPreset::Extended,
	};
	for (DropoutTimingPreset preset : presets) {
		if (DropoutTimingEquals(values, ValuesForDropoutTimingPreset(preset))) return preset;
	}
	return DropoutTimingPreset::Custom;
}

inline const char* DropoutTimingPresetLabel(DropoutTimingPreset preset)
{
	switch (preset) {
		case DropoutTimingPreset::Conservative:
			return "Conservative";
		case DropoutTimingPreset::Balanced:
			return "Balanced";
		case DropoutTimingPreset::Extended:
			return "Extended";
		case DropoutTimingPreset::Custom:
			return "Custom";
	}
	return "Custom";
}

inline const char* DropoutTimingPresetHelp(DropoutTimingPreset preset)
{
	switch (preset) {
		case DropoutTimingPreset::Conservative:
			return "Shortest bridge. Best when a wrong synthetic pose is worse than a dropped tracker.";
		case DropoutTimingPreset::Balanced:
			return "Default bridge length. Keeps brief tracking hiccups hidden and drops stale poses quickly.";
		case DropoutTimingPreset::Extended:
			return "Longer bridge for noisy rooms. Use only if the synthetic pose remains believable.";
		case DropoutTimingPreset::Custom:
			return "Manual timing values are active.";
	}
	return "Manual timing values are active.";
}

} // namespace phantom::ui
