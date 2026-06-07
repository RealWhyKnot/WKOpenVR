#pragma once

#include "RoleCatalog.h"

namespace phantom::ui {

inline bool ShouldAttemptDriverConnection(bool vrConnected)
{
	return vrConnected;
}

inline bool ShouldShowDriverError(bool vrConnected, bool hasDriverError)
{
	return vrConnected && hasDriverError;
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

} // namespace phantom::ui
