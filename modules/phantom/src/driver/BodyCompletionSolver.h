#pragma once

#include "RoleCatalog.h"

#include <array>
#include <cstdint>

namespace phantom {

enum BodyCompletionSourceMask : uint16_t
{
	kBodySourceNone = 0,
	kBodySourceMeasured = 1u << 0,
	kBodySourceHmd = 1u << 1,
	kBodySourceController = 1u << 2,
	kBodySourceFloor = 1u << 3,
	kBodySourceContact = 1u << 4,
	kBodySourcePredicted = 1u << 5,
	kBodySourceHeld = 1u << 6,
};

enum class BodyCompletionMode : uint8_t
{
	None = 0,
	Measured,
	HmdRoot,
	ControllerIk,
	FloorContact,
	HeldContact,
	LowConfidence,
};

struct BodyCompletionPose
{
	double position[3] = {0.0, 0.0, 0.0};
	double rotation[4] = {1.0, 0.0, 0.0, 0.0}; // w, x, y, z
	double velocity[3] = {0.0, 0.0, 0.0};
};

struct BodyCompletionSensorPose
{
	BodyCompletionPose pose;
	bool valid = false;
	uint32_t age_ms = 0;
};

struct BodyCompletionPriors
{
	double floor_y_m = 0.0;
	double height_m = 1.70;
	double forward_yaw_rad = 0.0;
	double stance_width_m = 0.28;
	double shoulder_width_m = 0.38;
	double pelvis_width_m = 0.28;
	double upper_arm_m = 0.30;
	double lower_arm_m = 0.27;
	double upper_leg_m = 0.45;
	double lower_leg_m = 0.45;
	double virtual_min_confidence = 0.20;
	bool forward_estimated = false;
};

struct BodyCompletionInput
{
	double dt_seconds = 1.0 / 90.0;
	BodyCompletionPriors priors;
	BodyCompletionSensorPose hmd;
	BodyCompletionSensorPose left_controller;
	BodyCompletionSensorPose right_controller;
	std::array<BodyCompletionSensorPose, kBodyRoleCount> real_roles{};
	std::array<bool, kBodyRoleCount> enabled_roles{};
};

struct BodyCompletionRoleOutput
{
	BodyCompletionPose pose;
	float confidence = 0.0f;
	uint16_t source_mask = kBodySourceNone;
	BodyCompletionMode mode = BodyCompletionMode::None;
	uint32_t age_ms = 0;
	bool valid = false;
};

struct BodyCompletionResult
{
	std::array<BodyCompletionRoleOutput, kBodyRoleCount> roles{};
	float global_confidence = 0.0f;
};

struct BodyCompletionSolverRoleState
{
	BodyCompletionPose pose{};
	float confidence = 0.0f;
	bool valid = false;
};

struct BodyCompletionFootLockState
{
	double position[3] = {0.0, 0.0, 0.0};
	bool locked = false;
};

// Walking-gait tracker for the synthesized feet. While the headset sustains
// planar speed the feet take alternating discrete steps along the direction of
// travel (one foot planted in world space, the other swinging to a new target)
// instead of gliding with the pelvis. Purely dt-integrated so replays and unit
// tests stay deterministic.
struct BodyCompletionGaitState
{
	bool walking = false;
	double speed_ema_mps = 0.0;
	double dir[2] = {0.0, 1.0}; // planar unit direction of travel (x, z)
	double above_s = 0.0;
	double below_s = 0.0;
	int swing_foot = 0; // 0 = left, 1 = right
	double swing_t = 0.0;
	double swing_from[3] = {0.0, 0.0, 0.0};
	double swing_pos[3] = {0.0, 0.0, 0.0};
	double planted[2][3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
};

class BodyCompletionSolver
{
public:
	BodyCompletionSolver() = default;

	void Reset();
	BodyCompletionResult Solve(const BodyCompletionInput& input);

private:
	std::array<BodyCompletionSolverRoleState, kBodyRoleCount> previous_{};
	BodyCompletionFootLockState left_foot_lock_{};
	BodyCompletionFootLockState right_foot_lock_{};
	BodyCompletionGaitState gait_{};
};

const char* BodyCompletionModeLabel(BodyCompletionMode mode);

} // namespace phantom
