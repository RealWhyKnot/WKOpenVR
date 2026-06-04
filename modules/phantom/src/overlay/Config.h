#pragma once

#include "BlendCurves.h"
#include "RoleCatalog.h"

#include <cstdint>
#include <string>
#include <unordered_map>

// One T-pose-captured rigid offset for a body role. Persisted under the
// role's BodyRoleToKey() name in phantom.txt.
struct PhantomRoleOffset
{
	bool calibrated = false;
	double rel_position_x = 0.0;
	double rel_position_y = 0.0;
	double rel_position_z = 0.0;
	double rel_rotation_w = 1.0;
	double rel_rotation_x = 0.0;
	double rel_rotation_y = 0.0;
	double rel_rotation_z = 0.0;
};

struct PhantomSolverCalibration
{
	bool calibrated = false;
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
};

// Persisted Phantom overlay state. Saved to
// %LocalAppDataLow%\WKOpenVR\profiles\phantom.txt as plain key=value lines,
// matching the smoothing.txt / inputhealth.txt convention so a user can
// inspect or hand-edit if needed.
struct PhantomConfig
{
	// Master switch. When false the driver hot-path fast-paths to passthrough
	// for every device (pose history still records so a later flip-on has
	// back-history available, but no synthesis is applied).
	bool master_enabled = false;

	// Per-tracker dropout opt-in, keyed by serial. A device absent from the
	// map (or mapped to false) does not get dropout bridging even when
	// master_enabled is true. Allows the user to opt in only the body
	// trackers and leave HMD / controllers on passthrough.
	std::unordered_map<std::string, bool> dropout_enabled;

	// Tunable timeout-ladder values, all in milliseconds. Defaults track
	// BlendCurves.h. A future overlay may surface fewer than all of these as
	// sliders; the config persists whatever the overlay sets and the driver
	// treats 0 as "use the compiled-in default" for graceful skew handling.
	uint32_t blend_out_ms = phantom::DefaultTimings::kBlendOutMs;
	uint32_t blend_in_ms = phantom::DefaultTimings::kBlendInMs;
	uint32_t reckon_hold_ms = phantom::DefaultTimings::kReckonHoldMs;
	uint32_t synth_hold_ms = phantom::DefaultTimings::kSynthHoldMs;
	uint32_t lost_hold_ms = phantom::DefaultTimings::kLostHoldMs;

	// Phase 1.5: per-physical-tracker body-role assignment (serial -> role).
	// Drives both the IK fallback (for dropouts past 250 ms) and, in Phase
	// 2, the absent-mode "do not invent a virtual tracker for a role a real
	// device already holds" check.
	std::unordered_map<std::string, phantom::BodyRole> device_role;

	// Phase 1.5: per-body-role HMD-relative offset captured during the
	// T-pose wizard. Keyed by BodyRole. Roles absent from the map are
	// treated as uncalibrated and the driver falls back to dead reckoning.
	std::unordered_map<phantom::BodyRole, PhantomRoleOffset> role_offset;

	// Phase 2: per-body-role absent-mode toggle. When true (and the role
	// has a solver pose above the configured confidence threshold), the
	// driver publishes a virtual GenericTracker for the role with the
	// vive_tracker_<role> controller type so supported apps pick it up
	// automatically as a body-bound tracker.
	std::unordered_map<phantom::BodyRole, bool> virtual_enabled;

	PhantomSolverCalibration solver;
};

// Load from disk. On any read / parse error the on-disk file is ignored and
// a default-constructed PhantomConfig is returned.
PhantomConfig LoadPhantomConfig();

// Save to disk. Best-effort: failures are silently swallowed and the next
// save retries. Driver values land via IPC regardless of persistence success.
void SavePhantomConfig(const PhantomConfig& cfg);
