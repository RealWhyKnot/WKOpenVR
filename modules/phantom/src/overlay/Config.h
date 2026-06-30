#pragma once

#include "BlendCurves.h"
#include "RoleCatalog.h"
#include "TrackerModelCatalog.h"

#include <cstdint>
#include <string>
#include <unordered_map>

struct PhantomSolverSettings
{
	double virtual_min_confidence = 0.20;
};

// Persisted Phantom overlay state. Saved to
// %LocalAppDataLow%\WKOpenVR\profiles\phantom.txt as plain key=value lines,
// matching the smoothing.txt / inputhealth.txt convention so a user can
// inspect or hand-edit if needed.
struct PhantomConfig
{
	// Dropout bridge master switch. When false the driver hot-path fast-paths
	// to passthrough for every real device (pose history still records so a
	// later flip-on has back-history available, but no dropout synthesis is
	// applied). Absent-mode virtual trackers are controlled by virtual_enabled.
	bool master_enabled = false;

	// Per-tracker dropout opt-in, keyed by serial. A device absent from the
	// map (or mapped to false) does not get dropout bridging even when
	// the bridge master is true. Allows the user to opt in only the body
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

	// Per-physical-tracker body-role assignment (serial -> role). The driver
	// uses these roles as measured body anchors and blocks duplicate virtual
	// devices for roles that a real tracker already holds.
	std::unordered_map<std::string, phantom::BodyRole> device_role;

	// Origin of each device_role entry: true when the user hand-picked it in the
	// role dropdown, false when it came from automatic detection (Accept or
	// auto-save). Lets the UI badge manual vs automatic and keeps auto-save from
	// overwriting a hand-picked role. Keyed by serial, same as device_role.
	std::unordered_map<std::string, bool> role_manual;

	// When true, a high-confidence passive role detection from the driver is
	// saved as the persistent device_role automatically (so the mapping sticks
	// across sessions without the user pressing Accept). The driver already
	// applies confident detections live; this only controls persistence.
	bool auto_accept_roles = true;

	// When true, the driver snap-calibrates body roles automatically once the
	// user stands still with at least one unassigned tracker (zero-touch). A
	// manual Snap button is always available regardless.
	bool auto_snap = true;

	// Per-body-role absent-mode toggle. When true (and the role
	// has a solver pose above the configured confidence threshold), the
	// driver publishes a virtual GenericTracker for the role with the
	// vive_tracker_<role> controller type so supported apps pick it up
	// automatically as a body-bound tracker.
	std::unordered_map<phantom::BodyRole, bool> virtual_enabled;

	// SteamVR render model for the absent-mode virtual trackers. Cosmetic only;
	// the controller type still drives app role binding. Defaults to Vive Tracker
	// 3.0. Sent to the driver in PhantomConfig.render_model.
	phantom::TrackerModel tracker_model = phantom::TrackerModel::ViveTracker3;

	PhantomSolverSettings solver;
};

// Load from disk. On any read / parse error the on-disk file is ignored and
// a default-constructed PhantomConfig is returned.
PhantomConfig LoadPhantomConfig();

// Save to disk. Best-effort: failures are silently swallowed and the next
// save retries. Driver values land via IPC regardless of persistence success.
void SavePhantomConfig(const PhantomConfig& cfg);
