#pragma once

#include <openvr_driver.h>

#include <cstdint>
#include <string>

#include "inputhealth/EWMARollingMin.h"
#include "inputhealth/PageHinkley.h"
#include "inputhealth/PolarHistogram.h"
#include "inputhealth/WelfordAccumulator.h"

// Per-component statistics state. One of these lives in the driver-side
// InputHealth subsystem per VRInputComponentHandle_t the driver has seen
// through Create{Boolean,Scalar}Component. The detour bodies update the
// stats on the per-tick path; the background worker thread reads them
// (eventually behind a snapshot publish; not yet wired) to surface
// detection categories to the overlay.
//
// All fields default to "no observations yet". Reset is provided so the
// wizard's "start fresh" flow can wipe history per device serial.

namespace inputhealth {

// Axis role for paired 2D inputs (analog sticks). When two scalar
// components share the same path stem with X/Y suffixes, the X-side owns
// the polar histogram and the Y-side delegates polar updates to its
// partner. A stick that publishes its axes individually but is not paired
// (some custom drivers) ends up unpaired and just runs Welford + PH per
// axis -- no perimeter envelope tracking, but rest drift still works.
enum class AxisRole : uint8_t
{
	Unpaired = 0,
	StickX = 1,
	StickY = 2,
};

struct ComponentStats
{
	// Component path string, captured at CreateBoolean / CreateScalar time.
	// Stored per-handle so log lines and snapshot publishes can identify
	// the input by the same path strings the SteamVR Input bindings use
	// ("/input/joystick/x", "/input/trigger/value", etc.).
	std::string path;
	bool is_scalar = false;
	bool is_boolean = false;
	uint8_t scalar_type = 0;  // vr::EVRScalarType for scalar components
	uint8_t scalar_units = 0; // vr::EVRScalarUnits for scalar components

	// Property container the driver passed at Create{Boolean,Scalar}Component
	// time. Stable for the lifetime of the underlying tracked device. Stored
	// so HandleResetInputHealthStats can walk this map and resolve each entry
	// to its owning device's serial number on demand without taking a hot-path
	// query inside the input-update detour.
	vr::PropertyContainerHandle_t container_handle = vr::k_ulInvalidPropertyContainer;

	// Lazily resolved FNV-1a 64-bit hash of Prop_SerialNumber_String for the
	// owning device. Zero means "not yet resolved" -- the reset path resolves
	// it on demand via vr::VRProperties() and caches the result here.
	uint64_t device_serial_hash = 0;

	// Hot-path retry throttle for device_serial_hash. Some drivers attach
	// Prop_SerialNumber_String to the property container after the matching
	// Create{Boolean,Scalar}Component call has already returned, so the
	// Register helpers see an empty serial and leave device_serial_hash at
	// zero. The Update detours retry the resolve at most once per second
	// per component until it succeeds, then never again.
	uint64_t last_serial_resolve_attempt_us = 0;
	bool serial_resolution_logged = false;

	// Latched first-update-on-this-handle log so the deploy-validation step
	// has a clean grep-able signal that hooks fire end-to-end. Reset by
	// Stage 1A's Init clears the whole map; per-handle reset comes through
	// HandleResetInputHealthStats.
	bool first_update_logged = false;

	// =========================================================
	// Scalar-only stats. Unused (zero-sized at runtime) for booleans.
	// =========================================================

	// Streaming mean / variance over every value seen. The mean tracks the
	// long-running rest centroid for an axis at idle, the long-running
	// peak distribution for a trigger, etc.
	WelfordState welford;

	// Observed scalar range. This is intentionally a cheap raw min/max over
	// all samples, not a verdict: the overlay uses it to tell whether an
	// active trigger/stick exercise has covered enough range to be useful.
	bool scalar_range_initialized = false;
	float observed_min = 0.0f;
	float observed_max = 0.0f;

	// Two-sided PH on the value, with EWMA reference. For sticks: detects
	// rest centroid migration. For triggers: a one-sided variant is more
	// appropriate (only upward drift of rest matters), but the same struct
	// is reused with one_sided_positive=true via the params at update time.
	PageHinkleyState ph_drift;

	// EWMA-decayed rolling minimum. Only meaningful for triggers: tracks
	// the lowest rest-period value the trigger has reached, decayed slowly
	// toward the running mean so a stuck-up trigger eventually stops being
	// hidden by a single brief return-to-zero.
	EWMARollingMinState rest_min;

	// Most recent sample + its timestamp. The X-side of a paired axis pair
	// reads its partner's last_value to assemble the (x, y) tuple for the
	// polar histogram update. The Y-side just records its sample here.
	float last_value = 0.0f;
	uint64_t last_update_us = 0;

	// =========================================================
	// Pairing for 2D analog sticks.
	// =========================================================

	AxisRole axis_role = AxisRole::Unpaired;
	vr::VRInputComponentHandle_t partner_handle = 0;

	// Polar histogram only owned by AxisRole::StickX. Kept zero-initialised
	// for other roles so sizeof(ComponentStats) stays uniform; the field is
	// cheap (a few KiB) and avoiding a heap allocation keeps the per-handle
	// state cache-friendly.
	PolarHistogramState polar;

	// =========================================================
	// Boolean-only stats.
	// =========================================================

	// Total observed press transitions. Useful as a staleness signal: an
	// input no UpdateBoolean has ever fired for is either not exposed by
	// the driver or never pressed; either way the wizard skips it.
	uint64_t press_count = 0;

	// Latest boolean state, for transition detection.
	bool last_boolean = false;
	bool pending_state = false;
	uint64_t last_committed_us = 0;

	// Raw boolean transition timing. Used to learn short mechanical bounce
	// without waiting for hundreds of ordinary presses.
	uint64_t last_raw_transition_us = 0;
	uint64_t bounce_transition_count = 0;
	uint32_t bounce_max_interval_us = 0;
};

// Reset everything except the path / role / partner pairing. Used by
// HandleResetInputHealthStats(reset_passive=1) to wipe accumulated history
// without forgetting the topology mapping the wizard already discovered.
inline void ComponentStatsResetPassive(ComponentStats& s)
{
	WelfordReset(s.welford);
	PageHinkleyReset(s.ph_drift);
	EWMARollingMinReset(s.rest_min);
	PolarHistogramReset(s.polar);
	s.scalar_range_initialized = false;
	s.observed_min = 0.0f;
	s.observed_max = 0.0f;
	s.last_value = 0.0f;
	s.last_update_us = 0;
	s.press_count = 0;
	s.last_boolean = false;
	s.pending_state = false;
	s.last_committed_us = 0;
	s.last_raw_transition_us = 0;
	s.bounce_transition_count = 0;
	s.bounce_max_interval_us = 0;
	s.first_update_logged = false;
}

// Extract the X/Y axis pairing role from a component path. Returns one of:
//   AxisRole::StickX    if the path ends in "/x" (case-insensitive)
//   AxisRole::StickY    if the path ends in "/y" (case-insensitive)
//   AxisRole::Unpaired  otherwise
// `out_stem` is filled with the path with the trailing "/x" or "/y" stripped
// so callers can match X/Y handles into pairs by stem equality.
inline AxisRole ClassifyAxisRole(const std::string& path, std::string& out_stem)
{
	out_stem = path;
	if (path.size() < 2) return AxisRole::Unpaired;
	const char a = path[path.size() - 2];
	const char b = path[path.size() - 1];
	if (a == '/' && (b == 'x' || b == 'X')) {
		out_stem = path.substr(0, path.size() - 2);
		return AxisRole::StickX;
	}
	if (a == '/' && (b == 'y' || b == 'Y')) {
		out_stem = path.substr(0, path.size() - 2);
		return AxisRole::StickY;
	}
	return AxisRole::Unpaired;
}

} // namespace inputhealth
