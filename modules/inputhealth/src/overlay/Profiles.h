#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

// Per-device-serial profile persistence.
//
// Stored as `%LocalAppDataLow%\WKOpenVR\profiles\<serial>.json`,
// one file per device serial. Each file holds the wizard's "start fresh"
// baseline (factory-norm references), the user's per-category opt-in
// toggles, and the most recently observed health classification. Profiles
// are written when the wizard completes a step or when a passive detection
// crosses a threshold; they are read at startup to seed the diagnostics
// view before the driver has published its first snapshot.
//
// Only fields needed by the diagnostics tab and learned compensation pipeline
// are stored at this stage.

struct LearnedPathRecord
{
	std::string path;
	std::string kind;
	uint64_t sample_count = 0;
	bool ready = false;
	double learned_rest_offset = 0.0;
	double learned_stddev = 0.0;
	double learned_trigger_min = 0.0;
	double learned_trigger_max = 0.0;
	double learned_deadzone_radius = 0.0;
	uint32_t learned_debounce_us = 0;
	uint64_t last_updated_unix = 0;
	uint32_t drift_shift_resets = 0;
};

inline bool LearnedPathMaterialEqual(const LearnedPathRecord& a, const LearnedPathRecord& b)
{
	return a.path == b.path && a.kind == b.kind && a.ready == b.ready &&
	       a.learned_rest_offset == b.learned_rest_offset && a.learned_stddev == b.learned_stddev &&
	       a.learned_trigger_min == b.learned_trigger_min && a.learned_trigger_max == b.learned_trigger_max &&
	       a.learned_deadzone_radius == b.learned_deadzone_radius && a.learned_debounce_us == b.learned_debounce_us &&
	       a.last_updated_unix == b.last_updated_unix && a.drift_shift_resets == b.drift_shift_resets;
}

struct DeviceProfile
{
	std::string serial;       // empty if the profile is from a deleted device
	uint64_t serial_hash = 0; // FNV-1a 64 of `serial`
	std::string display_name; // last-known controller model name; UI hint only

	// Preferences. Defaults match the driver-side InputHealthConfig
	// defaults so a freshly-created profile does not surprise the user.
	bool enable_diagnostics_only = true;
	bool enable_rest_recenter = true;
	bool enable_trigger_remap = false;
	bool corrections_enabled = true;

	// Set once the rest-recenter default-flip migration has run for this profile
	// (see Decode in Profiles.cpp). Profiles saved before the default was flipped
	// on lack this marker, which triggers a one-time force-on of
	// enable_rest_recenter so existing users pick up the new default.
	bool rest_recenter_migrated = false;

	std::vector<LearnedPathRecord> learned_paths;
};

struct ProfileIoStats
{
	uint64_t attempted_saves = 0;
	uint64_t skipped_unchanged = 0;
	uint64_t actual_writes = 0;
	uint64_t failed_writes = 0;
	std::string last_save_reason;
};

class ProfileStore
{
public:
	// Load every existing profile from disk. Idempotent; a second call
	// re-scans the profiles directory and merges new entries.
	void LoadAll();

	// Persist `profile` to disk. Creates the profiles directory if
	// missing. Returns true on success; on failure the in-memory copy is
	// retained so the UI can show "unsaved" state instead of dropping the
	// edit silently.
	bool Save(const DeviceProfile& profile, const char* reason = nullptr);

	// Return a profile by serial-hash. Creates a default-initialized
	// profile if none exists, so the diagnostics tab can call this once
	// per visible device without checking for existence.
	DeviceProfile& GetOrCreate(uint64_t serial_hash);

	// Read-only iteration. Useful when the wizard wants to enumerate
	// every known device the user has previously calibrated.
	const std::unordered_map<uint64_t, DeviceProfile>& All() const { return profiles_; }
	const ProfileIoStats& Stats() const { return stats_; }

private:
	std::unordered_map<uint64_t, DeviceProfile> profiles_;
	ProfileIoStats stats_;
};
