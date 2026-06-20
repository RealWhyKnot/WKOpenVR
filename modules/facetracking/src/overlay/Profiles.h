#pragma once

#include "Protocol.h"

#include <array>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

struct FaceShapeTuningValue
{
	int scale_percent = protocol::FACETRACKING_SHAPE_TUNING_DEFAULT_PERCENT;
	int min_percent = protocol::FACETRACKING_SHAPE_TUNING_DEFAULT_MIN_PERCENT;
	int max_percent = protocol::FACETRACKING_SHAPE_TUNING_DEFAULT_MAX_PERCENT;
};

using FaceShapeScaleArray = std::array<FaceShapeTuningValue, protocol::FACETRACKING_EXPRESSION_COUNT>;

inline constexpr const char* kDefaultAvatarShapeTuningKey = "__default__";

FaceShapeScaleArray DefaultFaceShapeScales();
FaceShapeTuningValue DefaultFaceShapeTuningValue();
FaceShapeTuningValue ClampFaceShapeTuningValue(FaceShapeTuningValue value);
bool IsDefaultFaceShapeTuningValue(const FaceShapeTuningValue& value);
std::string NormalizeAvatarShapeTuningKey(std::string key);
bool IsDefaultFaceShapeScales(const FaceShapeScaleArray& values);
FaceShapeTuningValue CombineShapeTuningValue(const FaceShapeTuningValue& global, const FaceShapeTuningValue& avatar);
FaceShapeScaleArray CombineShapeTuning(const FaceShapeScaleArray& global, const FaceShapeScaleArray& avatar);

struct AvatarShapeTuningMetadata
{
	std::string custom_name;
	std::string auto_name;
	std::string last_used_utc;
	std::string config_path;
};

// Overlay-side settings for the FaceTracking feature.
// Serialised to %LocalAppDataLow%\WKOpenVR\profiles\facetracking.json.
// This file holds settings that the overlay itself needs across sessions.

struct FacetrackingProfile
{
	// --- wire-side settings (mirrored to the driver on connect) ---
	bool eyelid_sync_enabled = false;
	bool eyelid_sync_preserve_winks = true;
	int eyelid_sync_strength = 70; // 0..100
	int eyelid_sync_mode = protocol::FACETRACKING_EYELID_SYNC_MOST_CLOSED;

	bool vergence_lock_enabled = false;
	int vergence_lock_strength = 60; // 0..100

	// Legacy profile/wire field retained for compatibility. Normal runtime
	// keeps continuous calibration disabled.
	int continuous_calib_mode = 0;

	// Drives output_osc_enabled in FaceTrackingConfig: gates the driver's per-frame
	// OSC publish calls before they reach the router. To disable FT OSC output
	// entirely, clear this toggle; individual route filtering lives on the router tab.
	bool output_osc_enabled = true;

	int gaze_smoothing = 0;     // 0..100
	int openness_smoothing = 0; // 0..100

	// Avatar-expression shaping. These are preference controls, not baseline
	// compatibility behaviour, so new profiles keep them disabled.
	bool mouth_close_compensation_enabled = false;
	bool smile_mouth_open_assist_enabled = false;
	int smile_mouth_open_strength = 50; // 0..100
	bool idle_mouth_auto_close_enabled = false;
	bool eyelid_brow_sync_enabled = false;
	int eyelid_brow_sync_strength = 50; // 0..100

	// Which installed modules the user has toggled on in the Modules tab.
	// Empty list = a fresh profile; the overlay selects the first installed
	// module when pushing config. A populated list is the user's explicit
	// selection. The backend currently activates the first entry until the
	// host is upgraded to run multiple simultaneously. Serialised order is
	// load order -- preserve the user's row order so an upgrade to true
	// multi-run has a stable priority.
	std::vector<std::string> enabled_module_uuids;

	// Expression output tuning. Values are percentages in [0, 200], with 100
	// meaning pass-through. Final published output remains limited to 0..1.
	FaceShapeScaleArray global_shape_tuning = DefaultFaceShapeScales();
	std::map<std::string, FaceShapeScaleArray> avatar_shape_tuning;
	std::map<std::string, AvatarShapeTuningMetadata> avatar_shape_metadata;

	// --- overlay-only preferences ---
	bool show_raw_values = false;
	int last_tab_index = 0; // remembers which tab was active
};

FaceShapeScaleArray& ShapeTuningForAvatar(FacetrackingProfile& profile, const std::string& avatarKey);
const FaceShapeScaleArray* FindShapeTuningForAvatar(const FacetrackingProfile& profile, const std::string& avatarKey);
void PruneAvatarShapeTuning(FacetrackingProfile& profile, const std::string& avatarKey);
AvatarShapeTuningMetadata& MetadataForAvatar(FacetrackingProfile& profile, const std::string& avatarKey);
const AvatarShapeTuningMetadata* FindMetadataForAvatar(const FacetrackingProfile& profile,
                                                       const std::string& avatarKey);
std::string AvatarDisplayName(const std::string& avatarKey, const AvatarShapeTuningMetadata* metadata);
std::string AvatarDisplaySourceLabel(const std::string& avatarKey, const AvatarShapeTuningMetadata* metadata);
int64_t AvatarLastUsedUnixSeconds(const std::string& utc);
std::string FormatAvatarLastUsedAge(const std::string& utc, int64_t now_unix_seconds);
std::string FormatAvatarLastUsedAge(const std::string& utc);

class FacetrackingProfileStore
{
public:
	// Load from facetracking.json. Returns true on success. On failure the
	// `current` field retains its default-initialised values.
	bool Load();

	// Write `current` to facetracking.json. Returns true on success.
	bool Save() const;

	FacetrackingProfile current;
};
