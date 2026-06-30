#pragma once

#include <cstdint>

namespace phantom {

// Render model the absent-mode virtual trackers present to SteamVR. Purely
// cosmetic: Prop_ControllerType_String (vive_tracker_<role>) drives app role
// binding, so the model never affects tracking or OSC. Values are stable wire
// identifiers carried in PhantomConfig.render_model; do not renumber. 0 is the
// default so a zero-filled byte (older overlay or a fresh config) decodes to it.
enum class TrackerModel : uint8_t
{
	ViveTracker3 = 0, // default
	ViveTracker1 = 1,
	GenericTracker = 2,
};

constexpr uint8_t kTrackerModelCount = 3;

// SteamVR render-model name set on Prop_RenderModelName_String. The {htc}
// prefix resolves through the htc lighthouse driver that ships with SteamVR;
// generic_tracker lives in SteamVR core. All three are present under
// SteamVR\drivers\htc\resources\rendermodels and resources\rendermodels.
inline const char* TrackerModelRenderName(TrackerModel m)
{
	switch (m) {
		case TrackerModel::ViveTracker3:
			return "{htc}vr_tracker_vive_3_0";
		case TrackerModel::ViveTracker1:
			return "{htc}vr_tracker_vive_1_0";
		case TrackerModel::GenericTracker:
			return "generic_tracker";
	}
	return "{htc}vr_tracker_vive_3_0";
}

// Friendly label for the overlay dropdown + diagnostics logs.
inline const char* TrackerModelLabel(TrackerModel m)
{
	switch (m) {
		case TrackerModel::ViveTracker3:
			return "Vive Tracker 3.0";
		case TrackerModel::ViveTracker1:
			return "Vive Tracker 1.0";
		case TrackerModel::GenericTracker:
			return "Generic Tracker";
	}
	return "Vive Tracker 3.0";
}

// Short key persisted in phantom.txt, matching the device_role string-key
// convention. Round-trippable via TrackerModelFromKey.
inline const char* TrackerModelToKey(TrackerModel m)
{
	switch (m) {
		case TrackerModel::ViveTracker3:
			return "vive_3_0";
		case TrackerModel::ViveTracker1:
			return "vive_1_0";
		case TrackerModel::GenericTracker:
			return "generic";
	}
	return "vive_3_0";
}

inline TrackerModel TrackerModelFromKey(const char* s)
{
	if (!s || !*s) return TrackerModel::ViveTracker3;
	for (uint8_t i = 0; i < kTrackerModelCount; ++i) {
		const TrackerModel m = static_cast<TrackerModel>(i);
		const char* k = TrackerModelToKey(m);
		// Manual strcmp to keep this header dependency-free.
		const char* a = s;
		const char* b = k;
		while (*a && *b && *a == *b) {
			++a;
			++b;
		}
		if (*a == 0 && *b == 0) return m;
	}
	return TrackerModel::ViveTracker3;
}

} // namespace phantom
