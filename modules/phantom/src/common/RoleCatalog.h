#pragma once

#include <cstdint>

namespace phantom {

// Body-role assignment per physical tracker. Drives both the IK fallback
// (Phase 1.5) and the absent-mode virtual-tracker creation (Phase 2). The
// values are stable wire identifiers; do not renumber.
enum class BodyRole : uint8_t
{
    None         = 0,
    Hmd          = 1,
    LeftHand     = 2,
    RightHand    = 3,
    Waist        = 4,
    Chest        = 5,
    LeftFoot     = 6,
    RightFoot    = 7,
    LeftKnee     = 8,
    RightKnee    = 9,
    LeftElbow    = 10,
    RightElbow   = 11,
    LeftShoulder = 12,
    RightShoulder = 13,
};

constexpr uint8_t kBodyRoleCount = 14;

// Friendly label for the overlay UI + diagnostics logs.
inline const char* BodyRoleLabel(BodyRole r)
{
    switch (r) {
        case BodyRole::None:          return "None";
        case BodyRole::Hmd:           return "HMD";
        case BodyRole::LeftHand:      return "Left hand";
        case BodyRole::RightHand:     return "Right hand";
        case BodyRole::Waist:         return "Waist";
        case BodyRole::Chest:         return "Chest";
        case BodyRole::LeftFoot:      return "Left foot";
        case BodyRole::RightFoot:     return "Right foot";
        case BodyRole::LeftKnee:      return "Left knee";
        case BodyRole::RightKnee:     return "Right knee";
        case BodyRole::LeftElbow:     return "Left elbow";
        case BodyRole::RightElbow:    return "Right elbow";
        case BodyRole::LeftShoulder:  return "Left shoulder";
        case BodyRole::RightShoulder: return "Right shoulder";
    }
    return "?";
}

// SteamVR controller-type string published for absent-mode trackers. The
// virtual target set follows VRChat's supported extra tracker points:
// waist/hip, chest, feet, knees, and upper-arm/elbow. Shoulder roles stay in
// the internal solver, but are not separate virtual trackers.
inline const char* BodyRoleToControllerType(BodyRole r)
{
    switch (r) {
        case BodyRole::Waist:         return "vive_tracker_waist";
        case BodyRole::Chest:         return "vive_tracker_chest";
        case BodyRole::LeftFoot:      return "vive_tracker_left_foot";
        case BodyRole::RightFoot:     return "vive_tracker_right_foot";
        case BodyRole::LeftKnee:      return "vive_tracker_left_knee";
        case BodyRole::RightKnee:     return "vive_tracker_right_knee";
        case BodyRole::LeftElbow:     return "vive_tracker_left_elbow";
        case BodyRole::RightElbow:    return "vive_tracker_right_elbow";
        case BodyRole::None:
        case BodyRole::Hmd:
        case BodyRole::LeftHand:
        case BodyRole::RightHand:
        case BodyRole::LeftShoulder:
        case BodyRole::RightShoulder:
            return nullptr;
    }
    return nullptr;
}

// Short string identifier persisted in phantom.txt + used as a key in IPC
// messages. Round-trippable via BodyRoleFromString().
inline const char* BodyRoleToKey(BodyRole r)
{
    switch (r) {
        case BodyRole::None:          return "none";
        case BodyRole::Hmd:           return "hmd";
        case BodyRole::LeftHand:      return "left_hand";
        case BodyRole::RightHand:     return "right_hand";
        case BodyRole::Waist:         return "waist";
        case BodyRole::Chest:         return "chest";
        case BodyRole::LeftFoot:      return "left_foot";
        case BodyRole::RightFoot:     return "right_foot";
        case BodyRole::LeftKnee:      return "left_knee";
        case BodyRole::RightKnee:     return "right_knee";
        case BodyRole::LeftElbow:     return "left_elbow";
        case BodyRole::RightElbow:    return "right_elbow";
        case BodyRole::LeftShoulder:  return "left_shoulder";
        case BodyRole::RightShoulder: return "right_shoulder";
    }
    return "none";
}

inline BodyRole BodyRoleFromKey(const char* s)
{
    if (!s || !*s) return BodyRole::None;
    for (uint8_t i = 0; i < kBodyRoleCount; ++i) {
        const BodyRole r = static_cast<BodyRole>(i);
        const char* k = BodyRoleToKey(r);
        // Manual strcmp to keep this header dependency-free.
        const char* a = s; const char* b = k;
        while (*a && *b && *a == *b) { ++a; ++b; }
        if (*a == 0 && *b == 0) return r;
    }
    return BodyRole::None;
}

} // namespace phantom
