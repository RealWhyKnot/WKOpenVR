#pragma once

#include "inputhealth/PathPolicy.h"

#include <string>

// Input path classification shared between the overlay learning engine and
// the driver-side compensation cache. Both sides call ClassifyInputPath() so
// they agree on which paths receive compensation, which are observed-only, and
// which are silently ignored.
//
// Classification is path-string-only and has no runtime dependencies so it
// can be called from the driver detour, the overlay tick, and the test suite
// without linking any SteamVR or UI code.

namespace inputhealth {

enum class PathClass : uint8_t
{
    // Path is a trigger scalar: min/max remapping is appropriate.
    Trigger,

    // Path is an analog stick axis (x or y component): rest-offset + radial
    // deadzone compensation is appropriate.
    StickAxis,

    // Path is a controller button / binary scalar (grip, system, A/B, etc.):
    // rest-offset compensation is appropriate (debounce for booleans).
    ControllerButton,

    // Path is observable in the diagnostics UI but must not be pushed into
    // compensation. Examples: facial expression blendshapes, eye openness,
    // any path under /input/eye or /input/face.
    DiagnosticsOnly,

    // Path is not meaningful for any InputHealth feature. Silently ignored.
    // Examples: pupil dilation, proximity sensors, squeeze click, raw IMU.
    Unsupported,
};

// Returns true for every PathClass for which compensation may be applied.
// DiagnosticsOnly and Unsupported must not be pushed into compensation.
inline bool IsCompensationPath(PathClass cls)
{
    return cls == PathClass::Trigger ||
           cls == PathClass::StickAxis ||
           cls == PathClass::ControllerButton;
}

// Returns true for paths that should be visible in the diagnostics tab but
// must not be learned into persistent compensation.
inline bool IsDiagnosticsOnlyPath(PathClass cls)
{
    return cls == PathClass::DiagnosticsOnly;
}

// Path-string-only heuristic for persistent trigger min/max remapping.
// Force, pressure, grip value, and trackpad force are intentionally excluded;
// they are handled by the stricter path policy as capped idle-floor paths.
inline bool IsTriggerLikePath(const std::string &path)
{
    return IsTriggerRemapFamily(ClassifyPathFamily(path));
}

// Classify a /input/... path string. The path should be the canonical SteamVR
// input path as reported by CreateBooleanComponent / CreateScalarComponent.
//
// Classification order matters: more-specific checks precede broader ones so
// a path like "/input/trigger/value" resolves to Trigger rather than falling
// through to ControllerButton.
inline PathClass ClassifyInputPath(const std::string &path)
{
    switch (ClassifyPathFamily(path)) {
        case PathFamily::TriggerValue:
            return PathClass::Trigger;
        case PathFamily::ThumbstickAxis:
            return PathClass::StickAxis;
        case PathFamily::ForceSensor:
        case PathFamily::GripValue:
            return PathClass::Trigger;
        case PathFamily::ControllerButton:
            return PathClass::ControllerButton;
        case PathFamily::TrackpadAxis:
        case PathFamily::FingerCapsense:
        case PathFamily::DiagnosticsOnly:
            return PathClass::DiagnosticsOnly;
        case PathFamily::Unsupported:
            return PathClass::Unsupported;
    }
    return PathClass::Unsupported;
}

} // namespace inputhealth
