#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>

// Persisted Smoothing overlay state. Saved to
// %LocalAppDataLow%\WKOpenVR\profiles\smoothing.txt as plain key=value
// lines so it's trivially editable by hand if needed.
struct SmoothingConfig
{
    // Finger smoothing (Index Knuckles bone arrays).
    int      smoothness     = 0;           // 0..100, global fallback; 0 = pass-through
    uint16_t finger_mask    = 0x03FF;      // protocol::kAllFingersMask -- all 10 fingers

    // Per-finger strength override (0..100). Indexed the same way as
    // finger_mask bits: 0..4 = left thumb..pinky, 5..9 = right thumb..pinky.
    // Value 0 means "use the global smoothness above" so a default-constructed
    // config behaves like a v12 build with a single global strength.
    int      per_finger_smoothness[10] = {0,0,0,0,0,0,0,0,0,0};

    // Per-tracker pose-prediction suppression, keyed by serial number so
    // values survive a device reconnecting under a fresh OpenVR ID. 0..100;
    // serials absent from the map are treated as 0 (off). Driver applies
    // (1 - value/100) to velocity / acceleration / poseTimeOffset at pose
    // update time.
    std::unordered_map<std::string, int> trackerSmoothness;

    // Compatibility key for the old smart-smoothing toggle. Position smoothing
    // now always uses the speed-adaptive one-euro filter whenever a tracker
    // smoothness value is nonzero. Dev builds still forward this flag to preview
    // rotation filtering; release builds hide the control and keep raw rotation.
    bool smart_smoothing = false;
};

// Load from disk. On any read / parse error the on-disk file is ignored and
// a default-constructed SmoothingConfig is returned (so first launch and
// corrupt-file launch both produce the same sensible defaults).
SmoothingConfig LoadConfig();

// Save to disk. Best-effort: failures (locked file, missing dir) are
// silently swallowed -- the next save will retry. The driver gets the live
// value via IPC regardless of persistence success.
void SaveConfig(const SmoothingConfig &cfg);
