#pragma once

#include <string>

// Developer live-tuning knobs. DEV BUILDS ONLY: a dev_tuning.ini dropped in the
// driver resources directory is re-read at runtime so numeric constants can be
// tweaked while SteamVR keeps running -- no rebuild, no restart, no leaving VR.
// In release builds nothing is ever read from disk and Get() returns the
// caller's default, so behavior is identical to a build with no tuning file.
//
// File format: one "key = value" per line. '#' or ';' begins a comment (whole
// line or trailing). Blank lines are ignored. The value is parsed as a double.
// Unknown or malformed lines are skipped, so a typo degrades to "that key keeps
// its default" rather than a crash or a bad value.
//
// Usage at a call site: replace a hard-coded constant with
//     devtuning::Get("smoothing.finger_max_strength", 0.95)
// The literal stays the default, so removing/omitting the file restores the
// exact original value.
namespace openvr_pair::common::devtuning {

// Live value for key, or def if the key is absent, unparsed, or (on release) in
// all cases. Cheap: one snapshot copy + hash lookup. Safe from hot paths.
double Get(const char* key, double def);

// Re-read path if its last-write time changed since the previous call. A cheap
// no-op when unchanged; intended to be driven from a throttled driver loop
// (~1 Hz). Returns true only on the calls where a reload actually happened (so
// the caller can log it). No-op returning false on release builds.
bool MaybeReloadFromFile(const std::wstring& path);

// Test seam: replace the active snapshot from an in-memory buffer, bypassing the
// filesystem and the release gate. Pass an empty string to clear all keys.
void ApplyTextForTest(const std::string& text);

} // namespace openvr_pair::common::devtuning
