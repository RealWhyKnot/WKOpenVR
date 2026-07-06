#pragma once

#include <cstdint>
#include <cstdlib>
#include <string>

namespace spacecal::calibration_experiments {

// Bits 1<<0 (headset offset auto-correct), 1<<1 (witness auto-calibrate),
// 1<<2 (witness continuous correction), 1<<4 (geometry-shift restart), and
// 1<<5 (micro-reanchor of witness-corroborated sub-30 cm frame jumps; the
// trigger never occurred in ~30 h of real captures) are retired; do not
// reuse them -- the values are persisted in the experimental_flags column
// of archived v5 recordings.
enum ExperimentFlag : uint32_t
{
	// Geometry-precision confidence fusion of continuous re-solves.
	ConfidenceFusion = 1u << 3,
};

inline bool EnvFlagEnabled(const char* value)
{
	if (!value) return false;
	std::string s(value);
	for (char& c : s) {
		if (c >= 'A' && c <= 'Z') c = static_cast<char>(c - 'A' + 'a');
	}
	return s == "1" || s == "true" || s == "yes" || s == "on";
}

inline bool HiddenExperimentsEnabled()
{
	return EnvFlagEnabled(std::getenv("WKOPENVR_CALIBRATION_EXPERIMENT_DIAGNOSTICS"));
}

inline std::string ToggleAnnotation(const char* optionName, bool enabled)
{
	std::string out(optionName ? optionName : "unknown_experiment");
	out += "_toggled: source=ui enabled=";
	out += enabled ? "1" : "0";
	return out;
}

} // namespace spacecal::calibration_experiments
