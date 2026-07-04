#pragma once

#include <cstdint>
#include <cstdlib>
#include <string>

namespace spacecal::calibration_experiments {

enum ExperimentFlag : uint32_t
{
	HeadsetOffsetAutoCorrect = 1u << 0,
	// Witness-based continuous drift correction (offline-validated on real
	// recordings: ~50-89% typical sub-30 cm drift reduction). Auto-calibrate
	// snapshots the HMD<->witness baseline offset; correction applies the
	// slew-limited step that closes the witness-vs-calibration drift.
	WitnessOffsetAutoCalibrate = 1u << 1,
	WitnessContinuousCorrection = 1u << 2,
	// Geometry-precision confidence fusion of continuous re-solves.
	ConfidenceFusion = 1u << 3,
	// Geometry-shift fires restart continuous calibration (legacy behaviour).
	GeometryShiftRestart = 1u << 4,
	// Witness-corroborated sub-30 cm frame jumps are absorbed immediately.
	MicroReanchor = 1u << 5,
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
