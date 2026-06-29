#pragma once

#include <cstdint>
#include <cstdlib>
#include <string>

namespace spacecal::calibration_experiments {

enum ExperimentFlag : uint32_t
{
	HeadsetOffsetAutoCorrect = 1u << 0,
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
