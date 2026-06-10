#pragma once

#include <algorithm>
#include <cstddef>
#include <string>
#include <vector>

namespace captions {

enum class SileroVadModelFormat
{
	Unknown = 0,
	LegacyHcState,
	MergedState,
};

inline const char* SileroVadModelFormatName(SileroVadModelFormat format)
{
	switch (format) {
		case SileroVadModelFormat::LegacyHcState:
			return "legacy-hc-state";
		case SileroVadModelFormat::MergedState:
			return "merged-state";
		default:
			return "unknown";
	}
}

inline bool SileroVadHasNode(const std::vector<std::string>& names, const char* name)
{
	return std::find(names.begin(), names.end(), name) != names.end();
}

inline SileroVadModelFormat ClassifySileroVadModelContract(size_t input_count,
                                                           const std::vector<std::string>& input_names,
                                                           size_t output_count,
                                                           const std::vector<std::string>& output_names)
{
	if (input_count == 3 && output_count == 2 && SileroVadHasNode(input_names, "input") &&
	    SileroVadHasNode(input_names, "state") && SileroVadHasNode(input_names, "sr") &&
	    SileroVadHasNode(output_names, "output") && SileroVadHasNode(output_names, "stateN")) {
		return SileroVadModelFormat::MergedState;
	}
	if (input_count == 4 && output_count == 3 && SileroVadHasNode(input_names, "input") &&
	    SileroVadHasNode(input_names, "sr") && SileroVadHasNode(input_names, "h") &&
	    SileroVadHasNode(input_names, "c") && SileroVadHasNode(output_names, "output") &&
	    SileroVadHasNode(output_names, "hn") && SileroVadHasNode(output_names, "cn")) {
		return SileroVadModelFormat::LegacyHcState;
	}
	return SileroVadModelFormat::Unknown;
}

inline std::string JoinSileroVadNodeNames(const std::vector<std::string>& names)
{
	std::string out;
	for (const auto& name : names) {
		if (!out.empty()) out += ",";
		out += name;
	}
	return out;
}

} // namespace captions
