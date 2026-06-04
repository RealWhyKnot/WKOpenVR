#pragma once

#include "Protocol.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>

namespace inputhealth::diagnostics {
struct PolarSummary
{
	int bins_with_samples = 0;
	int weak_bins = 0;
	double coverage = 0.0;
	double entropy = 0.0;
	float min_r = 0.0f;
	float mean_r = 0.0f;
	bool enough_coverage = false;
};

inline const char* AxisRoleName(uint8_t role)
{
	switch (role) {
		case 0:
			return "-";
		case 1:
			return "stick.x";
		case 2:
			return "stick.y";
		default:
			return "?";
	}
}

inline const char* KindName(const protocol::InputHealthSnapshotBody& b)
{
	if (b.is_scalar) return "scalar";
	if (b.is_boolean) return "bool";
	return "?";
}

inline double SampleStdDev(const protocol::InputHealthSnapshotBody& b)
{
	if (b.welford_count < 2) return 0.0;
	return std::sqrt(b.welford_m2 / static_cast<double>(b.welford_count - 1));
}

inline size_t BoundedPathLength(const char* path)
{
	size_t n = 0;
	while (n < protocol::INPUTHEALTH_PATH_LEN && path[n] != '\0')
		++n;
	return n;
}

inline std::string LowerPath(const protocol::InputHealthSnapshotBody& b)
{
	const size_t n = BoundedPathLength(b.path);
	std::string out(b.path, b.path + n);
	for (char& c : out) {
		c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
	}
	return out;
}

inline bool LooksLikeTriggerValue(const protocol::InputHealthSnapshotBody& b)
{
	const std::string p = LowerPath(b);
	return p.find("trigger") != std::string::npos &&
	       (p.find("value") != std::string::npos || p.rfind("/input/trigger", 0) == 0);
}

inline PolarSummary SummarizePolar(const protocol::InputHealthSnapshotBody& b)
{
	PolarSummary s;
	constexpr uint16_t kMinSamplesPerBin = 3;
	const int binCount = static_cast<int>(protocol::INPUTHEALTH_POLAR_BIN_COUNT);

	uint64_t totalCount = 0;
	for (int i = 0; i < binCount; ++i)
		totalCount += b.polar_count[i];

	float minR = std::numeric_limits<float>::max();
	float sumR = 0.0f;
	for (int i = 0; i < binCount; ++i) {
		if (b.polar_count[i] >= kMinSamplesPerBin && b.polar_max_r[i] > 0.0f) {
			++s.bins_with_samples;
			minR = std::min(minR, b.polar_max_r[i]);
			sumR += b.polar_max_r[i];
		}
		if (totalCount > 0 && b.polar_count[i] > 0) {
			const double p = static_cast<double>(b.polar_count[i]) / static_cast<double>(totalCount);
			s.entropy -= p * std::log(p);
		}
	}

	if (totalCount > 0) {
		s.entropy /= std::log(static_cast<double>(binCount));
	}
	s.coverage = static_cast<double>(s.bins_with_samples) / static_cast<double>(binCount);
	if (s.bins_with_samples > 0) {
		s.min_r = minR;
		s.mean_r = sumR / static_cast<float>(s.bins_with_samples);
	}

	s.enough_coverage = s.coverage >= 0.70 && s.entropy >= 0.75 && b.polar_global_max_r >= 0.60f;
	if (s.enough_coverage) {
		const float weakThreshold = b.polar_global_max_r * 0.75f;
		for (int i = 0; i < binCount; ++i) {
			if (b.polar_count[i] >= kMinSamplesPerBin && b.polar_max_r[i] > 0.0f && b.polar_max_r[i] < weakThreshold) {
				++s.weak_bins;
			}
		}
	}
	return s;
}
} // namespace inputhealth::diagnostics
