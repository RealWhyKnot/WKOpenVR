#pragma once

#include "BodyCompletionSolver.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace phantom::analysis {

struct ScalarStats
{
	double mean = 0.0;
	double rms = 0.0;
	double p95 = 0.0;
	double max = 0.0;
	uint32_t count = 0;
};

struct PoseErrorStats
{
	ScalarStats position_m;
	ScalarStats orientation_deg;
};

struct ContinuityStats
{
	double max_step_m = 0.0;
	double max_orientation_step_deg = 0.0;
	uint32_t teleport_count = 0;
	uint32_t invalid_count = 0;
	uint32_t sample_count = 0;
};

struct FootSkateStats
{
	uint32_t planted_frames = 0;
	uint32_t skating_frames = 0;
	double total_slide_m = 0.0;
	double max_frame_slide_m = 0.0;
};

struct ConfidenceCalibrationStats
{
	static constexpr uint32_t kBinCount = 5;

	std::array<uint32_t, kBinCount> counts{};
	std::array<double, kBinCount> mean_error_m{};
	uint32_t monotonic_violations = 0;
};

inline bool IsFinite3(const double v[3])
{
	return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

inline bool IsFiniteQuat(const double q[4])
{
	return std::isfinite(q[0]) && std::isfinite(q[1]) && std::isfinite(q[2]) && std::isfinite(q[3]);
}

inline double Clamp(double value, double lo, double hi)
{
	return std::max(lo, std::min(hi, value));
}

inline double DistanceM(const double a[3], const double b[3])
{
	const double dx = a[0] - b[0];
	const double dy = a[1] - b[1];
	const double dz = a[2] - b[2];
	return std::sqrt(dx * dx + dy * dy + dz * dz);
}

inline double PlanarDistanceM(const double a[3], const double b[3])
{
	const double dx = a[0] - b[0];
	const double dz = a[2] - b[2];
	return std::sqrt(dx * dx + dz * dz);
}

inline double QuaternionAngularErrorDeg(const double a[4], const double b[4])
{
	if (!IsFiniteQuat(a) || !IsFiniteQuat(b)) {
		return std::numeric_limits<double>::infinity();
	}
	double dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
	dot = std::abs(dot);
	dot = Clamp(dot, 0.0, 1.0);
	return (2.0 * std::acos(dot)) * (180.0 / 3.14159265358979323846);
}

inline ScalarStats SummarizeScalars(std::vector<double> values)
{
	ScalarStats stats;
	values.erase(std::remove_if(values.begin(), values.end(), [](double v) { return !std::isfinite(v); }),
	             values.end());
	stats.count = static_cast<uint32_t>(values.size());
	if (values.empty()) return stats;

	double sum = 0.0;
	double sum_sq = 0.0;
	stats.max = values.front();
	for (double value : values) {
		sum += value;
		sum_sq += value * value;
		stats.max = std::max(stats.max, value);
	}
	stats.mean = sum / static_cast<double>(values.size());
	stats.rms = std::sqrt(sum_sq / static_cast<double>(values.size()));

	std::sort(values.begin(), values.end());
	const size_t p95_index =
	    std::min(values.size() - 1, static_cast<size_t>(std::ceil(0.95 * static_cast<double>(values.size())) - 1.0));
	stats.p95 = values[p95_index];
	return stats;
}

inline PoseErrorStats ComputePoseErrorStats(const std::vector<BodyCompletionPose>& estimated,
                                            const std::vector<BodyCompletionPose>& truth)
{
	PoseErrorStats stats;
	const size_t count = std::min(estimated.size(), truth.size());
	std::vector<double> position_errors;
	std::vector<double> orientation_errors;
	position_errors.reserve(count);
	orientation_errors.reserve(count);

	for (size_t i = 0; i < count; ++i) {
		if (!IsFinite3(estimated[i].position) || !IsFinite3(truth[i].position)) continue;
		position_errors.push_back(DistanceM(estimated[i].position, truth[i].position));
		orientation_errors.push_back(QuaternionAngularErrorDeg(estimated[i].rotation, truth[i].rotation));
	}

	stats.position_m = SummarizeScalars(std::move(position_errors));
	stats.orientation_deg = SummarizeScalars(std::move(orientation_errors));
	return stats;
}

inline ContinuityStats ComputeContinuityStats(const std::vector<BodyCompletionPose>& samples, double max_step_m,
                                              double max_orientation_step_deg)
{
	ContinuityStats stats;
	stats.sample_count = static_cast<uint32_t>(samples.size());
	if (samples.empty()) return stats;

	for (const auto& sample : samples) {
		if (!IsFinite3(sample.position) || !IsFiniteQuat(sample.rotation)) {
			++stats.invalid_count;
		}
	}

	for (size_t i = 1; i < samples.size(); ++i) {
		if (!IsFinite3(samples[i - 1].position) || !IsFinite3(samples[i].position) ||
		    !IsFiniteQuat(samples[i - 1].rotation) || !IsFiniteQuat(samples[i].rotation)) {
			continue;
		}
		const double step = DistanceM(samples[i - 1].position, samples[i].position);
		const double rot_step = QuaternionAngularErrorDeg(samples[i - 1].rotation, samples[i].rotation);
		stats.max_step_m = std::max(stats.max_step_m, step);
		stats.max_orientation_step_deg = std::max(stats.max_orientation_step_deg, rot_step);
		if (step > max_step_m || rot_step > max_orientation_step_deg) {
			++stats.teleport_count;
		}
	}
	return stats;
}

inline FootSkateStats ComputeFootSkateStats(const std::vector<BodyCompletionPose>& foot_samples, double floor_y_m,
                                            double planted_height_m, double max_planted_step_m)
{
	FootSkateStats stats;
	for (size_t i = 1; i < foot_samples.size(); ++i) {
		const auto& prev = foot_samples[i - 1];
		const auto& cur = foot_samples[i];
		if (!IsFinite3(prev.position) || !IsFinite3(cur.position)) continue;
		const bool planted = std::abs(prev.position[1] - floor_y_m) <= planted_height_m &&
		                     std::abs(cur.position[1] - floor_y_m) <= planted_height_m;
		if (!planted) continue;
		++stats.planted_frames;
		const double slide = PlanarDistanceM(prev.position, cur.position);
		stats.total_slide_m += slide;
		stats.max_frame_slide_m = std::max(stats.max_frame_slide_m, slide);
		if (slide > max_planted_step_m) {
			++stats.skating_frames;
		}
	}
	return stats;
}

inline ConfidenceCalibrationStats ComputeConfidenceCalibration(const std::vector<float>& confidence,
                                                               const std::vector<double>& error_m)
{
	ConfidenceCalibrationStats stats;
	const size_t count = std::min(confidence.size(), error_m.size());
	std::array<double, ConfidenceCalibrationStats::kBinCount> sums{};
	for (size_t i = 0; i < count; ++i) {
		if (!std::isfinite(confidence[i]) || !std::isfinite(error_m[i])) continue;
		const double clamped = Clamp(confidence[i], 0.0, 1.0);
		const size_t bin = std::min<size_t>(ConfidenceCalibrationStats::kBinCount - 1,
		                                    static_cast<size_t>(clamped * ConfidenceCalibrationStats::kBinCount));
		++stats.counts[bin];
		sums[bin] += error_m[i];
	}

	for (size_t i = 0; i < ConfidenceCalibrationStats::kBinCount; ++i) {
		if (stats.counts[i] > 0) {
			stats.mean_error_m[i] = sums[i] / static_cast<double>(stats.counts[i]);
		}
	}

	double previous = std::numeric_limits<double>::infinity();
	for (size_t i = 0; i < ConfidenceCalibrationStats::kBinCount; ++i) {
		if (stats.counts[i] == 0) continue;
		if (stats.mean_error_m[i] > previous + 1e-9) {
			++stats.monotonic_violations;
		}
		previous = stats.mean_error_m[i];
	}
	return stats;
}

inline bool OutputWouldPublish(const BodyCompletionRoleOutput& out, double virtual_min_confidence)
{
	return out.valid && out.confidence >= virtual_min_confidence;
}

} // namespace phantom::analysis
