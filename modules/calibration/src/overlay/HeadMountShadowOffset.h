#pragma once

#include <Eigen/Geometry>

#include <cmath>
#include <cstddef>
#include <limits>

namespace spacecal::headmount {

constexpr double kShadowNoopTranslationM = 0.015;
constexpr double kShadowNoopRotationDeg = 3.0;
constexpr double kShadowStableTranslationM = 0.010;
constexpr double kShadowStableRotationDeg = 2.0;
constexpr double kShadowMaxTranslationM = 0.35;
constexpr double kShadowMaxRotationDeg = 45.0;
constexpr double kShadowResidualMaxMm = 5.0;
constexpr double kShadowProfileHealthyRmsMm = 10.0;
constexpr int kShadowRequiredStableWindows = 3;
constexpr double kShadowSourceResetQuietSec = 2.0;
constexpr double kShadowFreshPoseMaxAgeMs = 150.0;
constexpr size_t kShadowMaxWindowSamplesWithoutReadiness = 120;

struct OffsetDelta
{
	double translationM = 0.0;
	double rotationDeg = 0.0;
};

inline OffsetDelta ComputeOffsetDelta(const Eigen::AffineCompact3d& a, const Eigen::AffineCompact3d& b)
{
	OffsetDelta out;
	out.translationM = (a.translation() - b.translation()).norm();
	const Eigen::Quaterniond aq(a.linear());
	const Eigen::Quaterniond bq(b.linear());
	out.rotationDeg = aq.normalized().angularDistance(bq.normalized()) * (180.0 / EIGEN_PI);
	if (!std::isfinite(out.translationM)) {
		out.translationM = std::numeric_limits<double>::infinity();
	}
	if (!std::isfinite(out.rotationDeg)) {
		out.rotationDeg = std::numeric_limits<double>::infinity();
	}
	return out;
}

inline bool IsMeaningfulOffsetDelta(const OffsetDelta& delta)
{
	return delta.translationM >= kShadowNoopTranslationM || delta.rotationDeg >= kShadowNoopRotationDeg;
}

inline bool IsPlausibleOffsetDelta(const OffsetDelta& delta)
{
	return std::isfinite(delta.translationM) && std::isfinite(delta.rotationDeg) &&
	       delta.translationM <= kShadowMaxTranslationM && delta.rotationDeg <= kShadowMaxRotationDeg;
}

inline bool IsStableOffsetDelta(const OffsetDelta& delta)
{
	return std::isfinite(delta.translationM) && std::isfinite(delta.rotationDeg) &&
	       delta.translationM <= kShadowStableTranslationM && delta.rotationDeg <= kShadowStableRotationDeg;
}

struct ShadowGateInput
{
	bool toggleEnabled = true;
	bool windowSolved = false;
	bool posesFresh = false;
	bool targetMatches = false;
	bool profileHealthy = false;
	bool residualOk = false;
	bool sourceSettled = false;
	bool fallbackQuiet = false;
	bool mismatchPlausible = false;
	bool mismatchMeaningful = false;
	bool candidateStable = false;
	int stableWindowCount = 0;
	int requiredStableWindows = kShadowRequiredStableWindows;
};

struct ShadowGateResult
{
	bool readyToApply = false;
	bool wouldApply = false;
	const char* reason = "unknown";
};

inline ShadowGateResult EvaluateShadowGate(const ShadowGateInput& in)
{
	ShadowGateResult out;

	if (!in.windowSolved) {
		out.reason = "window_not_solved";
		return out;
	}
	if (!in.posesFresh) {
		out.reason = "stale_pose";
		return out;
	}
	if (!in.targetMatches) {
		out.reason = "target_mismatch";
		return out;
	}
	if (!in.profileHealthy) {
		out.reason = "profile_unhealthy";
		return out;
	}
	if (!in.residualOk) {
		out.reason = "residual_high";
		return out;
	}
	if (!in.sourceSettled) {
		out.reason = "source_reset_recent";
		return out;
	}
	if (!in.fallbackQuiet) {
		out.reason = "driver_synth_fallback_active";
		return out;
	}
	if (!in.mismatchPlausible) {
		out.reason = "delta_implausible";
		return out;
	}
	if (!in.mismatchMeaningful) {
		out.reason = "delta_below_threshold";
		return out;
	}
	if (!in.candidateStable) {
		out.reason = "offset_unstable";
		return out;
	}
	if (in.stableWindowCount < in.requiredStableWindows) {
		out.reason = "waiting_for_stable_windows";
		return out;
	}

	out.wouldApply = true;
	if (!in.toggleEnabled) {
		out.reason = "toggle_disabled";
		return out;
	}

	out.readyToApply = true;
	out.reason = "ready";
	return out;
}

} // namespace spacecal::headmount
