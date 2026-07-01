#pragma once

// Pure helpers for the experimental witness-based continuous drift correction.
//
// Geometry (see Calibration.cpp TickHeadMountInvariantShadow): with the calibration
// reference == HMD and target == head-mounted witness tracker,
//     localOffset = hmd^-1 * (targetToReference * tracker)
// is a rigid HMD-relative offset that is constant while the calibration tracks the
// witness and wanders as the SLAM frame drifts. T1 snapshots its stable mean as the
// baseline (headFromTracker); T2 applies a slew-limited step that walks the applied
// calibration translation back toward that baseline.
//
// Kept free of CalibrationContext/OpenVR so the accumulator + frame mapping are
// unit-testable. Offline-validated on real recordings (WitnessDriftReplay.h): the
// slew-limited model reduces typical sub-30 cm drift ~50-89%.

#include "ContinuousCorrection.h"

#include <Eigen/Geometry>
#include <cmath>
#include <limits>

namespace spacecal::witness_correction {

// T1: accumulate localOffset translation samples until the estimate is stable
// enough to commit as the baseline offset (no manual offset wizard needed).
constexpr int kBaselineMinSamples = 60;
constexpr double kBaselineMinSec = 3.0;
constexpr double kBaselineMaxStdM = 0.010; // 10 mm spread across the window

struct BaselineAccumulator
{
	int count = 0;
	Eigen::Vector3d sum = Eigen::Vector3d::Zero();
	double sumSqNorm = 0.0;
	double firstTime = 0.0;
	bool started = false;

	void Reset()
	{
		count = 0;
		sum = Eigen::Vector3d::Zero();
		sumSqNorm = 0.0;
		firstTime = 0.0;
		started = false;
	}

	void Add(const Eigen::Vector3d& localOffset, double time)
	{
		if (!started) {
			started = true;
			firstTime = time;
		}
		++count;
		sum += localOffset;
		sumSqNorm += localOffset.squaredNorm();
	}

	Eigen::Vector3d Mean() const
	{
		return count > 0 ? Eigen::Vector3d(sum / static_cast<double>(count)) : Eigen::Vector3d::Zero();
	}

	// Population std of the sample translations (metres), sqrt(E[|x|^2] - |E[x]|^2).
	double StdM() const
	{
		if (count <= 0) return std::numeric_limits<double>::infinity();
		const double var = sumSqNorm / static_cast<double>(count) - Mean().squaredNorm();
		return std::sqrt(var > 0.0 ? var : 0.0);
	}

	double ElapsedSec(double now) const { return started ? (now - firstTime) : 0.0; }

	bool Ready(double now) const
	{
		return count >= kBaselineMinSamples && ElapsedSec(now) >= kBaselineMinSec && StdM() <= kBaselineMaxStdM;
	}
};

// T2: the per-tick delta (centimetres) to add to the applied calibration
// translation to walk localOffset back toward the baseline, using the shipped
// slew/dead-band step (dead-band adapts to madFloorM so noisy tracking is not
// chased). Returns zero when the drift is inside the dead-band or above the 30 cm
// cap (the latter is a relocalization the recovery path owns). hmdWorldRot maps the
// HMD-local correction into the calibration (reference) frame; * 100 converts the
// metre step to the centimetre units of calibratedTranslation.
inline Eigen::Vector3d CorrectionDeltaCm(const Eigen::Vector3d& localOffset, const Eigen::Vector3d& baselineOffset,
                                         const Eigen::Matrix3d& hmdWorldRot, double madFloorM, double dt)
{
	const Eigen::Vector3d driftVec = localOffset - baselineOffset;
	const double drift = driftVec.norm();
	const double step = spacecal::cont_correction::CorrectionStepM(drift, madFloorM, dt);
	if (step <= 0.0 || drift < 1e-9) return Eigen::Vector3d::Zero();
	const Eigen::Vector3d deltaLocal = -step * (driftVec / drift); // move toward baseline
	return (hmdWorldRot * deltaLocal) * 100.0;
}

// Runaway / non-convergence guard. The per-tick step is already capped (slew +
// 30 cm error gate), but the loop can still walk the calibration away without
// bound if it fights the solver against a bad baseline: field logs showed 56.8 cm
// applied while the measured drift never fell below ~42 mm. Two independent trips
// bound that: a hard cumulative cap, and a "corrected a lot but drift never shrank
// over a window" convergence check. On a trip the caller re-baselines (a witness-
// captured offset) and backs off -- self-healing, no user action.
constexpr double kMaxCumulativeCorrectionM = 0.20; // below the 0.30 m reloc cap; above any legit shrinking total
constexpr double kNonConvergeWindowSec = 60.0;
constexpr double kNonConvergeMinAppliedCm = 5.0; // only judge convergence once this much was applied in-window
constexpr double kNonConvergeMinDriftMm = 20.0;  // drift still this high after correcting => loop not working

struct RunawayGuardState
{
	double appliedTotalCm = 0.0;
	double windowStartTime = -1.0;
	double appliedAtWindowStartCm = 0.0;
	double lastDriftMm = 0.0;
};

enum class GuardVerdict
{
	Ok,
	TripCumulative,
	TripNonConverge
};

// Evaluate the guard for this correction tick. `appliedTotalCm` is the cumulative
// applied correction, `driftMm` the current witness-vs-calibration drift, `now`
// the tick time. Advances the convergence window in place. Trips when cumulative
// meets the cap, or when a full window elapsed in which >= kNonConvergeMinAppliedCm
// was applied yet drift stayed >= kNonConvergeMinDriftMm.
inline GuardVerdict EvaluateRunawayGuard(RunawayGuardState& g, double appliedTotalCm, double driftMm, double now)
{
	g.appliedTotalCm = appliedTotalCm;
	g.lastDriftMm = driftMm;
	if (appliedTotalCm * 0.01 >= kMaxCumulativeCorrectionM) return GuardVerdict::TripCumulative;
	if (g.windowStartTime < 0.0) {
		g.windowStartTime = now;
		g.appliedAtWindowStartCm = appliedTotalCm;
	}
	if (now - g.windowStartTime >= kNonConvergeWindowSec) {
		const bool wasCorrecting = (appliedTotalCm - g.appliedAtWindowStartCm) >= kNonConvergeMinAppliedCm;
		const bool notShrinking = driftMm >= kNonConvergeMinDriftMm;
		g.windowStartTime = now; // reset the window regardless of verdict
		g.appliedAtWindowStartCm = appliedTotalCm;
		if (wasCorrecting && notShrinking) return GuardVerdict::TripNonConverge;
	}
	return GuardVerdict::Ok;
}

inline void ResetRunawayGuard(RunawayGuardState& g)
{
	g = RunawayGuardState{};
}

} // namespace spacecal::witness_correction
