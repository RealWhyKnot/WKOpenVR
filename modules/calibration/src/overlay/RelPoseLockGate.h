#pragma once

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>

// Accept gating for the lock-relative-position continuous solve.
//
// The locked branch averages the banked relative pose over the sample window
// and historically applied every candidate that fit within the 100 mm
// ValidateCalibration floor -- roughly four raw playspace writes per second
// on a healthy rig, each free to move the world as far as the fit landed.
// These gates bring the branch to parity with the incremental path (quality
// band, max-error cap, prior ratio) and bound how far a single accept may
// move the applied calibration:
//
//   - steps below the deadband are churn unless the fit meaningfully
//     improves; holding the prior keeps the world still,
//   - steps above the cap only land through a classified event (warm-restart
//     grace, session-first candidate) or through consensus: several
//     consecutive candidates agreeing on the same far-away answer mean the
//     reference frame really moved even though no observable event fired
//     (the asleep-headset re-anchor shape).
namespace spacecal::relpose_lock {

// Quality parity with the incremental relpose accept.
constexpr double kAcceptBandM = 0.010;
constexpr double kAcceptBandCalibratedM = 0.025;

// Movement bounds on a single accept.
constexpr double kStepDeadbandCm = 1.0;
constexpr double kStepImproveFraction = 0.10;
constexpr double kStepMaxCm = 10.0;

// Oversize escape: consecutive candidates that agree within the spread.
constexpr int kOversizeConsensusCount = 8;
constexpr double kOversizeConsensusSpreadCm = 2.0;

enum class LockedAccept
{
	Accept,
	HoldPrior,
	AcceptConsensusStep,
};

struct OversizeConsensusState
{
	int streak = 0;
	Eigen::Vector3d lastCandidateCm = Eigen::Vector3d::Zero();
};

struct LockedAcceptInputs
{
	double candidateErrorM = INFINITY; // relpose fit error of this candidate
	double priorErrorM = INFINITY;     // fit error of the held calibration
	bool havePrior = false;            // a validated calibration is held
	bool relPosCalibrated = false;     // banked relpose widens the band
	double notWorseRatio = 1.5;        // continuousCalibrationThreshold
	double maxErrorM = 0.005;          // relPoseMaxError parity cap
	double stepCm = 0.0;               // |candidate - held| translation
	bool stepGateBypassed = false;     // warm-restart grace / first candidate
};

struct LockedAcceptDecision
{
	LockedAccept action = LockedAccept::Accept;
	const char* rejectTag = nullptr; // set when action == HoldPrior
};

inline LockedAcceptDecision EvaluateLockedAccept(const LockedAcceptInputs& in, const Eigen::Vector3d& candidateCm,
                                                 OversizeConsensusState& consensus)
{
	LockedAcceptDecision out;
	auto hold = [&](const char* tag, bool resetConsensus) {
		out.action = LockedAccept::HoldPrior;
		out.rejectTag = tag;
		if (resetConsensus) consensus.streak = 0;
		return out;
	};

	// Callers that deliberately widen the max-error cap (offline solvers, the
	// A/B harnesses) widen the band with it; live passes a cap tighter than
	// the band, so the parity constants govern there.
	const double bandM = std::max(in.relPosCalibrated ? kAcceptBandCalibratedM : kAcceptBandM, in.maxErrorM);
	if (!(in.candidateErrorM < bandM)) {
		return hold("relpose_band", true);
	}
	if (in.candidateErrorM > in.maxErrorM) {
		return hold("relpose_max_error", true);
	}
	if (in.havePrior && std::isfinite(in.priorErrorM) && in.candidateErrorM > in.priorErrorM * in.notWorseRatio) {
		return hold("relpose_worse_than_prior", true);
	}

	if (!in.stepGateBypassed && in.havePrior) {
		if (in.stepCm > kStepMaxCm) {
			const bool agrees =
			    consensus.streak > 0 && (candidateCm - consensus.lastCandidateCm).norm() <= kOversizeConsensusSpreadCm;
			consensus.streak = agrees ? consensus.streak + 1 : 1;
			consensus.lastCandidateCm = candidateCm;
			if (consensus.streak >= kOversizeConsensusCount) {
				consensus.streak = 0;
				out.action = LockedAccept::AcceptConsensusStep;
				return out;
			}
			return hold("relpose_step_oversized", false);
		}
		const bool improves = std::isfinite(in.priorErrorM) &&
		                      (in.priorErrorM - in.candidateErrorM) >= kStepImproveFraction * in.priorErrorM;
		if (in.stepCm < kStepDeadbandCm && !improves) {
			return hold("relpose_step_deadband", true);
		}
	}

	consensus.streak = 0;
	return out;
}

} // namespace spacecal::relpose_lock
