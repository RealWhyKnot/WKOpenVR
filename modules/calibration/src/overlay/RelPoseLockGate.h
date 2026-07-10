#pragma once

#include <Eigen/Dense>

#include <algorithm>
#include <array>
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
//   - steps between the deadband and the cap land only when a rolling
//     median of recent candidates has itself departed the held
//     calibration: on a rig with a long lever arm to the playspace
//     origin, individually well-fitting candidates scatter several cm,
//     and accepting each one walked the world ~200 times an hour (field
//     log 2026-07-10). The median separates that scatter from real
//     drift -- zero-mean noise leaves it at the held answer while a
//     persistent shift moves it within a window fill,
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

// In-band drift follower: an in-band step is real drift only when the
// component-wise median of the last kDriftWindowCandidates quality-passing
// candidates sits at least kDriftStepMinCm from the held calibration AND
// the candidate itself is a typical member of that cluster. For ~30
// candidates with a few cm of scatter the median's standard error is
// sub-cm, so steady-state noise stays statistically quiet while a
// persistent shift fires within one window fill.
constexpr int kDriftWindowCandidates = 30;
constexpr double kDriftStepMinCm = 1.5;
constexpr double kDriftCandidateAgreeCm = 1.0;

enum class LockedAccept
{
	Accept,
	HoldPrior,
	AcceptConsensusStep,
	AcceptDriftStep,
};

struct OversizeConsensusState
{
	int streak = 0;
	Eigen::Vector3d lastCandidateCm = Eigen::Vector3d::Zero();
};

// Ring of the most recent quality-passing sub-cap candidate translations
// (absolute, cm). Reset whenever the held frame moves by other means (a
// bypassed accept during grace, a consensus step) -- the old cluster then
// describes a dead frame.
struct SmallStepDriftState
{
	int count = 0;
	int next = 0;
	std::array<Eigen::Vector3d, kDriftWindowCandidates> ringCm{};
};

inline void PushDriftCandidate(SmallStepDriftState& st, const Eigen::Vector3d& candidateCm)
{
	st.ringCm[st.next] = candidateCm;
	st.next = (st.next + 1) % kDriftWindowCandidates;
	if (st.count < kDriftWindowCandidates) ++st.count;
}

inline Eigen::Vector3d DriftWindowMedianCm(const SmallStepDriftState& st)
{
	Eigen::Vector3d median = Eigen::Vector3d::Zero();
	std::array<double, kDriftWindowCandidates> axis{};
	for (int a = 0; a < 3; ++a) {
		for (int i = 0; i < st.count; ++i) axis[static_cast<size_t>(i)] = st.ringCm[static_cast<size_t>(i)][a];
		const int mid = st.count / 2;
		std::nth_element(axis.begin(), axis.begin() + mid, axis.begin() + st.count);
		median[a] = axis[static_cast<size_t>(mid)];
	}
	return median;
}

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

	// Held calibration translation (cm); reference point for the drift
	// follower's median-departure test. Meaningful only when havePrior.
	Eigen::Vector3d heldCm = Eigen::Vector3d::Zero();
};

struct LockedAcceptDecision
{
	LockedAccept action = LockedAccept::Accept;
	const char* rejectTag = nullptr; // set when action == HoldPrior
};

inline LockedAcceptDecision EvaluateLockedAccept(const LockedAcceptInputs& in, const Eigen::Vector3d& candidateCm,
                                                 OversizeConsensusState& consensus, SmallStepDriftState& drift)
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
				// The held frame is about to move far; the cluster the ring
				// describes dies with it.
				drift = {};
				out.action = LockedAccept::AcceptConsensusStep;
				return out;
			}
			return hold("relpose_step_oversized", false);
		}

		// Every quality-passing sub-cap candidate joins the drift window --
		// including sub-deadband ones, which anchor the median at the
		// steady-state answer.
		PushDriftCandidate(drift, candidateCm);

		if (in.stepCm < kStepDeadbandCm) {
			const bool improves = std::isfinite(in.priorErrorM) &&
			                      (in.priorErrorM - in.candidateErrorM) >= kStepImproveFraction * in.priorErrorM;
			if (!improves) {
				return hold("relpose_step_deadband", true);
			}
		}
		else {
			// In-band step: land it only when the candidate cluster's median
			// has genuinely departed the held answer AND this candidate is a
			// typical member of the cluster (not an extremal fit -- this also
			// keeps the applied rotation typical rather than an outlier's).
			// After an accept the held answer sits at the median, so the
			// condition goes quiet on its own until real drift resumes.
			if (drift.count < kDriftWindowCandidates) {
				return hold("relpose_step_drift_pending", true);
			}
			const Eigen::Vector3d medianCm = DriftWindowMedianCm(drift);
			if ((medianCm - in.heldCm).norm() < kDriftStepMinCm ||
			    (candidateCm - medianCm).norm() > kDriftCandidateAgreeCm) {
				return hold("relpose_step_drift_pending", true);
			}
			consensus.streak = 0;
			out.action = LockedAccept::AcceptDriftStep;
			return out;
		}
	}
	else if (in.stepGateBypassed && in.havePrior) {
		// Grace-window and fusion accepts move the held frame directly; the
		// ring's cluster belongs to the frame before the event.
		drift = {};
	}

	consensus.streak = 0;
	return out;
}

} // namespace spacecal::relpose_lock
