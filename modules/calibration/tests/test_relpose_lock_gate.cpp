#include <gtest/gtest.h>

#include <Eigen/Dense>

#include "RelPoseLockGate.h"

using spacecal::relpose_lock::EvaluateLockedAccept;
using spacecal::relpose_lock::LockedAccept;
using spacecal::relpose_lock::LockedAcceptInputs;
using spacecal::relpose_lock::OversizeConsensusState;
using spacecal::relpose_lock::SmallStepDriftState;

namespace {

LockedAcceptInputs HealthyInputs()
{
	LockedAcceptInputs in;
	in.candidateErrorM = 0.002;
	in.priorErrorM = 0.002;
	in.havePrior = true;
	in.relPosCalibrated = false;
	in.notWorseRatio = 1.5;
	in.maxErrorM = 0.005;
	in.stepCm = 2.0;
	in.stepGateBypassed = false;
	return in;
}

Eigen::Vector3d Cand(double xCm)
{
	return Eigen::Vector3d(xCm, 0.0, 0.0);
}

// Feed one candidate at x-offset `xCm` from a held calibration at the
// origin, keeping stepCm consistent with the geometry.
spacecal::relpose_lock::LockedAcceptDecision Feed(double xCm, OversizeConsensusState& consensus,
                                                  SmallStepDriftState& drift)
{
	auto in = HealthyInputs();
	in.stepCm = std::abs(xCm);
	return EvaluateLockedAccept(in, Cand(xCm), consensus, drift);
}

} // namespace

// --- Quality gates (band / cap / prior ratio) --------------------------------

TEST(RelPoseLockGateTest, RejectsAboveAcceptBand)
{
	OversizeConsensusState st;
	SmallStepDriftState drift;
	auto in = HealthyInputs();
	in.candidateErrorM = 0.012; // above the 10 mm fresh band
	const auto d = EvaluateLockedAccept(in, Cand(2.0), st, drift);
	EXPECT_EQ(d.action, LockedAccept::HoldPrior);
	EXPECT_STREQ(d.rejectTag, "relpose_band");
}

TEST(RelPoseLockGateTest, WiderBandWhenRelPoseCalibrated)
{
	// A banked relpose widens the band 10 -> 25 mm; the same candidate gets
	// past the band and is caught by the max-error cap instead.
	OversizeConsensusState st;
	SmallStepDriftState drift;
	auto in = HealthyInputs();
	in.candidateErrorM = 0.012;
	in.relPosCalibrated = true;
	const auto d = EvaluateLockedAccept(in, Cand(2.0), st, drift);
	EXPECT_EQ(d.action, LockedAccept::HoldPrior);
	EXPECT_STREQ(d.rejectTag, "relpose_max_error");
}

TEST(RelPoseLockGateTest, WidenedMaxErrorCapWidensBand)
{
	// Offline harnesses pass a deliberately loose cap; the band follows it so
	// they can score raw candidates the live gate would refuse. Sub-deadband
	// step with a clear improvement keeps this focused on the band logic.
	OversizeConsensusState st;
	SmallStepDriftState drift;
	auto in = HealthyInputs();
	in.candidateErrorM = 0.055;
	in.priorErrorM = 0.075;
	in.maxErrorM = 1.0;
	in.stepCm = 0.5;
	const auto d = EvaluateLockedAccept(in, Cand(0.5), st, drift);
	EXPECT_EQ(d.action, LockedAccept::Accept);
}

TEST(RelPoseLockGateTest, MaxErrorCapParity)
{
	OversizeConsensusState st;
	SmallStepDriftState drift;
	auto in = HealthyInputs();
	in.candidateErrorM = 0.006; // inside the band, above the cap
	const auto d = EvaluateLockedAccept(in, Cand(2.0), st, drift);
	EXPECT_EQ(d.action, LockedAccept::HoldPrior);
	EXPECT_STREQ(d.rejectTag, "relpose_max_error");
}

TEST(RelPoseLockGateTest, HoldsWhenWorseThanPriorByRatio)
{
	OversizeConsensusState st;
	SmallStepDriftState drift;
	auto in = HealthyInputs();
	in.priorErrorM = 0.001;
	in.candidateErrorM = 0.002; // 2x worse than prior, ratio 1.5
	const auto d = EvaluateLockedAccept(in, Cand(2.0), st, drift);
	EXPECT_EQ(d.action, LockedAccept::HoldPrior);
	EXPECT_STREQ(d.rejectTag, "relpose_worse_than_prior");
}

// --- Step deadband (sub-1 cm) -------------------------------------------------

TEST(RelPoseLockGateTest, HoldsBelowStepDeadband)
{
	OversizeConsensusState st;
	SmallStepDriftState drift;
	auto in = HealthyInputs();
	in.stepCm = 0.4;
	const auto d = EvaluateLockedAccept(in, Cand(0.4), st, drift);
	EXPECT_EQ(d.action, LockedAccept::HoldPrior);
	EXPECT_STREQ(d.rejectTag, "relpose_step_deadband");
}

TEST(RelPoseLockGateTest, ImprovementBeatsDeadband)
{
	OversizeConsensusState st;
	SmallStepDriftState drift;
	auto in = HealthyInputs();
	in.stepCm = 0.4;
	in.priorErrorM = 0.004;
	in.candidateErrorM = 0.002; // 50% better than prior
	const auto d = EvaluateLockedAccept(in, Cand(0.4), st, drift);
	EXPECT_EQ(d.action, LockedAccept::Accept);
}

// --- In-band drift follower (1-10 cm) -----------------------------------------
//
// A single in-band candidate is not evidence of drift: on a rig with a long
// lever arm to the playspace origin, individually well-fitting candidates
// scatter several cm, and accepting each one walked the applied calibration
// ~200 times an hour (field log 2026-07-10). An in-band step lands only when
// the rolling candidate median has itself departed the held calibration.

TEST(RelPoseLockGateTest, InBandStepHeldUntilDriftConsensus)
{
	OversizeConsensusState st;
	SmallStepDriftState drift;
	const auto d = EvaluateLockedAccept(HealthyInputs(), Cand(2.0), st, drift);
	EXPECT_EQ(d.action, LockedAccept::HoldPrior);
	EXPECT_STREQ(d.rejectTag, "relpose_step_drift_pending");
	EXPECT_EQ(drift.count, 1) << "The held candidate still joins the window";
}

TEST(RelPoseLockGateTest, PersistentOffsetFiresDriftStep)
{
	// Steady-state candidates hover at ~prior error; a strict per-sample
	// improve gate would starve drift correction entirely. A persistent
	// cluster ~3 cm from the held answer must land once the window agrees.
	OversizeConsensusState st;
	SmallStepDriftState drift;
	const double cluster[3] = {2.7, 3.0, 3.3};
	spacecal::relpose_lock::LockedAcceptDecision last;
	int accepts = 0;
	for (int i = 0; i < spacecal::relpose_lock::kDriftWindowCandidates; ++i) {
		last = Feed(cluster[i % 3], st, drift);
		if (last.action != LockedAccept::HoldPrior) ++accepts;
	}
	EXPECT_EQ(accepts, 1) << "Exactly one drift step once the window fills";
	EXPECT_EQ(last.action, LockedAccept::AcceptDriftStep);
}

TEST(RelPoseLockGateTest, ScatteredCandidatesNeverFireDriftStep)
{
	// Zero-mean scatter around the held answer: the median stays at the
	// held calibration, so no amount of churn fires a drift step.
	OversizeConsensusState st;
	SmallStepDriftState drift;
	const double scatter[3] = {2.0, 0.3, -2.0};
	for (int i = 0; i < 4 * spacecal::relpose_lock::kDriftWindowCandidates; ++i) {
		const auto d = Feed(scatter[i % 3], st, drift);
		ASSERT_EQ(d.action, LockedAccept::HoldPrior) << "iteration " << i;
	}
}

TEST(RelPoseLockGateTest, DriftStepQuietAfterAccept)
{
	// After a drift accept the caller's held calibration sits at the
	// cluster median; the same cluster then reads as sub-deadband steps
	// and the follower goes quiet on its own.
	OversizeConsensusState st;
	SmallStepDriftState drift;
	const double cluster[3] = {2.7, 3.0, 3.3};
	for (int i = 0; i < spacecal::relpose_lock::kDriftWindowCandidates; ++i) {
		Feed(cluster[i % 3], st, drift);
	}
	// Held has moved to the cluster; steps are now 0.0-0.3 cm.
	auto in = HealthyInputs();
	in.heldCm = Cand(3.0);
	for (int i = 0; i < spacecal::relpose_lock::kDriftWindowCandidates; ++i) {
		const double x = cluster[i % 3];
		in.stepCm = std::abs(x - 3.0);
		const auto d = EvaluateLockedAccept(in, Cand(x), st, drift);
		ASSERT_EQ(d.action, LockedAccept::HoldPrior) << "iteration " << i;
		ASSERT_STREQ(d.rejectTag, "relpose_step_deadband");
	}
}

TEST(RelPoseLockGateTest, BypassSkipsDriftGateAndResetsWindow)
{
	OversizeConsensusState st;
	SmallStepDriftState drift;
	// Partially fill the window with steady-state candidates.
	for (int i = 0; i < 5; ++i) {
		Feed(2.0, st, drift);
	}
	ASSERT_EQ(drift.count, 5);

	// A grace-window accept moves the held frame; the old cluster dies.
	auto in = HealthyInputs();
	in.stepGateBypassed = true;
	const auto d = EvaluateLockedAccept(in, Cand(2.0), st, drift);
	EXPECT_EQ(d.action, LockedAccept::Accept);
	EXPECT_EQ(drift.count, 0) << "Bypassed accepts restart the drift window";
}

// --- Oversize step + consensus escape -----------------------------------------

TEST(RelPoseLockGateTest, RejectsOversizedStepWithoutClassification)
{
	OversizeConsensusState st;
	SmallStepDriftState drift;
	auto in = HealthyInputs();
	in.stepCm = 400.0;
	const auto d = EvaluateLockedAccept(in, Cand(400.0), st, drift);
	EXPECT_EQ(d.action, LockedAccept::HoldPrior);
	EXPECT_STREQ(d.rejectTag, "relpose_step_oversized");
	EXPECT_EQ(st.streak, 1);
	EXPECT_EQ(drift.count, 0) << "Oversized candidates belong to another frame, not the drift window";
}

TEST(RelPoseLockGateTest, BypassAllowsOversizedStepDuringGrace)
{
	OversizeConsensusState st;
	SmallStepDriftState drift;
	auto in = HealthyInputs();
	in.stepCm = 400.0;
	in.stepGateBypassed = true;
	const auto d = EvaluateLockedAccept(in, Cand(400.0), st, drift);
	EXPECT_EQ(d.action, LockedAccept::Accept);
}

TEST(RelPoseLockGateTest, ConsensusEscapeAcceptsAfterAgreeingStreak)
{
	OversizeConsensusState st;
	SmallStepDriftState drift;
	// Seed the drift window so the escape's reset is observable.
	for (int i = 0; i < 5; ++i) {
		Feed(2.0, st, drift);
	}
	auto in = HealthyInputs();
	in.stepCm = 400.0;
	for (int i = 0; i < spacecal::relpose_lock::kOversizeConsensusCount - 1; ++i) {
		const auto d = EvaluateLockedAccept(in, Cand(400.0 + 0.1 * i), st, drift);
		ASSERT_EQ(d.action, LockedAccept::HoldPrior) << "hold " << i;
		ASSERT_STREQ(d.rejectTag, "relpose_step_oversized");
	}
	const auto escaped = EvaluateLockedAccept(in, Cand(400.0), st, drift);
	EXPECT_EQ(escaped.action, LockedAccept::AcceptConsensusStep);
	EXPECT_EQ(st.streak, 0) << "the escape consumes the streak";
	EXPECT_EQ(drift.count, 0) << "the escape moves the held frame; the drift window restarts";
}

TEST(RelPoseLockGateTest, DisagreeingOversizedCandidatesNeverEscape)
{
	OversizeConsensusState st;
	SmallStepDriftState drift;
	auto in = HealthyInputs();
	in.stepCm = 400.0;
	for (int i = 0; i < 4 * spacecal::relpose_lock::kOversizeConsensusCount; ++i) {
		// Alternate far apart: each candidate breaks the previous streak.
		const auto d = EvaluateLockedAccept(in, Cand(i % 2 == 0 ? 400.0 : 300.0), st, drift);
		ASSERT_EQ(d.action, LockedAccept::HoldPrior) << "iteration " << i;
	}
	EXPECT_LE(st.streak, 1);
}

TEST(RelPoseLockGateTest, QualityRejectResetsConsensusStreak)
{
	OversizeConsensusState st;
	SmallStepDriftState drift;
	auto in = HealthyInputs();
	in.stepCm = 400.0;
	EvaluateLockedAccept(in, Cand(400.0), st, drift);
	EvaluateLockedAccept(in, Cand(400.0), st, drift);
	ASSERT_EQ(st.streak, 2);

	auto bad = in;
	bad.candidateErrorM = 0.050; // fails the band
	const auto d = EvaluateLockedAccept(bad, Cand(400.0), st, drift);
	EXPECT_EQ(d.action, LockedAccept::HoldPrior);
	EXPECT_EQ(st.streak, 0);
}

TEST(RelPoseLockGateTest, FirstCandidateWithoutPriorAccepts)
{
	OversizeConsensusState st;
	SmallStepDriftState drift;
	auto in = HealthyInputs();
	in.havePrior = false;
	in.priorErrorM = INFINITY;
	in.stepCm = 0.0;
	const auto d = EvaluateLockedAccept(in, Cand(0.0), st, drift);
	EXPECT_EQ(d.action, LockedAccept::Accept);
}

// --- Constants pinned ---------------------------------------------------------

static_assert(spacecal::relpose_lock::kDriftWindowCandidates == 30,
              "kDriftWindowCandidates changed -- rebalance against the solve cadence: the window is the "
              "drift-correction latency and the noise averager at once");
static_assert(spacecal::relpose_lock::kDriftStepMinCm == 1.5,
              "kDriftStepMinCm changed -- drift below this stays uncorrected; keep it near the persist "
              "deadband so the answer is stable on disk too");
static_assert(spacecal::relpose_lock::kDriftCandidateAgreeCm == 1.0,
              "kDriftCandidateAgreeCm changed -- the applied candidate must stay a typical cluster member "
              "or the accepted rotation can be an outlier's");
