#include <gtest/gtest.h>

#include <Eigen/Dense>

#include "RelPoseLockGate.h"

using spacecal::relpose_lock::EvaluateLockedAccept;
using spacecal::relpose_lock::LockedAccept;
using spacecal::relpose_lock::LockedAcceptInputs;
using spacecal::relpose_lock::OversizeConsensusState;

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

} // namespace

TEST(RelPoseLockGateTest, AcceptsWithinBandAndAboveDeadband)
{
	OversizeConsensusState st;
	const auto d = EvaluateLockedAccept(HealthyInputs(), Cand(2.0), st);
	EXPECT_EQ(d.action, LockedAccept::Accept);
	EXPECT_EQ(d.rejectTag, nullptr);
}

TEST(RelPoseLockGateTest, RejectsAboveAcceptBand)
{
	OversizeConsensusState st;
	auto in = HealthyInputs();
	in.candidateErrorM = 0.012; // above the 10 mm fresh band
	const auto d = EvaluateLockedAccept(in, Cand(2.0), st);
	EXPECT_EQ(d.action, LockedAccept::HoldPrior);
	EXPECT_STREQ(d.rejectTag, "relpose_band");
}

TEST(RelPoseLockGateTest, WiderBandWhenRelPoseCalibrated)
{
	// A banked relpose widens the band 10 -> 25 mm; the same candidate gets
	// past the band and is caught by the max-error cap instead.
	OversizeConsensusState st;
	auto in = HealthyInputs();
	in.candidateErrorM = 0.012;
	in.relPosCalibrated = true;
	const auto d = EvaluateLockedAccept(in, Cand(2.0), st);
	EXPECT_EQ(d.action, LockedAccept::HoldPrior);
	EXPECT_STREQ(d.rejectTag, "relpose_max_error");
}

TEST(RelPoseLockGateTest, WidenedMaxErrorCapWidensBand)
{
	// Offline harnesses pass a deliberately loose cap; the band follows it so
	// they can score raw candidates the live gate would refuse.
	OversizeConsensusState st;
	auto in = HealthyInputs();
	in.candidateErrorM = 0.055;
	in.priorErrorM = 0.055;
	in.maxErrorM = 1.0;
	const auto d = EvaluateLockedAccept(in, Cand(2.0), st);
	EXPECT_EQ(d.action, LockedAccept::Accept);
}

TEST(RelPoseLockGateTest, MaxErrorCapParity)
{
	OversizeConsensusState st;
	auto in = HealthyInputs();
	in.candidateErrorM = 0.006; // inside the band, above the cap
	const auto d = EvaluateLockedAccept(in, Cand(2.0), st);
	EXPECT_EQ(d.action, LockedAccept::HoldPrior);
	EXPECT_STREQ(d.rejectTag, "relpose_max_error");
}

TEST(RelPoseLockGateTest, HoldsWhenWorseThanPriorByRatio)
{
	OversizeConsensusState st;
	auto in = HealthyInputs();
	in.priorErrorM = 0.001;
	in.candidateErrorM = 0.002; // 2x worse than prior, ratio 1.5
	const auto d = EvaluateLockedAccept(in, Cand(2.0), st);
	EXPECT_EQ(d.action, LockedAccept::HoldPrior);
	EXPECT_STREQ(d.rejectTag, "relpose_worse_than_prior");
}

TEST(RelPoseLockGateTest, SteadyStateEqualErrorStillAccepts)
{
	// Under lock, steady-state candidates hover at ~prior error. A strict
	// improve gate would starve drift correction; equal error must accept.
	OversizeConsensusState st;
	auto in = HealthyInputs();
	in.priorErrorM = 0.002;
	in.candidateErrorM = 0.002;
	const auto d = EvaluateLockedAccept(in, Cand(2.0), st);
	EXPECT_EQ(d.action, LockedAccept::Accept);
}

TEST(RelPoseLockGateTest, HoldsBelowStepDeadband)
{
	OversizeConsensusState st;
	auto in = HealthyInputs();
	in.stepCm = 0.4;
	const auto d = EvaluateLockedAccept(in, Cand(0.4), st);
	EXPECT_EQ(d.action, LockedAccept::HoldPrior);
	EXPECT_STREQ(d.rejectTag, "relpose_step_deadband");
}

TEST(RelPoseLockGateTest, ImprovementBeatsDeadband)
{
	OversizeConsensusState st;
	auto in = HealthyInputs();
	in.stepCm = 0.4;
	in.priorErrorM = 0.004;
	in.candidateErrorM = 0.002; // 50% better than prior
	const auto d = EvaluateLockedAccept(in, Cand(0.4), st);
	EXPECT_EQ(d.action, LockedAccept::Accept);
}

TEST(RelPoseLockGateTest, RejectsOversizedStepWithoutClassification)
{
	OversizeConsensusState st;
	auto in = HealthyInputs();
	in.stepCm = 400.0;
	const auto d = EvaluateLockedAccept(in, Cand(400.0), st);
	EXPECT_EQ(d.action, LockedAccept::HoldPrior);
	EXPECT_STREQ(d.rejectTag, "relpose_step_oversized");
	EXPECT_EQ(st.streak, 1);
}

TEST(RelPoseLockGateTest, BypassAllowsOversizedStepDuringGrace)
{
	OversizeConsensusState st;
	auto in = HealthyInputs();
	in.stepCm = 400.0;
	in.stepGateBypassed = true;
	const auto d = EvaluateLockedAccept(in, Cand(400.0), st);
	EXPECT_EQ(d.action, LockedAccept::Accept);
}

TEST(RelPoseLockGateTest, ConsensusEscapeAcceptsAfterAgreeingStreak)
{
	OversizeConsensusState st;
	auto in = HealthyInputs();
	in.stepCm = 400.0;
	for (int i = 0; i < spacecal::relpose_lock::kOversizeConsensusCount - 1; ++i) {
		const auto d = EvaluateLockedAccept(in, Cand(400.0 + 0.1 * i), st);
		ASSERT_EQ(d.action, LockedAccept::HoldPrior) << "hold " << i;
		ASSERT_STREQ(d.rejectTag, "relpose_step_oversized");
	}
	const auto escaped = EvaluateLockedAccept(in, Cand(400.0), st);
	EXPECT_EQ(escaped.action, LockedAccept::AcceptConsensusStep);
	EXPECT_EQ(st.streak, 0) << "the escape consumes the streak";
}

TEST(RelPoseLockGateTest, DisagreeingOversizedCandidatesNeverEscape)
{
	OversizeConsensusState st;
	auto in = HealthyInputs();
	in.stepCm = 400.0;
	for (int i = 0; i < 4 * spacecal::relpose_lock::kOversizeConsensusCount; ++i) {
		// Alternate far apart: each candidate breaks the previous streak.
		const auto d = EvaluateLockedAccept(in, Cand(i % 2 == 0 ? 400.0 : 300.0), st);
		ASSERT_EQ(d.action, LockedAccept::HoldPrior) << "iteration " << i;
	}
	EXPECT_LE(st.streak, 1);
}

TEST(RelPoseLockGateTest, QualityRejectResetsConsensusStreak)
{
	OversizeConsensusState st;
	auto in = HealthyInputs();
	in.stepCm = 400.0;
	EvaluateLockedAccept(in, Cand(400.0), st);
	EvaluateLockedAccept(in, Cand(400.0), st);
	ASSERT_EQ(st.streak, 2);

	auto bad = in;
	bad.candidateErrorM = 0.050; // fails the band
	const auto d = EvaluateLockedAccept(bad, Cand(400.0), st);
	EXPECT_EQ(d.action, LockedAccept::HoldPrior);
	EXPECT_EQ(st.streak, 0);
}

TEST(RelPoseLockGateTest, FirstCandidateWithoutPriorAccepts)
{
	OversizeConsensusState st;
	auto in = HealthyInputs();
	in.havePrior = false;
	in.priorErrorM = INFINITY;
	in.stepCm = 0.0;
	const auto d = EvaluateLockedAccept(in, Cand(0.0), st);
	EXPECT_EQ(d.action, LockedAccept::Accept);
}
