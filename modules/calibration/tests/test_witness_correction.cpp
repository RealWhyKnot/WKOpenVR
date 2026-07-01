// Unit tests for the witness-correction pure helpers (WitnessCorrection.h):
// the T1 baseline accumulator and the T2 slew-limited, frame-mapped correction
// step. The end-to-end drift reduction is validated separately on real
// recordings via WitnessDriftReplay.h.

#include "WitnessCorrection.h"

#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <cmath>

using spacecal::witness_correction::BaselineAccumulator;
using spacecal::witness_correction::CorrectionDeltaCm;
using spacecal::witness_correction::EvaluateRunawayGuard;
using spacecal::witness_correction::GuardVerdict;
using spacecal::witness_correction::ResetRunawayGuard;
using spacecal::witness_correction::RunawayGuardState;

namespace {
constexpr double kDt = 1.0 / 3.5; // continuous-cal cadence
}

TEST(WitnessBaselineAccumulatorTest, NotReadyBeforeEnoughStableSamples)
{
	BaselineAccumulator acc;
	for (int i = 0; i < 30; ++i)
		acc.Add(Eigen::Vector3d(0.1, -0.05, 0.08), i * 0.1);
	EXPECT_FALSE(acc.Ready(30 * 0.1)) << "30 samples < the 60-sample floor";
}

TEST(WitnessBaselineAccumulatorTest, ReadyWhenStableAndCommitsMean)
{
	BaselineAccumulator acc;
	const Eigen::Vector3d off(0.10, -0.05, 0.08);
	for (int i = 0; i < 70; ++i)
		acc.Add(off, i * 0.1); // 70 samples over ~7 s, zero spread
	EXPECT_TRUE(acc.Ready(70 * 0.1));
	EXPECT_LT((acc.Mean() - off).norm(), 1e-9);
	EXPECT_LT(acc.StdM(), 1e-4) << "identical samples -> ~zero spread (float cancellation aside)";
}

TEST(WitnessBaselineAccumulatorTest, NotReadyWhenNoisy)
{
	BaselineAccumulator acc;
	for (int i = 0; i < 100; ++i) {
		// Alternate 0 and 30 mm -> ~15 mm std, above the 10 mm stability floor.
		acc.Add(Eigen::Vector3d((i % 2) ? 0.03 : 0.0, 0.0, 0.0), i * 0.1);
	}
	EXPECT_GT(acc.StdM(), 0.010);
	EXPECT_FALSE(acc.Ready(100 * 0.1));
}

TEST(WitnessBaselineAccumulatorTest, ResetClears)
{
	BaselineAccumulator acc;
	for (int i = 0; i < 70; ++i)
		acc.Add(Eigen::Vector3d(0.1, 0, 0), i * 0.1);
	acc.Reset();
	EXPECT_EQ(acc.count, 0);
	EXPECT_FALSE(acc.Ready(100.0));
}

TEST(WitnessCorrectionDeltaTest, InsideDeadbandIsZero)
{
	// 2 mm drift, below the 3 mm adaptive dead-band floor -> no correction.
	const Eigen::Vector3d d =
	    CorrectionDeltaCm(Eigen::Vector3d(0.002, 0, 0), Eigen::Vector3d::Zero(), Eigen::Matrix3d::Identity(), 0.0, kDt);
	EXPECT_LT(d.norm(), 1e-9);
}

TEST(WitnessCorrectionDeltaTest, AboveThirtyCentimetreCapIsZero)
{
	// 40 cm drift is a relocalization (recovery's job), not slow drift.
	const Eigen::Vector3d d =
	    CorrectionDeltaCm(Eigen::Vector3d(0.40, 0, 0), Eigen::Vector3d::Zero(), Eigen::Matrix3d::Identity(), 0.0, kDt);
	EXPECT_LT(d.norm(), 1e-9);
}

TEST(WitnessCorrectionDeltaTest, MidDriftStepsTowardBaseline)
{
	// 5 cm drift along +x, identity HMD rotation -> step points -x, magnitude = slew*dt.
	const Eigen::Vector3d d =
	    CorrectionDeltaCm(Eigen::Vector3d(0.05, 0, 0), Eigen::Vector3d::Zero(), Eigen::Matrix3d::Identity(), 0.0, kDt);
	EXPECT_LT(d.x(), 0.0) << "correction opposes the drift";
	EXPECT_NEAR(d.y(), 0.0, 1e-9);
	EXPECT_NEAR(d.z(), 0.0, 1e-9);
	// slew 3 mm/s * dt seconds, expressed in cm.
	const double expectedCm = spacecal::cont_correction::kCorrectionSlewMps * kDt * 100.0;
	EXPECT_NEAR(d.norm(), expectedCm, 1e-6);
}

TEST(WitnessCorrectionDeltaTest, MapsThroughHmdRotation)
{
	// Drift along +x in the HMD-local frame; a 90 deg yaw maps the -x correction to -y.
	const Eigen::Matrix3d rot = Eigen::AngleAxisd(EIGEN_PI / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
	const Eigen::Vector3d d = CorrectionDeltaCm(Eigen::Vector3d(0.05, 0, 0), Eigen::Vector3d::Zero(), rot, 0.0, kDt);
	EXPECT_NEAR(d.x(), 0.0, 1e-6);
	EXPECT_LT(d.y(), 0.0);
}

TEST(WitnessCorrectionDeltaTest, AdaptiveDeadbandWidensWithMadFloor)
{
	// 8 mm drift: corrected with a 3 mm floor, but ignored when the mad floor is 12 mm.
	const Eigen::Vector3d drift(0.008, 0, 0);
	const Eigen::Vector3d lowFloor =
	    CorrectionDeltaCm(drift, Eigen::Vector3d::Zero(), Eigen::Matrix3d::Identity(), 0.003, kDt);
	const Eigen::Vector3d highFloor =
	    CorrectionDeltaCm(drift, Eigen::Vector3d::Zero(), Eigen::Matrix3d::Identity(), 0.012, kDt);
	EXPECT_GT(lowFloor.norm(), 0.0);
	EXPECT_LT(highFloor.norm(), 1e-9) << "a wide mad floor suppresses correction of small drift";
}

// --- Runaway / non-convergence guard (the safety net that bounds the field
// 56.8 cm divergence). Cap = 0.20 m; window = 60 s / >=5 cm applied / >=20 mm drift.

TEST(WitnessRunawayGuardTest, TripsOnCumulativeCap)
{
	RunawayGuardState g;
	// 25 cm cumulative exceeds the 20 cm cap regardless of drift or window.
	EXPECT_EQ(EvaluateRunawayGuard(g, 25.0, 5.0, 0.0), GuardVerdict::TripCumulative);
}

TEST(WitnessRunawayGuardTest, OkBelowCapWithinWindow)
{
	RunawayGuardState g;
	EXPECT_EQ(EvaluateRunawayGuard(g, 3.0, 5.0, 0.0), GuardVerdict::Ok);
}

TEST(WitnessRunawayGuardTest, NonConvergeTripsWhenDriftStaysHigh)
{
	RunawayGuardState g;
	EXPECT_EQ(EvaluateRunawayGuard(g, 0.0, 42.0, 0.0), GuardVerdict::Ok); // opens the window
	// 61 s later: >=5 cm applied yet drift never fell below 20 mm -> loop not working.
	EXPECT_EQ(EvaluateRunawayGuard(g, 8.0, 42.0, 61.0), GuardVerdict::TripNonConverge);
}

TEST(WitnessRunawayGuardTest, NonConvergeNoTripWhenDriftShrinks)
{
	RunawayGuardState g;
	EvaluateRunawayGuard(g, 0.0, 42.0, 0.0);
	// Applied a lot, but drift dropped under the 20 mm floor -> the loop is working.
	EXPECT_EQ(EvaluateRunawayGuard(g, 8.0, 5.0, 61.0), GuardVerdict::Ok);
}

TEST(WitnessRunawayGuardTest, NonConvergeNoTripWhenLittleApplied)
{
	RunawayGuardState g;
	EvaluateRunawayGuard(g, 0.0, 42.0, 0.0);
	// Only 2 cm applied over the window (<5 cm) -> not enough correction to judge.
	EXPECT_EQ(EvaluateRunawayGuard(g, 2.0, 42.0, 61.0), GuardVerdict::Ok);
}

TEST(WitnessRunawayGuardTest, WindowResetsAndJudgesIndependently)
{
	RunawayGuardState g;
	EvaluateRunawayGuard(g, 0.0, 42.0, 0.0);                                // window A opens
	EXPECT_EQ(EvaluateRunawayGuard(g, 2.0, 42.0, 61.0), GuardVerdict::Ok);  // window A: <5 cm, resets to B at t=61
	// Window B (from t=61): +8 cm applied, drift still high -> trips independently.
	EXPECT_EQ(EvaluateRunawayGuard(g, 10.0, 42.0, 122.0), GuardVerdict::TripNonConverge);
}

TEST(WitnessRunawayGuardTest, ResetClears)
{
	RunawayGuardState g;
	EvaluateRunawayGuard(g, 10.0, 42.0, 0.0);
	ResetRunawayGuard(g);
	EXPECT_EQ(g.appliedTotalCm, 0.0);
	EXPECT_LT(g.windowStartTime, 0.0);
}
