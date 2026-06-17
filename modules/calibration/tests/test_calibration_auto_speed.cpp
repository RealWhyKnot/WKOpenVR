#include "CalibrationAutoSpeed.h"

#include <gtest/gtest.h>

#include <limits>

namespace cas = spacecal::calibration_speed;

TEST(CalibrationAutoSpeedTest, FitRmsBucketsMatchDocumentedThresholds)
{
	EXPECT_EQ(cas::BucketForObservedFitRmsMm(0.0), cas::AutoSpeedBucket::Fast);
	EXPECT_EQ(cas::BucketForObservedFitRmsMm(4.99), cas::AutoSpeedBucket::Fast);
	EXPECT_EQ(cas::BucketForObservedFitRmsMm(5.0), cas::AutoSpeedBucket::Slow);
	EXPECT_EQ(cas::BucketForObservedFitRmsMm(9.99), cas::AutoSpeedBucket::Slow);
	EXPECT_EQ(cas::BucketForObservedFitRmsMm(10.0), cas::AutoSpeedBucket::VerySlow);
}

TEST(CalibrationAutoSpeedTest, NonFiniteFitFallsBackToFast)
{
	EXPECT_EQ(cas::BucketForObservedFitRmsMm(std::numeric_limits<double>::infinity()), cas::AutoSpeedBucket::Fast);
	EXPECT_EQ(cas::BucketForObservedFitRmsMm(std::numeric_limits<double>::quiet_NaN()), cas::AutoSpeedBucket::Fast);
}

TEST(CalibrationAutoSpeedTest, CandidateFitRmsBeatsRawMotionSpread)
{
	const double rawWorldPositionSpreadMm = 720.0;
	const double candidateFitRmsMm = 3.0;

	EXPECT_EQ(cas::BucketForObservedFitRmsMm(candidateFitRmsMm), cas::AutoSpeedBucket::Fast);
	EXPECT_EQ(cas::BucketForObservedFitRmsMm(rawWorldPositionSpreadMm), cas::AutoSpeedBucket::VerySlow);
}

TEST(CalibrationAutoSpeedTest, CurrentFitUsedWhenCandidateIsUnavailable)
{
	const double selected = cas::SelectObservedFitRmsMm(std::numeric_limits<double>::infinity(), 7.0);

	EXPECT_DOUBLE_EQ(selected, 7.0);
	EXPECT_EQ(cas::BucketForObservedFitRmsMm(selected), cas::AutoSpeedBucket::Slow);
}

TEST(CalibrationAutoSpeedTest, ZeroFitRmsIsTreatedAsUnavailable)
{
	EXPECT_FALSE(cas::IsUsableFitRmsMm(0.0));

	const double selected = cas::SelectObservedFitRmsMm(0.0, 8.0);
	EXPECT_DOUBLE_EQ(selected, 8.0);
	EXPECT_EQ(cas::BucketForObservedFitRmsMm(selected), cas::AutoSpeedBucket::Slow);
}

TEST(CalibrationAutoSpeedTest, MissingFitRmsReturnsNaN)
{
	const double selected = cas::SelectObservedFitRmsMm(0.0, 0.0);
	EXPECT_TRUE(std::isnan(selected));
	EXPECT_EQ(cas::BucketForObservedFitRmsMm(selected), cas::AutoSpeedBucket::Fast);
}

TEST(CalibrationAutoSpeedTest, ReducibleDriftUsesFastWhileConverging)
{
	cas::AutoSpeedState state;
	const auto decision = cas::ResolveAutoSpeed(state, 32.0, 4.0);

	EXPECT_EQ(decision.state.phase, cas::AutoSpeedPhase::Converging);
	EXPECT_EQ(decision.bucket, cas::AutoSpeedBucket::Fast);
	EXPECT_DOUBLE_EQ(decision.reducibleMm, 28.0);
}

TEST(CalibrationAutoSpeedTest, SettledCleanFitUsesFast)
{
	cas::AutoSpeedState state;
	const auto decision = cas::ResolveAutoSpeed(state, 4.0, 3.0);

	EXPECT_EQ(decision.state.phase, cas::AutoSpeedPhase::Settled);
	EXPECT_EQ(decision.bucket, cas::AutoSpeedBucket::Fast);
}

TEST(CalibrationAutoSpeedTest, SettledNoisyFitUsesNoiseFloorBucket)
{
	cas::AutoSpeedState state;
	const auto decision = cas::ResolveAutoSpeed(state, 12.5, 12.0);

	EXPECT_EQ(decision.state.phase, cas::AutoSpeedPhase::Settled);
	EXPECT_EQ(decision.bucket, cas::AutoSpeedBucket::VerySlow);
	EXPECT_DOUBLE_EQ(decision.reducibleMm, 0.5);
}

TEST(CalibrationAutoSpeedTest, ConvergingRequiresDwellBeforeSettling)
{
	cas::AutoSpeedState state;
	state.phase = cas::AutoSpeedPhase::Converging;

	cas::AutoSpeedDecision decision;
	for (int i = 0; i < cas::kSettleDwellTicks - 1; ++i) {
		decision = cas::ResolveAutoSpeed(state, 4.5, 3.0);
		state = decision.state;
		EXPECT_EQ(state.phase, cas::AutoSpeedPhase::Converging);
		EXPECT_EQ(decision.bucket, cas::AutoSpeedBucket::Fast);
	}

	decision = cas::ResolveAutoSpeed(state, 4.5, 3.0);
	EXPECT_EQ(decision.state.phase, cas::AutoSpeedPhase::Settled);
	EXPECT_EQ(decision.bucket, cas::AutoSpeedBucket::Fast);
}

TEST(CalibrationAutoSpeedTest, HysteresisPreventsFlapping)
{
	cas::AutoSpeedState state;
	state.phase = cas::AutoSpeedPhase::Converging;

	auto decision = cas::ResolveAutoSpeed(state, 7.0, 3.0);
	EXPECT_EQ(decision.state.phase, cas::AutoSpeedPhase::Converging);
	EXPECT_EQ(decision.state.settleTicks, 0);

	decision = cas::ResolveAutoSpeed(decision.state, 9.5, 3.0);
	EXPECT_EQ(decision.state.phase, cas::AutoSpeedPhase::Converging);
	EXPECT_EQ(decision.state.settleTicks, 0);
}

TEST(CalibrationAutoSpeedTest, MissingFreshFitTreatsCorrectionAsReducible)
{
	const auto decision = cas::ResolveAutoSpeed({}, 15.0, std::numeric_limits<double>::quiet_NaN());

	EXPECT_EQ(decision.state.phase, cas::AutoSpeedPhase::Converging);
	EXPECT_EQ(decision.bucket, cas::AutoSpeedBucket::Fast);
	EXPECT_DOUBLE_EQ(decision.reducibleMm, 15.0);
}
