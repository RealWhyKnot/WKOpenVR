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
