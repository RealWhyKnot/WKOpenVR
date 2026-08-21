#include "CalibrationAutoSpeed.h"

#include <gtest/gtest.h>

#include <limits>

namespace cas = spacecal::calibration_speed;

TEST(CalibrationAutoSpeedTest, CurrentFitUsedWhenCandidateIsUnavailable)
{
	EXPECT_DOUBLE_EQ(cas::SelectObservedFitRmsMm(std::numeric_limits<double>::infinity(), 7.0), 7.0);
}

TEST(CalibrationAutoSpeedTest, ZeroFitRmsIsTreatedAsUnavailable)
{
	EXPECT_FALSE(cas::IsUsableFitRmsMm(0.0));
	EXPECT_DOUBLE_EQ(cas::SelectObservedFitRmsMm(0.0, 8.0), 8.0);
}

TEST(CalibrationAutoSpeedTest, MissingFitRmsReturnsNaN)
{
	EXPECT_TRUE(std::isnan(cas::SelectObservedFitRmsMm(0.0, 0.0)));
}

TEST(CalibrationAutoSpeedTest, CandidateFitPreferredWhenUsable)
{
	EXPECT_DOUBLE_EQ(cas::SelectObservedFitRmsMm(3.0, 8.0), 3.0);
}
