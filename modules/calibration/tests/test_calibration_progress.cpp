#include <gtest/gtest.h>

#include "CalibrationProgress.h"

namespace progress = spacecal::calibration_progress;

TEST(CalibrationProgressTest, IdleStatesUseSampleFill)
{
	EXPECT_DOUBLE_EQ(0.25, progress::OneShotReadyScore(
		CalibrationState::None,
		25,
		100,
		0.0,
		0.0));
	EXPECT_DOUBLE_EQ(0.5, progress::OneShotReadyScore(
		CalibrationState::Continuous,
		50,
		100,
		0.0,
		0.0));
}

TEST(CalibrationProgressTest, BeginStateShowsNotReady)
{
	EXPECT_DOUBLE_EQ(0.0, progress::OneShotReadyScore(
		CalibrationState::Begin,
		100,
		100,
		1.0,
		1.0));
}

TEST(CalibrationProgressTest, RotationPhaseCannotReachFullReady)
{
	EXPECT_DOUBLE_EQ(0.5, progress::OneShotReadyScore(
		CalibrationState::Rotation,
		200,
		200,
		progress::kOneShotRotationReadyDiversity,
		0.0));
}

TEST(CalibrationProgressTest, RotationPhaseNeedsSamplesAndMotion)
{
	EXPECT_DOUBLE_EQ(0.25, progress::OneShotReadyScore(
		CalibrationState::Rotation,
		200,
		200,
		progress::kOneShotRotationReadyDiversity * 0.5,
		0.0));
	EXPECT_DOUBLE_EQ(0.25, progress::OneShotReadyScore(
		CalibrationState::Rotation,
		100,
		200,
		progress::kOneShotRotationReadyDiversity,
		0.0));
}

TEST(CalibrationProgressTest, TranslationPhaseStartsAtHalf)
{
	EXPECT_DOUBLE_EQ(0.5, progress::OneShotReadyScore(
		CalibrationState::Translation,
		0,
		200,
		0.0,
		0.0));
}

TEST(CalibrationProgressTest, TranslationPhaseReachesFullOnlyWhenReady)
{
	EXPECT_DOUBLE_EQ(1.0, progress::OneShotReadyScore(
		CalibrationState::Translation,
		200,
		200,
		0.0,
		progress::kOneShotTranslationReadyDiversity));
}

TEST(CalibrationProgressTest, TranslationPhaseNeedsSamplesAndMotion)
{
	EXPECT_DOUBLE_EQ(0.75, progress::OneShotReadyScore(
		CalibrationState::Translation,
		200,
		200,
		0.0,
		progress::kOneShotTranslationReadyDiversity * 0.5));
	EXPECT_DOUBLE_EQ(0.75, progress::OneShotReadyScore(
		CalibrationState::Translation,
		100,
		200,
		0.0,
		progress::kOneShotTranslationReadyDiversity));
}

TEST(CalibrationProgressTest, SampleFillHandlesInvalidTargets)
{
	EXPECT_DOUBLE_EQ(0.0, progress::SampleFillScore(10, 0));
	EXPECT_DOUBLE_EQ(0.0, progress::SampleFillScore(10, -1));
	EXPECT_DOUBLE_EQ(1.0, progress::SampleFillScore(300, 200));
}
