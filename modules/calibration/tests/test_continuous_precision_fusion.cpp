// Confidence-weighted continuous-fusion tests.
//
// Pins the pure helpers in ContinuousPrecisionFusion.h: a calibration
// measurement's trust falls with its lever arm, the fusion gain self-normalises
// against accumulated confidence, and -- the point of the whole thing -- a
// calibration banked with good (near-origin) geometry barely moves when a far
// session floods it with low-confidence readings.

#include "ContinuousPrecisionFusion.h"

#include <gtest/gtest.h>

#include <Eigen/Geometry>

namespace pf = spacecal::precision;

namespace {
Eigen::AffineCompact3d T(double x, double y, double z)
{
	Eigen::AffineCompact3d a = Eigen::AffineCompact3d::Identity();
	a.translation() = Eigen::Vector3d(x, y, z);
	return a;
}
} // namespace

TEST(ContinuousPrecisionFusionTest, PrecisionFallsWithLeverArm)
{
	// Near origin is far more precise than far from origin, monotonically.
	EXPECT_GT(pf::MeasurementPrecision(0.04), pf::MeasurementPrecision(1.0));
	EXPECT_GT(pf::MeasurementPrecision(1.0), pf::MeasurementPrecision(12.0));
	// Regularizer keeps it finite at the origin.
	EXPECT_LT(pf::MeasurementPrecision(0.0), 1.0 / pf::kLeverRegM2 + 1e-9);
	EXPECT_GT(pf::MeasurementPrecision(0.0), 0.0);
}

TEST(ContinuousPrecisionFusionTest, GainSelfNormalises)
{
	const double m = pf::MeasurementPrecision(1.0);
	EXPECT_DOUBLE_EQ(pf::FusionGain(0.0, m), 1.0) << "first measurement is adopted whole";
	// More accumulated confidence -> smaller gain, strictly.
	EXPECT_LT(pf::FusionGain(100.0, m), pf::FusionGain(10.0, m));
	EXPECT_LT(pf::FusionGain(1e6, m), 1e-3) << "a well-established estimate barely moves";
}

TEST(ContinuousPrecisionFusionTest, FuseEndpointsAndMidpoint)
{
	const Eigen::AffineCompact3d a = T(1, 2, 3);
	const Eigen::AffineCompact3d b = T(3, 4, 5);
	EXPECT_LT((pf::Fuse(a, b, 0.0).translation() - a.translation()).norm(), 1e-12);
	EXPECT_LT((pf::Fuse(a, b, 1.0).translation() - b.translation()).norm(), 1e-12);
	EXPECT_LT((pf::Fuse(a, b, 0.5).translation() - Eigen::Vector3d(2, 3, 4)).norm(), 1e-12);
	// Gain is clamped, so out-of-range values don't overshoot.
	EXPECT_LT((pf::Fuse(a, b, 2.0).translation() - b.translation()).norm(), 1e-12);
}

// The headline property: bank a calibration from near-origin readings, then run
// a far-the-whole-time session of biased readings. The banked calibration must
// stay close to the truth -- far readings carry too little confidence to drag it.
TEST(ContinuousPrecisionFusionTest, BankedCalibrationSurvivesFarSession)
{
	const Eigen::AffineCompact3d truth = T(0.5, 0.3, -0.2);
	const Eigen::AffineCompact3d wrong = T(3.0, -2.0, 1.0); // what a far, biased solve reports

	// Establish near origin (lever arm ~0.45 m -> squared ~0.2): converges fast.
	Eigen::AffineCompact3d est = Eigen::AffineCompact3d::Identity();
	double accum = 0.0;
	for (int i = 0; i < 120; ++i) {
		const double m = pf::MeasurementPrecision(0.2);
		est = pf::Fuse(est, truth, pf::FusionGain(accum, m));
		accum = std::min(accum + m, pf::kMaxConfidence);
	}
	ASSERT_LT((est.translation() - truth.translation()).norm(), 0.01) << "should converge near origin";

	// A far-from-origin gain must now be tiny compared to the near-origin one.
	EXPECT_LT(pf::FusionGain(accum, pf::MeasurementPrecision(12.0)), 0.01);

	// Flood a long far session of a biased reading. Overwriting would land at
	// `wrong` (~3.6 m off); the fusion must hold most of the ground -- it resists
	// strongly, though a *sustained* bias still leaks slowly (which is why
	// persisting earned confidence is the follow-up).
	const double fullJump = (wrong.translation() - truth.translation()).norm();
	for (int i = 0; i < 600; ++i) {
		const double m = pf::MeasurementPrecision(12.0); // ~2.4 m from each origin
		est = pf::Fuse(est, wrong, pf::FusionGain(accum, m));
		accum = std::min(accum + m, pf::kMaxConfidence);
	}
	const double drift = (est.translation() - truth.translation()).norm();
	EXPECT_LT(drift, 0.20 * fullJump) << "far session dragged the banked calibration off by " << drift << " m of "
	                                  << fullJump << " m";
}

TEST(ContinuousPrecisionFusionTest, StaleSeedBreakerTripsAfterConsecutiveDisagreements)
{
	int streak = 0;
	for (int i = 0; i < pf::kStaleSeedTripCount - 1; ++i) {
		EXPECT_FALSE(pf::NoteSeedDisagreement(streak, 2.5)) << "trip " << i << " fired early";
	}
	EXPECT_TRUE(pf::NoteSeedDisagreement(streak, 2.5));
	EXPECT_EQ(streak, 0) << "streak must reset after a trip";
}

TEST(ContinuousPrecisionFusionTest, StaleSeedBreakerStreakResetsOnAgreement)
{
	int streak = 0;
	EXPECT_FALSE(pf::NoteSeedDisagreement(streak, 3.0));
	EXPECT_FALSE(pf::NoteSeedDisagreement(streak, 3.0));
	// One in-band candidate (the far-from-origin noise profile: mostly sub-metre
	// scatter with isolated large outliers) clears the streak.
	EXPECT_FALSE(pf::NoteSeedDisagreement(streak, 0.4));
	EXPECT_EQ(streak, 0);
	EXPECT_FALSE(pf::NoteSeedDisagreement(streak, 3.0));
	EXPECT_FALSE(pf::NoteSeedDisagreement(streak, 3.0));
	EXPECT_TRUE(pf::NoteSeedDisagreement(streak, 3.0));
}

TEST(ContinuousPrecisionFusionTest, StaleSeedBreakerBoundaryAtDisagreeThreshold)
{
	int streak = 0;
	for (int i = 0; i < 10 * pf::kStaleSeedTripCount; ++i) {
		EXPECT_FALSE(pf::NoteSeedDisagreement(streak, pf::kStaleSeedDisagreeM - 0.01)) << "sub-threshold tripped";
	}
	for (int i = 0; i < pf::kStaleSeedTripCount - 1; ++i) {
		EXPECT_FALSE(pf::NoteSeedDisagreement(streak, pf::kStaleSeedDisagreeM));
	}
	EXPECT_TRUE(pf::NoteSeedDisagreement(streak, pf::kStaleSeedDisagreeM));
}
