// Sequential-validation tests: the SE(3) Log/Exp convention pins and the
// Wald test's decision behaviour under valid-profile vs frame-moved residual
// streams.

#include <gtest/gtest.h>

#include "LeverArmCovariance.h"
#include "Se3Log.h"
#include "SprtValidation.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <random>

namespace se3 = spacecal::se3;
namespace sprt = spacecal::sprt;
namespace levercov = spacecal::levercov;

// ---------------------------------------------------------------------------
// Se3Log.h convention pins.

// Ordering pin: [rho(3); phi(3)], translation FIRST. For a pure translation,
// phi = 0 and rho equals the translation exactly.
TEST(Se3LogTest, PureTranslationOrderingPin)
{
	Eigen::AffineCompact3d T = Eigen::AffineCompact3d::Identity();
	T.translation() = Eigen::Vector3d(0.1, -0.2, 0.3);
	const auto xi = se3::LogSE3(T);
	EXPECT_LT((xi.head<3>() - T.translation()).norm(), 1e-12);
	EXPECT_LT(xi.tail<3>().norm(), 1e-12);
}

TEST(Se3LogTest, ExpLogRoundTrip)
{
	const struct
	{
		Eigen::Vector3d rho;
		Eigen::Vector3d phi;
	} cases[] = {
	    {{0.1, 0.2, -0.3}, {0.3, -0.2, 0.5}},    {{-1.0, 2.0, 0.5}, {0.0, 0.0, 0.0}},
	    {{0.01, 0.0, 0.0}, {1e-7, -2e-7, 1e-7}}, // tiny-angle Taylor branch
	    {{0.5, -0.5, 0.5}, {0.0, 3.10, 0.0}},    // near pi
	    {{2.0, 0.0, -1.0}, {0.6, 0.6, 0.6}},
	};
	for (const auto& c : cases) {
		Eigen::Matrix<double, 6, 1> xi;
		xi.head<3>() = c.rho;
		xi.tail<3>() = c.phi;
		const auto back = se3::LogSE3(se3::ExpSE3(xi));
		EXPECT_LT((back - xi).norm(), 1e-9) << "rho=" << c.rho.transpose() << " phi=" << c.phi.transpose();
	}
}

TEST(Se3LogTest, LogExpRoundTripOnTransforms)
{
	Eigen::AffineCompact3d T(Eigen::AngleAxisd(1.1, Eigen::Vector3d(0.5, -1.0, 0.25).normalized()));
	T.translation() = Eigen::Vector3d(1.5, -0.7, 2.2);
	const Eigen::AffineCompact3d back = se3::ExpSE3(se3::LogSE3(T));
	EXPECT_LT((back.matrix() - T.matrix()).norm(), 1e-9);
}

// ---------------------------------------------------------------------------
// SPRT decisions.

// Thresholds move the right way with the error rates.
TEST(SprtValidationTest, ThresholdMonotonicity)
{
	sprt::SprtParams loose;
	sprt::SprtParams strict;
	strict.alpha = 0.001;
	strict.beta = 0.005;
	EXPECT_GT(sprt::UpperThreshold(strict), sprt::UpperThreshold(loose));
	EXPECT_LT(sprt::LowerThreshold(strict), sprt::LowerThreshold(loose));
}

TEST(SprtValidationTest, OutcomeMapping)
{
	EXPECT_EQ(sprt::ToValidationOutcome(sprt::SprtDecision::Continue),
	          spacecal::warm_restart::ValidationOutcome::Inconclusive);
	EXPECT_EQ(sprt::ToValidationOutcome(sprt::SprtDecision::AcceptH0),
	          spacecal::warm_restart::ValidationOutcome::Settled);
	EXPECT_EQ(sprt::ToValidationOutcome(sprt::SprtDecision::AcceptH1),
	          spacecal::warm_restart::ValidationOutcome::Failed);
}

// No verdict before the minimum-sample floor, however extreme the sample --
// the one-sample-verdict failure shape of the old validator must not return.
TEST(SprtValidationTest, MinimumSampleFloorHoldsVerdict)
{
	sprt::SprtState s;
	const sprt::SprtParams p;
	for (int i = 1; i < p.minDecisionSamples; ++i) {
		EXPECT_EQ(sprt::Step(s, 1e6, p), sprt::SprtDecision::Continue) << "n=" << i;
	}
	EXPECT_EQ(sprt::Step(s, 1e6, p), sprt::SprtDecision::AcceptH1);
}

// A valid profile produces chi^2(6) whitened residuals; the test must settle
// (accept H0) close to the minimum floor, and never declare the frame moved.
TEST(SprtValidationTest, ChiSquaredStreamSettles)
{
	std::mt19937 rng(7);
	std::normal_distribution<double> gauss(0.0, 1.0);
	const sprt::SprtParams p;

	for (int trial = 0; trial < 20; ++trial) {
		sprt::SprtState s;
		sprt::SprtDecision d = sprt::SprtDecision::Continue;
		int n = 0;
		while (d == sprt::SprtDecision::Continue && n < 100) {
			double d2 = 0.0;
			for (int k = 0; k < p.dof; ++k) {
				const double g = gauss(rng);
				d2 += g * g;
			}
			d = sprt::Step(s, d2, p);
			++n;
		}
		EXPECT_EQ(d, sprt::SprtDecision::AcceptH0) << "trial=" << trial << " n=" << n << " llr=" << s.llr;
		EXPECT_LE(n, 30) << "settling should happen near the minimum floor";
	}
}

// A 30 cm frame move at a ~1 m lever arm produces residuals the whitening
// makes enormous: the test must fail the profile right at the minimum floor.
TEST(SprtValidationTest, FrameMoveFailsFast)
{
	const Eigen::Vector3d refT(1.0, 0.2, -0.3);
	const Eigen::Vector3d tgtT(0.8, 0.1, 0.1);
	Eigen::Matrix<double, 6, 1> residual = Eigen::Matrix<double, 6, 1>::Zero();
	residual(0) = 0.30; // 30 cm

	const double d2 =
	    levercov::MahalanobisSq(residual, refT, tgtT, levercov::kDefaultSigmaThetaRad, levercov::kDefaultSigmaJitterM);
	const sprt::SprtParams p;
	sprt::SprtState s;
	sprt::SprtDecision d = sprt::SprtDecision::Continue;
	int n = 0;
	while (d == sprt::SprtDecision::Continue && n < 100) {
		d = sprt::Step(s, d2, p);
		++n;
	}
	EXPECT_EQ(d, sprt::SprtDecision::AcceptH1);
	EXPECT_LE(n, 10);
}

// The whitened verdict is lever-arm invariant: the same physical residual
// magnitude relative to the local noise decides identically near the origin
// and far from it, which is the property the fixed-millimetre thresholds
// lacked.
TEST(SprtValidationTest, VerdictConsistentAcrossLeverArms)
{
	const sprt::SprtParams p;
	auto decideAt = [&](double leverM) {
		const Eigen::Vector3d refT(leverM, 0.0, 0.0);
		const Eigen::Vector3d tgtT(leverM * 0.9, 0.1, 0.0);
		Eigen::Matrix<double, 6, 1> residual = Eigen::Matrix<double, 6, 1>::Zero();
		residual(1) = 0.30; // tangential 30 cm -- a real frame move anywhere
		const double d2 = levercov::MahalanobisSq(residual, refT, tgtT, levercov::kDefaultSigmaThetaRad,
		                                          levercov::kDefaultSigmaJitterM);
		sprt::SprtState s;
		sprt::SprtDecision d = sprt::SprtDecision::Continue;
		int n = 0;
		while (d == sprt::SprtDecision::Continue && n < 200) {
			d = sprt::Step(s, d2, p);
			++n;
		}
		return d;
	};
	EXPECT_EQ(decideAt(0.5), sprt::SprtDecision::AcceptH1);
	EXPECT_EQ(decideAt(3.5), sprt::SprtDecision::AcceptH1);
	EXPECT_EQ(decideAt(7.0), sprt::SprtDecision::AcceptH1);
}
