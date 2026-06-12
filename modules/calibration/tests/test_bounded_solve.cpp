// Robust / bounded continuous-solve tests. Pins the prior-pull, slew-clamp, and
// common-mode-rejection helpers that bound how far a single tick can move the
// calibration. See BoundedSolve.h for the rationale.

#include "BoundedSolve.h"

#include <gtest/gtest.h>

namespace bs = spacecal::bounded_solve;

namespace {
constexpr double kPi = 3.14159265358979323846;
Eigen::AffineCompact3d MakePose(const Eigen::Vector3d& t, const Eigen::Quaterniond& q)
{
	Eigen::AffineCompact3d p;
	p.translation() = t;
	p.linear() = q.normalized().toRotationMatrix();
	return p;
}
Eigen::Quaterniond RotZ(double deg)
{
	return Eigen::Quaterniond(Eigen::AngleAxisd(deg * kPi / 180.0, Eigen::Vector3d::UnitZ()));
}
} // namespace

// --- ApplyPrior -------------------------------------------------------------

TEST(BoundedSolveTest, PriorLambdaZeroIsSolve)
{
	const auto solved = MakePose({1, 2, 3}, RotZ(30));
	const auto prior = MakePose({9, 9, 9}, RotZ(-40));
	const auto out = bs::ApplyPrior(solved, prior, 0.0);
	EXPECT_TRUE(out.translation().isApprox(solved.translation(), 1e-12));
}

TEST(BoundedSolveTest, PriorLambdaOneIsPrior)
{
	const auto solved = MakePose({1, 2, 3}, RotZ(30));
	const auto prior = MakePose({9, 9, 9}, RotZ(-40));
	const auto out = bs::ApplyPrior(solved, prior, 1.0);
	EXPECT_TRUE(out.translation().isApprox(prior.translation(), 1e-12));
}

TEST(BoundedSolveTest, PriorMidpointTranslationLerp)
{
	const auto solved = MakePose({0, 0, 0}, RotZ(0));
	const auto prior = MakePose({1, 2, 3}, RotZ(0));
	const auto out = bs::ApplyPrior(solved, prior, 0.5);
	EXPECT_TRUE(out.translation().isApprox(Eigen::Vector3d(0.5, 1.0, 1.5), 1e-12));
}

// --- ClampStep --------------------------------------------------------------

TEST(BoundedSolveTest, ClampStepNoOpWhenUnderLimit)
{
	const auto prev = MakePose({0, 0, 0}, RotZ(0));
	const auto proposed = MakePose({0.02, 0, 0}, RotZ(1));
	const auto out = bs::ClampStep(prev, proposed, /*maxTransM*/ 0.05, /*maxRotRad*/ 0.05);
	EXPECT_TRUE(out.translation().isApprox(proposed.translation(), 1e-12));
}

TEST(BoundedSolveTest, ClampStepCapsTranslation)
{
	const auto prev = MakePose({0, 0, 0}, RotZ(0));
	const auto proposed = MakePose({0.30, 0, 0}, RotZ(0));
	const auto out = bs::ClampStep(prev, proposed, /*maxTransM*/ 0.05, /*maxRotRad*/ 0.0);
	// Capped to 5 cm along the same direction.
	EXPECT_TRUE(out.translation().isApprox(Eigen::Vector3d(0.05, 0, 0), 1e-9));
}

TEST(BoundedSolveTest, ClampStepCapsRotation)
{
	const auto prev = MakePose({0, 0, 0}, RotZ(0));
	const auto proposed = MakePose({0, 0, 0}, RotZ(10));
	const double maxRad = 2.0 * kPi / 180.0;
	const auto out = bs::ClampStep(prev, proposed, /*maxTransM*/ 0.0, maxRad);
	const double ang = Eigen::Quaterniond(prev.rotation()).angularDistance(Eigen::Quaterniond(out.rotation()));
	EXPECT_NEAR(ang, maxRad, 1e-9);
}

// --- IsCommonModeJump -------------------------------------------------------

TEST(BoundedSolveTest, CommonModeRejectsCoherentJump)
{
	// Ref and target both translate ~0.5 m the same direction: a frame shift.
	EXPECT_TRUE(bs::IsCommonModeJump({0.5, 0, 0}, {0.5, 0, 0}));
	// Slightly noisy but still coherent in magnitude and direction.
	EXPECT_TRUE(bs::IsCommonModeJump({0.5, 0, 0}, {0.48, 0.01, 0.0}));
}

TEST(BoundedSolveTest, CommonModeAllowsRealRelativeMotion)
{
	// Target essentially still while ref moves -> genuine relative motion.
	EXPECT_FALSE(bs::IsCommonModeJump({0.5, 0, 0}, {0.0, 0, 0}));
	// Magnitudes disagree badly (0.5 vs 0.06) -> not common mode.
	EXPECT_FALSE(bs::IsCommonModeJump({0.5, 0, 0}, {0.06, 0, 0}));
}

TEST(BoundedSolveTest, CommonModeAllowsOpposingJump)
{
	// Equal magnitude but opposite direction -> dirCos negative -> not common.
	EXPECT_FALSE(bs::IsCommonModeJump({0.5, 0, 0}, {-0.5, 0, 0}));
}

TEST(BoundedSolveTest, CommonModeIgnoresSubThresholdJitter)
{
	// Both below the 5 cm min jump -> never classified as a frame shift.
	EXPECT_FALSE(bs::IsCommonModeJump({0.01, 0, 0}, {0.01, 0, 0}));
}

// --- Pinned constants -------------------------------------------------------

static_assert(bs::kCommonModeMinJumpM == 0.05, "kCommonModeMinJumpM changed -- keep aligned with the reloc detector");
static_assert(bs::kCommonModeCoherence == 0.85,
              "kCommonModeCoherence changed -- review false-positive risk on real motion");
