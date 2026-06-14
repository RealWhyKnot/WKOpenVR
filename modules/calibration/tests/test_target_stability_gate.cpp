#include <gtest/gtest.h>

#include "TargetStabilityGate.h"

using namespace spacecal::target_stability;

namespace {

// Drive the EWMA across `count` ticks where `invalidPattern(i)` decides whether
// the target was untracked on tick i.
template <typename Fn> double RunEwma(double start, int count, Fn invalidPattern)
{
	double e = start;
	for (int i = 0; i < count; ++i) {
		e = UpdateInvalidEwma(e, invalidPattern(i), kSolveDeferEwmaAlpha);
	}
	return e;
}

} // namespace

TEST(TargetStabilityGate, EwmaRisesTowardOneWhenTargetStaysInvalid)
{
	const double e = RunEwma(0.0, 200, [](int) { return true; });
	EXPECT_GT(e, 0.95);
}

TEST(TargetStabilityGate, EwmaDecaysToZeroWhenTargetStaysValid)
{
	const double e = RunEwma(1.0, 200, [](int) { return false; });
	EXPECT_LT(e, 0.05);
}

TEST(TargetStabilityGate, HealthySessionNeverDefers)
{
	// One brief dropout every 50 ticks (~2% invalid) -- a healthy link. The EWMA
	// must stay well under the defer threshold so the solve is never starved.
	const double e = RunEwma(0.0, 600, [](int i) { return (i % 50) == 0; });
	EXPECT_LT(e, kSolveDeferInvalidFraction);
	EXPECT_FALSE(ShouldDeferSolve(e, kSolveDeferInvalidFraction));
}

TEST(TargetStabilityGate, SustainedInstabilityDefers)
{
	// ~70% of recent ticks untracked -- a clear instability burst.
	const double e = RunEwma(0.0, 400, [](int i) { return (i % 10) < 7; });
	EXPECT_GT(e, kSolveDeferInvalidFraction);
	EXPECT_TRUE(ShouldDeferSolve(e, kSolveDeferInvalidFraction));
}

TEST(TargetStabilityGate, ThresholdIsStrictlyGreaterThan)
{
	EXPECT_FALSE(ShouldDeferSolve(0.5, 0.5));
	EXPECT_TRUE(ShouldDeferSolve(0.5001, 0.5));
}

TEST(TargetStabilityGate, EwmaStaysInUnitInterval)
{
	double e = 0.0;
	for (int i = 0; i < 1000; ++i) {
		e = UpdateInvalidEwma(e, (i % 3) == 0, kSolveDeferEwmaAlpha);
		EXPECT_GE(e, 0.0);
		EXPECT_LE(e, 1.0);
	}
}
