#include "FrameHitchLogic.h"

#include <gtest/gtest.h>

namespace {

using openvr_pair::overlay::FrameHitchGate;
using openvr_pair::overlay::FrameHitchSample;
using openvr_pair::overlay::IsFrameHitchWarning;

TEST(FrameHitchLogic, ComputesFrameGapFromMonotonicFrameStarts)
{
	FrameHitchGate gate;
	EXPECT_DOUBLE_EQ(0.0, gate.BeginFrame(10.0));
	EXPECT_NEAR(16.0, gate.BeginFrame(10.016), 0.001);
	EXPECT_DOUBLE_EQ(0.0, gate.BeginFrame(10.010));
	EXPECT_NEAR(20.0, gate.BeginFrame(10.030), 0.001);
}

TEST(FrameHitchLogic, DetectsWarningThresholds)
{
	EXPECT_FALSE(IsFrameHitchWarning({249.9, 249.9, 74.9}));
	EXPECT_TRUE(IsFrameHitchWarning({250.0, 0.0, 0.0}));
	EXPECT_TRUE(IsFrameHitchWarning({0.0, 250.0, 0.0}));
	EXPECT_TRUE(IsFrameHitchWarning({0.0, 0.0, 75.0}));
}

TEST(FrameHitchLogic, ThrottlesRepeatedWarnings)
{
	FrameHitchGate gate;
	const FrameHitchSample sample{0.0, 300.0, 0.0};

	auto first = gate.Evaluate(20.0, sample);
	EXPECT_TRUE(first.shouldLog);
	EXPECT_EQ(0u, first.suppressedSinceLastLog);

	auto suppressedA = gate.Evaluate(20.5, sample);
	EXPECT_FALSE(suppressedA.shouldLog);

	auto suppressedB = gate.Evaluate(20.75, sample);
	EXPECT_FALSE(suppressedB.shouldLog);

	auto next = gate.Evaluate(21.0, sample);
	EXPECT_TRUE(next.shouldLog);
	EXPECT_EQ(2u, next.suppressedSinceLastLog);
}

TEST(FrameHitchLogic, IgnoresNonWarningsWithoutResettingSuppression)
{
	FrameHitchGate gate;
	const FrameHitchSample warning{300.0, 0.0, 0.0};
	const FrameHitchSample normal{10.0, 20.0, 3.0};

	EXPECT_TRUE(gate.Evaluate(30.0, warning).shouldLog);
	EXPECT_FALSE(gate.Evaluate(30.5, warning).shouldLog);
	EXPECT_FALSE(gate.Evaluate(30.75, normal).shouldLog);

	auto next = gate.Evaluate(31.0, warning);
	EXPECT_TRUE(next.shouldLog);
	EXPECT_EQ(1u, next.suppressedSinceLastLog);
}

} // namespace
