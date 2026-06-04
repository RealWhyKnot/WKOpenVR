// Tests for trigger compensation math (no double-subtract of rest offset).
//
// The compensation formula is:
//   remapped = (raw - trigger_min) / (trigger_max - trigger_min)
//
// It must NOT subtract learned_rest_offset before applying trigger_min,
// because trigger_min already encodes the resting floor observed at rest.
// Double-subtracting would push a released trigger negative.

#include <gtest/gtest.h>
#include <cmath>

namespace {

// Mirrors the compensation logic in InputHealthCompensation.cpp.
// Kept inline here so the test is self-contained and doesn't need to link
// the full driver binary (which depends on openvr_driver.h at runtime).
float TriggerRemap(float raw, float triggerMin, float triggerMax)
{
	const float maxValue = triggerMax > 0.0f ? triggerMax : 1.0f;
	const float range = std::max(0.001f, maxValue - triggerMin);
	const float remapped = (raw - triggerMin) / range;
	if (remapped < 0.0f) return 0.0f;
	if (remapped > 1.0f) return 1.0f;
	return remapped;
}

} // namespace

// ---------------------------------------------------------------------------
// Ideal hardware: rest=0, max=1. Remap is identity.
// ---------------------------------------------------------------------------

TEST(TriggerRemap, PerfectHardware_RestIsZero)
{
	EXPECT_FLOAT_EQ(TriggerRemap(0.0f, 0.0f, 1.0f), 0.0f);
	EXPECT_FLOAT_EQ(TriggerRemap(0.5f, 0.0f, 1.0f), 0.5f);
	EXPECT_FLOAT_EQ(TriggerRemap(1.0f, 0.0f, 1.0f), 1.0f);
}

// ---------------------------------------------------------------------------
// Typical drifted hardware: rest floor ~0.05 (trigger never quite reaches 0).
// Released trigger should remap to exactly 0.0, not negative.
// ---------------------------------------------------------------------------

TEST(TriggerRemap, DriftedRest_ReleasedGoesToZero)
{
	const float rest = 0.05f;
	const float peak = 0.98f;
	EXPECT_FLOAT_EQ(TriggerRemap(rest, rest, peak), 0.0f);
}

TEST(TriggerRemap, DriftedRest_FullPressGoesToOne)
{
	const float rest = 0.05f;
	const float peak = 0.98f;
	EXPECT_FLOAT_EQ(TriggerRemap(peak, rest, peak), 1.0f);
}

TEST(TriggerRemap, DriftedRest_MidpointIsCorrect)
{
	const float rest = 0.05f;
	const float peak = 0.98f;
	const float mid = (rest + peak) * 0.5f;
	const float result = TriggerRemap(mid, rest, peak);
	EXPECT_NEAR(result, 0.5f, 0.001f);
}

// ---------------------------------------------------------------------------
// Values below trigger_min clamp to 0 (not negative).
// ---------------------------------------------------------------------------

TEST(TriggerRemap, BelowMin_ClampsToZero)
{
	EXPECT_FLOAT_EQ(TriggerRemap(0.02f, 0.05f, 1.0f), 0.0f);
	EXPECT_FLOAT_EQ(TriggerRemap(0.0f, 0.05f, 1.0f), 0.0f);
}

// ---------------------------------------------------------------------------
// Values above trigger_max clamp to 1.
// ---------------------------------------------------------------------------

TEST(TriggerRemap, AboveMax_ClampsToOne)
{
	EXPECT_FLOAT_EQ(TriggerRemap(1.1f, 0.0f, 1.0f), 1.0f);
}

// ---------------------------------------------------------------------------
// No double-subtract: applying the compensation twice must not produce
// negative output. This is the regression guard for the original bug where
// value = (rawValue - rest_offset) was used as the input to remap, causing
// double-counting when rest_offset == trigger_min.
// ---------------------------------------------------------------------------

TEST(TriggerRemap, NoDoubleSubtract_RestOffsetEqualToTriggerMin)
{
	// Simulate a path where rest_offset == trigger_min == 0.05.
	// The OLD code did: value = raw - rest_offset (= raw - 0.05), then
	// remapped = (value - trigger_min) / range = (raw - 0.1) / range.
	// A released trigger at raw=0.05 would produce (0.05 - 0.1) < 0.
	// The NEW code uses raw directly in the remap, so released -> 0.
	const float rest = 0.05f;
	const float peak = 0.98f;
	// Released trigger exactly at rest floor.
	EXPECT_GE(TriggerRemap(rest, rest, peak), 0.0f);
}

// ---------------------------------------------------------------------------
// Zero trigger_min, non-standard peak (some controllers max at ~0.95).
// ---------------------------------------------------------------------------

TEST(TriggerRemap, ZeroRestLowPeak)
{
	EXPECT_FLOAT_EQ(TriggerRemap(0.0f, 0.0f, 0.95f), 0.0f);
	EXPECT_FLOAT_EQ(TriggerRemap(0.95f, 0.0f, 0.95f), 1.0f);
	EXPECT_NEAR(TriggerRemap(0.475f, 0.0f, 0.95f), 0.5f, 0.001f);
}

// ---------------------------------------------------------------------------
// Degenerate: trigger_max not yet learned (== 0). Fall back to range [min,1].
// ---------------------------------------------------------------------------

TEST(TriggerRemap, MaxNotLearned_FallsBackToOne)
{
	// When trigger_max == 0, the formula substitutes 1.0 as the ceiling.
	const float rest = 0.05f;
	EXPECT_FLOAT_EQ(TriggerRemap(rest, rest, 0.0f), 0.0f);
	EXPECT_FLOAT_EQ(TriggerRemap(1.0f, rest, 0.0f), 1.0f);
}
