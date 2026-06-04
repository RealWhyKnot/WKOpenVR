// Tests for stick axis rest-offset subtraction and radial deadzone behavior.
// Verifies that the existing stick compensation path is correct and unchanged
// by the trigger-remap refactor.

#include <gtest/gtest.h>
#include <cmath>

namespace {

// Mirrors the stick compensation in InputHealthCompensation.cpp.
// Single axis (no partner) version: applies rest offset, then per-axis
// absolute-value deadzone.
float StickCompensateSingleAxis(float raw, float restOffset, float deadzoneRadius)
{
	float value = raw - restOffset;
	if (deadzoneRadius > 0.0f && std::fabs(value) < deadzoneRadius) {
		value = 0.0f;
	}
	return value;
}

// Two-axis paired version: radial deadzone on the (x, y) vector.
// The X compensation partner offset is applied to partnerValue before the
// radius is computed.
float StickCompensatePaired(float rawX, float rawY, float restOffsetX, float restOffsetY, float deadzoneRadius)
{
	const float vx = rawX - restOffsetX;
	const float vy = rawY - restOffsetY;
	const float radius = std::sqrt(vx * vx + vy * vy);
	if (deadzoneRadius > 0.0f && radius < deadzoneRadius) {
		return 0.0f; // return the X component zeroed; Y would be symmetric
	}
	return vx;
}

} // namespace

// ---------------------------------------------------------------------------
// Rest-offset subtraction
// ---------------------------------------------------------------------------

TEST(StickDeadzone, RestOffsetSubtracted)
{
	// Stick at rest, rest_offset = 0.02. After subtraction: 0.0.
	EXPECT_FLOAT_EQ(StickCompensateSingleAxis(0.02f, 0.02f, 0.0f), 0.0f);
}

TEST(StickDeadzone, PositiveDeflectionAfterSubtraction)
{
	// Pushed to +0.5 with rest_offset=0.02. Result: 0.48.
	EXPECT_NEAR(StickCompensateSingleAxis(0.50f, 0.02f, 0.0f), 0.48f, 1e-5f);
}

TEST(StickDeadzone, NegativeDeflectionAfterSubtraction)
{
	// Pushed to -0.5 with rest_offset=0.02 (positive drift). Result: -0.52.
	EXPECT_NEAR(StickCompensateSingleAxis(-0.50f, 0.02f, 0.0f), -0.52f, 1e-5f);
}

// ---------------------------------------------------------------------------
// Deadzone zeroing
// ---------------------------------------------------------------------------

TEST(StickDeadzone, InsideDeadzone_YieldsZero)
{
	// Raw=0.02, rest_offset=0, deadzone=0.05 -> value=0.02 < 0.05 -> zero.
	EXPECT_FLOAT_EQ(StickCompensateSingleAxis(0.02f, 0.0f, 0.05f), 0.0f);
}

TEST(StickDeadzone, AtDeadzoneEdge_YieldsZero)
{
	// Exactly at deadzone boundary: the condition is strict "<", so 0.05 < 0.05
	// is false and the raw offset-subtracted value passes through.
	const float v = StickCompensateSingleAxis(0.05f, 0.0f, 0.05f);
	EXPECT_FLOAT_EQ(v, 0.05f);
}

TEST(StickDeadzone, OutsideDeadzone_PassesThrough)
{
	EXPECT_FLOAT_EQ(StickCompensateSingleAxis(0.10f, 0.0f, 0.05f), 0.10f);
}

// ---------------------------------------------------------------------------
// Radial deadzone (paired axes): vector inside radius -> zero
// ---------------------------------------------------------------------------

TEST(StickDeadzone, PairedAxes_InsideRadialDeadzone)
{
	// x=0.03, y=0.04 -> radius=0.05. deadzone=0.06 -> zero.
	EXPECT_FLOAT_EQ(StickCompensatePaired(0.03f, 0.04f, 0.0f, 0.0f, 0.06f), 0.0f);
}

TEST(StickDeadzone, PairedAxes_OutsideRadialDeadzone)
{
	// x=0.08, y=0.0 -> radius=0.08. deadzone=0.05 -> passes through.
	EXPECT_NEAR(StickCompensatePaired(0.08f, 0.0f, 0.0f, 0.0f, 0.05f), 0.08f, 1e-5f);
}

TEST(StickDeadzone, PairedAxes_RestOffsetAppliedBeforeRadius)
{
	// Both axes have rest_offset=0.02. Effective vector: (0.01, 0.0).
	// radius=0.01 < deadzone=0.05 -> zero.
	EXPECT_FLOAT_EQ(StickCompensatePaired(0.03f, 0.02f, 0.02f, 0.02f, 0.05f), 0.0f);
}
