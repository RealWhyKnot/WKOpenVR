// Drift circuit-breaker tests. Pins the freeze/release thresholds that automate
// the manual headset-lock intervention when the relative-pose MAD runs away.
// See DriftBreaker.h for the rationale.

#include "DriftBreaker.h"

#include <gtest/gtest.h>

namespace db = spacecal::drift_breaker;

// --- ShouldFreeze -----------------------------------------------------------

TEST(DriftBreakerTest, NoFreezeWhenQuiet)
{
	// Healthy steady state: 6 mm against a 5 mm floor, K=8 (trip 40), cap 60.
	EXPECT_FALSE(db::ShouldFreeze(6.0, 5.0, 8.0, 60.0));
}

TEST(DriftBreakerTest, FreezeOnFloorMultiple)
{
	// 45 mm >= 8 * 5 mm -> floor gate trips (cap disabled here).
	EXPECT_TRUE(db::ShouldFreeze(45.0, 5.0, 8.0, /*cap*/ 0.0));
}

TEST(DriftBreakerTest, FreezeOnAbsCapWhenFloorUnreliable)
{
	// The bad session's floor climbed to 100+ mm, which would defeat a
	// floor-only gate. With floor=0 the floor gate is disabled but the
	// absolute cap still trips at runaway MAD.
	EXPECT_TRUE(db::ShouldFreeze(400.0, /*floor*/ 0.0, 8.0, /*cap*/ 60.0));
}

TEST(DriftBreakerTest, AbsCapBoundary)
{
	EXPECT_TRUE(db::ShouldFreeze(60.0, 0.0, 0.0, 60.0));  // >= cap
	EXPECT_FALSE(db::ShouldFreeze(59.9, 0.0, 0.0, 60.0)); // just under
}

TEST(DriftBreakerTest, ZeroKAndZeroCapDisables)
{
	// Both gates disabled -> never freeze, even at extreme MAD.
	EXPECT_FALSE(db::ShouldFreeze(400.0, 5.0, /*K*/ 0.0, /*cap*/ 0.0));
}

TEST(DriftBreakerTest, FloorMultBoundary)
{
	EXPECT_TRUE(db::ShouldFreeze(40.0, 5.0, 8.0, 0.0));  // exactly 8*5
	EXPECT_FALSE(db::ShouldFreeze(39.9, 5.0, 8.0, 0.0)); // just under
}

// --- ShouldRelease ----------------------------------------------------------

TEST(DriftBreakerTest, ReleaseHysteresis)
{
	// Tripped against floor=5, K=8 -> trip level 40, release level 0.5*40=20.
	EXPECT_TRUE(db::ShouldRelease(19.0, 5.0, 8.0));  // back under 20 -> release
	EXPECT_FALSE(db::ShouldRelease(25.0, 5.0, 8.0)); // still elevated -> hold freeze
	EXPECT_FALSE(db::ShouldRelease(20.0, 5.0, 8.0)); // boundary: not strictly under
}

TEST(DriftBreakerTest, ReleaseWhenFloorGateDisabled)
{
	// No floor hysteresis level to hold against -> release immediately and let
	// the cap gate re-trip if MAD is still high.
	EXPECT_TRUE(db::ShouldRelease(400.0, /*floor*/ 0.0, 8.0));
	EXPECT_TRUE(db::ShouldRelease(400.0, 5.0, /*K*/ 0.0));
}

// --- Pinned constants -------------------------------------------------------

static_assert(db::kDefaultMadMult == 8.0,
              "kDefaultMadMult changed -- 8x sits above normal AUTO-lock churn; review before tuning");
static_assert(db::kDefaultAbsCapMm == 60.0,
              "kDefaultAbsCapMm changed -- 60 mm is the floor-independent runaway backstop; review before tuning");
static_assert(db::kReleaseHysteresisMult == 0.5,
              "kReleaseHysteresisMult changed -- the 2:1 band prevents freeze flapping");
