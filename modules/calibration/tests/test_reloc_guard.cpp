// Post-relocalization sample-quarantine tests. Pins the half-open window
// semantics the live sample-intake path uses to drop poses for a settle window
// after a Quest SLAM relocalization. See RelocGuard.h for the rationale.

#include "RelocGuard.h"

#include <gtest/gtest.h>

namespace rg = spacecal::reloc_guard;

// --- ShouldQuarantineSample -------------------------------------------------

TEST(RelocGuardTest, NeverRelocReturnsFalse)
{
	// Sentinel lastRelocTime < 0 means no relocalization has been seen yet;
	// nothing should ever be quarantined.
	EXPECT_FALSE(rg::ShouldQuarantineSample(100.0, -1.0, 1.0));
	EXPECT_FALSE(rg::ShouldQuarantineSample(0.0, -1e9, 1.0));
}

TEST(RelocGuardTest, InsideWindowQuarantines)
{
	// Event at t=9.5, window 1.0 s -> [9.5, 10.5). A sample at 10.0 (age 0.5)
	// is inside the window and dropped.
	EXPECT_TRUE(rg::ShouldQuarantineSample(10.0, 9.5, 1.0));
	EXPECT_TRUE(rg::ShouldQuarantineSample(9.5, 9.5, 1.0)); // age 0 is inside
}

TEST(RelocGuardTest, AtWindowEdgeReleases)
{
	// age == window is the half-open boundary: released (resume sampling).
	EXPECT_FALSE(rg::ShouldQuarantineSample(10.5, 9.5, 1.0));
}

TEST(RelocGuardTest, PastWindowReleases)
{
	EXPECT_FALSE(rg::ShouldQuarantineSample(11.0, 9.5, 1.0));
}

TEST(RelocGuardTest, ZeroOrNegativeWindowDisables)
{
	// A zeroed window behaves as "toggle off", not "quarantine everything".
	EXPECT_FALSE(rg::ShouldQuarantineSample(9.6, 9.5, 0.0));
	EXPECT_FALSE(rg::ShouldQuarantineSample(9.6, 9.5, -1.0));
}

TEST(RelocGuardTest, NegativeAgeReleases)
{
	// `now` before the event (clock anomaly) must not quarantine.
	EXPECT_FALSE(rg::ShouldQuarantineSample(9.0, 9.5, 1.0));
}

// --- QuarantineActive (early-release on restabilize) ------------------------

TEST(RelocGuardTest, EarlyClearOnRestabilize)
{
	// Inside the window but the live MAD already dropped under floor*mult
	// (6 <= 5*1.5) -> stop dropping samples early.
	EXPECT_FALSE(rg::QuarantineActive(9.6, 9.5, 1.0, /*liveMad*/ 6.0, /*floor*/ 5.0, /*mult*/ 1.5));
	// Still-elevated MAD inside the window -> keep quarantining.
	EXPECT_TRUE(rg::QuarantineActive(9.6, 9.5, 1.0, /*liveMad*/ 20.0, /*floor*/ 5.0, /*mult*/ 1.5));
}

TEST(RelocGuardTest, EarlyClearDisabledByNonPositiveInputs)
{
	// floor or mult <= 0 disables the early-release check; falls back to the
	// plain time window.
	EXPECT_TRUE(rg::QuarantineActive(9.6, 9.5, 1.0, 6.0, /*floor*/ 0.0, 1.5));
	EXPECT_TRUE(rg::QuarantineActive(9.6, 9.5, 1.0, 6.0, 5.0, /*mult*/ 0.0));
	// And outside the window it is still released regardless.
	EXPECT_FALSE(rg::QuarantineActive(11.0, 9.5, 1.0, 20.0, 5.0, 1.5));
}

// --- Pinned constants -------------------------------------------------------

static_assert(rg::kDefaultQuarantineSec == 1.0,
              "kDefaultQuarantineSec changed -- review the settle-window rationale in RelocGuard.h");
static_assert(rg::kMinQuarantineSec == 0.0, "kMinQuarantineSec changed -- a zero window must mean 'disabled'");
