// Locked-style snap-recovery tests. Pins which (style, toggle, corroboration,
// state) combinations open the gentle corroborated fast-reanchor in locked
// styles, and confirms the destructive Clear path stays Continuous-only.
// See LockedSnapRecovery.h for the rationale.

#include "LockedSnapRecovery.h"

#include <gtest/gtest.h>

namespace ls = spacecal::locked_snap;

// --- GentleSnapAllowedInLockedStyle -----------------------------------------

TEST(LockedSnapRecoveryTest, ContinuousNotHandledByLockedHelper)
{
	// Continuous keeps its existing eligibility via the caller's recoveryEligible
	// path; this helper returns false for Continuous so the two do not double-fire.
	EXPECT_FALSE(ls::GentleSnapAllowedInLockedStyle(TrackingStyle::Continuous, /*on*/ true,
	                                                /*corroborated*/ true, CalibrationState::Continuous));
}

TEST(LockedSnapRecoveryTest, LockedGentleAllowedWhenCorroboratedAndOn)
{
	EXPECT_TRUE(ls::GentleSnapAllowedInLockedStyle(TrackingStyle::LockedWithRecovery, true, true,
	                                               CalibrationState::Continuous));
}

TEST(LockedSnapRecoveryTest, LockedGentleBlockedWhenNotCorroborated)
{
	// An uncorroborated jump (head tracker did NOT confirm a snap) must not
	// fast-reanchor in a locked style.
	EXPECT_FALSE(ls::GentleSnapAllowedInLockedStyle(TrackingStyle::LockedWithRecovery, true, false,
	                                                CalibrationState::Continuous));
}

TEST(LockedSnapRecoveryTest, LockedGentleBlockedWhenToggleOff)
{
	EXPECT_FALSE(ls::GentleSnapAllowedInLockedStyle(TrackingStyle::LockedWithRecovery, false, true,
	                                                CalibrationState::Continuous));
}

TEST(LockedSnapRecoveryTest, HardTrackerLockSameAsLocked)
{
	EXPECT_TRUE(ls::GentleSnapAllowedInLockedStyle(TrackingStyle::HardTrackerLock, true, true,
	                                               CalibrationState::ContinuousStandby));
}

TEST(LockedSnapRecoveryTest, StateMustBeContinuousOrStandby)
{
	// Outside the continuous-cal states the gentle path is inert even in a
	// locked style with a corroborated snap.
	EXPECT_FALSE(
	    ls::GentleSnapAllowedInLockedStyle(TrackingStyle::LockedWithRecovery, true, true, CalibrationState::Editing));
}

// --- DestructiveRecoveryAllowed (unchanged by the toggle) -------------------

TEST(LockedSnapRecoveryTest, DestructiveStaysContinuousOnly)
{
	EXPECT_TRUE(ls::DestructiveRecoveryAllowed(TrackingStyle::Continuous));
	EXPECT_FALSE(ls::DestructiveRecoveryAllowed(TrackingStyle::LockedWithRecovery));
	EXPECT_FALSE(ls::DestructiveRecoveryAllowed(TrackingStyle::HardTrackerLock));
	EXPECT_FALSE(ls::DestructiveRecoveryAllowed(TrackingStyle::Manual));
}
