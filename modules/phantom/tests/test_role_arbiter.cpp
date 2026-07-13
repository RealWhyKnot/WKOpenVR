#include "RoleArbiter.h"

#include <gtest/gtest.h>

using namespace phantom;

namespace {
constexpr uint32_t kPastHold = 100000; // well past the default hold_ms
}

// A confident inference adopts a role onto an unassigned, untagged slot.
TEST(RoleArbiter, AdoptsFromNone)
{
	RoleArbiterParams p;
	EXPECT_EQ(DecideRole(BodyRole::None, false, BodyRole::Waist, 0.80f, 0.0f, kPastHold, p), RoleAction::Adopt);
}

// Below the adopt bar, an unassigned slot stays put.
TEST(RoleArbiter, DoesNotAdoptBelowThreshold)
{
	RoleArbiterParams p;
	EXPECT_EQ(DecideRole(BodyRole::None, false, BodyRole::Waist, 0.40f, 0.0f, kPastHold, p), RoleAction::Keep);
}

// Snapped / manual roles (userTagged) are never auto-managed.
TEST(RoleArbiter, NeverTouchesUserTagged)
{
	RoleArbiterParams p;
	EXPECT_EQ(DecideRole(BodyRole::Waist, true, BodyRole::LeftFoot, 0.99f, 0.10f, kPastHold, p), RoleAction::Keep);
}

// Within the hold window, a slot does not change even on a strong disagreement.
TEST(RoleArbiter, HoldDebouncesReassign)
{
	RoleArbiterParams p; // hold_ms = 1500
	EXPECT_EQ(DecideRole(BodyRole::Waist, false, BodyRole::LeftFoot, 0.99f, 0.10f, 200, p), RoleAction::Keep);
}

// Past the hold window, a clearly-better role wins by the reassign margin.
TEST(RoleArbiter, ReassignsPastHoldAndMargin)
{
	RoleArbiterParams p;
	EXPECT_EQ(DecideRole(BodyRole::Waist, false, BodyRole::LeftFoot, 0.90f, 0.50f, kPastHold, p), RoleAction::Reassign);
}

// A new role only a hair better than the held one does not flap.
TEST(RoleArbiter, DoesNotReassignWithoutMargin)
{
	RoleArbiterParams p; // reassign_margin = 0.15
	EXPECT_EQ(DecideRole(BodyRole::Waist, false, BodyRole::LeftFoot, 0.62f, 0.55f, kPastHold, p), RoleAction::Keep);
}

// The held auto-role is dropped once it no longer fits and nothing replaces it.
TEST(RoleArbiter, DropsWhenHeldRoleNoLongerFits)
{
	RoleArbiterParams p; // drop_conf = 0.30
	EXPECT_EQ(DecideRole(BodyRole::Waist, false, BodyRole::None, 0.0f, 0.10f, kPastHold, p), RoleAction::Drop);
}

// A still-fitting role is kept even if this cycle had no confident pick.
TEST(RoleArbiter, KeepsWhenHeldRoleStillFits)
{
	RoleArbiterParams p;
	EXPECT_EQ(DecideRole(BodyRole::Waist, false, BodyRole::None, 0.0f, 0.50f, kPastHold, p), RoleAction::Keep);
}

// A sustained, clearly-better contradiction corrects a snap-sourced sticky
// role only after the full streak of consecutive confident cycles.
TEST(StickyOverride, TriggersOnlyAfterFullStreak)
{
	RoleArbiterParams p;
	StickyOverrideState s;
	for (int i = 0; i < 4; ++i) {
		EXPECT_FALSE(UpdateStickyOverride(s, BodyRole::LeftFoot, BodyRole::RightFoot, 0.85f, 0.20f, p));
	}
	EXPECT_TRUE(UpdateStickyOverride(s, BodyRole::LeftFoot, BodyRole::RightFoot, 0.85f, 0.20f, p));
	// State resets after a trigger; the next cycle starts a fresh streak.
	EXPECT_FALSE(UpdateStickyOverride(s, BodyRole::LeftFoot, BodyRole::RightFoot, 0.85f, 0.20f, p));
}

// One weak cycle resets the streak: transient wrong inference cannot flip a
// snap role.
TEST(StickyOverride, WeakCycleResetsStreak)
{
	RoleArbiterParams p;
	StickyOverrideState s;
	for (int i = 0; i < 4; ++i) {
		EXPECT_FALSE(UpdateStickyOverride(s, BodyRole::LeftFoot, BodyRole::RightFoot, 0.85f, 0.20f, p));
	}
	// Dips below adopt_conf + extra_margin (0.75).
	EXPECT_FALSE(UpdateStickyOverride(s, BodyRole::LeftFoot, BodyRole::RightFoot, 0.70f, 0.20f, p));
	for (int i = 0; i < 4; ++i) {
		EXPECT_FALSE(UpdateStickyOverride(s, BodyRole::LeftFoot, BodyRole::RightFoot, 0.85f, 0.20f, p));
	}
	EXPECT_TRUE(UpdateStickyOverride(s, BodyRole::LeftFoot, BodyRole::RightFoot, 0.85f, 0.20f, p));
}

// A changing candidate restarts the streak; only one sustained role wins.
TEST(StickyOverride, CandidateFlipRestartsStreak)
{
	RoleArbiterParams p;
	StickyOverrideState s;
	for (int i = 0; i < 4; ++i) {
		EXPECT_FALSE(UpdateStickyOverride(s, BodyRole::LeftFoot, BodyRole::RightFoot, 0.85f, 0.20f, p));
	}
	EXPECT_FALSE(UpdateStickyOverride(s, BodyRole::LeftFoot, BodyRole::Waist, 0.85f, 0.20f, p));
	for (int i = 0; i < 3; ++i) {
		EXPECT_FALSE(UpdateStickyOverride(s, BodyRole::LeftFoot, BodyRole::Waist, 0.85f, 0.20f, p));
	}
	EXPECT_TRUE(UpdateStickyOverride(s, BodyRole::LeftFoot, BodyRole::Waist, 0.85f, 0.20f, p));
}

// Confidence high in absolute terms but not clearly better than the held fit
// never corrects: the margin keeps a near-tie from flapping a snap role.
TEST(StickyOverride, RequiresMarginOverHeldFit)
{
	RoleArbiterParams p; // reassign_margin = 0.15
	StickyOverrideState s;
	for (int i = 0; i < 10; ++i) {
		EXPECT_FALSE(UpdateStickyOverride(s, BodyRole::LeftFoot, BodyRole::RightFoot, 0.80f, 0.70f, p));
	}
}

// Agreement or no inference never builds a streak.
TEST(StickyOverride, AgreementIsNotContradiction)
{
	RoleArbiterParams p;
	StickyOverrideState s;
	for (int i = 0; i < 10; ++i) {
		EXPECT_FALSE(UpdateStickyOverride(s, BodyRole::LeftFoot, BodyRole::LeftFoot, 0.95f, 0.95f, p));
		EXPECT_FALSE(UpdateStickyOverride(s, BodyRole::LeftFoot, BodyRole::None, 0.95f, 0.10f, p));
	}
}
