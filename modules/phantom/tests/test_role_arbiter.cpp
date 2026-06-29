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
