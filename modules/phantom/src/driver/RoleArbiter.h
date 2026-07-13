#pragma once

#include "RoleCatalog.h"

#include <cstdint>

namespace phantom {

// Continuous self-correction for auto-detected tracker roles.
//
// Passive inference auto-adopts a role onto an untagged tracker once it is
// confident. But a first guess can be wrong, a tracker can be moved to a
// different body point mid-session, and a role can stop fitting. The arbiter
// turns the latest inference into one decision per auto-managed slot. It only
// ever touches auto-adopted slots: roles the user snapped or hand-picked stay
// sticky (the caller passes userTagged=true for those). A hold timer plus a
// reassign margin keep a borderline tracker from flapping between two roles.

enum class RoleAction : uint8_t
{
	Keep = 0,     // leave the slot's role as-is
	Adopt = 1,    // take the inferred role onto a role-less slot
	Reassign = 2, // switch to a different, clearly-better role
	Drop = 3      // release the role; it no longer fits and nothing replaces it
};

struct RoleArbiterParams
{
	float adopt_conf = 0.60f;      // adopt/reassign target must clear this
	float reassign_margin = 0.15f; // new role must beat the held role's fit by this
	float drop_conf = 0.30f;       // below this the held auto-role is released
	uint32_t hold_ms = 1500;       // min dwell before a slot may change again
};

// current        - role the slot holds now (None if unassigned)
// userTagged      - true if snapped/manual (sticky): always Keep
// inferred        - this cycle's best-guess role for the slot's tracker
// inferredConf    - confidence backing `inferred`
// currentRoleConf - how well the tracker still fits the role it holds
// msSinceChange   - ms since the slot's role last changed (debounce)
inline RoleAction DecideRole(BodyRole current, bool userTagged, BodyRole inferred, float inferredConf,
                             float currentRoleConf, uint32_t msSinceChange, const RoleArbiterParams& params = {})
{
	if (userTagged) {
		return RoleAction::Keep;
	}

	if (current == BodyRole::None) {
		if (inferred != BodyRole::None && inferredConf >= params.adopt_conf) {
			return RoleAction::Adopt;
		}
		return RoleAction::Keep;
	}

	// Debounce: hold a freshly-set role briefly before reconsidering it.
	if (msSinceChange < params.hold_ms) {
		return RoleAction::Keep;
	}

	// Held role no longer fits and nothing confident replaces it -> release.
	if (currentRoleConf < params.drop_conf && (inferred == BodyRole::None || inferredConf < params.adopt_conf)) {
		return RoleAction::Drop;
	}

	// A different role now fits clearly better -> switch.
	if (inferred != BodyRole::None && inferred != current && inferredConf >= params.adopt_conf &&
	    inferredConf >= currentRoleConf + params.reassign_margin) {
		return RoleAction::Reassign;
	}

	return RoleAction::Keep;
}

// Correction gate for snap-sourced sticky roles. A snap can be wrong (two feet
// side by side read nearly the same), and a sticky role would otherwise never
// heal. This keeps snap roles immune to ordinary inference churn but lets a
// sustained, clearly-better contradiction through: the same candidate role must
// beat the held role by the reassign margin AND clear adopt_conf + extra_margin
// on `required_streak` consecutive inference cycles (~1 Hz each). Any weaker
// cycle or a different candidate resets the streak. Never drops -- only swaps
// to the contradicting role. Manual roles must not be routed through this.
struct StickyOverrideState
{
	BodyRole candidate = BodyRole::None;
	uint8_t streak = 0;
};

inline bool UpdateStickyOverride(StickyOverrideState& state, BodyRole current, BodyRole inferred, float inferredConf,
                                 float currentFit, const RoleArbiterParams& params = {}, float extra_margin = 0.15f,
                                 uint8_t required_streak = 5)
{
	const bool contradicts = inferred != BodyRole::None && current != BodyRole::None && inferred != current &&
	                         inferredConf >= params.adopt_conf + extra_margin &&
	                         inferredConf >= currentFit + params.reassign_margin;
	if (!contradicts) {
		state = StickyOverrideState{};
		return false;
	}
	if (state.candidate != inferred) {
		state.candidate = inferred;
		state.streak = 1;
	}
	else if (state.streak < required_streak) {
		++state.streak;
	}
	if (state.streak < required_streak) {
		return false;
	}
	state = StickyOverrideState{};
	return true;
}

} // namespace phantom
