#pragma once

// Pure recovery-action policy for the calibration output/recovery layer.
//
// Decides whether a relocalization event or a failed warm-restart should hold
// the current calibration, gently re-anchor to the saved profile, or -- only as
// a true last resort -- destructively clear and recalibrate from scratch.
//
// Background: field logs showed destructive clears firing on corroborated
// universe flips and on warm-restart validation failures even when a saved
// profile existed and a Lighthouse witness puck could confirm the situation.
// A destructive clear wipes the user's calibration and restarts cold, so it
// must be the exception, not the default. These pure functions encode that
// policy so it is unit-testable without a live OpenVR runtime.

#include <cstdint>

namespace spacecal::recovery {

enum class RecoveryAction : uint8_t
{
	Hold = 0,          // keep the current calibration; take no corrective action
	ReanchorToProfile, // gently re-anchor to the saved profile (ramped re-apply)
	DestructiveClear,  // wipe and recalibrate from scratch (true last resort)
};

// Destructive clear is permitted only as a true last resort: the witness cannot
// corroborate, there is no saved profile to fall back to, AND warm-restart
// validation has already failed. If any one of these is false, a
// non-destructive action is available and must be preferred.
constexpr bool DestructiveClearAllowed(bool witnessInvalid, bool hasSavedProfile, bool warmRestartFailed)
{
	return witnessInvalid && !hasSavedProfile && warmRestartFailed;
}

// Relocalization (HMD pose-jump) recovery decision. Evaluated only after the
// snap-corroboration fast-reanchor branch has been ruled out, i.e. the head did
// NOT stay still during the jump.
//
//   witnessValid         - the witness produced a valid displacement this tick
//   witnessSaysHeadMoved - that displacement exceeded the still threshold
//   hasSavedProfile      - a saved calibration profile exists
//   warmRestartFailed    - warm-restart validation is currently Failed
//
// A valid witness that moved with the HMD means real physical motion, not a
// frame jump -> the calibration is still valid -> Hold. (This was the dominant
// false-positive destructive clear.) Otherwise prefer a gentle re-anchor to the
// saved profile; destroy only when DestructiveClearAllowed.
constexpr RecoveryAction ChooseRelocRecoveryAction(bool witnessValid, bool witnessSaysHeadMoved, bool hasSavedProfile,
                                                   bool warmRestartFailed)
{
	if (witnessValid && witnessSaysHeadMoved) return RecoveryAction::Hold;
	if (hasSavedProfile) return RecoveryAction::ReanchorToProfile;
	if (DestructiveClearAllowed(/*witnessInvalid=*/!witnessValid, hasSavedProfile, warmRestartFailed))
		return RecoveryAction::DestructiveClear;
	return RecoveryAction::Hold;
}

// Bounded re-anchor retries before warm-restart gives up and holds.
constexpr int kWarmRestartMaxReanchors = 2;

// Warm-restart validation-failure decision. When a witness is present we trust
// the saved profile and re-anchor again (bounded retries) rather than
// destroying the user's calibration. When retries are exhausted or no witness
// is present, hold the profile unless a destructive clear is truly the last
// resort -- which requires no saved profile, so at warm restart (where a
// profile always exists) this holds rather than clears.
//
// frameReanchorWitnessed: the episode began with a witnessed world-frame move
// (corroborated SLAM snap, reloc re-anchor, or a >= eviction-length away gap
// on an inside-out headset). In that case the saved profile describes the OLD
// frame -- re-applying it is the wrong side of the disagreement, and doing so
// produced a re-anchor ping-pong at multi-metre amplitude in field logs
// (2026-07-10: three re-apply/re-snap cycles per away gap at 3.5-7.3 m).
// Hold the re-solved frame and let continuous calibration converge; the
// saved profile stays on disk untouched for the next classified event.
constexpr RecoveryAction ChooseWarmRestartFailureAction(bool frameReanchorWitnessed, bool witnessPresent,
                                                        bool hasSavedProfile, int reanchorCount, int maxReanchors)
{
	if (frameReanchorWitnessed) return RecoveryAction::Hold;
	if (witnessPresent && reanchorCount < maxReanchors) return RecoveryAction::ReanchorToProfile;
	if (DestructiveClearAllowed(/*witnessInvalid=*/!witnessPresent, hasSavedProfile, /*warmRestartFailed=*/true))
		return RecoveryAction::DestructiveClear;
	return RecoveryAction::Hold;
}

} // namespace spacecal::recovery
