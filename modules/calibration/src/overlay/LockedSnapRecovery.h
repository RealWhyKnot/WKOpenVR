#pragma once

// Pure helpers for the locked-style snap-recovery toggle.
//
// HMD-pose-event recovery (the path that reacts to a >=30 cm Quest relocaliza-
// tion) is gated to the Continuous tracking style only -- see
// TrackingStyleAllowsHmdPoseEventRecovery in TrackingStyle.h. That gate wraps
// the entire recovery block in CalibrationRecoveryTick.cpp, including the gentle
// corroborated fast-reanchor that snap-suppression produces when the head-mount
// tracker confirms the head did not physically move (the textbook Quest universe
// flip: HMD jumps a metre, lighthouse-tracked head stays put). The result is
// that in LockedWithRecovery / HardTrackerLock -- the very styles that have a
// head tracker to corroborate with -- a corroborated snap is logged as
// `auto_recover_skipped styleOK=0` and nothing re-anchors the body-tracker
// calibration. Field logs show two ~1.5 m flips skipped this way.
//
// This toggle opens ONLY the gentle, non-destructive corroborated path in the
// locked styles. The destructive full-recovery path (calibration Clear + cold
// continuous-cal restart) stays Continuous-only regardless of the toggle: in a
// locked style the headset is already driven by the head tracker, so a clear is
// never the right response -- a re-anchor to the saved profile is.
//
// All functions are pure: no CalCtx access, no vr::* calls.

#include "TrackingStyle.h" // TrackingStyle, CalibrationState, TrackingStyleAllowsHmdPoseEventRecovery

namespace spacecal::locked_snap {

// True when the gentle corroborated snap fast-reanchor should be allowed to run
// in a locked tracking style. Requires all of:
//   - the experimental toggle is on,
//   - the jump was corroborated as a snap (head tracker confirmed near-zero
//     displacement; the caller computes this via
//     snap_suppression::IsJumpClassifiedAsSnap),
//   - the style is LockedWithRecovery or HardTrackerLock,
//   - the calibration state is Continuous or ContinuousStandby (same state
//     eligibility the Continuous path already requires).
//
// Continuous keeps its existing eligibility through the caller's normal
// recoveryEligible path; this helper deliberately returns false for Continuous
// so the two paths do not double-fire.
inline bool GentleSnapAllowedInLockedStyle(TrackingStyle style, bool toggleOn, bool snapCorroborated,
                                           CalibrationState state)
{
	if (!toggleOn || !snapCorroborated) return false;

	const bool lockedStyle = style == TrackingStyle::LockedWithRecovery || style == TrackingStyle::HardTrackerLock;
	if (!lockedStyle) return false;

	return state == CalibrationState::Continuous || state == CalibrationState::ContinuousStandby;
}

// True when the destructive full-recovery (Clear + restart) path is allowed for
// this style. Unchanged by the toggle: it remains Continuous-only, matching the
// existing TrackingStyleAllowsHmdPoseEventRecovery contract.
inline bool DestructiveRecoveryAllowed(TrackingStyle style)
{
	return TrackingStyleAllowsHmdPoseEventRecovery(style);
}

} // namespace spacecal::locked_snap
