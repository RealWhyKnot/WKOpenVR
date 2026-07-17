#pragma once

// Cycle-level "should this profile-apply use smooth blending or snap?"
// decision. Used by the overlay's ScanAndApplyProfile when constructing
// the SetDeviceTransform payload.lerp field. Pinned as a pure helper so
// the snap-bypass contract (post-recovery, freshly-adopted device, one-
// shot mode) is testable without instantiating the full overlay state
// machine.
//
// History: this header also carried regime-based still-floor helpers
// (ClassifyCorrection / StillFloor / Regime) for the original 2026-05-04
// recalibrateOnMovement implementation. The cycle-level snap decision is
// independent of that removed floor logic and stays here.

namespace spacecal::motiongate {

constexpr double kFirstContinuousSnapJumpCm = 3.0;

// Upper bound on the first-candidate snap. Below this the jump is a normal
// "calibration takes effect" correction and lands instantly; above it the
// candidate is more likely a bad first solve (stale buffer, wrong target)
// than a real playspace move, so it blends in over the driver's lerp
// instead of teleporting the world. Real frame moves are handled by the
// warm-restart path, which validates before snapping. 50 cm sits an order
// of magnitude above per-solve noise while still letting genuine
// setup-scale corrections land in one step.
constexpr double kFirstContinuousSnapMaxCm = 50.0;

// Returns false (i.e. SNAP -- driver assigns transform := target without
// blending) in three cases:
//   1. We're not in continuous state -- only continuous mode lerps; one-
//      shot finalisations always snap to truth.
//   2. The device is freshly adopted -- its `transform` is identity-or-
//      stale; blending in from there would look like a slow drift.
//   3. The current cycle is a post-recovery snap (snapThisCycle=true) --
//      RecoverFromWedgedCalibration set the flag so the brand-new cal
//      lands discontinuously.
//
// Otherwise returns true (smooth blend via BlendTransform).
constexpr bool ShouldBlendCycle(bool inContinuousState, bool isFreshlyAdopted, bool snapThisCycle)
{
	return inContinuousState && !isFreshlyAdopted && !snapThisCycle;
}

// First accepted continuous candidate after entering continuous mode gets
// special handling: if it differs from the profile that was just loaded by
// more than the solve's own noise floor, snap so the user sees the calibration
// take effect immediately instead of waiting on a blend from stale state.
// Bounded above by kFirstContinuousSnapMaxCm: an oversized first candidate
// blends instead of snapping, so one bad opening solve cannot teleport the
// applied calibration in a single frame. This bound holds in every mode --
// the step-limiting locked-accept gate is an opt-in check, but the first
// candidate lands before any of that machinery engages.
constexpr bool ShouldSnapFirstContinuousCandidate(bool inContinuousState, bool hasAcceptedSnapshot,
                                                  bool hasGuardBaseline, double jumpCm, double solveUncertaintyCm)
{
	const double snapThresholdCm =
	    solveUncertaintyCm > kFirstContinuousSnapJumpCm ? solveUncertaintyCm : kFirstContinuousSnapJumpCm;
	return inContinuousState && !hasAcceptedSnapshot && hasGuardBaseline && jumpCm >= snapThresholdCm &&
	       jumpCm <= kFirstContinuousSnapMaxCm;
}

} // namespace spacecal::motiongate
