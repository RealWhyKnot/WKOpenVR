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
// recalibrateOnMovement implementation. Those were replaced on 2026-05-24
// by the absolute mm/sec time-rate cap in SlewRateCap.h, which avoids the
// visible snap that the 90 percent Large-regime floor produced after long
// stationary stretches. The cycle-level snap decision is independent of
// the floor and stays here.

namespace spacecal::motiongate {

constexpr double kFirstContinuousSnapJumpCm = 3.0;
constexpr double kMaxFirstContinuousSnapJumpCm = 25.0;

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
constexpr bool ShouldBlendCycle(bool inContinuousState,
                                bool isFreshlyAdopted,
                                bool snapThisCycle) {
    return inContinuousState && !isFreshlyAdopted && !snapThisCycle;
}

// First accepted continuous candidate after entering continuous mode gets
// special handling: if it differs enough from the profile that was just
// loaded/applied, snap so the user sees the calibration take effect instead
// of waiting for motion-gated slew to catch up. Large corrections still
// blend so a bad first solve cannot move the whole playspace in one frame.
constexpr bool ShouldSnapFirstContinuousCandidate(bool inContinuousState,
                                                  bool hasAcceptedSnapshot,
                                                  bool hasGuardBaseline,
                                                  double jumpCm) {
    return inContinuousState
        && !hasAcceptedSnapshot
        && hasGuardBaseline
        && jumpCm >= kFirstContinuousSnapJumpCm
        && jumpCm <= kMaxFirstContinuousSnapJumpCm;
}

}  // namespace spacecal::motiongate
