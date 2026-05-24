#pragma once

// Slew-rate cap for the driver-side BlendTransform. Bounds how fast the
// user-visible `transform` is allowed to converge toward `targetTransform`,
// in absolute mm/sec and deg/sec, regardless of how large the pending
// correction is.
//
// Why a cap, not a regime-based percentage floor: the predecessor used
// a 10/50/90 percent still-floor by Tiny/Normal/Large classification,
// which closed ~85 percent of the gap per second when stationary for any
// Large-regime correction. After a long stationary stretch in which
// continuous-cal accumulated a few cm of pending correction, that regime
// fired the moment the user moved, producing a visible jump. The
// imperceptible-drift threshold in VR is ~0.5 mm/sec lateral; a hard cap
// at that rate guarantees no visible jump regardless of how much correction
// accumulated while still.
//
// Two-rate split: when the user moves, motion masks the catch-up, so the
// cap can be much higher. BlendRate() linearly interpolates the stationary
// and moving rates by the existing 0..1 motionGate signal computed inside
// BlendTransform (the same per-frame device-motion magnitude the old
// implementation already produced).
//
// Pure functions -- no state. Pinned by test_slew_rate_cap.cpp.

namespace spacecal::slew {

// Convergence guards: when the pending step is below this magnitude the cap
// cannot be exceeded regardless of the proposed lerp, so we skip the divide
// and return the proposed lerp unchanged. Values are in metres / radians,
// matching the BlendTransform call site.
constexpr double kConvergedPosM   = 1e-5;   // 0.01 mm
constexpr double kConvergedRotRad = 1e-6;   // ~0.00006 deg

// Apply the slew-rate cap to a proposed lerp factor.
//
//   proposedLerp    -- time-based lerp factor the caller already produced
//                      via `dt * GetTransformRate(currentRate)`.
//   pendingPosM     -- |target.translation - current.translation|, metres.
//   pendingRotRad   -- angularDistance(target.rotation, current.rotation),
//                      radians.
//   capPosMPerSec   -- maximum permitted position step per second, metres.
//                      Caller blends stationary and moving rates by
//                      motionGate before passing in (see BlendRate below).
//   capRotRadPerSec -- maximum permitted rotation step per second, radians.
//   dt              -- seconds since the prior BlendTransform call. The
//                      hard cap on this tick is (capX * dt).
//
// Returns the scaled lerp. Always in [0, proposedLerp]. The slower axis
// dominates: if rotation would overshoot but position would not, both get
// throttled by the rotation-dictated scale so the transform composes
// correctly (we cannot scale one axis without the other when both share
// the lerp parameter).
constexpr double ApplyCap(double proposedLerp,
                          double pendingPosM,
                          double pendingRotRad,
                          double capPosMPerSec,
                          double capRotRadPerSec,
                          double dt) {
    if (!(proposedLerp > 0.0) || !(dt > 0.0)) return 0.0;

    const double maxStepPosM   = capPosMPerSec   * dt;
    const double maxStepRotRad = capRotRadPerSec * dt;

    const double wouldStepPosM   = pendingPosM   * proposedLerp;
    const double wouldStepRotRad = pendingRotRad * proposedLerp;

    const double scalePos = (wouldStepPosM   > maxStepPosM   && pendingPosM   > kConvergedPosM)
                              ? (maxStepPosM   / wouldStepPosM)   : 1.0;
    const double scaleRot = (wouldStepRotRad > maxStepRotRad && pendingRotRad > kConvergedRotRad)
                              ? (maxStepRotRad / wouldStepRotRad) : 1.0;

    const double scale = (scalePos < scaleRot) ? scalePos : scaleRot;
    return proposedLerp * scale;
}

// Linearly blend the stationary and moving rate caps by motionGate (0..1).
// The gate is clamped defensively so a misbehaved caller cannot widen the
// effective rate beyond moving or invert past stationary.
constexpr double BlendRate(double stationaryRate,
                           double movingRate,
                           double motionGate) {
    if (motionGate < 0.0) motionGate = 0.0;
    if (motionGate > 1.0) motionGate = 1.0;
    return stationaryRate + (movingRate - stationaryRate) * motionGate;
}

}  // namespace spacecal::slew
