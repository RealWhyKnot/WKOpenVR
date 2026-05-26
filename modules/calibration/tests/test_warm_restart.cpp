// Warm-restart engage-decision tests. Pins the conditions under which a
// proximity-sensor false -> true edge engages the saved-profile snap path
// described in WarmRestart.h. The fix landed because a real session
// (spacecal_log.2026-05-25T14-05-35.txt) took ~7 minutes to settle after
// the user set the HMD down for a drink and came back, with the avatar
// "flying away and flying back" until the rolling sample buffer refilled
// against the saved offset. Each test below corresponds to one of the
// gate's input conditions; if any of them inverts in the future the
// engage path either over-fires (snaps on a sensor blip) or under-fires
// (drops back into the slow re-validation that this fix exists to skip).

#include "WarmRestart.h"

#include <gtest/gtest.h>

namespace wr = spacecal::warm_restart;

namespace {

// Baseline "healthy warm restart" input -- all conditions met. Individual
// tests below flip one field at a time to confirm each gate fires alone.
constexpr wr::EngageInput kHealthyWarmRestart = {
    /*wasPresent=*/false,
    /*nowPresent=*/true,
    /*awayForSeconds=*/30.0,
    /*validProfile=*/true,
    /*stateEligible=*/true,
};

}  // namespace

// --- Rising edge requirement ------------------------------------------------

TEST(WarmRestartTest, EngagesOnRisingEdge) {
    EXPECT_TRUE(wr::ShouldEngage(kHealthyWarmRestart));
}

TEST(WarmRestartTest, DoesNotEngageOnFallingEdge) {
    // User just took the HMD off. We saved away-since-now elsewhere; this
    // tick should not engage anything.
    wr::EngageInput in = kHealthyWarmRestart;
    in.wasPresent = true;
    in.nowPresent = false;
    EXPECT_FALSE(wr::ShouldEngage(in));
}

TEST(WarmRestartTest, DoesNotEngageOnSteadyTrue) {
    // User is wearing the HMD continuously -- no edge to act on.
    wr::EngageInput in = kHealthyWarmRestart;
    in.wasPresent = true;
    in.nowPresent = true;
    EXPECT_FALSE(wr::ShouldEngage(in));
}

TEST(WarmRestartTest, DoesNotEngageOnSteadyFalse) {
    // HMD sitting on a desk; no one is wearing it. No edge.
    wr::EngageInput in = kHealthyWarmRestart;
    in.wasPresent = false;
    in.nowPresent = false;
    EXPECT_FALSE(wr::ShouldEngage(in));
}

// --- Minimum-away threshold -------------------------------------------------

// A sub-threshold blip should be filtered out -- some HMD runtimes report
// brief proximity drops on radio glitches that don't reflect the user
// removing the headset. Without this gate, every passing glitch would
// trigger a snap + grace cycle and re-disturb the calibration.
TEST(WarmRestartTest, DoesNotEngageOnSubThresholdBlip) {
    wr::EngageInput in = kHealthyWarmRestart;
    in.awayForSeconds = 1.0;
    EXPECT_FALSE(wr::ShouldEngage(in))
        << "Brief proximity drops are sensor noise, not real warm restarts";
}

TEST(WarmRestartTest, EngagesAtExactlyThreshold) {
    // The threshold is inclusive: a clean kMinAwaySeconds away counts.
    // Don't widen this inadvertently -- the user-visible latency between
    // putting the HMD back on and feeling tracked correctly is bounded
    // below by this value (you can't snap until you're sure it's a real
    // wake, but the user is staring at "flying away" the whole time).
    wr::EngageInput in = kHealthyWarmRestart;
    in.awayForSeconds = wr::kMinAwaySeconds;
    EXPECT_TRUE(wr::ShouldEngage(in));
}

TEST(WarmRestartTest, DoesNotEngageJustBelowThreshold) {
    wr::EngageInput in = kHealthyWarmRestart;
    in.awayForSeconds = wr::kMinAwaySeconds - 0.01;
    EXPECT_FALSE(wr::ShouldEngage(in));
}

// --- Valid-profile requirement ----------------------------------------------

// Without a saved profile there's nothing to snap to. Engaging the grace
// window here would just bypass the gate for the first calibration ever,
// silently accepting whatever the solver produces in the first 30 s --
// a regression risk for fresh installs where the user is still calibrating
// for the first time.
TEST(WarmRestartTest, DoesNotEngageWithoutValidProfile) {
    wr::EngageInput in = kHealthyWarmRestart;
    in.validProfile = false;
    EXPECT_FALSE(wr::ShouldEngage(in));
}

// --- State eligibility ------------------------------------------------------

// One-shot wizard phases (Begin / Rotation / Translation / Editing) all
// have their own UX and motion gates; the warm-restart fast path is for
// continuous mode only. The non-continuous case isn't expressible in this
// pure helper without enumerating CalibrationState; the caller flattens
// the membership check to a single bool. This test pins the contract that
// when the caller says "not eligible," engage stays off.
TEST(WarmRestartTest, DoesNotEngageWhenStateNotEligible) {
    wr::EngageInput in = kHealthyWarmRestart;
    in.stateEligible = false;
    EXPECT_FALSE(wr::ShouldEngage(in));
}

// --- Compile-time sanity ----------------------------------------------------

// ShouldEngage is constexpr because the input is all PODs and the logic
// is pure boolean. Pinning the constexpr evaluations keeps anyone from
// silently breaking constexpr-eligibility by, e.g., adding logging or a
// non-constexpr helper inside the gate.
static_assert(wr::ShouldEngage(kHealthyWarmRestart),
    "Healthy input must evaluate to engage at compile time");
static_assert(!wr::ShouldEngage({false, false, 30.0, true, true}),
    "No rising edge means no engage, at compile time");
static_assert(!wr::ShouldEngage({false, true, 1.0, true, true}),
    "Sub-threshold away duration means no engage, at compile time");
static_assert(!wr::ShouldEngage({false, true, 30.0, false, true}),
    "Missing saved profile means no engage, at compile time");
static_assert(!wr::ShouldEngage({false, true, 30.0, true, false}),
    "Wrong state means no engage, at compile time");

// --- Threshold values pinned ------------------------------------------------

// kMinAwaySeconds and kGraceSamples both get tuned by reading session logs;
// pin the current values so a change is forced through the test suite as a
// signal that the tuning rationale should be documented.
static_assert(wr::kMinAwaySeconds == 5.0,
    "kMinAwaySeconds changed -- update calibration_robustness memos");
static_assert(wr::kGraceSamples == 100,
    "kGraceSamples changed -- ~30 s of grace at 3.5 Hz; verify the new "
    "value with the latest tick-rate measurement");
static_assert(wr::kMaxAwaySeconds == 4.0 * 3600.0,
    "kMaxAwaySeconds changed -- update the cold-cal handoff rationale");
static_assert(wr::kPositionJumpFastPathM == 0.30,
    "kPositionJumpFastPathM changed -- update the dead-proximity-sensor "
    "fallback rationale");
static_assert(wr::kColdStartGraceTicks == 100,
    "kColdStartGraceTicks changed -- ~30 s of session warmup at 3.5 Hz");

// --- Pose-jump fast-path (B1) ----------------------------------------------

// Single-signal fast-path for HMDs whose activity-level signal is
// unreliable. A 30+ cm HMD displacement during "away" is unambiguous
// evidence of repositioning, regardless of how long the away gap was
// (even sub-kMinAwaySeconds). Built for Quest variants over Link with
// flaky proximity, IMU stillness not met on wobbly desks, etc.

TEST(WarmRestartTest, PoseJumpFastPathEngagesBelowMinAway) {
    // Brief gap (1 s, way below kMinAwaySeconds=5) but with a 50 cm
    // physical HMD displacement -- engage.
    wr::EngageInput in = kHealthyWarmRestart;
    in.awayForSeconds = 1.0;
    in.awayPositionDeltaM = 0.50;
    EXPECT_TRUE(wr::ShouldEngage(in))
        << "Large HMD displacement bypasses the kMinAwaySeconds gate";
}

TEST(WarmRestartTest, PoseJumpFastPathRequiresThreshold) {
    // Brief gap + small displacement (20 cm) -- does NOT engage. Sub-
    // threshold position deltas are casual head movement, not warm-restart.
    wr::EngageInput in = kHealthyWarmRestart;
    in.awayForSeconds = 1.0;
    in.awayPositionDeltaM = 0.20;
    EXPECT_FALSE(wr::ShouldEngage(in));
}

TEST(WarmRestartTest, PoseJumpFastPathStillNeedsRisingEdge) {
    // Even with a huge position jump, a steady-true reading is not a
    // warm-restart event -- no edge to act on.
    wr::EngageInput in = kHealthyWarmRestart;
    in.wasPresent = true;
    in.nowPresent = true;
    in.awayPositionDeltaM = 5.0;
    EXPECT_FALSE(wr::ShouldEngage(in));
}

TEST(WarmRestartTest, PoseJumpFastPathStillNeedsValidProfile) {
    wr::EngageInput in = kHealthyWarmRestart;
    in.awayForSeconds = 1.0;
    in.awayPositionDeltaM = 0.50;
    in.validProfile = false;
    EXPECT_FALSE(wr::ShouldEngage(in));
}

TEST(WarmRestartTest, PoseJumpAtExactlyThresholdEngages) {
    wr::EngageInput in = kHealthyWarmRestart;
    in.awayForSeconds = 1.0;
    in.awayPositionDeltaM = wr::kPositionJumpFastPathM;
    EXPECT_TRUE(wr::ShouldEngage(in)) << "Boundary value is inclusive (>=)";
}

// --- Max-away ceiling (B2) -------------------------------------------------

// Multi-day absences are unsafe to snap because base stations could have
// shifted, room layout could have changed, or the rig could have been
// physically moved while powered down. Falls through to cold cal which
// re-validates everything.

TEST(WarmRestartTest, MaxAwayCeilingSuppresses) {
    wr::EngageInput in = kHealthyWarmRestart;
    in.awayForSeconds = wr::kMaxAwaySeconds + 60.0;  // just over 4 h
    EXPECT_FALSE(wr::ShouldEngage(in))
        << "Beyond kMaxAwaySeconds the snap is unsafe without re-validation";
}

TEST(WarmRestartTest, MaxAwayCeilingBoundaryIsInclusive) {
    wr::EngageInput in = kHealthyWarmRestart;
    in.awayForSeconds = wr::kMaxAwaySeconds;  // exactly at boundary
    EXPECT_TRUE(wr::ShouldEngage(in))
        << "Exactly at the ceiling still engages (<= boundary)";
}

TEST(WarmRestartTest, PoseJumpBypassesMaxAwayCeiling) {
    // 8 h away AND a 1 m physical jump -- the jump is unambiguous evidence
    // that the saved profile must be re-evaluated, regardless of the
    // ceiling. The pose-jump fast-path bypasses time-window checks
    // entirely.
    wr::EngageInput in = kHealthyWarmRestart;
    in.awayForSeconds = 8.0 * 3600.0;
    in.awayPositionDeltaM = 1.0;
    EXPECT_TRUE(wr::ShouldEngage(in))
        << "Pose-jump fast-path bypasses kMaxAwaySeconds";
}

// --- Cold-start safety (B4) ------------------------------------------------

// Session startup with HMD off + user puts it on shortly = rising edge
// that looks like a warm restart, but there's no prior session state to
// "snap back to". Cold-start grace covers the first ~30 s.

TEST(WarmRestartTest, ColdStartSuppressesEngage) {
    wr::EngageInput in = kHealthyWarmRestart;
    in.tickId = 5;  // very early in session
    EXPECT_FALSE(wr::ShouldEngage(in))
        << "First few ticks of a session must not warm-restart";
}

TEST(WarmRestartTest, ColdStartGraceReleasesAtThreshold) {
    wr::EngageInput in = kHealthyWarmRestart;
    in.tickId = wr::kColdStartGraceTicks;  // exactly at boundary
    EXPECT_TRUE(wr::ShouldEngage(in)) << "Boundary is inclusive";
}

TEST(WarmRestartTest, ColdStartGraceAlsoBlocksPoseJump) {
    // Pose jump fast-path is also gated by cold-start grace -- session
    // startup with a large pose delta (e.g. HMD picked up off the desk
    // for the first time) must not engage.
    wr::EngageInput in = kHealthyWarmRestart;
    in.tickId = 5;
    in.awayForSeconds = 1.0;
    in.awayPositionDeltaM = 0.50;
    EXPECT_FALSE(wr::ShouldEngage(in));
}

// --- Validation phase (B3) -------------------------------------------------

// EvaluateValidation classifies the post-snap convergence into Settled
// (early-end grace), Failed (trigger recovery), or Inconclusive (ride out
// the window). Pure helper; caller threads samples_since_snap and
// graceEndedThisTick.

TEST(WarmRestartValidationTest, SettledWhenMadConvergesBelowThreshold) {
    // 5 mm MAD after 25 samples = comfortably settled.
    EXPECT_EQ(wr::EvaluateValidation(/*madFloorM=*/0.005,
                                      /*samplesSinceSnap=*/25,
                                      /*graceEndedThisTick=*/false),
              wr::ValidationOutcome::Settled);
}

TEST(WarmRestartValidationTest, SettledRequiresMinimumSamples) {
    // 5 mm MAD but only 10 samples in -- too early to declare settled.
    // The min-samples gate prevents declaring victory on a lucky early
    // tick before the solver has accumulated real evidence.
    EXPECT_EQ(wr::EvaluateValidation(/*madFloorM=*/0.005,
                                      /*samplesSinceSnap=*/10,
                                      /*graceEndedThisTick=*/false),
              wr::ValidationOutcome::Inconclusive);
}

TEST(WarmRestartValidationTest, SettledRequiresNonZeroMad) {
    // MAD floor of zero means "no observations yet" -- not the same as
    // "actually settled at sub-mm". Defensive: don't declare settled
    // until we have at least one real reading.
    EXPECT_EQ(wr::EvaluateValidation(/*madFloorM=*/0.0,
                                      /*samplesSinceSnap=*/25,
                                      /*graceEndedThisTick=*/false),
              wr::ValidationOutcome::Inconclusive);
}

TEST(WarmRestartValidationTest, FailedAtGraceEndWithHighMad) {
    // 30 mm MAD at grace end -> Failed -> caller triggers recovery.
    EXPECT_EQ(wr::EvaluateValidation(/*madFloorM=*/0.030,
                                      /*samplesSinceSnap=*/100,
                                      /*graceEndedThisTick=*/true),
              wr::ValidationOutcome::Failed);
}

TEST(WarmRestartValidationTest, FailedRequiresGraceEnd) {
    // Mid-grace with high MAD -> wait it out. The solver may still
    // converge. Only at grace end do we commit to Failed.
    EXPECT_EQ(wr::EvaluateValidation(/*madFloorM=*/0.030,
                                      /*samplesSinceSnap=*/50,
                                      /*graceEndedThisTick=*/false),
              wr::ValidationOutcome::Inconclusive);
}

TEST(WarmRestartValidationTest, BetweenThresholdsAtGraceEndIsInconclusive) {
    // 15 mm MAD at grace end -- not settled, not failed. Profile stays;
    // caller logs and moves on.
    EXPECT_EQ(wr::EvaluateValidation(/*madFloorM=*/0.015,
                                      /*samplesSinceSnap=*/100,
                                      /*graceEndedThisTick=*/true),
              wr::ValidationOutcome::Inconclusive);
}

TEST(WarmRestartValidationTest, BoundaryAtSettledThreshold) {
    // Boundary is strict less-than -- 8 mm exactly does NOT settle.
    // Prevents declaring settled when sitting right at the edge.
    EXPECT_EQ(wr::EvaluateValidation(wr::kValidatedSettledMadM,
                                      /*samplesSinceSnap=*/25,
                                      /*graceEndedThisTick=*/false),
              wr::ValidationOutcome::Inconclusive);
    EXPECT_EQ(wr::EvaluateValidation(wr::kValidatedSettledMadM - 0.0001,
                                      /*samplesSinceSnap=*/25,
                                      /*graceEndedThisTick=*/false),
              wr::ValidationOutcome::Settled);
}

TEST(WarmRestartValidationTest, BoundaryAtFailedThreshold) {
    // Boundary is strict greater-than -- 20 mm exactly does NOT fail.
    EXPECT_EQ(wr::EvaluateValidation(wr::kValidatedFailedMadM,
                                      /*samplesSinceSnap=*/100,
                                      /*graceEndedThisTick=*/true),
              wr::ValidationOutcome::Inconclusive);
    EXPECT_EQ(wr::EvaluateValidation(wr::kValidatedFailedMadM + 0.0001,
                                      /*samplesSinceSnap=*/100,
                                      /*graceEndedThisTick=*/true),
              wr::ValidationOutcome::Failed);
}

// --- Validation correctness path (B5) --------------------------------------
//
// Pre-fix, EvaluateValidation took (madFloor, samples, graceEnded) only and
// reported Settled whenever the dispersion floor was below threshold. A snap
// that landed on a stale saved profile -- low post-snap dispersion but a
// systematic offset between the applied calibration and the live samples --
// would be incorrectly reported as Settled. The fix adds a post-snap bias
// input (mean RMS retargeting error of the applied calibration over the
// post-snap window) and refuses to Settle without bias evidence. Rotation
// bias does not need its own input because rotation errors propagate to
// retargeting error via the tracker lever arm: a 2 deg rotation error on
// a 30 cm arm produces ~10 mm of bias, which is well above the fail
// threshold.

TEST(WarmRestartValidationTest, StaleProfileLowNoise_ReturnsFailed) {
    // The bug shape: post-snap samples are quiet (4 mm MAD, well under
    // the 8 mm Settled cap) but the applied calibration is producing
    // 25 mm of mean retargeting error. Pre-fix this was reported as
    // Settled. Post-fix the bias check fires Failed mid-grace because
    // 25 mm > kFailBiasTransM (15 mm). The mid-band shape -- low MAD
    // + bias between accept (5 mm) and fail (15 mm) -- is tested
    // separately in BiasGateBlocksSettledEvenWithLowMad below.
    const wr::ValidationInputs in{
        /*madFloorM=*/0.004,
        /*samplesSinceSnap=*/25,
        /*graceEndedThisTick=*/false,
        /*meanBiasTransM=*/0.025,
    };
    EXPECT_EQ(wr::EvaluateValidation(in), wr::ValidationOutcome::Failed)
        << "Stable post-snap dispersion does not imply a correct snap; "
           "the bias check must reject this";
}

TEST(WarmRestartValidationTest, LargeNoiseLowBias_StaysInconclusiveUntilGraceEnd) {
    // High dispersion, low bias: the snap target looks right on average
    // but the samples are noisy. Mid-grace this is Inconclusive (let the
    // solver keep working); at grace end the MAD-Failed path fires.
    wr::ValidationInputs in{
        /*madFloorM=*/0.018,
        /*samplesSinceSnap=*/50,
        /*graceEndedThisTick=*/false,
        /*meanBiasTransM=*/0.002,
    };
    EXPECT_EQ(wr::EvaluateValidation(in), wr::ValidationOutcome::Inconclusive);
    in.samplesSinceSnap = 100;
    in.graceEndedThisTick = true;
    EXPECT_EQ(wr::EvaluateValidation(in), wr::ValidationOutcome::Inconclusive)
        << "0.018 m is below kValidatedFailedMadM (0.020 m); only MAD strictly "
           "greater fails, otherwise grace end is Inconclusive";

    // Now push MAD past the failed threshold at grace end.
    in.madFloorM = 0.025;
    EXPECT_EQ(wr::EvaluateValidation(in), wr::ValidationOutcome::Failed);
}

TEST(WarmRestartValidationTest, RotationBiasPropagatesViaLeverArm) {
    // Rotation errors of the applied calibration manifest as translation
    // bias on a tracker mounted at the end of a lever arm. A 2 deg
    // rotation error on a 30 cm arm is ~10.5 mm of retargeting error,
    // which is above the 5 mm Settled bias cap. This test pins the
    // contract that the translation bias signal catches rotation
    // failures too, without requiring a separate rotation parameter.
    constexpr double kLeverArmM = 0.30;
    constexpr double kRotErrorRad = 2.0 * 3.14159265358979323846 / 180.0;
    constexpr double kTranslBiasFromRotation = kLeverArmM * kRotErrorRad;
    // Confirm the algebra before asserting on the validator.
    static_assert(kTranslBiasFromRotation > wr::kAcceptBiasTransM,
        "Test setup invariant: lever-arm bias must exceed accept cap");

    const wr::ValidationInputs in{
        /*madFloorM=*/0.003,             // dispersion clean
        /*samplesSinceSnap=*/30,
        /*graceEndedThisTick=*/false,
        /*meanBiasTransM=*/kTranslBiasFromRotation,
    };
    EXPECT_NE(wr::EvaluateValidation(in), wr::ValidationOutcome::Settled)
        << "Rotation error large enough to fail the bias gate via the "
           "lever arm must not be reported as Settled";
}

TEST(WarmRestartValidationTest, BiasGateBlocksSettledEvenWithLowMad) {
    // Companion to StaleProfileLowNoise: at exactly the Settled MAD
    // threshold minus epsilon (the case the original validator
    // reported Settled), the bias gate refuses the verdict if the
    // post-snap bias exceeds kAcceptBiasTransM.
    const wr::ValidationInputs lowDispersionLowBias{
        wr::kValidatedSettledMadM - 0.001, 25, false, 0.002};
    EXPECT_EQ(wr::EvaluateValidation(lowDispersionLowBias),
              wr::ValidationOutcome::Settled)
        << "Low dispersion + low bias is the only configuration that "
           "should Settle";

    const wr::ValidationInputs lowDispersionHighBias{
        wr::kValidatedSettledMadM - 0.001, 25, false,
        wr::kAcceptBiasTransM + 0.001};
    EXPECT_NE(wr::EvaluateValidation(lowDispersionHighBias),
              wr::ValidationOutcome::Settled)
        << "Just above the Settled bias cap must not Settle even with "
           "clean dispersion";
}

// Boundary tests on the bias thresholds. Mirror BoundaryAtSettledThreshold
// and BoundaryAtFailedThreshold above but for the bias axis.

TEST(WarmRestartValidationTest, BiasBoundaryAtAcceptThreshold) {
    // <= is the Settled gate: exactly at the accept threshold still
    // settles when MAD is clean. Just above does not.
    const wr::ValidationInputs atCap{
        0.005, 25, false, wr::kAcceptBiasTransM};
    EXPECT_EQ(wr::EvaluateValidation(atCap), wr::ValidationOutcome::Settled);

    const wr::ValidationInputs justAbove{
        0.005, 25, false, wr::kAcceptBiasTransM + 0.0001};
    EXPECT_EQ(wr::EvaluateValidation(justAbove),
              wr::ValidationOutcome::Inconclusive);
}

TEST(WarmRestartValidationTest, BiasBoundaryAtFailThreshold) {
    // > is the Failed gate: exactly at the fail threshold is still
    // Inconclusive (other paths may decide). Just above fails.
    const wr::ValidationInputs atCap{
        0.004, 25, false, wr::kFailBiasTransM};
    EXPECT_EQ(wr::EvaluateValidation(atCap),
              wr::ValidationOutcome::Inconclusive);

    const wr::ValidationInputs justAbove{
        0.004, 25, false, wr::kFailBiasTransM + 0.0001};
    EXPECT_EQ(wr::EvaluateValidation(justAbove),
              wr::ValidationOutcome::Failed);
}

TEST(WarmRestartValidationTest, BiasGateRequiresMinSamples) {
    // Bias-Failed needs enough post-snap samples to be confident the
    // mean is real. With only 10 samples in, a high bias reading is
    // ignored (Inconclusive); the solver may yet converge.
    const wr::ValidationInputs in{
        0.004, 10, false, wr::kFailBiasTransM + 0.005};
    EXPECT_EQ(wr::EvaluateValidation(in),
              wr::ValidationOutcome::Inconclusive);
}

// --- Pin the bias-threshold constants --------------------------------------

static_assert(wr::kAcceptBiasTransM == 0.005,
    "kAcceptBiasTransM changed -- review the lever-arm rotation propagation "
    "rationale in RotationBiasPropagatesViaLeverArm");
static_assert(wr::kFailBiasTransM == 0.015,
    "kFailBiasTransM changed -- review the stale-profile fail rationale; "
    "this controls how quickly a wrong snap is recovered");
static_assert(wr::kFailBiasTransM > wr::kAcceptBiasTransM,
    "Fail bias must exceed accept bias or the validator has no Inconclusive "
    "band on the bias axis");
