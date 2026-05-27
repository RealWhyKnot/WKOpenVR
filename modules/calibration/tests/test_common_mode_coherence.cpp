// Common-mode coherence score tests. Pins the score formula and the
// suppress-threshold semantics that the geometry-shift detector's fire
// site uses to distinguish pair-local shifts (fire the recovery path)
// from shared-frame events (runtime relocalization -- suppress the fire).
// See CommonModeCoherence.h for the rationale.

#include "CommonModeCoherence.h"

#include <gtest/gtest.h>

namespace cm = spacecal::coherence;

// --- ComputeCoherenceScore --------------------------------------------------

TEST(CommonModeCoherenceTest, EmptyExtrasReturnsZero) {
    // No second opinion -> no coherence evidence -> 0. Caller is
    // expected to also gate on kMinExtrasForCoherence before
    // interpreting the score as a suppression signal.
    EXPECT_DOUBLE_EQ(cm::ComputeCoherenceScore(5.0, {}), 0.0);
}

TEST(CommonModeCoherenceTest, NonPositivePrimaryReturnsZero) {
    // Primary ratio non-positive means the primary detector skipped the
    // ratio computation (rolling median below the floor). The coherence
    // formula is undefined here; fall back to "no evidence".
    EXPECT_DOUBLE_EQ(cm::ComputeCoherenceScore(0.0, {5.0, 5.0}), 0.0);
    EXPECT_DOUBLE_EQ(cm::ComputeCoherenceScore(-1.0, {5.0, 5.0}), 0.0);
}

TEST(CommonModeCoherenceTest, PerfectCoherenceReturnsOne) {
    // Extras spiked exactly as hard as the primary -> perfect coherence
    // -> common-mode verdict.
    EXPECT_DOUBLE_EQ(cm::ComputeCoherenceScore(5.0, {5.0, 5.0, 5.0}), 1.0);
}

TEST(CommonModeCoherenceTest, ExtrasHigherThanPrimaryClampsToOne) {
    // Extras can spike harder than the primary (e.g. their trackers
    // happened to be closer to the disturbed base station). The score
    // is clamped so the formula is monotone in coherence and bounded.
    EXPECT_DOUBLE_EQ(cm::ComputeCoherenceScore(5.0, {8.0, 9.0, 10.0}), 1.0);
}

TEST(CommonModeCoherenceTest, NoCoherenceReturnsNearZero) {
    // Primary spiked 10x, extras stayed near their baselines (ratio
    // ~1.0). Median extra ratio / primary ratio = 1.0 / 10.0 = 0.1.
    EXPECT_NEAR(cm::ComputeCoherenceScore(10.0, {1.0, 1.0, 1.0}), 0.1, 1e-9);
}

TEST(CommonModeCoherenceTest, UsesMedianNotMean) {
    // One outlier extra should not move the score: median resists, mean
    // would not. Primary=5, extras=[5, 5, 100]. Median=5. Score=1.0.
    // If we used mean=36.67, score would clamp to 1.0 too -- so test
    // the asymmetric case: primary=5, extras=[1, 1, 100]. Median=1,
    // score=0.2. Mean would be 34, giving score=1.0.
    EXPECT_NEAR(cm::ComputeCoherenceScore(5.0, {1.0, 1.0, 100.0}), 0.2, 1e-9);
}

TEST(CommonModeCoherenceTest, SingleExtraIsItsOwnMedian) {
    // With one extra, the median IS that extra's ratio. Confirms the
    // n=1 case behaves as the n>1 case extrapolates.
    EXPECT_NEAR(cm::ComputeCoherenceScore(5.0, {5.0}), 1.0, 1e-9);
    EXPECT_NEAR(cm::ComputeCoherenceScore(5.0, {1.0}), 0.2, 1e-9);
}

TEST(CommonModeCoherenceTest, EvenCountMedianIsUpperMiddle) {
    // Sort = [2, 4, 6, 8]. Middle index (size/2) = 2 -> value 6.
    // (This pins the specific median definition; the implementation
    // uses the upper-middle entry for even counts to avoid double
    // arithmetic on small fixed-size lists.)
    EXPECT_NEAR(cm::ComputeCoherenceScore(10.0, {2.0, 4.0, 6.0, 8.0}),
                0.6, 1e-9);
}

// --- ShouldSuppressFire -----------------------------------------------------

TEST(CommonModeCoherenceTest, SuppressRequiresEnoughExtras) {
    // Even a perfect 1.0 score with no extras does not suppress: the
    // score is by definition 0 with empty extras, but defensive
    // double-check at the suppress gate.
    EXPECT_FALSE(cm::ShouldSuppressFire(1.0, 0));
}

TEST(CommonModeCoherenceTest, SuppressFiresAtAndAboveThreshold) {
    EXPECT_TRUE(cm::ShouldSuppressFire(cm::kSuppressThreshold, 1));
    EXPECT_TRUE(cm::ShouldSuppressFire(cm::kSuppressThreshold + 0.01, 1));
    EXPECT_TRUE(cm::ShouldSuppressFire(1.0, 3));
}

TEST(CommonModeCoherenceTest, SuppressBlocksBelowThreshold) {
    EXPECT_FALSE(cm::ShouldSuppressFire(cm::kSuppressThreshold - 0.01, 3));
    EXPECT_FALSE(cm::ShouldSuppressFire(0.0, 3));
    EXPECT_FALSE(cm::ShouldSuppressFire(0.5, 5));
}

// --- Pinned constants -------------------------------------------------------

// kSuppressThreshold and kMinExtrasForCoherence are tuning parameters
// that affect every multi-pair session. Pin them so a change is forced
// through review with the rationale spelled out.
static_assert(cm::kSuppressThreshold == 0.70,
    "kSuppressThreshold changed -- review the bias-toward-firing rationale "
    "in CommonModeCoherence.h before relaxing or tightening");
static_assert(cm::kMinExtrasForCoherence == 1,
    "kMinExtrasForCoherence changed -- single-extra sessions are the "
    "common multi-system layout; raising this disables coherence checks "
    "for them");
