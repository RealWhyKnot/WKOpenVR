// Pin tests for the sustained quality-rejection breaker
// (QualityRejectionBreaker.h). The verdict stream rejects in short streaks
// during healthy operation (measured 2026-07-15/16 baselines: max 71-80
// consecutive) and without bound when the input geometry is structurally
// broken (2026-07-16 session: 1,523 consecutive over 2.4 h). The breaker
// must never engage inside the healthy envelope, must engage on the broken
// pattern, and must release on a single accepted verdict.

#include <gtest/gtest.h>

#include "QualityRejectionBreaker.h"

using spacecal::quality_breaker::kEngageConsecutiveRejects;
using spacecal::quality_breaker::kEngageMinSustainSec;
using spacecal::quality_breaker::Next;
using spacecal::quality_breaker::State;

namespace {

// Feed `count` rejections spaced `dtSec` apart, starting at t=0.
State FeedRejects(State s, int count, double dtSec)
{
	for (int i = 0; i < count; ++i) {
		s = Next(s, /*wouldAccept=*/false, /*nowSec=*/i * dtSec);
	}
	return s;
}

} // namespace

TEST(QualityBreaker, HealthyEnvelopeNeverEngages)
{
	// The measured healthy maximum streak (80), at the observed ~5.6 s
	// verdict cadence, stays well clear of the breaker.
	const State s = FeedRejects({}, 80, 5.6);
	EXPECT_FALSE(s.engaged);
	EXPECT_EQ(s.consecutiveRejects, 80);
}

TEST(QualityBreaker, BrokenSessionPatternEngages)
{
	// The 2026-07-16 pattern: uninterrupted rejections at ~5.6 s cadence.
	// Count and sustain thresholds are both crossed by 150 rejections.
	const State s = FeedRejects({}, kEngageConsecutiveRejects, 5.6);
	EXPECT_TRUE(s.engaged);
}

TEST(QualityBreaker, CountAloneIsNotEnough)
{
	// A burst of fast verdicts (0.1 s apart) reaches the count threshold in
	// 15 s -- far under the sustain floor. Must not engage.
	const State s = FeedRejects({}, kEngageConsecutiveRejects, 0.1);
	EXPECT_FALSE(s.engaged);
}

TEST(QualityBreaker, SustainAloneIsNotEnough)
{
	// Long-running but sparse rejections: 100 rejections over 10 minutes
	// exceeds the sustain floor but not the count threshold.
	State s = FeedRejects({}, 100, 6.0);
	EXPECT_GE(100 * 6.0, kEngageMinSustainSec);
	EXPECT_FALSE(s.engaged);
}

TEST(QualityBreaker, SingleAcceptReleasesAndResets)
{
	State s = FeedRejects({}, kEngageConsecutiveRejects, 5.6);
	ASSERT_TRUE(s.engaged);
	s = Next(s, /*wouldAccept=*/true, /*nowSec=*/10000.0);
	EXPECT_FALSE(s.engaged);
	EXPECT_EQ(s.consecutiveRejects, 0);
	EXPECT_DOUBLE_EQ(s.firstRejectSec, 0.0);
}

TEST(QualityBreaker, AcceptMidStreakRestartsTheCount)
{
	State s = FeedRejects({}, kEngageConsecutiveRejects - 1, 5.6);
	s = Next(s, /*wouldAccept=*/true, /*nowSec=*/900.0);
	EXPECT_EQ(s.consecutiveRejects, 0);
	// A fresh streak must earn both thresholds again from zero.
	s = FeedRejects(s, kEngageConsecutiveRejects - 1, 5.6);
	EXPECT_FALSE(s.engaged);
}

TEST(QualityBreaker, StaysEngagedWhileRejectionsContinue)
{
	State s = FeedRejects({}, kEngageConsecutiveRejects, 5.6);
	ASSERT_TRUE(s.engaged);
	s = Next(s, /*wouldAccept=*/false, /*nowSec=*/2000.0);
	EXPECT_TRUE(s.engaged);
}

// constexpr pins: the threshold relationship the design depends on. The
// engage count must sit clearly above the measured healthy maximum streak.
static_assert(kEngageConsecutiveRejects >= 2 * 80 - 10,
              "engage count must stay well above the measured healthy max streak (80)");
static_assert(Next(State{}, false, 0.0).consecutiveRejects == 1, "a rejection extends the streak");
static_assert(!Next(State{}, false, 0.0).engaged, "one rejection must not engage");
static_assert(Next(State{}, true, 0.0).consecutiveRejects == 0, "an accept resets the streak");
