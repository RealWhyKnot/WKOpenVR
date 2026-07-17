// Tests for the continuous-mode persistence cadence. Continuous calibration
// updates the in-memory offset every accepted candidate, but the registry copy
// is read only at startup, so the per-tick SaveProfile was pure I/O for a value
// nobody reads mid-session (live sessions logged ~1.2 registry writes/sec of
// sub-millimetre drift). ShouldPersistContinuous decides when a save is worth
// doing; this pins the cadence contract: first candidate forces, a meaningful
// move forces, otherwise persist at most once per interval.

#include <gtest/gtest.h>

#include "ContinuousPersistDecision.h"
#include "FileLog.h"

using spacecal::persist::kContinuousSaveDeltaCm;
using spacecal::persist::kContinuousSaveIntervalSec;
using spacecal::persist::ShouldPersistContinuous;

// The first accepted candidate after continuous calibration starts always
// persists, regardless of timing or delta — the next session needs a baseline.
TEST(ContinuousPersist, FirstCandidateAlwaysPersists)
{
	// now == lastSave, zero delta: only the first-candidate flag forces it.
	EXPECT_TRUE(ShouldPersistContinuous(/*now=*/100.0, /*lastSave=*/100.0, /*deltaCm=*/0.0,
	                                    /*isFirstCandidate=*/true));
}

// Steady state: same instant, no movement, not the first candidate -> skip.
TEST(ContinuousPersist, SteadyStateWithinIntervalSkips)
{
	EXPECT_FALSE(ShouldPersistContinuous(/*now=*/100.0, /*lastSave=*/100.0, /*deltaCm=*/0.0,
	                                     /*isFirstCandidate=*/false));
	// Just under the interval, tiny sub-deadband jitter: still skip.
	EXPECT_FALSE(ShouldPersistContinuous(/*now=*/100.0 + kContinuousSaveIntervalSec - 0.01,
	                                     /*lastSave=*/100.0, /*deltaCm=*/kContinuousSaveDeltaCm * 0.5,
	                                     /*isFirstCandidate=*/false));
}

// The interval gate fires once enough time has elapsed since the last persist,
// so rotation-only drift (which this delta doesn't capture) still reaches disk.
TEST(ContinuousPersist, IntervalElapsedPersists)
{
	EXPECT_TRUE(ShouldPersistContinuous(/*now=*/100.0 + kContinuousSaveIntervalSec, /*lastSave=*/100.0,
	                                    /*deltaCm=*/0.0, /*isFirstCandidate=*/false));
	EXPECT_TRUE(ShouldPersistContinuous(/*now=*/100.0 + kContinuousSaveIntervalSec + 5.0, /*lastSave=*/100.0,
	                                    /*deltaCm=*/0.0, /*isFirstCandidate=*/false));
}

// A meaningful move persists immediately even within the interval, so a real
// relocalization or large adjustment doesn't wait out the cadence.
TEST(ContinuousPersist, MeaningfulMovePersistsEarly)
{
	EXPECT_TRUE(ShouldPersistContinuous(/*now=*/100.0, /*lastSave=*/100.0,
	                                    /*deltaCm=*/kContinuousSaveDeltaCm + 0.001,
	                                    /*isFirstCandidate=*/false));
	// At exactly the deadband it does NOT fire (strictly greater-than); the
	// interval gate will eventually catch it.
	EXPECT_FALSE(ShouldPersistContinuous(/*now=*/100.0, /*lastSave=*/100.0,
	                                     /*deltaCm=*/kContinuousSaveDeltaCm,
	                                     /*isFirstCandidate=*/false));
}

// Bounded-staleness sequence: simulate a steady drift of sub-deadband steps and
// confirm persists happen on a cadence (first, then ~every interval), not every
// tick. Models the live failure mode (~1.2 writes/sec) collapsing to ~0.5/sec.
TEST(ContinuousPersist, DriftSequenceThrottlesToCadence)
{
	double lastSave = -1e9; // construction default
	double persistedTransCm = 0.0;
	double currentTransCm = 0.0;
	int persists = 0;
	const double dtPerTick = 0.05;       // 20 Hz tick
	const double driftPerTickCm = 0.002; // 0.02 mm/tick, well under the deadband
	const int ticks = 200;               // 10 s of session

	for (int i = 0; i < ticks; ++i) {
		const double now = i * dtPerTick;
		currentTransCm += driftPerTickCm;
		const double deltaCm = currentTransCm - persistedTransCm;
		const bool isFirst = (i == 0);
		if (ShouldPersistContinuous(now, lastSave, deltaCm, isFirst)) {
			++persists;
			lastSave = now;
			persistedTransCm = currentTransCm;
		}
	}

	// Without the throttle this would be 200 saves. With it: the first candidate
	// plus interval-gated saves over 10 s at a 2 s interval -> ~6, not 200.
	EXPECT_GE(persists, 4);
	EXPECT_LE(persists, 8);
	EXPECT_LT(persists, ticks / 4);
}

// Absolute-value pins for the deadband: sub-centimetre solver drift must ride
// the interval cadence (the earlier 1 mm deadband sat below solve noise, so
// nearly every accepted candidate bypassed the interval -- 16k registry writes
// in one 5.3 h session), while a real centimetre-scale move reaches disk
// immediately.
TEST(ContinuousPersist, SubCentimeterDriftWaitsForInterval)
{
	EXPECT_FALSE(ShouldPersistContinuous(/*now=*/100.5, /*lastSave=*/100.0, /*deltaCm=*/0.9,
	                                     /*isFirstCandidate=*/false));
}

TEST(ContinuousPersist, CentimeterMovePersistsEarly)
{
	EXPECT_TRUE(ShouldPersistContinuous(/*now=*/100.5, /*lastSave=*/100.0, /*deltaCm=*/1.1,
	                                    /*isFirstCandidate=*/false));
}

// constexpr-evaluation pin: the decision folds at compile time, so a refactor
// that accidentally makes it runtime-only (or flips the first-candidate force)
// breaks the build rather than silently regressing the cadence.
static_assert(ShouldPersistContinuous(0.0, 0.0, 0.0, /*isFirstCandidate=*/true),
              "first continuous candidate must always persist");
static_assert(!ShouldPersistContinuous(0.0, 0.0, 0.0, /*isFirstCandidate=*/false),
              "steady-state zero-delta within interval must not persist");

// ---------------------------------------------------------------------------
// Oversized-delta persist guard (ShouldDeferAnomalousPersist). A translation
// more than kDeferDeltaCm from the last persisted value must prove itself by
// dwell or by consecutive agreeing attempts before it reaches the registry;
// a 30392 cm transient was persisted once and poisoned the next launch.
// Delta-based on purpose -- absolute magnitude clamps broke rigs whose
// legitimate calibration values are large.
// ---------------------------------------------------------------------------
using spacecal::persist::kDeferAgreeingAttempts;
using spacecal::persist::kDeferDeltaCm;
using spacecal::persist::kDeferDwellSec;
using spacecal::persist::ShouldDeferAnomalousPersist;

TEST(AnomalousPersistGuard, NormalDeltasNeverDefer)
{
	EXPECT_FALSE(ShouldDeferAnomalousPersist(/*deltaCm=*/0.0, /*dwellSec=*/0.0, /*agreeing=*/0));
	EXPECT_FALSE(ShouldDeferAnomalousPersist(/*deltaCm=*/50.0, 0.0, 0));
	EXPECT_FALSE(ShouldDeferAnomalousPersist(kDeferDeltaCm, 0.0, 0)) << "boundary is inclusive-persist";
}

TEST(AnomalousPersistGuard, FreshOversizedDeltaDefers)
{
	EXPECT_TRUE(ShouldDeferAnomalousPersist(kDeferDeltaCm + 0.01, /*dwellSec=*/0.0, /*agreeing=*/1));
	EXPECT_TRUE(ShouldDeferAnomalousPersist(/*deltaCm=*/30392.0, 0.0, 1))
	    << "the persisted-wedge magnitude from the 2026-05-19 trace must defer";
}

TEST(AnomalousPersistGuard, DwellProvesTheValue)
{
	EXPECT_TRUE(ShouldDeferAnomalousPersist(200.0, kDeferDwellSec - 0.01, /*agreeing=*/1));
	EXPECT_FALSE(ShouldDeferAnomalousPersist(200.0, kDeferDwellSec, 1)) << "held long enough: persist";
}

TEST(AnomalousPersistGuard, AgreeingAttemptsProveTheValue)
{
	EXPECT_TRUE(ShouldDeferAnomalousPersist(200.0, /*dwellSec=*/1.0, kDeferAgreeingAttempts - 1));
	EXPECT_FALSE(ShouldDeferAnomalousPersist(200.0, 1.0, kDeferAgreeingAttempts))
	    << "enough consecutive agreeing attempts: persist without waiting out the dwell";
}

static_assert(!ShouldDeferAnomalousPersist(100.0, 0.0, 0), "at the delta bound, persist normally");
static_assert(ShouldDeferAnomalousPersist(100.01, 0.0, 1), "just past the bound with no proof, defer");
static_assert(!ShouldDeferAnomalousPersist(1000.0, 10.0, 1), "dwell threshold reached, persist");
static_assert(!ShouldDeferAnomalousPersist(1000.0, 0.0, 3), "agreeing-attempt threshold reached, persist");

// ---------------------------------------------------------------------------
// Diagnostic-log deferred device-sync policy (openvr_pair::common::ShouldFlushLog).
// ---------------------------------------------------------------------------
using openvr_pair::common::kLogFlushBytes;
using openvr_pair::common::kLogFlushIntervalMs;
using openvr_pair::common::ShouldFlushLog;

// Nothing accumulated, no time elapsed -> don't sync. This is the per-line case
// that previously forced a FlushFileBuffers ~22x/sec.
TEST(LogFlushPolicy, FreshNothingPendingSkips)
{
	EXPECT_FALSE(ShouldFlushLog(/*bytesSinceFlush=*/0, /*msSinceFlush=*/0));
	EXPECT_FALSE(ShouldFlushLog(/*bytesSinceFlush=*/128, /*msSinceFlush=*/5));
}

// The byte threshold forces a sync so a burst of large rows can't sit unsynced.
TEST(LogFlushPolicy, ByteThresholdForcesSync)
{
	EXPECT_FALSE(ShouldFlushLog(kLogFlushBytes - 1, 0));
	EXPECT_TRUE(ShouldFlushLog(kLogFlushBytes, 0));
	EXPECT_TRUE(ShouldFlushLog(kLogFlushBytes * 4, 0));
}

// The time threshold bounds the unsynced crash tail to one interval even when
// only a trickle of bytes is written.
TEST(LogFlushPolicy, TimeThresholdForcesSync)
{
	EXPECT_FALSE(ShouldFlushLog(16, kLogFlushIntervalMs - 1));
	EXPECT_TRUE(ShouldFlushLog(16, kLogFlushIntervalMs));
	EXPECT_TRUE(ShouldFlushLog(0, kLogFlushIntervalMs + 1000));
}

static_assert(!ShouldFlushLog(0, 0), "no pending data and no elapsed time must not sync");
static_assert(ShouldFlushLog(kLogFlushBytes, 0), "byte threshold must force a sync");
static_assert(ShouldFlushLog(0, kLogFlushIntervalMs), "interval must force a sync");
