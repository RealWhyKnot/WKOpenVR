#include "HostSupervisorBase.h"

#include <gtest/gtest.h>

namespace {

using openvr_pair::common::DecideHostExitAction;
using openvr_pair::common::HostExitAction;
using openvr_pair::common::kBackoffStartMs;
using openvr_pair::common::kCircuitBreakerThreshold;
using openvr_pair::common::kFastExitThresholdMs;
using openvr_pair::common::kSingletonRetryDelayMs;

TEST(HostExitAction, StaleGenerationExitIsIgnored)
{
	// Restart() replaced the tracked process while the monitor waited on the
	// old handle: the signaled exit is not the tracked host's. Nothing may be
	// reaped or counted -- the handle now belongs to a live replacement.
	const HostExitAction a = DecideHostExitAction(/*generationMatches=*/false, /*exitCode=*/259, /*uptimeMs=*/314,
	                                              /*consecutiveFastExitsBefore=*/1, kBackoffStartMs);
	EXPECT_TRUE(a.ignore);
	EXPECT_FALSE(a.halt);
	EXPECT_EQ(a.consecutiveFastExits, 1);
}

TEST(HostExitAction, SingletonExitPacesRetryAndSkipsCounter)
{
	// Codes 3/4: another instance holds the singleton mutex. Not a failure
	// (no fast-exit count), but never an immediate respawn either -- an
	// instant retry loops against a mutex owner whose pipe isn't up yet.
	for (DWORD code = 3; code <= 4; ++code) {
		const HostExitAction a = DecideHostExitAction(true, code, /*uptimeMs=*/110,
		                                              /*consecutiveFastExitsBefore=*/2, kBackoffStartMs);
		EXPECT_FALSE(a.ignore);
		EXPECT_FALSE(a.halt);
		EXPECT_EQ(a.consecutiveFastExits, 2);
		EXPECT_EQ(a.respawnDelayMs, kSingletonRetryDelayMs);
		EXPECT_GT(a.respawnDelayMs, 0);
	}
}

TEST(HostExitAction, FastExitIncrementsCounterAndUsesBackoff)
{
	const HostExitAction a = DecideHostExitAction(true, /*exitCode=*/1, kFastExitThresholdMs - 1,
	                                              /*consecutiveFastExitsBefore=*/0, kBackoffStartMs);
	EXPECT_FALSE(a.ignore);
	EXPECT_FALSE(a.halt);
	EXPECT_EQ(a.consecutiveFastExits, 1);
	EXPECT_EQ(a.respawnDelayMs, kBackoffStartMs);
}

TEST(HostExitAction, SlowExitResetsCounter)
{
	const HostExitAction a = DecideHostExitAction(true, /*exitCode=*/1, kFastExitThresholdMs,
	                                              /*consecutiveFastExitsBefore=*/3, kBackoffStartMs);
	EXPECT_EQ(a.consecutiveFastExits, 0);
	EXPECT_FALSE(a.halt);
}

TEST(HostExitAction, CircuitBreakerTripsAtThreshold)
{
	const HostExitAction below =
	    DecideHostExitAction(true, /*exitCode=*/1, /*uptimeMs=*/0, kCircuitBreakerThreshold - 2, kBackoffStartMs);
	EXPECT_FALSE(below.halt);

	const HostExitAction at =
	    DecideHostExitAction(true, /*exitCode=*/1, /*uptimeMs=*/0, kCircuitBreakerThreshold - 1, kBackoffStartMs);
	EXPECT_TRUE(at.halt);
	EXPECT_EQ(at.consecutiveFastExits, kCircuitBreakerThreshold);
}

TEST(HostExitAction, NormalExitUsesCurrentBackoff)
{
	const HostExitAction a = DecideHostExitAction(true, /*exitCode=*/0, kFastExitThresholdMs + 1,
	                                              /*consecutiveFastExitsBefore=*/0, /*backoffMs=*/8000);
	EXPECT_EQ(a.consecutiveFastExits, 0);
	EXPECT_EQ(a.respawnDelayMs, 8000);
}

} // namespace
