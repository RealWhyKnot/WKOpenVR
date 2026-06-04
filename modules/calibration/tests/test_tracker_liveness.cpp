// Tests for the tracker-liveness detector. The detector exists to catch the
// "Running_OK + poseIsValid stays true but the pose hash never changes" case
// SteamVR exposes for silently-disconnected Vive trackers -- the bug captured
// in spacecal_log.2026-05-13T00-48-51.txt where the head-mounted reference
// Vive tracker went silent for ~21 minutes and the calibration transform
// drifted ~7 cm while profile_saved kept persisting the drifted value.
//
// State is caller-owned; the same detector runs independently for referenceID
// and targetID. The header thresholds are pinned by static_asserts at the
// constants themselves; tests here pin the edge behaviour.

#include <gtest/gtest.h>

#include "TrackerLiveness.h"

using spacecal::liveness::IsOffline;
using spacecal::liveness::kFrozenPoseThresholdSec;
using spacecal::liveness::kHmdMovedThresholdMps;
using spacecal::liveness::kReconnectDebounceSec;
using spacecal::liveness::Reset;
using spacecal::liveness::TickTrackerLiveness;
using spacecal::liveness::TrackerLivenessInputs;
using spacecal::liveness::TrackerLivenessState;

namespace {

// Helper to drive a few ticks at a fixed tick rate. Builds the input struct
// the same way the production caller does.
TrackerLivenessInputs MakeInput(uint64_t hash, bool connected, double hmdMps, double emaUpdateSec, double now)
{
	TrackerLivenessInputs in{};
	in.posHash = hash;
	in.deviceIsConnected = connected;
	in.hmdSpeedMps = hmdMps;
	in.lastEmaUpdateSec = emaUpdateSec;
	in.now = now;
	return in;
}

} // namespace

// ---------------------------------------------------------------------------
// Healthy session: pose hash changes every tick, HMD has natural motion.
// Detector must never fire and IsOffline must never report true.
// ---------------------------------------------------------------------------
TEST(TrackerLivenessTest, HealthySession_HashChangesEveryTick_NeverFires)
{
	TrackerLivenessState s{};
	for (int i = 0; i < 1000; ++i) {
		const double now = i * 0.05; // 50 ms tick (matches CalibrationTick gate)
		const uint64_t hash = static_cast<uint64_t>(i) + 1;
		const auto in = MakeInput(hash, /*connected=*/true, /*hmd=*/0.30,
		                          /*ema=*/now, now);
		EXPECT_FALSE(TickTrackerLiveness(s, in)) << "tick " << i;
		EXPECT_FALSE(IsOffline(s)) << "tick " << i;
	}
}

// ---------------------------------------------------------------------------
// Stationary tracker, stationary user: HMD speed stays at zero. Even though
// the pose hash is permanently frozen, we must NOT fire -- the user is
// legitimately sitting still with a tracker resting on a table. This is the
// load-bearing false-positive guard.
// ---------------------------------------------------------------------------
TEST(TrackerLivenessTest, StationaryTrackerStationaryHmd_NeverFires)
{
	TrackerLivenessState s{};
	const uint64_t frozenHash = 0xCAFE'BABE'DEAD'BEEFull;
	for (int i = 0; i < 1000; ++i) {
		const double now = i * 0.05;
		const auto in = MakeInput(frozenHash, /*connected=*/true,
		                          /*hmd=*/0.0, /*ema=*/now, now);
		EXPECT_FALSE(TickTrackerLiveness(s, in)) << "tick " << i;
		EXPECT_FALSE(IsOffline(s)) << "tick " << i;
	}
}

// ---------------------------------------------------------------------------
// Stationary tracker, moving user: the bug scenario. Pose hash frozen, HMD
// has crossed the 0.02 m/s threshold. Within the debounce window we don't
// fire; once the window elapses we fire exactly once on the edge.
// ---------------------------------------------------------------------------
TEST(TrackerLivenessTest, FrozenTrackerMovingHmd_FiresOnceAfterDebounce)
{
	TrackerLivenessState s{};
	const uint64_t frozenHash = 0x1111'2222'3333'4444ull;

	// Pre-window: 6 s with frozen hash + moving HMD. No fire (under 8 s).
	bool sawFireEarly = false;
	for (int i = 0; i < 120; ++i) { // 120 * 0.05 = 6 s
		const double now = i * 0.05;
		const auto in = MakeInput(frozenHash, /*connected=*/true,
		                          /*hmd=*/0.30, /*ema=*/now, now);
		sawFireEarly = sawFireEarly || TickTrackerLiveness(s, in);
		EXPECT_FALSE(IsOffline(s)) << "tick " << i;
	}
	EXPECT_FALSE(sawFireEarly) << "Detector fired before kFrozenPoseThresholdSec elapsed";

	// Just past the threshold: this tick should fire on the edge.
	const double fireTime = kFrozenPoseThresholdSec + 0.05;
	const auto fireIn = MakeInput(frozenHash, true, 0.30, fireTime, fireTime);
	EXPECT_TRUE(TickTrackerLiveness(s, fireIn));
	EXPECT_TRUE(IsOffline(s));

	// Subsequent ticks (still frozen) must not re-fire; the edge is one-shot.
	for (int i = 0; i < 50; ++i) {
		const double now = fireTime + 0.05 + i * 0.05;
		const auto in = MakeInput(frozenHash, true, 0.30, now, now);
		EXPECT_FALSE(TickTrackerLiveness(s, in));
		EXPECT_TRUE(IsOffline(s));
	}
}

// ---------------------------------------------------------------------------
// deviceIsConnected fast path: when SteamVR does flip the per-pose
// `deviceIsConnected` flag false, fire on the very next tick regardless of
// the hash window. Useful for the cases where SteamVR is honest about the
// disconnect.
// ---------------------------------------------------------------------------
TEST(TrackerLivenessTest, DeviceIsConnectedFalse_FiresImmediately)
{
	TrackerLivenessState s{};

	// Healthy ticks first.
	for (int i = 0; i < 10; ++i) {
		const auto in = MakeInput((uint64_t)i + 1, true, 0.30, i * 0.05, i * 0.05);
		EXPECT_FALSE(TickTrackerLiveness(s, in));
	}

	// Connected flag flips false this tick. Hash is still changing.
	const auto in = MakeInput(/*hash=*/0xABCD, /*connected=*/false,
	                          /*hmd=*/0.30, /*ema=*/0.6, /*now=*/0.6);
	EXPECT_TRUE(TickTrackerLiveness(s, in));
	EXPECT_TRUE(IsOffline(s));
}

// ---------------------------------------------------------------------------
// Online -> offline -> online cycle with reconnect debounce. Verify:
//   - Fire exactly once on the offline edge.
//   - IsOffline stays true through the offline window.
//   - A single coincidental hash change does NOT prematurely declare online.
//   - Sustained hash changes for kReconnectDebounceSec restore online.
//   - Reset() returns state to a clean baseline.
// ---------------------------------------------------------------------------
TEST(TrackerLivenessTest, OfflineToOnline_RequiresReconnectDebounce)
{
	TrackerLivenessState s{};
	const uint64_t frozenHash = 0xFADE'CAFE'BAAD'F00Dull;

	// Drive to offline.
	int fireCount = 0;
	for (int i = 0; i < 250; ++i) { // 12.5 s of frozen hash + moving HMD
		const double now = i * 0.05;
		const auto in = MakeInput(frozenHash, true, 0.30, now, now);
		if (TickTrackerLiveness(s, in)) ++fireCount;
	}
	EXPECT_EQ(fireCount, 1) << "Offline edge must fire exactly once";
	EXPECT_TRUE(IsOffline(s));

	// Coincidental single hash change. Must NOT restore online -- starts a
	// reconnect debounce window only.
	{
		const double now = 12.55;
		const auto in = MakeInput(/*hash=*/frozenHash + 1, true, 0.30, now, now);
		EXPECT_FALSE(TickTrackerLiveness(s, in));
		EXPECT_TRUE(IsOffline(s)) << "Single hash change must not snap online before debounce";
	}

	// Hash freezes again (the "blip" was an artefact). Reconnect debounce
	// aborts and we stay offline.
	for (int i = 0; i < 20; ++i) {
		const double now = 12.60 + i * 0.05;
		const auto in = MakeInput(frozenHash + 1, true, 0.30, now, now);
		EXPECT_FALSE(TickTrackerLiveness(s, in));
		EXPECT_TRUE(IsOffline(s));
	}

	// Now real reconnect: hash changes every tick for > kReconnectDebounceSec.
	bool wentOnline = false;
	for (int i = 0; i < 100; ++i) { // 5 s of healthy pose changes
		const double now = 14.0 + i * 0.05;
		const uint64_t hash = 0x10000ull + static_cast<uint64_t>(i);
		const auto in = MakeInput(hash, true, 0.30, now, now);
		EXPECT_FALSE(TickTrackerLiveness(s, in));
		if (!IsOffline(s)) {
			wentOnline = true;
			break;
		}
	}
	EXPECT_TRUE(wentOnline) << "Sustained hash changes must restore online after debounce";

	// Reset returns to baseline.
	s.offlineSinceSec = 100.0;
	Reset(s);
	EXPECT_FALSE(IsOffline(s));
	EXPECT_LT(s.poseHashSinceSec, 0.0);
	EXPECT_LT(s.offlineSinceSec, 0.0);
	EXPECT_LT(s.reconnectSinceSec, 0.0);
	EXPECT_EQ(s.lastPoseHash, 0u);
	EXPECT_FALSE(s.hmdMovedDuringFreeze);
}

// ---------------------------------------------------------------------------
// EMA gap alone (without frozen hash) must never fire. The EMA gap is a
// corroborating signal only -- a quiet session with low motion can show a
// large gap without anything being wrong.
// ---------------------------------------------------------------------------
TEST(TrackerLivenessTest, EmaGapAloneWithLiveHash_NeverFires)
{
	TrackerLivenessState s{};
	// Hash changes every tick (device live), but EMA gap grows unboundedly.
	for (int i = 0; i < 1500; ++i) { // 75 s
		const double now = i * 0.05;
		const auto in = MakeInput((uint64_t)i + 1, true, 0.30,
		                          /*ema=*/0.0, // never advances
		                          now);
		EXPECT_FALSE(TickTrackerLiveness(s, in)) << "tick " << i;
		EXPECT_FALSE(IsOffline(s)) << "tick " << i;
	}
}

// ---------------------------------------------------------------------------
// Independence: two state instances driven by different inputs do not
// interfere. Mirrors the production use of one state for reference and
// one for target -- a regression here would mean a stale reference state
// could falsely suppress a healthy target.
// ---------------------------------------------------------------------------
TEST(TrackerLivenessTest, TwoStates_AreIndependent)
{
	TrackerLivenessState ref{};
	TrackerLivenessState tgt{};

	const uint64_t refFrozen = 0xAAAA'AAAA'AAAA'AAAAull;
	// Drive ref to offline, target stays healthy.
	for (int i = 0; i < 250; ++i) {
		const double now = i * 0.05;
		const auto refIn = MakeInput(refFrozen, true, 0.30, now, now);
		const auto tgtIn = MakeInput((uint64_t)i + 1, true, 0.30, now, now);
		TickTrackerLiveness(ref, refIn);
		TickTrackerLiveness(tgt, tgtIn);
	}
	EXPECT_TRUE(IsOffline(ref));
	EXPECT_FALSE(IsOffline(tgt));
}
