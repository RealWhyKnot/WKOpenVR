#pragma once

// Persistence cadence for the continuously-updated calibration offset. Pure,
// header-only, testable in isolation (tests/test_continuous_persist.cpp).
//
// Continuous calibration updates the in-memory offset on every accepted
// candidate (~once per solve tick). The registry copy of that offset is read
// only once, at startup (ParseProfile) -- the driver receives offsets over IPC,
// and recovery uses in-memory snapshots -- so serializing a ~5.5 KB profile and
// writing it to HKCU on every tick is pure I/O for a value nobody reads until
// the next session. Live sessions logged ~1.2 registry writes/sec of mostly
// sub-millimetre solver drift.
//
// We instead persist on a cadence: immediately on the first continuous
// candidate, then at most every kContinuousSaveIntervalSec, or sooner if the
// offset has moved more than kContinuousSaveDeltaCm since the last persist. The
// in-memory offset stays current every tick regardless, so tracking is
// unaffected; only the throwaway persistence frequency changes. A pending
// (unpersisted) offset is flushed when continuous mode ends and on shutdown so
// the latest value always survives to the next session.

namespace spacecal::persist {

// At most one persist every 2 s in steady state. Bounds the worst-case staleness
// of the on-disk copy to ~2 s of micro-drift, which only matters if the process
// dies before the next periodic save -- the next session simply loads a value a
// couple of seconds older and continuous cal re-converges immediately.
constexpr double kContinuousSaveIntervalSec = 2.0;

// Persist early when the offset jumps by more than this since the last persist,
// so a real move (relocalization, large adjustment) reaches disk promptly rather
// than waiting out the interval. 1.0 cm sits above per-solve noise (the earlier
// 1 mm value was below it, so ordinary solver drift bypassed the interval cap
// on nearly every accepted candidate -- 16k registry writes in one session) and
// matches the locked-accept step deadband: anything the solver is allowed to
// apply below this lands on the interval cadence instead.
constexpr double kContinuousSaveDeltaCm = 1.0;

// Returns true when the continuous-mode offset should be persisted to the
// registry this tick. `offsetDeltaCm` is the distance from the last persisted
// translation to the current one. `isFirstCandidate` forces a save on the first
// accepted candidate after continuous calibration starts.
constexpr bool ShouldPersistContinuous(double nowSec, double lastSaveSec, double offsetDeltaCm, bool isFirstCandidate,
                                       double intervalSec = kContinuousSaveIntervalSec,
                                       double deltaCm = kContinuousSaveDeltaCm)
{
	if (isFirstCandidate) return true;
	if (offsetDeltaCm > deltaCm) return true;
	// Unconditional on elapsed time: rotation-only drift moves no translation
	// but still changes the profile, and the save-side content hash already
	// skips truly identical payloads.
	if ((nowSec - lastSaveSec) >= intervalSec) return true;
	return false;
}

} // namespace spacecal::persist
