// AUTO Lock hysteresis + stationary gate decision tests.
//
// Covers the pure helpers in AutoLockHysteresis.h. The intent is to pin the
// hysteresis contract: a tracker hovering near the single 5mm threshold of
// the old detector flapped every few seconds; with separate enter (3mm) and
// leave (8mm) thresholds plus a stationary-HMD commit gate, the same input
// trace should produce zero flips.

#include "AutoLockHysteresis.h"

#include <gtest/gtest.h>

namespace al = spacecal::autolock;

// --- Hysteresis: enter behavior ---------------------------------------------

// While currently UNLOCKED, the detector requires tight evidence (3mm / 0.7
// deg) before flipping to locked. Sloppier values that the old 5mm threshold
// would have accepted now stay unlocked.
TEST(AutoLockHysteresisTest, EnterLockedRequiresTightTranslation)
{
    EXPECT_FALSE(al::VerdictWithHysteresis(/*translStdDev=*/0.004,
                                           /*rotMaxAngle=*/0.001,
                                           /*prevLocked=*/false))
        << "4 mm stddev is too loose to enter locked under the new contract";

    EXPECT_TRUE(al::VerdictWithHysteresis(/*translStdDev=*/0.002,
                                          /*rotMaxAngle=*/0.001,
                                          /*prevLocked=*/false))
        << "2 mm stddev with small rotation should be locked";
}

TEST(AutoLockHysteresisTest, EnterLockedRequiresTightRotation)
{
    const double tenthDeg = 0.1 * EIGEN_PI / 180.0;
    const double oneDeg = 1.0 * EIGEN_PI / 180.0;

    EXPECT_TRUE(al::VerdictWithHysteresis(0.001, tenthDeg, /*prevLocked=*/false));
    EXPECT_FALSE(al::VerdictWithHysteresis(0.001, oneDeg, /*prevLocked=*/false))
        << "1 deg max-from-median is too loose to enter locked";
}

// --- Hysteresis: leave behavior ---------------------------------------------

// While currently LOCKED, the detector tolerates modest noise (up to 8mm /
// 1.5 deg) before unlocking. Sub-threshold jitter that the old detector
// would have rejected now remains locked.
TEST(AutoLockHysteresisTest, StaysLockedThroughDeadbandNoise)
{
    // 5mm is comfortably above the 3mm enter threshold but well under the
    // 8mm leave threshold. The old detector flipped at 5mm; the new one
    // should keep us locked.
    EXPECT_TRUE(al::VerdictWithHysteresis(0.005, 0.005, /*prevLocked=*/true));

    // 7mm is still under the leave threshold; stays locked.
    EXPECT_TRUE(al::VerdictWithHysteresis(0.007, 0.005, /*prevLocked=*/true));
}

TEST(AutoLockHysteresisTest, LeavesLockedOnGenuineLooseness)
{
    EXPECT_FALSE(al::VerdictWithHysteresis(0.012, 0.001, /*prevLocked=*/true))
        << "12 mm stddev is too loose -- should unlock";

    const double twoDeg = 2.0 * EIGEN_PI / 180.0;
    EXPECT_FALSE(al::VerdictWithHysteresis(0.001, twoDeg, /*prevLocked=*/true))
        << "2 deg max-from-median is too loose -- should unlock";
}

// --- Hysteresis: deadband persistence (the headline regression) -------------

// Simulate a trace whose stddev oscillates between 4mm and 6mm -- the exact
// flap pattern the user's head-mounted Vive tracker exhibits near the old
// 5mm boundary. Under the new contract: starts unlocked, never reaches the
// 3mm enter threshold -> stays unlocked. Never flips.
TEST(AutoLockHysteresisTest, FourToSixMillimeterFlapStaysUnlocked)
{
    bool locked = false;
    for (int i = 0; i < 100; ++i) {
        const double stddev = (i & 1) ? 0.004 : 0.006;
        const bool next = al::VerdictWithHysteresis(stddev, 0.001, locked);
        EXPECT_FALSE(next) << "iteration " << i << " stddev=" << stddev;
        locked = next;
    }
}

// Same oscillation but starting from a locked state (the tracker WAS rigidly
// attached but is now mildly jittery). Under the new contract: 6mm is still
// inside the leave threshold (8mm) -> stays locked.
TEST(AutoLockHysteresisTest, FourToSixMillimeterFlapStaysLockedOnceLocked)
{
    bool locked = true;
    for (int i = 0; i < 100; ++i) {
        const double stddev = (i & 1) ? 0.004 : 0.006;
        const bool next = al::VerdictWithHysteresis(stddev, 0.001, locked);
        EXPECT_TRUE(next) << "iteration " << i << " stddev=" << stddev;
        locked = next;
    }
}

// A genuine unlock event (the user pulls the tracker off the headset) crosses
// the leave threshold and the detector commits. Confirms the deadband doesn't
// prevent legitimate state changes.
TEST(AutoLockHysteresisTest, GenuineLooseMotionTransitionsThroughDeadband)
{
    // Start locked, ramp from rigid (1mm) up through the deadband to genuinely
    // loose (15mm). Detector should hold locked through the 4-7mm region and
    // unlock once we exceed 8mm.
    bool locked = true;
    const double trace[] = {0.001, 0.003, 0.005, 0.007, 0.009, 0.011, 0.013, 0.015};
    bool sawUnlock = false;
    for (double stddev : trace) {
        const bool next = al::VerdictWithHysteresis(stddev, 0.001, locked);
        if (locked && !next) sawUnlock = true;
        locked = next;
    }
    EXPECT_TRUE(sawUnlock);
    EXPECT_FALSE(locked);
}

// --- Stationary gate --------------------------------------------------------

TEST(AutoLockHysteresisTest, StationaryGatePassesOnLowSpeed)
{
    EXPECT_TRUE(al::HmdIsStationary(0.0));
    EXPECT_TRUE(al::HmdIsStationary(0.01));
    EXPECT_TRUE(al::HmdIsStationary(0.04));
}

TEST(AutoLockHysteresisTest, StationaryGateFailsOnMotion)
{
    // A casual head turn produces speeds in the 0.1-0.3 m/s range. Walking
    // is 1+ m/s. Both should block the flip commit.
    EXPECT_FALSE(al::HmdIsStationary(0.1));
    EXPECT_FALSE(al::HmdIsStationary(0.5));
    EXPECT_FALSE(al::HmdIsStationary(2.0));
}

// Boundary value: exactly at the threshold counts as NOT stationary.
// Strict less-than avoids commit-on-marginal-motion which would be
// indistinguishable from the old single-threshold behavior near boundary.
TEST(AutoLockHysteresisTest, StationaryThresholdIsStrictLessThan)
{
    EXPECT_FALSE(al::HmdIsStationary(al::kStationaryHmdMps));
}
