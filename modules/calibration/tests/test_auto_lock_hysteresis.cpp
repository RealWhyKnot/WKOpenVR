// AUTO Lock hysteresis + stationary gate decision tests.
//
// Covers the pure helpers in AutoLockHysteresis.h. The intent is to pin the
// hysteresis contract: a tracker hovering near the single 5mm threshold of
// the old detector flapped every few seconds; with separate enter (3mm) and
// leave (8mm) thresholds plus a stationary-HMD commit gate, the same input
// trace should produce zero flips.

#include "AutoLockHysteresis.h"

#include <gtest/gtest.h>

#include <deque>

namespace al = spacecal::autolock;

namespace {

// Helper: build a relative-pose sample at the given translation, identity
// rotation. Tests that vary rotation use the overload below.
Eigen::AffineCompact3d Sample(double x, double y, double z)
{
    Eigen::AffineCompact3d a = Eigen::AffineCompact3d::Identity();
    a.translation() = Eigen::Vector3d(x, y, z);
    return a;
}

Eigen::AffineCompact3d SampleRot(const Eigen::Quaterniond& q)
{
    Eigen::AffineCompact3d a = Eigen::AffineCompact3d::Identity();
    a.linear() = q.toRotationMatrix();
    return a;
}

}  // namespace

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

// --- Reanchor suppression --------------------------------------------------

// Default unset deadline (0.0) never suppresses, regardless of current time.
// Important so a fresh session before any reanchor has fired doesn't
// accidentally hold every queued flip.
TEST(AutoLockHysteresisTest, ReanchorSuppressionUnsetByDefault)
{
    EXPECT_FALSE(al::ShouldSuppressForReanchor(/*now=*/0.0, /*suppressUntil=*/0.0));
    EXPECT_FALSE(al::ShouldSuppressForReanchor(/*now=*/100.0, /*suppressUntil=*/0.0));
    EXPECT_FALSE(al::ShouldSuppressForReanchor(/*now=*/1e9, /*suppressUntil=*/0.0));
}

// While `now` sits inside the window (now < suppressUntil), the gate holds.
// The reanchor that armed the window was emitted at suppressUntil - 2.0 s
// per kReanchorSuppressSeconds; we exercise both fresh-after-reanchor and
// nearly-expired points inside the band.
TEST(AutoLockHysteresisTest, ReanchorSuppressionHoldsInsideWindow)
{
    const double armedAt = 1000.0;
    const double until   = armedAt + al::kReanchorSuppressSeconds;

    EXPECT_TRUE(al::ShouldSuppressForReanchor(/*now=*/armedAt, until));
    EXPECT_TRUE(al::ShouldSuppressForReanchor(/*now=*/armedAt + 0.5, until));
    EXPECT_TRUE(al::ShouldSuppressForReanchor(/*now=*/until - 1e-6, until));
}

// Once `now >= suppressUntil`, the gate releases. Boundary uses `<` so the
// exact-equal sample is the first allowed tick; the next tick onward also
// passes. A legitimate flip queued during the window must be commit-able
// the moment the window expires.
TEST(AutoLockHysteresisTest, ReanchorSuppressionReleasesAtAndAfterDeadline)
{
    const double until = 1000.0;

    EXPECT_FALSE(al::ShouldSuppressForReanchor(/*now=*/until, until));
    EXPECT_FALSE(al::ShouldSuppressForReanchor(/*now=*/until + 1e-6, until));
    EXPECT_FALSE(al::ShouldSuppressForReanchor(/*now=*/until + 10.0, until));
}

// --- MAD-based robust deviation metrics ------------------------------------

// 28 tight samples (sub-mm scatter around origin) plus 2 outliers at 25mm.
// The previous sqrt(variance) metric would compute ~6mm and unlock the
// detector mid-session; MAD ignores the outliers and stays at the
// steady-state noise level. This is the exact failure mode the 2026-05-21
// log showed: pending LOCKs cancelled by transient pose spikes.
TEST(AutoLockHysteresisTest, RobustTranslDeviationRejectsOutlierSpike)
{
    std::deque<Eigen::AffineCompact3d> history;
    // Smooth triangular ramp across ±0.5mm so the componentwise median sits
    // near origin (alternating-sign distributions land the median at one
    // mode and double the apparent MAD).
    for (int i = 0; i < 28; ++i) {
        const double t = ((double)i - 13.5) / 13.5;  // -1 to +1
        history.push_back(Sample(0.0005 * t, 0.0002 * t, 0.0));
    }
    // Two outliers at 25mm along x.
    history.push_back(Sample(0.025, 0.0, 0.0));
    history.push_back(Sample(0.025, 0.0, 0.0));

    const double mad = al::RobustTranslDeviation(history);
    EXPECT_LT(mad, 0.003)
        << "MAD should ignore the two 25mm outliers and stay below the 3mm "
           "enter threshold. Got " << mad << " m.";
}

// 30 samples drifting linearly across 0-15mm represent an actual rigidity
// change (tracker slowly slipping). MAD should rise enough to trip the 8mm
// leave threshold.
TEST(AutoLockHysteresisTest, RobustTranslDeviationDetectsSustainedDrift)
{
    std::deque<Eigen::AffineCompact3d> history;
    for (int i = 0; i < 30; ++i) {
        const double x = 0.015 * (double)i / 29.0;
        history.push_back(Sample(x, 0.0, 0.0));
    }
    const double mad = al::RobustTranslDeviation(history);
    EXPECT_GT(mad, 0.005)
        << "Sustained drift across 15mm should produce MAD large enough to "
           "be meaningful. Got " << mad << " m.";
}

// Rotation analogue of the outlier-rejection test. 28 samples at identity
// + 2 large-angle outliers; MAD should stay below the 0.7 deg enter
// threshold.
TEST(AutoLockHysteresisTest, RobustRotDeviationRejectsOutlierSpike)
{
    std::deque<Eigen::AffineCompact3d> history;
    for (int i = 0; i < 28; ++i) {
        // Tiny rotation noise (~0.05 deg).
        const double sign = (i & 1) ? 1.0 : -1.0;
        const double angle = sign * 0.05 * EIGEN_PI / 180.0;
        history.push_back(SampleRot(
            Eigen::Quaterniond(Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()))));
    }
    // Two outliers at 5 degrees.
    const double bigAngle = 5.0 * EIGEN_PI / 180.0;
    history.push_back(SampleRot(
        Eigen::Quaterniond(Eigen::AngleAxisd(bigAngle, Eigen::Vector3d::UnitZ()))));
    history.push_back(SampleRot(
        Eigen::Quaterniond(Eigen::AngleAxisd(bigAngle, Eigen::Vector3d::UnitZ()))));

    const double mad = al::RobustRotDeviation(history);
    const double enterThresh = 0.7 * EIGEN_PI / 180.0;
    EXPECT_LT(mad, enterThresh)
        << "Rotation MAD should ignore the two 5-deg outliers. Got " << mad
        << " rad (" << (mad * 180.0 / EIGEN_PI) << " deg).";
}

// Empty history returns zero rather than NaN/garbage. The detector caller
// gates on history size before calling these helpers, but the helpers
// themselves should be safe to call on an empty deque.
TEST(AutoLockHysteresisTest, RobustDeviationOnEmptyHistoryReturnsZero)
{
    std::deque<Eigen::AffineCompact3d> history;
    EXPECT_DOUBLE_EQ(al::RobustTranslDeviation(history), 0.0);
    EXPECT_DOUBLE_EQ(al::RobustRotDeviation(history), 0.0);
}

// --- End-to-end: bimodal Quest+Lighthouse noise pattern --------------------

// Synthesize the exact failure mode from the 2026-05-21 log:
// most samples are tight (sub-2mm), but every ~10th sample is a transient
// spike at 10-15mm. Drive RobustTranslDeviation -> VerdictWithHysteresis;
// confirm pending LOCK survives the spikes (previous sqrt(variance) would
// have unlocked within one tick of each spike).
TEST(AutoLockHysteresisTest, BimodalNoiseStaysLockedThroughSpikes)
{
    std::deque<Eigen::AffineCompact3d> history;
    bool locked = false;
    int flipCount = 0;

    // Seed with 30 tight samples to enter the locked state -- smooth ramp
    // so the median sits near origin and MAD reflects the actual scatter.
    for (int i = 0; i < 30; ++i) {
        const double t = ((double)i - 14.5) / 14.5;  // -1 to +1
        history.push_back(Sample(0.0005 * t, 0.0, 0.0));
    }
    {
        const bool verdict = al::VerdictWithHysteresis(
            al::RobustTranslDeviation(history),
            al::RobustRotDeviation(history),
            locked);
        if (verdict != locked) ++flipCount;
        locked = verdict;
    }
    EXPECT_TRUE(locked) << "Initial tight window should enter locked.";

    // Now feed 100 more samples: tight noise with 10% spike rate at 12mm.
    for (int i = 0; i < 100; ++i) {
        const bool isSpike = (i % 10 == 5);
        const double t = ((double)(i % 13) - 6.0) / 6.0;  // smooth -1..+1
        const double x = isSpike ? 0.012 : 0.0005 * t;
        history.push_back(Sample(x, 0.0, 0.0));
        if (history.size() > al::kHistoryMax) history.pop_front();

        const bool verdict = al::VerdictWithHysteresis(
            al::RobustTranslDeviation(history),
            al::RobustRotDeviation(history),
            locked);
        if (verdict != locked) ++flipCount;
        locked = verdict;
    }

    // Under the old metric this would flap 10+ times. Under MAD we expect
    // ≤1 flip (initial enter); ideally zero unlocks across the 10% spike
    // rate.
    EXPECT_LE(flipCount, 1)
        << "Bimodal noise (10% spikes) should not flap the lock. flipCount="
        << flipCount;
    EXPECT_TRUE(locked) << "Should remain locked at end of run.";
}

// Same noise but starts unlocked. The detector should latch to locked
// after the initial 30-sample window earns evidence, then stay locked.
// The 2026-05-21 log showed 4 pending-LOCK events that all got cancelled;
// here we want 1 transition to locked and 0 unlocks.
TEST(AutoLockHysteresisTest, BimodalNoiseEnterLockOnceAndHold)
{
    std::deque<Eigen::AffineCompact3d> history;
    bool locked = false;
    int lockEvents = 0;
    int unlockEvents = 0;

    for (int i = 0; i < 60; ++i) {
        const bool isSpike = (i % 10 == 5);
        const double t = ((double)(i % 13) - 6.0) / 6.0;
        const double x = isSpike ? 0.012 : 0.0005 * t;
        history.push_back(Sample(x, 0.0, 0.0));
        if (history.size() > al::kHistoryMax) history.pop_front();

        if (history.size() < al::kSamplesNeeded) continue;

        const bool verdict = al::VerdictWithHysteresis(
            al::RobustTranslDeviation(history),
            al::RobustRotDeviation(history),
            locked);
        if (verdict && !locked) ++lockEvents;
        if (!verdict && locked) ++unlockEvents;
        locked = verdict;
    }

    EXPECT_GE(lockEvents, 1) << "Should enter locked at least once.";
    EXPECT_EQ(unlockEvents, 0)
        << "Should never unlock under bimodal 10%-spike noise. unlockEvents="
        << unlockEvents;
}

// --- Panic-level deviation predicate ----------------------------------------

// Catches clearly-broken rigid attachments that the pending-flip queue
// would otherwise hold under the stationary-HMD gate for up to 5 s. Sized
// against the 2026-05-22 field log: median locked MAD was 2.5 mm, worst-
// case outlier 669 mm. The 40 mm / 5 deg thresholds bracket those.

TEST(AutoLockPanicTest, TranslationBoundary)
{
    EXPECT_FALSE(al::IsPanicLevelDeviation(0.039, 0.0))
        << "39 mm is one mm below the panic floor -- should defer to the "
           "normal pending-flip path";
    EXPECT_TRUE(al::IsPanicLevelDeviation(0.040, 0.0))
        << "40 mm sits exactly on the panic floor -- should fire";
    EXPECT_TRUE(al::IsPanicLevelDeviation(0.041, 0.0));
}

TEST(AutoLockPanicTest, RotationBoundary)
{
    const double fourNineDeg = 4.9 * EIGEN_PI / 180.0;
    const double fiveDeg     = 5.0 * EIGEN_PI / 180.0;
    const double fiveOneDeg  = 5.1 * EIGEN_PI / 180.0;

    EXPECT_FALSE(al::IsPanicLevelDeviation(0.0, fourNineDeg));
    EXPECT_TRUE(al::IsPanicLevelDeviation(0.0, fiveDeg));
    EXPECT_TRUE(al::IsPanicLevelDeviation(0.0, fiveOneDeg));
}

TEST(AutoLockPanicTest, EitherAxisTriggers)
{
    // 50 mm translation + 10 deg rotation -- both panic-level.
    const double tenDeg = 10.0 * EIGEN_PI / 180.0;
    EXPECT_TRUE(al::IsPanicLevelDeviation(0.050, tenDeg));

    // The 2026-05-22 field outlier: 669 mm with no extreme rotation.
    EXPECT_TRUE(al::IsPanicLevelDeviation(0.669, 0.0));

    // Below panic on translation, above on rotation: still fires.
    EXPECT_TRUE(al::IsPanicLevelDeviation(0.010, tenDeg));
}

// Panic does not replace the hysteresis verdict -- the 8 mm leave
// threshold still releases locks normally, the queue and stationary gate
// still arbitrate timing. Panic is a strict superset trigger for the
// pathological tail. Pin the contract that callers must consult both.
TEST(AutoLockPanicTest, DoesNotSubsumeHysteresis)
{
    // 9 mm: past the 8 mm leave threshold but well under the 40 mm panic
    // floor. The hysteresis verdict (currently locked) says unlock; the
    // panic predicate says no. Callers route through the normal queue.
    EXPECT_FALSE(al::IsPanicLevelDeviation(0.009, 0.0));
    EXPECT_FALSE(al::VerdictWithHysteresis(0.009, 0.0, /*prevLocked=*/true));

    // 2 mm: below both. Locked stays locked.
    EXPECT_FALSE(al::IsPanicLevelDeviation(0.002, 0.0));
    EXPECT_TRUE(al::VerdictWithHysteresis(0.002, 0.0, /*prevLocked=*/true));
}

// --- Commit-gate decision ---------------------------------------------------

// Shared between the primary and extras detectors. Pin the three-way
// decision: stationary lets any flip through, unlock-timeout escapes
// sustained motion, reanchor-suppress blocks even when stationary, and
// lock-direction flips have no escape.

TEST(AutoLockCommitGateTest, StationaryCommitsImmediately)
{
    const auto d = al::EvaluateCommitGate(
        /*pendingFlipTo=*/true,
        /*hmdSpeedMps=*/0.02,        // below kStationaryHmdMps (0.05)
        /*now=*/100.0,
        /*reanchorSuppressUntil=*/0.0,
        /*pendingHeldSec=*/0.1);
    EXPECT_TRUE(d.commit);
    EXPECT_STREQ(d.mode, "stationary_gate");
}

TEST(AutoLockCommitGateTest, HmdMotionHoldsLockFlipForever)
{
    // Lock direction (pendingFlipTo=true) has no timeout escape. Even with
    // a huge held duration, sustained motion keeps the flip pending.
    const auto d = al::EvaluateCommitGate(
        /*pendingFlipTo=*/true,
        /*hmdSpeedMps=*/0.50,        // well above stationary
        /*now=*/1000.0,
        /*reanchorSuppressUntil=*/0.0,
        /*pendingHeldSec=*/30.0);    // way past kAutoLockUnlockMaxWaitSeconds
    EXPECT_FALSE(d.commit);
    EXPECT_STREQ(d.mode, "held");
}

TEST(AutoLockCommitGateTest, UnlockTimeoutEscapesMotion)
{
    const auto d = al::EvaluateCommitGate(
        /*pendingFlipTo=*/false,     // unlock direction
        /*hmdSpeedMps=*/0.50,        // moving
        /*now=*/1000.0,
        /*reanchorSuppressUntil=*/0.0,
        /*pendingHeldSec=*/al::kAutoLockUnlockMaxWaitSeconds);
    EXPECT_TRUE(d.commit);
    EXPECT_STREQ(d.mode, "unlock_timeout");
}

TEST(AutoLockCommitGateTest, UnlockBelowTimeoutStaysHeld)
{
    const double half = al::kAutoLockUnlockMaxWaitSeconds * 0.5;
    const auto d = al::EvaluateCommitGate(
        /*pendingFlipTo=*/false,
        /*hmdSpeedMps=*/0.50,
        /*now=*/1000.0,
        /*reanchorSuppressUntil=*/0.0,
        /*pendingHeldSec=*/half);
    EXPECT_FALSE(d.commit);
    EXPECT_STREQ(d.mode, "held");
}

TEST(AutoLockCommitGateTest, ReanchorSuppressBlocksEvenStationary)
{
    // Stationary + reanchor still active -> held. The reanchor window
    // exists to hide post-reanchor stddev spikes from the lock decision.
    const auto d = al::EvaluateCommitGate(
        /*pendingFlipTo=*/true,
        /*hmdSpeedMps=*/0.0,
        /*now=*/100.0,
        /*reanchorSuppressUntil=*/100.5,   // still suppressing
        /*pendingHeldSec=*/0.2);
    EXPECT_FALSE(d.commit);
    EXPECT_STREQ(d.mode, "held");
}

TEST(AutoLockCommitGateTest, UnlockTimeoutBeatsStationaryInTag)
{
    // When both conditions could commit, the tag should reflect the
    // timeout escape rather than the stationary gate. Matches the
    // committed_via shape of the existing diagnostic.
    const auto d = al::EvaluateCommitGate(
        /*pendingFlipTo=*/false,
        /*hmdSpeedMps=*/0.0,                // stationary
        /*now=*/1000.0,
        /*reanchorSuppressUntil=*/0.0,
        /*pendingHeldSec=*/al::kAutoLockUnlockMaxWaitSeconds + 1.0);
    EXPECT_TRUE(d.commit);
    EXPECT_STREQ(d.mode, "unlock_timeout");
}
