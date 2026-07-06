// Tests for the snap-suppression pure helpers in SnapSuppression.h.
//
// Covers the snap-suppression helper cases:
//   1. Speed_gate_max_of_hmd_and_tracker
//   2. Speed_gate_falls_back_when_tracker_invalid
//   3. Jump_detector_classifies_slam_snap
//   4. Jump_detector_real_jump_passes_through
//   5. Geometry_shift_coherence_uses_tracker_displacement
//   6. Off_mode_unchanged_recovery

#include "SnapSuppression.h"

#include <gtest/gtest.h>

namespace ss = spacecal::snap_suppression;

// ---------------------------------------------------------------------------
// Site 1: EffectiveSpeedMps (AUTO Lock stationary gate)
// ---------------------------------------------------------------------------

TEST(SnapSuppression, Speed_gate_max_of_hmd_and_tracker)
{
	// Corroborate mode + valid tracker reporting motion: effective speed is the
	// tracker speed even when the HMD is still.
	const double hmdSpeed = 0.01;     // ~stationary
	const double trackerSpeed = 0.50; // clearly moving
	const double result = ss::EffectiveSpeedMps(HeadMountMode::Corroborate, hmdSpeed, trackerSpeed);
	EXPECT_DOUBLE_EQ(result, trackerSpeed);
}

TEST(SnapSuppression, Speed_gate_hmd_dominates_when_higher)
{
	// HMD is faster: effective speed is the HMD speed.
	const double hmdSpeed = 0.80;
	const double trackerSpeed = 0.10;
	const double result = ss::EffectiveSpeedMps(HeadMountMode::Corroborate, hmdSpeed, trackerSpeed);
	EXPECT_DOUBLE_EQ(result, hmdSpeed);
}

TEST(SnapSuppression, Speed_gate_falls_back_when_tracker_invalid)
{
	// Tracker invalid (sentinel -1.0): fall back to HMD speed.
	const double hmdSpeed = 0.20;
	const double trackerInvalid = -1.0;
	const double result = ss::EffectiveSpeedMps(HeadMountMode::Corroborate, hmdSpeed, trackerInvalid);
	EXPECT_DOUBLE_EQ(result, hmdSpeed);
}

TEST(SnapSuppression, Speed_gate_off_mode_ignores_tracker)
{
	// Off mode: always return HMD speed regardless of tracker value.
	const double hmdSpeed = 0.05;
	const double trackerSpeed = 0.95;
	EXPECT_DOUBLE_EQ(ss::EffectiveSpeedMps(HeadMountMode::Off, hmdSpeed, trackerSpeed), hmdSpeed);
}

TEST(SnapSuppression, Speed_gate_autopaired_mode_ignores_tracker)
{
	// AutoPaired does not imply Corroborate: tracker not consulted.
	const double hmdSpeed = 0.05;
	const double trackerSpeed = 0.95;
	EXPECT_DOUBLE_EQ(ss::EffectiveSpeedMps(HeadMountMode::AutoPaired, hmdSpeed, trackerSpeed), hmdSpeed);
}

TEST(SnapSuppression, Speed_gate_driversynth_uses_tracker)
{
	// DriverSynth implies Corroborate: tracker is consulted.
	const double hmdSpeed = 0.01;
	const double trackerSpeed = 0.60;
	EXPECT_DOUBLE_EQ(ss::EffectiveSpeedMps(HeadMountMode::DriverSynth, hmdSpeed, trackerSpeed), trackerSpeed);
}

// ---------------------------------------------------------------------------
// Site 504: IsJumpClassifiedAsSnap (30 cm jump detector)
// ---------------------------------------------------------------------------

TEST(SnapSuppression, Jump_detector_classifies_slam_snap)
{
	// HMD > 30 cm, tracker < 2 cm, mode >= Corroborate: SLAM snap.
	EXPECT_TRUE(ss::IsJumpClassifiedAsSnap(HeadMountMode::Corroborate,
	                                       0.35,   // hmd: 35 cm
	                                       0.01)); // tracker: 1 cm
}

TEST(SnapSuppression, Jump_detector_real_jump_passes_through)
{
	// Both HMD and tracker exceed their respective thresholds: genuine jump.
	EXPECT_FALSE(ss::IsJumpClassifiedAsSnap(HeadMountMode::Corroborate,
	                                        0.50,   // hmd: 50 cm
	                                        0.30)); // tracker: 30 cm (well above 2 cm)
}

TEST(SnapSuppression, Jump_detector_small_hmd_jump_not_classified)
{
	// HMD below the 30 cm threshold: not a qualifying snap event.
	EXPECT_FALSE(ss::IsJumpClassifiedAsSnap(HeadMountMode::Corroborate,
	                                        0.10, // hmd: 10 cm -- below kSnapHmdJumpM
	                                        0.005));
}

TEST(SnapSuppression, Jump_detector_tracker_invalid_no_snap)
{
	// Tracker invalid (sentinel -1.0): cannot corroborate, returns false.
	EXPECT_FALSE(ss::IsJumpClassifiedAsSnap(HeadMountMode::Corroborate, 0.40, -1.0));
}

TEST(SnapSuppression, Jump_detector_off_mode_no_snap)
{
	// Off mode: never classifies as snap even with obvious signals.
	EXPECT_FALSE(ss::IsJumpClassifiedAsSnap(HeadMountMode::Off, 0.50, 0.001));
}

TEST(SnapSuppression, Jump_detector_autopaired_no_snap)
{
	// AutoPaired does not enable corroboration.
	EXPECT_FALSE(ss::IsJumpClassifiedAsSnap(HeadMountMode::AutoPaired, 0.50, 0.001));
}

// Exact-boundary cases (thresholds are non-strict inequalities in the
// required direction per the plan).
TEST(SnapSuppression, Jump_detector_exactly_at_hmd_threshold)
{
	// hmdDelta == kSnapHmdJumpM: should fire (>= threshold).
	EXPECT_TRUE(ss::IsJumpClassifiedAsSnap(HeadMountMode::Corroborate,
	                                       ss::kSnapHmdJumpM, // exactly 30 cm
	                                       0.005));
}

TEST(SnapSuppression, Jump_detector_tracker_exactly_at_max_disp)
{
	// trackerDelta == kSnapTrackerMaxDispM: NOT a snap (strict <).
	EXPECT_FALSE(ss::IsJumpClassifiedAsSnap(HeadMountMode::Corroborate, 0.40,
	                                        ss::kSnapTrackerMaxDispM)); // exactly 2 cm -- not < threshold
}

// ---------------------------------------------------------------------------
// Site 708: ShouldUseTrackerDisplacement (geometry-shift coherence source)
// ---------------------------------------------------------------------------

TEST(SnapSuppression, Geometry_shift_coherence_uses_tracker_displacement)
{
	// Corroborate + valid tracker reading: use tracker displacement.
	EXPECT_TRUE(ss::ShouldUseTrackerDisplacement(HeadMountMode::Corroborate, 0.015));
}

TEST(SnapSuppression, Geometry_shift_off_mode_uses_hmd_imu)
{
	// Off mode: always use velocity-integrated HMD estimate.
	EXPECT_FALSE(ss::ShouldUseTrackerDisplacement(HeadMountMode::Off, 0.015));
}

TEST(SnapSuppression, Geometry_shift_tracker_invalid_uses_hmd_imu)
{
	// Tracker invalid (sentinel): fall back to HMD estimate.
	EXPECT_FALSE(ss::ShouldUseTrackerDisplacement(HeadMountMode::Corroborate, -1.0));
}

TEST(SnapSuppression, Geometry_shift_zero_tracker_displacement_accepted)
{
	// Zero is a valid tracker displacement (perfectly still).
	EXPECT_TRUE(ss::ShouldUseTrackerDisplacement(HeadMountMode::Corroborate, 0.0));
}

// ---------------------------------------------------------------------------
// Off_mode_unchanged_recovery: verify every function returns the "unchanged"
// value when mode == Off, regardless of other inputs.
// ---------------------------------------------------------------------------

TEST(SnapSuppression, Off_mode_unchanged_recovery)
{
	const HeadMountMode off = HeadMountMode::Off;

	// Speed gate: returns HMD speed unchanged.
	EXPECT_DOUBLE_EQ(ss::EffectiveSpeedMps(off, 0.3, 0.9), 0.3);

	// Jump detector: never classifies as snap.
	EXPECT_FALSE(ss::IsJumpClassifiedAsSnap(off, 1.0, 0.001));

	// Geometry-shift source: always HMD-IMU.
	EXPECT_FALSE(ss::ShouldUseTrackerDisplacement(off, 0.0));
}

// ---------------------------------------------------------------------------
// EffectiveHeadMountMode: witness presence promotes Off -> Corroborate so the
// witness works in Continuous/Manual styles (which set config mode = Off).
// ---------------------------------------------------------------------------

TEST(SnapSuppression, EffectiveMode_promotes_off_when_witness_present)
{
	// Continuous/Manual set config mode Off; a bound witness promotes to Corroborate.
	EXPECT_EQ(ss::EffectiveHeadMountMode(HeadMountMode::Off, /*witnessPresent=*/true), HeadMountMode::Corroborate);
}

TEST(SnapSuppression, EffectiveMode_promotes_autopaired_when_witness_present)
{
	// AutoPaired (1) is below Corroborate (2): promoted when a witness is present.
	EXPECT_EQ(ss::EffectiveHeadMountMode(HeadMountMode::AutoPaired, true), HeadMountMode::Corroborate);
}

TEST(SnapSuppression, EffectiveMode_preserves_higher_mode)
{
	// DriverSynth already implies Corroborate: never downgraded.
	EXPECT_EQ(ss::EffectiveHeadMountMode(HeadMountMode::DriverSynth, true), HeadMountMode::DriverSynth);
	EXPECT_EQ(ss::EffectiveHeadMountMode(HeadMountMode::Corroborate, true), HeadMountMode::Corroborate);
}

TEST(SnapSuppression, EffectiveMode_unchanged_when_no_witness)
{
	// No witness puck: nothing changes, so non-witness setups behave as before.
	EXPECT_EQ(ss::EffectiveHeadMountMode(HeadMountMode::Off, /*witnessPresent=*/false), HeadMountMode::Off);
	EXPECT_EQ(ss::EffectiveHeadMountMode(HeadMountMode::AutoPaired, false), HeadMountMode::AutoPaired);
	EXPECT_EQ(ss::EffectiveHeadMountMode(HeadMountMode::DriverSynth, false), HeadMountMode::DriverSynth);
}

TEST(SnapSuppression, EffectiveMode_enables_snap_classification_in_continuous)
{
	// The core fix: in Continuous (config mode Off) with a witness puck bound,
	// a corroborated universe flip now classifies as a snap instead of taking
	// the destructive path. Without promotion this returned false.
	const HeadMountMode configOff = HeadMountMode::Off;               // Continuous preset
	EXPECT_FALSE(ss::IsJumpClassifiedAsSnap(configOff, 0.85, 0.005)); // old behavior: never corroborates

	const HeadMountMode eff = ss::EffectiveHeadMountMode(configOff, /*witnessPresent=*/true);
	EXPECT_TRUE(ss::IsJumpClassifiedAsSnap(eff, 0.85, 0.005)); // 85 cm HMD flip, 5 mm puck move -> snap
}

// ---------------------------------------------------------------------------
// Pinned threshold constants (per plan spec -- must not drift without review)
// ---------------------------------------------------------------------------

static_assert(ss::kSnapHmdJumpM == 0.30, "kSnapHmdJumpM changed from 0.30 -- update the plan spec before tuning");
static_assert(ss::kSnapTrackerMaxDispM == 0.02,
              "kSnapTrackerMaxDispM changed from 0.02 -- update the plan spec before tuning");
