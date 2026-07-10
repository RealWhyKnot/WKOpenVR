// Tests for the pure recovery-action policy in RecoveryPolicy.h.
//
// The policy decides, without a live runtime, whether a relocalization or a
// failed warm-restart should Hold the calibration, gently ReanchorToProfile, or
// (last resort) DestructiveClear. The central guarantee is that a destructive
// clear never fires while a saved profile or a corroborating witness exists.

#include "RecoveryPolicy.h"

#include <gtest/gtest.h>

namespace rp = spacecal::recovery;

// ---------------------------------------------------------------------------
// DestructiveClearAllowed: requires ALL of (witness invalid, no profile, wr failed)
// ---------------------------------------------------------------------------

TEST(RecoveryPolicy, Destructive_requires_all_three)
{
	EXPECT_TRUE(rp::DestructiveClearAllowed(/*witnessInvalid=*/true, /*hasSavedProfile=*/false,
	                                        /*warmRestartFailed=*/true));
	// Any single guard flips it off.
	EXPECT_FALSE(rp::DestructiveClearAllowed(false, false, true)); // witness valid
	EXPECT_FALSE(rp::DestructiveClearAllowed(true, true, true));   // profile exists
	EXPECT_FALSE(rp::DestructiveClearAllowed(true, false, false)); // wr not failed
}

// ---------------------------------------------------------------------------
// ChooseRelocRecoveryAction
// ---------------------------------------------------------------------------

TEST(RecoveryPolicy, Reloc_witness_corroborates_real_motion_holds)
{
	// Witness valid and moved with the HMD -> genuine motion, calibration still
	// valid -> Hold (the dominant false-positive destructive clear).
	EXPECT_EQ(rp::ChooseRelocRecoveryAction(/*witnessValid=*/true, /*witnessSaysHeadMoved=*/true,
	                                        /*hasSavedProfile=*/true, /*warmRestartFailed=*/false),
	          rp::RecoveryAction::Hold);
	// Holds even with no profile -- real motion needs no recovery.
	EXPECT_EQ(rp::ChooseRelocRecoveryAction(true, true, false, true), rp::RecoveryAction::Hold);
}

TEST(RecoveryPolicy, Reloc_uncorroborated_with_profile_reanchors)
{
	// Witness invalid (can't corroborate) but a profile exists -> gentle re-anchor.
	EXPECT_EQ(rp::ChooseRelocRecoveryAction(/*witnessValid=*/false, /*witnessSaysHeadMoved=*/false,
	                                        /*hasSavedProfile=*/true, /*warmRestartFailed=*/true),
	          rp::RecoveryAction::ReanchorToProfile);
}

TEST(RecoveryPolicy, Reloc_no_profile_no_witness_wr_failed_destroys)
{
	// True last resort: no witness, no profile, warm-restart already failed.
	EXPECT_EQ(rp::ChooseRelocRecoveryAction(/*witnessValid=*/false, /*witnessSaysHeadMoved=*/false,
	                                        /*hasSavedProfile=*/false, /*warmRestartFailed=*/true),
	          rp::RecoveryAction::DestructiveClear);
}

TEST(RecoveryPolicy, Reloc_no_profile_but_wr_not_failed_holds)
{
	// No profile and witness invalid, but warm-restart hasn't failed -> not yet
	// last resort -> Hold rather than destroy.
	EXPECT_EQ(rp::ChooseRelocRecoveryAction(false, false, false, false), rp::RecoveryAction::Hold);
}

// ---------------------------------------------------------------------------
// ChooseWarmRestartFailureAction
// ---------------------------------------------------------------------------

TEST(RecoveryPolicy, WarmRestart_witness_present_reanchors_until_cap)
{
	// Witness present + retries remaining -> re-anchor (the take-off/put-back-on fix).
	EXPECT_EQ(rp::ChooseWarmRestartFailureAction(/*frameReanchorWitnessed=*/false, /*witnessPresent=*/true,
	                                             /*hasSavedProfile=*/true, /*reanchorCount=*/0,
	                                             rp::kWarmRestartMaxReanchors),
	          rp::RecoveryAction::ReanchorToProfile);
	EXPECT_EQ(rp::ChooseWarmRestartFailureAction(false, true, true, rp::kWarmRestartMaxReanchors - 1,
	                                             rp::kWarmRestartMaxReanchors),
	          rp::RecoveryAction::ReanchorToProfile);
}

TEST(RecoveryPolicy, WarmRestart_retries_exhausted_holds)
{
	// Cap reached: a profile still exists, so hold rather than destroy.
	EXPECT_EQ(rp::ChooseWarmRestartFailureAction(false, true, true, rp::kWarmRestartMaxReanchors,
	                                             rp::kWarmRestartMaxReanchors),
	          rp::RecoveryAction::Hold);
}

TEST(RecoveryPolicy, WarmRestart_no_witness_with_profile_holds)
{
	// No witness but a saved profile exists -> hold (destructive needs no profile).
	EXPECT_EQ(rp::ChooseWarmRestartFailureAction(/*frameReanchorWitnessed=*/false, /*witnessPresent=*/false,
	                                             /*hasSavedProfile=*/true, /*reanchorCount=*/0,
	                                             rp::kWarmRestartMaxReanchors),
	          rp::RecoveryAction::Hold);
}

TEST(RecoveryPolicy, WarmRestart_no_witness_no_profile_destroys)
{
	// No witness and no profile, warm-restart failed -> last resort destructive.
	EXPECT_EQ(rp::ChooseWarmRestartFailureAction(false, false, false, 0, rp::kWarmRestartMaxReanchors),
	          rp::RecoveryAction::DestructiveClear);
}

TEST(RecoveryPolicy, WarmRestart_witnessed_frame_move_holds_resolved_frame)
{
	// The episode began with a witnessed world-frame move (corroborated SLAM
	// snap, reloc re-anchor, or an eviction-length away gap). The saved
	// profile describes the OLD frame; re-applying it re-opens the step-gate
	// bypass and produced two teleports per validation-failure cycle in
	// field logs. Hold wins regardless of witness presence or retry budget.
	EXPECT_EQ(rp::ChooseWarmRestartFailureAction(/*frameReanchorWitnessed=*/true, /*witnessPresent=*/true,
	                                             /*hasSavedProfile=*/true, /*reanchorCount=*/0,
	                                             rp::kWarmRestartMaxReanchors),
	          rp::RecoveryAction::Hold);
	// Even the destructive last-resort shape is overridden by provenance:
	// the frame moved, so the solver's answer is the trustworthy side.
	EXPECT_EQ(rp::ChooseWarmRestartFailureAction(true, false, false, 0, rp::kWarmRestartMaxReanchors),
	          rp::RecoveryAction::Hold);
}

static_assert(rp::kWarmRestartMaxReanchors == 2,
              "kWarmRestartMaxReanchors changed -- update the plan spec before tuning");
