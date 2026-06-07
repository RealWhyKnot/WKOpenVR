// Unit tests for facetracking::EyelidSync.
//
// Coverage:
//   - Strength 0 leaves the frame untouched.
//   - Strength 100 on a symmetric input keeps both eyes at the same value.
//   - Strength 100 can target either the most closed or most open eye.
//   - preserve_winks=true with sustained large asymmetry and high confidence
//     bypasses sync (a deliberate wink survives).
//   - Single-frame asymmetric spike does not pop both eyes.

#include "EyelidSync.h"
#include "Protocol.h"

#include <gtest/gtest.h>

#include <chrono>
#include <thread>

namespace {

protocol::FaceTrackingFrameBody MakeFrame(float lid_l, float lid_r, float conf_l = 1.0f, float conf_r = 1.0f)
{
	protocol::FaceTrackingFrameBody f{};
	f.eye_openness_l = lid_l;
	f.eye_openness_r = lid_r;
	f.eye_confidence_l = conf_l;
	f.eye_confidence_r = conf_r;
	f.flags = 0x1; // eye valid
	return f;
}

} // namespace

TEST(EyelidSync, StrengthZeroIsNoOp)
{
	facetracking::EyelidSync sync;
	protocol::FaceTrackingFrameBody f = MakeFrame(0.3f, 0.9f);
	sync.Apply(f, 0, true);

	EXPECT_FLOAT_EQ(f.eye_openness_l, 0.3f);
	EXPECT_FLOAT_EQ(f.eye_openness_r, 0.9f);
}

TEST(EyelidSync, SymmetricInputUnchanged)
{
	facetracking::EyelidSync sync;
	protocol::FaceTrackingFrameBody f = MakeFrame(0.5f, 0.5f);

	// Run many frames so the temporal smoother converges; sync should
	// converge to the symmetric value.
	for (int i = 0; i < 50; ++i) {
		f = MakeFrame(0.5f, 0.5f);
		sync.Apply(f, 100, false);
	}
	EXPECT_NEAR(f.eye_openness_l, 0.5f, 0.01f);
	EXPECT_NEAR(f.eye_openness_r, 0.5f, 0.01f);
}

TEST(EyelidSync, MostClosedModeUsesLowerOpenness)
{
	facetracking::EyelidSync sync;
	protocol::FaceTrackingFrameBody f{};

	for (int i = 0; i < 200; ++i) {
		f = MakeFrame(0.9f, 0.1f, 0.3f, 0.3f);
		sync.Apply(f, 100, false, protocol::FACETRACKING_EYELID_SYNC_MOST_CLOSED);
	}

	EXPECT_NEAR(f.eye_openness_l, 0.1f, 0.02f);
	EXPECT_NEAR(f.eye_openness_r, 0.1f, 0.02f);
}

TEST(EyelidSync, MostOpenModeUsesHigherOpenness)
{
	facetracking::EyelidSync sync;
	protocol::FaceTrackingFrameBody f{};

	for (int i = 0; i < 200; ++i) {
		f = MakeFrame(0.9f, 0.1f, 0.3f, 0.3f);
		sync.Apply(f, 100, false, protocol::FACETRACKING_EYELID_SYNC_MOST_OPEN);
	}

	EXPECT_NEAR(f.eye_openness_l, 0.9f, 0.02f);
	EXPECT_NEAR(f.eye_openness_r, 0.9f, 0.02f);
}

TEST(EyelidSync, DefaultModeUsesMoreClosedEye)
{
	facetracking::EyelidSync sync;

	protocol::FaceTrackingFrameBody f{};
	for (int i = 0; i < 200; ++i) {
		f = MakeFrame(0.2f, 0.4f, 0.3f, 0.3f);
		sync.Apply(f, 100, true);
	}

	EXPECT_NEAR(f.eye_openness_l, 0.2f, 0.05f);
	EXPECT_NEAR(f.eye_openness_r, 0.2f, 0.05f);
}

TEST(EyelidSync, WinkSurvivesPreserveOn)
{
	facetracking::EyelidSync sync;

	// Sustained large asymmetry (|0.9 - 0.1| = 0.8 > 0.45) with high
	// confidence on both eyes.  After 120 ms of dwell, preserve_winks should
	// bypass sync and the asymmetry should persist.
	protocol::FaceTrackingFrameBody f{};
	for (int i = 0; i < 60; ++i) {
		f = MakeFrame(0.9f, 0.1f, 0.95f, 0.95f);
		sync.Apply(f, 100, true);
		std::this_thread::sleep_for(std::chrono::milliseconds(3));
	}

	// After the 120 ms dwell window has elapsed, the wink should be preserved:
	// |lid_l - lid_r| stays close to its original asymmetry rather than being
	// forced together.
	const float diff = std::abs(f.eye_openness_l - f.eye_openness_r);
	EXPECT_GT(diff, 0.3f);
}

TEST(EyelidSync, WinkCollapsedWhenPreserveOff)
{
	facetracking::EyelidSync sync;

	// Same large asymmetry as above but with preserve_winks=false.  The sync
	// should drive both eyes toward the selected target and collapse the
	// asymmetry.
	protocol::FaceTrackingFrameBody f{};
	for (int i = 0; i < 200; ++i) {
		f = MakeFrame(0.9f, 0.1f, 0.95f, 0.95f);
		sync.Apply(f, 100, false);
	}

	const float diff = std::abs(f.eye_openness_l - f.eye_openness_r);
	EXPECT_LT(diff, 0.1f);
}

TEST(EyelidSync, SingleFrameGlitchSmoothed)
{
	facetracking::EyelidSync sync;

	// Settle on a symmetric baseline.
	protocol::FaceTrackingFrameBody f{};
	for (int i = 0; i < 50; ++i) {
		f = MakeFrame(0.7f, 0.7f);
		sync.Apply(f, 100, false);
	}

	// Inject a single-frame asymmetric glitch. The smoothed output should not
	// pop all the way to the one-frame target.
	f = MakeFrame(0.7f, 0.1f);
	sync.Apply(f, 100, false);

	EXPECT_GT(f.eye_openness_l, 0.25f);
	EXPECT_GT(f.eye_openness_r, 0.25f);
	EXPECT_LT(f.eye_openness_l, 0.7f);
	EXPECT_LT(f.eye_openness_r, 0.7f);
}

TEST(EyelidSync, SmootherFastCloseSlowOpen)
{
	// A blink (closing: target below current) must track fast so it reaches the
	// avatar, while reopening is gentler/cosmetic. At a ~16 ms (60 Hz) frame the
	// close should cover well over half the gap; the open well under half.
	const double dt = 0.016;
	const float closed = facetracking::EyelidSync::SmoothToward(1.0f, 0.0f, dt);
	const float opened = facetracking::EyelidSync::SmoothToward(0.0f, 1.0f, dt);
	EXPECT_LT(closed, 0.5f); // closing covers >50% of the gap (blink survives)
	EXPECT_GT(opened, 0.0f);
	EXPECT_LT(opened, 0.5f);          // reopening covers <50% of the gap
	EXPECT_GT(1.0f - closed, opened); // close moves a larger fraction than open
	// dt <= 0 uses a nominal step; long gaps snap to the target.
	EXPECT_GT(facetracking::EyelidSync::SmoothToward(0.2f, 0.8f, 0.0), 0.2f);
	EXPECT_LT(facetracking::EyelidSync::SmoothToward(0.2f, 0.8f, 0.0), 0.8f);
	EXPECT_FLOAT_EQ(facetracking::EyelidSync::SmoothToward(0.2f, 0.8f, 1.0), 0.8f);
}
