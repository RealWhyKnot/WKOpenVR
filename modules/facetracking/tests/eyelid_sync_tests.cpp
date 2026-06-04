// Unit tests for facetracking::EyelidSync.
//
// Coverage:
//   - Strength 0 leaves the frame untouched.
//   - Strength 100 on a symmetric input keeps both eyes at the same value.
//   - Strength 100 on an asymmetric low-confidence input narrows the gap.
//   - preserve_winks=true with sustained large asymmetry and high confidence
//     bypasses sync (a deliberate wink survives).
//   - Single-frame asymmetric spike does not pop both eyes (80 ms smoothing).

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

TEST(EyelidSync, AsymmetricLowConfidenceConverges)
{
	facetracking::EyelidSync sync;

	// Asymmetric input below the wink threshold and with low confidence so
	// the wink detector never trips, even with preserve_winks=true.  After
	// enough frames the synced output should pull toward the confidence-
	// weighted mean.
	protocol::FaceTrackingFrameBody f{};
	for (int i = 0; i < 200; ++i) {
		f = MakeFrame(0.2f, 0.4f, 0.3f, 0.3f);
		sync.Apply(f, 100, true);
	}

	// Both eyes should be approximately equal and inside the original [0.2, 0.4]
	// range.  Tolerance is wide because the temporal smoothing is asymmetric
	// and the confidence-weighted mean depends on both confidences (equal here).
	EXPECT_NEAR(f.eye_openness_l, f.eye_openness_r, 0.05f);
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
	// should drive both eyes toward the confidence-weighted mean and the
	// asymmetry should collapse.
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

	// Inject a single-frame asymmetric glitch.  With the 80 ms temporal
	// smoothing on top of confidence blending, the smoothed output should not
	// pop -- it should remain near the prior symmetric value.
	f = MakeFrame(0.7f, 0.1f);
	sync.Apply(f, 100, false);

	EXPECT_NEAR(f.eye_openness_l, 0.7f, 0.2f);
	EXPECT_NEAR(f.eye_openness_r, 0.7f, 0.2f);
}
