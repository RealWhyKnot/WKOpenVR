// Tests for the pure constant-velocity re-anchor ramp math in ReanchorRamp.h.

#include "ReanchorRamp.h"

#include <gtest/gtest.h>

#include <algorithm>

namespace rr = spacecal::reanchor;

namespace {
// 90 Hz frame time.
constexpr double kDt = 1.0 / 90.0;
constexpr double kMaxTransStep = rr::kReanchorSlewTransMps * kDt;
constexpr double kMaxRotStep = rr::kReanchorSlewRotRadps * kDt;
} // namespace

TEST(ReanchorRamp, Small_delta_reaches_in_one_frame)
{
	// A 0.1 mm correction is under the per-frame cap (~0.5 mm) -> full fraction.
	EXPECT_DOUBLE_EQ(rr::ComputeReanchorFraction(0.0001, 0.0, kMaxTransStep, kMaxRotStep), 1.0);
}

TEST(ReanchorRamp, At_cap_delta_is_full)
{
	// A delta exactly at the per-frame cap completes this frame.
	EXPECT_DOUBLE_EQ(rr::ComputeReanchorFraction(kMaxTransStep, 0.0, kMaxTransStep, kMaxRotStep), 1.0);
}

TEST(ReanchorRamp, Zero_delta_is_complete)
{
	EXPECT_DOUBLE_EQ(rr::ComputeReanchorFraction(0.0, 0.0, kMaxTransStep, kMaxRotStep), 1.0);
}

TEST(ReanchorRamp, Large_translation_is_capped)
{
	// 0.8 m flip: the per-frame step is the cap, so the fraction is tiny and the
	// world-space step equals the cap (constant velocity, not front-loaded).
	const double transFull = 0.8;
	const double frac = rr::ComputeReanchorFraction(transFull, 0.0, kMaxTransStep, kMaxRotStep);
	EXPECT_LT(frac, 1.0);
	EXPECT_NEAR(frac * transFull, kMaxTransStep, 1e-12);
}

TEST(ReanchorRamp, Rotation_can_bind)
{
	// Large rotation, negligible translation -> rotation cap limits the step.
	const double rotFull = 3.14159; // ~180 deg
	const double frac = rr::ComputeReanchorFraction(0.0, rotFull, kMaxTransStep, kMaxRotStep);
	EXPECT_LT(frac, 1.0);
	EXPECT_NEAR(frac * rotFull, kMaxRotStep, 1e-12);
}

TEST(ReanchorRamp, Binding_cap_is_the_smaller_fraction)
{
	// Both exceed their caps; the fraction is the min so neither axis exceeds its
	// per-frame budget.
	const double transFull = 0.8; // trans fraction ~ kMaxTransStep/0.8
	const double rotFull = 0.1;   // rot fraction ~ kMaxRotStep/0.1 (larger)
	const double frac = rr::ComputeReanchorFraction(transFull, rotFull, kMaxTransStep, kMaxRotStep);
	const double transFrac = kMaxTransStep / transFull;
	const double rotFrac = kMaxRotStep / rotFull;
	EXPECT_DOUBLE_EQ(frac, std::min(transFrac, rotFrac));
}

TEST(ReanchorRamp, Settles_08m_flip_in_about_18s)
{
	// Iterate the ramp on a pure 0.8 m translation and count frames to converge.
	double remaining = 0.8;
	int frames = 0;
	while (remaining > 1e-4 && frames < 100000) {
		const double frac = rr::ComputeReanchorFraction(remaining, 0.0, kMaxTransStep, kMaxRotStep);
		remaining -= frac * remaining;
		++frames;
	}
	const double seconds = frames * kDt;
	// ~0.8 / 0.045 = ~17.8 s; allow a small band for the final sub-cap frame.
	EXPECT_GE(seconds, 16.0);
	EXPECT_LE(seconds, 20.0);
}

static_assert(rr::kReanchorSlewTransMps == 0.045,
              "kReanchorSlewTransMps changed -- revisit the ramp-duration tests with it");
static_assert(rr::kReanchorSlewRotRadps == 0.0785,
              "kReanchorSlewRotRadps changed -- revisit the ramp-duration tests with it");
