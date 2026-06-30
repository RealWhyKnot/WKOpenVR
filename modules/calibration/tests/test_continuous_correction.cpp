// Tests for the pure continuous sub-30 cm witness-correction math.

#include "ContinuousCorrection.h"

#include <gtest/gtest.h>

namespace cc = spacecal::cont_correction;

namespace {
constexpr double kTickDt = 1.0 / 3.5; // continuous-cal tick cadence (~0.286 s)
constexpr double kBudget = cc::kCorrectionSlewMps * kTickDt;
} // namespace

TEST(ContinuousCorrection, Inside_deadband_no_step)
{
	// 2 mm drift, default 3 mm floor -> no correction.
	EXPECT_DOUBLE_EQ(cc::CorrectionStepM(0.002, 0.0, kTickDt), 0.0);
}

TEST(ContinuousCorrection, Mad_floor_widens_deadband)
{
	// 8 mm drift but mad_floor is 10 mm -> still inside the band.
	EXPECT_DOUBLE_EQ(cc::CorrectionStepM(0.008, 0.010, kTickDt), 0.0);
}

TEST(ContinuousCorrection, Large_drift_left_to_recovery)
{
	// 35 cm drift exceeds kMaxCorrectionM -> this slow loop yields; recovery owns it.
	EXPECT_DOUBLE_EQ(cc::CorrectionStepM(0.35, 0.003, kTickDt), 0.0);
}

TEST(ContinuousCorrection, In_band_is_slew_limited)
{
	// 6 cm drift: remaining over the band far exceeds the per-tick budget -> step
	// is the budget (constant-velocity convergence).
	EXPECT_DOUBLE_EQ(cc::CorrectionStepM(0.06, 0.003, kTickDt), kBudget);
}

TEST(ContinuousCorrection, Near_band_edge_steps_only_the_remainder)
{
	// Just over the band by less than one budget -> step only the remainder so we
	// converge to the band edge, not past it.
	const double deadband = cc::kDeadbandFloorM;      // 3 mm
	const double errorM = deadband + (kBudget * 0.5); // half a budget over the band
	EXPECT_DOUBLE_EQ(cc::CorrectionStepM(errorM, 0.0, kTickDt), kBudget * 0.5);
}

TEST(ContinuousCorrection, Zero_dt_no_step)
{
	EXPECT_DOUBLE_EQ(cc::CorrectionStepM(0.06, 0.003, 0.0), 0.0);
}

static_assert(cc::kCorrectionSlewMps >= 0.002 && cc::kCorrectionSlewMps <= 0.005,
              "kCorrectionSlewMps left the 2-5 mm/s band -- update the plan spec before tuning");
static_assert(cc::kMaxCorrectionM == 0.30, "kMaxCorrectionM changed -- update the plan spec before tuning");
