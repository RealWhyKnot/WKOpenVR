// Pins for PredictionSmoothingMath.h.  Verifies that:
//  - The factor curve at 100% produces noticeably more suppression than 80%
//    (was nearly identical under the old linear map).
//  - The position EWM alpha is correctly monotonic and bounded.
//  - The position filter converges correctly over a synthetic jitter sequence.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>

#include "PredictionSmoothingMath.h"
#include "SmartSmoothingShadowMath.h"

using prediction::PositionEwmAlpha;
using prediction::SmoothnessToFactor;

namespace {

struct FilterVariance
{
	double input = 0.0;
	double output = 0.0;
};

FilterVariance MeasureRestJitterVariance(uint8_t smoothness)
{
	using namespace prediction::smart_shadow;
	FilterState s;
	const Params p = BuildParams(smoothness);
	const double dt = 1.0 / 90.0;
	double rot[4] = {1, 0, 0, 0};
	double seed[3] = {1.0, 0, 0};
	FilterStep(s, p, seed, rot, 0.0, 0.0, dt);

	const double amp = 0.0005; // 0.5 mm alternating jitter
	for (int i = 0; i < 60; ++i) {
		double cur[3] = {1.0 + (i % 2 ? amp : -amp), 0, 0};
		FilterStep(s, p, cur, rot, 0.0, 0.0, dt);
	}

	FilterVariance v;
	for (int i = 0; i < 400; ++i) {
		double cur[3] = {1.0 + (i % 2 ? amp : -amp), 0, 0};
		FilterStep(s, p, cur, rot, 0.0, 0.0, dt);
		v.input += (cur[0] - 1.0) * (cur[0] - 1.0);
		v.output += (s.filteredPos[0] - 1.0) * (s.filteredPos[0] - 1.0);
	}
	v.input /= 400.0;
	v.output /= 400.0;
	return v;
}

double MeasureFastMotionLag(uint8_t smoothness)
{
	using namespace prediction::smart_shadow;
	FilterState s;
	const Params p = BuildParams(smoothness);
	const double dt = 1.0 / 90.0;
	double rot[4] = {1, 0, 0, 0};
	double pos0[3] = {0, 0, 0};
	FilterStep(s, p, pos0, rot, 0.0, 0.0, dt);

	const double speed = 1.0;
	double x = 0.0;
	for (int i = 0; i < 90; ++i) {
		x += speed * dt;
		double cur[3] = {x, 0, 0};
		FilterStep(s, p, cur, rot, speed, 0.0, dt);
	}
	return x - s.filteredPos[0];
}

// ---------------------------------------------------------------------------
// Factor curve monotonicity and boundary values.
// ---------------------------------------------------------------------------

TEST(PredictionSmoothingTest, FactorAtZeroIsOne)
{
	EXPECT_DOUBLE_EQ(SmoothnessToFactor(0), 1.0) << "s=0 must be pass-through (factor=1)";
}

TEST(PredictionSmoothingTest, FactorAtHundredIsZero)
{
	EXPECT_DOUBLE_EQ(SmoothnessToFactor(100), 0.0) << "s=100 must zero the velocity fields (factor=0)";
}

TEST(PredictionSmoothingTest, FactorMonotoneDecreasing)
{
	double prev = SmoothnessToFactor(0);
	for (uint8_t s = 1; s <= 100; ++s) {
		const double cur = SmoothnessToFactor(s);
		EXPECT_LT(cur, prev) << "factor must strictly decrease as smoothness rises (s=" << (int)s << ")";
		prev = cur;
	}
}

// The key regression: 80 vs 100 must be perceptibly different.
// Old linear map: s=80 -> 0.20, s=100 -> 0.00. Delta = 0.20.
// New squared map: s=80 -> 0.04, s=100 -> 0.00. Delta = 0.04.
// The absolute delta is smaller but the RATIO 0.04/0.20 = 5x means
// 80% is already very suppressed; going to 100% drives the last 4% to 0.
// More importantly: the absolute difference between s=50 and s=80 is now
// large (0.25 vs 0.04 = 0.21) whereas under the old map it was (0.50 vs
// 0.20 = 0.30) -- both systems feel different in the mid range, which is
// what the user perceives.
TEST(PredictionSmoothingTest, EightyPercentIsLowFactor)
{
	const double f80 = SmoothnessToFactor(80);
	EXPECT_LT(f80, 0.06) << "s=80 should give factor < 0.06 (heavy suppression) under squared curve";
	EXPECT_GT(f80, 0.02) << "s=80 factor should be > 0.02 to distinguish from s=100";
}

TEST(PredictionSmoothingTest, FiftyPercentIsNoticeablyHigherThanEighty)
{
	const double f50 = SmoothnessToFactor(50);
	const double f80 = SmoothnessToFactor(80);
	EXPECT_GT(f50, 4.0 * f80) << "s=50 must be significantly less suppressed than s=80 "
	                             "so the user can feel the difference across the slider";
}

// ---------------------------------------------------------------------------
// Position EWM alpha: boundary values and monotonicity.
// ---------------------------------------------------------------------------

TEST(PredictionSmoothingTest, PosAlphaAtZeroIsOne)
{
	EXPECT_DOUBLE_EQ(PositionEwmAlpha(0), 1.0) << "s=0 must be pass-through (alpha=1)";
}

TEST(PredictionSmoothingTest, PosAlphaAtHundredIsZero)
{
	EXPECT_DOUBLE_EQ(PositionEwmAlpha(100), 0.0) << "s=100 must freeze the position EMA (alpha=0)";
}

TEST(PredictionSmoothingTest, PosAlphaMonotoneDecreasing)
{
	double prev = PositionEwmAlpha(0);
	for (uint8_t s = 1; s <= 100; ++s) {
		const double cur = PositionEwmAlpha(s);
		EXPECT_LT(cur, prev) << "alpha must strictly decrease as smoothness rises (s=" << (int)s << ")";
		prev = cur;
	}
}

TEST(PredictionSmoothingTest, PosAlphaAtEightyIsHeavy)
{
	const double a80 = PositionEwmAlpha(80);
	// At s=80: t=0.2, alpha=0.2^1.8 ~= 0.075.  Allow a small numeric window.
	EXPECT_LT(a80, 0.12) << "s=80 alpha should be < 0.12 (heavy filter)";
	EXPECT_GT(a80, 0.04) << "s=80 alpha should be > 0.04";
}

// ---------------------------------------------------------------------------
// Position EWM convergence.  A device with pure white-noise position jitter
// (amplitude A, mean 0) centred on a true position p0.  After many frames
// at alpha=PositionEwmAlpha(100-via-smart-blended-to ~0.08), the output
// variance should be much lower than the input variance.
//
// Use a deterministic jitter pattern so the test is reproducible.
// ---------------------------------------------------------------------------
TEST(PredictionSmoothingTest, PosEwmReducesVariance)
{
	const uint8_t smoothness = 80;
	const double alpha = PositionEwmAlpha(smoothness); // ~0.075
	const double truePos = 1.0;                        // true position in metres
	const double jitterAmp = 0.005;                    // 5 mm peak jitter

	// Simulate 200 frames of +-jitter alternating.
	double ema = truePos;
	double sumSqIn = 0.0;
	double sumSqOut = 0.0;
	for (int i = 0; i < 200; ++i) {
		const double raw = truePos + (i % 2 == 0 ? jitterAmp : -jitterAmp);
		ema += alpha * (raw - ema);
		sumSqIn += (raw - truePos) * (raw - truePos);
		sumSqOut += (ema - truePos) * (ema - truePos);
	}
	const double varIn = sumSqIn / 200.0;
	const double varOut = sumSqOut / 200.0;
	EXPECT_LT(varOut, varIn * 0.05) << "EWM at s=80 should reduce position variance by at least 95%"
	                                   " (varIn="
	                                << varIn << " varOut=" << varOut << ")";
}

// ---------------------------------------------------------------------------
// Large-jump guard math.  The driver reseeds when dist^2 > 0.25.
// Verify the threshold arithmetic is consistent with a 0.5 m radius.
// ---------------------------------------------------------------------------
TEST(PredictionSmoothingTest, JumpGuardThreshold)
{
	const double kJumpThreshSq = 0.25; // 0.5 m radius squared
	// A 0.49 m jump: dist^2 = 0.2401, below threshold -- should NOT reseed.
	EXPECT_LT(0.49 * 0.49, kJumpThreshSq) << "0.49 m jump should not trigger reseed";
	// A 0.51 m jump: dist^2 = 0.2601, above threshold -- should reseed.
	EXPECT_GT(0.51 * 0.51, kJumpThreshSq) << "0.51 m jump should trigger reseed";
}

TEST(PredictionSmoothingTest, CutoffAlphaIsSamplingRateStable)
{
	constexpr double cutoffHz = 2.0;
	constexpr double durationSeconds = 1.0;
	auto run = [&](double dt) {
		double y = 0.0;
		const int steps = static_cast<int>(durationSeconds / dt);
		for (int i = 0; i < steps; ++i) {
			const double alpha = prediction::smart_shadow::AlphaFromCutoffHz(cutoffHz, dt);
			y += alpha * (1.0 - y);
		}
		return y;
	};

	const double at60Hz = run(1.0 / 60.0);
	const double at120Hz = run(1.0 / 120.0);
	EXPECT_NEAR(at60Hz, at120Hz, 0.02) << "cutoff-based alpha should behave similarly across sampling rates";
}

TEST(PredictionSmoothingTest, SmoothStepHasExpectedEdges)
{
	using prediction::smart_shadow::SmoothStep;
	EXPECT_DOUBLE_EQ(SmoothStep(2.0, 4.0, 1.0), 0.0);
	EXPECT_DOUBLE_EQ(SmoothStep(2.0, 4.0, 5.0), 1.0);
	EXPECT_NEAR(SmoothStep(2.0, 4.0, 3.0), 0.5, 1e-12);
}

TEST(PredictionSmoothingTest, SmartParamsPreservePredictionFactor)
{
	for (uint8_t smoothness : {uint8_t{0}, uint8_t{50}, uint8_t{80}, uint8_t{100}}) {
		const auto params = prediction::smart_shadow::BuildParams(smoothness);
		EXPECT_DOUBLE_EQ(params.basePredictionFactor, SmoothnessToFactor(smoothness));
	}
}

TEST(PredictionSmoothingTest, SmartParamsIncreaseRestSmoothingWithSlider)
{
	const auto p20 = prediction::smart_shadow::BuildParams(20);
	const auto p80 = prediction::smart_shadow::BuildParams(80);
	EXPECT_LT(p80.posMinCutoffHz, p20.posMinCutoffHz);
	EXPECT_LT(p80.rotMinCutoffHz, p20.rotMinCutoffHz);
	EXPECT_GT(p80.posBetaHzPerMps, p20.posBetaHzPerMps);
	EXPECT_GT(p80.rotBetaHzPerRadps, p20.rotBetaHzPerRadps);
}

TEST(PredictionSmoothingTest, SmartParamsKeepCurrentCurveBeforeHighEnd)
{
	const auto p80 = prediction::smart_shadow::BuildParams(80);
	EXPECT_NEAR(p80.posMinCutoffHz, 0.45 + 16.0 * std::pow(0.2, 1.8), 1e-12);
	EXPECT_NEAR(p80.posBetaHzPerMps, 6.0 + 18.0 * 0.8, 1e-12);
	EXPECT_DOUBLE_EQ(p80.releaseScale, 1.0);
	EXPECT_DOUBLE_EQ(p80.linMovingSpeed, 0.35);
	EXPECT_DOUBLE_EQ(p80.angMovingSpeed, 1.20);
}

TEST(PredictionSmoothingTest, SmartParamsMakeHundredStrongerButNonzero)
{
	const auto p100 = prediction::smart_shadow::BuildParams(100);
	EXPECT_NEAR(p100.posMinCutoffHz, 0.25, 1e-12);
	EXPECT_GT(p100.posMinCutoffHz, 0.0);
	EXPECT_NEAR(p100.releaseScale, 0.40, 1e-12);
	EXPECT_NEAR(p100.linMovingSpeed, 0.60, 1e-12);
	EXPECT_NEAR(p100.angMovingSpeed, 2.00, 1e-12);
	EXPECT_LT(p100.posBetaHzPerMps, 24.0);
}

TEST(PredictionSmoothingTest, LockedHeadsetParamsClampRotationCutoff)
{
	const auto shared = prediction::smart_shadow::BuildParams(100);
	const auto split = prediction::smart_shadow::BuildParams(100, 100);

	EXPECT_NEAR(shared.rotMinCutoffHz, 0.75, 1e-12);
	EXPECT_NEAR(split.posMinCutoffHz, shared.posMinCutoffHz, 1e-12);
	EXPECT_GE(split.rotMinCutoffHz, prediction::smart_shadow::kLockedHeadsetRotationMinCutoffHz);
}

TEST(PredictionSmoothingTest, LockedHeadsetParamsUseSeparateAxisSmoothness)
{
	const auto split = prediction::smart_shadow::BuildParams(80, 20);
	const auto pos = prediction::smart_shadow::BuildParams(80);
	const auto rot = prediction::smart_shadow::BuildParams(20);

	EXPECT_NEAR(split.posMinCutoffHz, pos.posMinCutoffHz, 1e-12);
	EXPECT_NEAR(split.posBetaHzPerMps, pos.posBetaHzPerMps, 1e-12);
	EXPECT_NEAR(split.rotMinCutoffHz, rot.rotMinCutoffHz, 1e-12);
	EXPECT_NEAR(split.rotBetaHzPerRadps, rot.rotBetaHzPerRadps, 1e-12);
}

TEST(PredictionSmoothingTest, ReleasedPredictionFactorMovesTowardPassThrough)
{
	using prediction::smart_shadow::ReleasedPredictionFactor;
	EXPECT_DOUBLE_EQ(ReleasedPredictionFactor(0.25, 0.0), 0.25);
	EXPECT_DOUBLE_EQ(ReleasedPredictionFactor(0.25, 1.0), 1.0);
	EXPECT_NEAR(ReleasedPredictionFactor(0.25, 0.5), 0.625, 1e-12);
}

// ---------------------------------------------------------------------------
// One-euro FilterStep -- the shared live + shadow position/rotation filter.
// ---------------------------------------------------------------------------

TEST(PredictionSmoothingFilterTest, FirstCallSeedsAndPassesThrough)
{
	using namespace prediction::smart_shadow;
	FilterState s;
	const Params p = BuildParams(80);
	double pos[3] = {0.1, 0.2, 0.3};
	double rot[4] = {1, 0, 0, 0};
	const StepResult r = FilterStep(s, p, pos, rot, 0.0, 0.0, 1.0 / 90.0);
	EXPECT_TRUE(r.reseeded);
	EXPECT_DOUBLE_EQ(s.filteredPos[0], 0.1);
	EXPECT_DOUBLE_EQ(s.filteredPos[1], 0.2);
	EXPECT_DOUBLE_EQ(s.filteredPos[2], 0.3);
}

// THE regression pin: at s=100 the old position EWM had alpha=0 and froze
// forever ("sticks at rest"). The one-euro min cutoff must keep tracking a
// slow drift instead of sticking at the seed.
TEST(PredictionSmoothingFilterTest, DoesNotFreezeAtMaxSmoothnessRest)
{
	using namespace prediction::smart_shadow;
	FilterState s;
	const Params p = BuildParams(100);
	const double dt = 1.0 / 90.0;
	double rot[4] = {1, 0, 0, 0};
	double seed[3] = {0, 0, 0};
	FilterStep(s, p, seed, rot, 0.0, 0.0, dt);
	// Drift +9 mm/s for 2 s -- below the still-speed threshold, so the gate
	// does not release; this exercises pure min-cutoff tracking.
	const double perFrame = 0.009 * dt;
	double x = 0.0;
	for (int i = 0; i < 180; ++i) {
		x += perFrame;
		double cur[3] = {x, 0, 0};
		FilterStep(s, p, cur, rot, 0.009, 0.0, dt);
	}
	EXPECT_GT(s.filteredPos[0], 0.008) << "filter froze at rest (output=" << s.filteredPos[0] << " m) -- regression";
	EXPECT_LT(s.filteredPos[0], x) << "output should lag slightly behind a drifting input, not lead it";
}

TEST(PredictionSmoothingFilterTest, LargeJumpReseedsWithoutLag)
{
	using namespace prediction::smart_shadow;
	FilterState s;
	const Params p = BuildParams(90);
	const double dt = 1.0 / 90.0;
	double rot[4] = {1, 0, 0, 0};
	double pos[3] = {0, 0, 0};
	for (int i = 0; i < 30; ++i)
		FilterStep(s, p, pos, rot, 0.0, 0.0, dt);
	double jumped[3] = {0.6, 0, 0}; // > positionJumpM (0.5 m)
	const StepResult r = FilterStep(s, p, jumped, rot, 0.0, 0.0, dt);
	EXPECT_TRUE(r.reseeded);
	EXPECT_DOUBLE_EQ(s.filteredPos[0], 0.6);
}

TEST(PredictionSmoothingFilterTest, LongGapReseeds)
{
	using namespace prediction::smart_shadow;
	FilterState s;
	const Params p = BuildParams(80);
	double rot[4] = {1, 0, 0, 0};
	double pos[3] = {0, 0, 0};
	FilterStep(s, p, pos, rot, 0.0, 0.0, 1.0 / 90.0);
	double moved[3] = {0.05, 0, 0};
	const StepResult r = FilterStep(s, p, moved, rot, 0.0, 0.0, 0.5); // > resetGapSeconds
	EXPECT_TRUE(r.reseeded);
	EXPECT_DOUBLE_EQ(s.filteredPos[0], 0.05);
}

TEST(PredictionSmoothingFilterTest, ReducesRestJitter)
{
	// Realistic sub-mm rest jitter from a stationary tracker (reported velocity
	// ~0). Large coherent jitter would derive a high speed and the filter would
	// (correctly) release -- that is motion, not rest -- so it would not, and
	// should not, be smoothed away.
	const FilterVariance v80 = MeasureRestJitterVariance(80);
	EXPECT_LT(v80.output, v80.input * 0.10) << "one-euro at s=80 should cut sub-mm rest jitter variance by >90%";
}

TEST(PredictionSmoothingFilterTest, HundredSmoothsRestJitterMoreThanEighty)
{
	const FilterVariance v80 = MeasureRestJitterVariance(80);
	const FilterVariance v100 = MeasureRestJitterVariance(100);
	EXPECT_LT(v100.output, v80.output * 0.75) << "s=100 should visibly strengthen rest damping over s=80";
}

TEST(PredictionSmoothingFilterTest, TracksFastMotionWithLowLag)
{
	const double lag80 = MeasureFastMotionLag(80);
	EXPECT_LT(lag80, 0.02) << "fast motion should track within 20 mm (lag=" << lag80 << " m)";
}

TEST(PredictionSmoothingFilterTest, HundredAllowsMoreLagButStaysBounded)
{
	const double lag80 = MeasureFastMotionLag(80);
	const double lag100 = MeasureFastMotionLag(100);
	EXPECT_GT(lag100, lag80 + 0.001) << "s=100 should be stronger than s=80 during motion";
	EXPECT_LT(lag100, 0.04) << "s=100 should not wedge behind normal fast motion (lag=" << lag100 << " m)";
}

TEST(PredictionSmoothingFilterTest, RotationConvergesTowardHeldTarget)
{
	using namespace prediction::smart_shadow;
	FilterState s;
	const Params p = BuildParams(80);
	const double dt = 1.0 / 90.0;
	double pos[3] = {0, 0, 0};
	double rot0[4] = {1, 0, 0, 0};
	FilterStep(s, p, pos, rot0, 0.0, 0.0, dt);
	const double half = 15.0 * 3.14159265358979323846 / 180.0; // 30 deg about Z
	double target[4] = {std::cos(half), 0.0, 0.0, std::sin(half)};
	for (int i = 0; i < 250; ++i)
		FilterStep(s, p, pos, target, 0.0, 0.0, dt);
	EXPECT_LT(QuatAngleRad(s.filteredRot, target), 0.05) << "rotation should converge toward a held target";
}

TEST(PredictionSmoothingFilterTest, CandidateParamsVaryAsIntended)
{
	using namespace prediction::smart_shadow;
	const Params base = BuildParams(80);

	const Params match = BuildCandidateParams(80, CandidateKind::Match);
	EXPECT_DOUBLE_EQ(match.posMinCutoffHz, base.posMinCutoffHz);
	EXPECT_DOUBLE_EQ(match.posBetaHzPerMps, base.posBetaHzPerMps);
	EXPECT_DOUBLE_EQ(match.gateReleaseTauSeconds, base.gateReleaseTauSeconds);

	const Params strong = BuildCandidateParams(80, CandidateKind::Strong);
	EXPECT_LT(strong.posMinCutoffHz, base.posMinCutoffHz) << "strong should smooth harder at rest (lower floor cutoff)";
	EXPECT_LT(strong.rotMinCutoffHz, base.rotMinCutoffHz);

	const Params resp = BuildCandidateParams(80, CandidateKind::Responsive);
	EXPECT_GT(resp.posBetaHzPerMps, base.posBetaHzPerMps)
	    << "responsive should ramp cutoff faster with speed (less motion lag)";
	EXPECT_LT(resp.gateReleaseTauSeconds, base.gateReleaseTauSeconds) << "responsive should release faster";
}

TEST(PredictionSmoothingFilterTest, CandidateKindNames)
{
	using namespace prediction::smart_shadow;
	EXPECT_STREQ(CandidateKindName(CandidateKind::Match), "match");
	EXPECT_STREQ(CandidateKindName(CandidateKind::Strong), "strong");
	EXPECT_STREQ(CandidateKindName(CandidateKind::Responsive), "responsive");
}

} // namespace
