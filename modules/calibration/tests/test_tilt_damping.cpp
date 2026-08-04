// Tilt-damped locked-relpose accept tests (GravityAlignment.h DampTilt wired
// into ComputeIncremental behind SetTiltDamping).
//
// Contract: yaw + translation follow each accepted candidate at full speed;
// the swing (roll/pitch) component moves toward the candidate on a slow
// exponential (kTiltTimeConstantSec) and never exceeds kMaxTiltRad. A real
// constant floor tilt must be CONVERGED TO, not zeroed -- the hard yaw
// projection was refuted on corpus replay precisely because it erased a real
// tilt; the damper only suppresses the per-solve random walk.

#include "CalibrationCalc.h"
#include "GravityAlignment.h"

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <cmath>
#include <vector>

namespace gravity = spacecal::gravity;

namespace {

Eigen::AffineCompact3d MakeCal(double yawRad, double tiltRad, const Eigen::Vector3d& trans)
{
	Eigen::AffineCompact3d c(Eigen::Quaterniond(Eigen::AngleAxisd(yawRad, Eigen::Vector3d::UnitY())) *
	                         Eigen::Quaterniond(Eigen::AngleAxisd(tiltRad, Eigen::Vector3d::UnitX())));
	c.translation() = trans;
	return c;
}

Eigen::AffineCompact3d MakeRef(double yaw, const Eigen::Vector3d& trans)
{
	Eigen::AffineCompact3d a(Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitY())));
	a.pretranslate(trans);
	return a;
}

Pose AffineToPose(const Eigen::AffineCompact3d& a)
{
	Pose p;
	p.rot = a.rotation();
	p.trans = a.translation();
	return p;
}

// Sample whose per-sample estimate (with relpose S = identity) is exactly `cx`.
Sample ExactSample(const Eigen::AffineCompact3d& ref, const Eigen::AffineCompact3d& cx, double t)
{
	Eigen::AffineCompact3d target = cx.inverse() * ref;
	return Sample(AffineToPose(ref), AffineToPose(target), t);
}

CalibrationCalc MakeLockedCalc(bool tiltDamping)
{
	CalibrationCalc calc;
	calc.lockRelativePosition = true;
	calc.setRelativeTransformation(Eigen::AffineCompact3d::Identity(), true);
	// Mirror the default (enhanced-checks-off) live pipeline the damper ships
	// into: the locked accept gate stands down, every validated candidate
	// applies. The damper's behaviour under the gates is covered by the
	// session-replay goldens instead.
	calc.SetLockedAcceptGate(false);
	calc.SetTiltDamping(tiltDamping);
	return calc;
}

// Push a fresh batch of samples all reporting calibration `cx` and run one
// compute tick. `t` advances by 10 s per call so each accept runs at the
// filter's dt clamp (alpha = 1 - exp(-10/tau) per accept).
bool AcceptOnce(CalibrationCalc& calc, const Eigen::AffineCompact3d& cx, double& t)
{
	for (int i = 0; i < 12; ++i) {
		const double yaw = 0.05 * i;
		const Eigen::Vector3d trans(0.05 * std::sin(0.7 * i), 0.05 * std::cos(0.5 * i), 0.05 * std::sin(0.3 * i));
		calc.PushSample(ExactSample(MakeRef(yaw, trans), cx, t));
		t += 0.01;
	}
	t += 10.0;
	bool lerp = false;
	return calc.ComputeIncremental(lerp, /*threshold=*/1.5, /*relPoseMaxError=*/1.0, /*ignoreOutliers=*/false);
}

double AppliedTiltDeg(const CalibrationCalc& calc)
{
	return gravity::TiltAngleDeg(Eigen::Quaterniond(calc.Transformation().rotation()));
}

} // namespace

// Alternating +/-3 deg candidate tilt (mean zero) must be absorbed. The
// solver averages over its whole rolling sample window, so the per-compute
// candidate is the window mean; yaw and translation are held constant in the
// stream so they must come through exactly while the tilt is damped. Seeded
// (the live session-start shape) so the damper has a prior from accept one.
TEST(TiltDampingTest, NoisyTiltCandidatesAreSuppressed)
{
	auto calc = MakeLockedCalc(/*tiltDamping=*/true);
	const Eigen::Vector3d trans(0.2, 0.0, 0.1);
	calc.SeedEstimatedTransformation(MakeCal(0.8, 0.0, trans), /*annotate=*/false);
	const double noise = 3.0 * EIGEN_PI / 180.0;
	double t = 0.0;
	double maxAppliedTilt = 0.0;
	for (int k = 0; k < 40; ++k) {
		const double tilt = (k % 2 == 0) ? noise : -noise;
		ASSERT_TRUE(AcceptOnce(calc, MakeCal(0.8, tilt, trans), t)) << k;
		maxAppliedTilt = std::max(maxAppliedTilt, AppliedTiltDeg(calc));
		// Translation follows the solver candidate at full speed; the stream
		// holds it constant, so the applied value must match it exactly.
		EXPECT_LT((calc.Transformation().translation() - trans).norm(), 1e-6) << k;
	}
	EXPECT_LT(maxAppliedTilt, 0.75) << "3-deg alternating tilt noise should damp to well under a degree";
	// Yaw tracked at full speed the whole time.
	const Eigen::Quaterniond yawOnly(Eigen::AngleAxisd(0.8, Eigen::Vector3d::UnitY()));
	EXPECT_LT(gravity::YawTwist(Eigen::Quaterniond(calc.Transformation().rotation())).angularDistance(yawOnly), 1e-6);
}

// A constant 1.5-deg real floor tilt is converged to, not zeroed. This is the
// anti-hard-projection contract.
TEST(TiltDampingTest, RealConstantTiltIsPreserved)
{
	auto calc = MakeLockedCalc(/*tiltDamping=*/true);
	const double tiltDeg = 1.5;
	const Eigen::AffineCompact3d cTrue = MakeCal(-0.4, tiltDeg * EIGEN_PI / 180.0, Eigen::Vector3d(0.1, 0.05, -0.2));
	double t = 0.0;
	// 120 accepts x alpha(10s/300s) covers ~4 time constants.
	for (int k = 0; k < 120; ++k) {
		ASSERT_TRUE(AcceptOnce(calc, cTrue, t)) << k;
	}
	EXPECT_GT(AppliedTiltDeg(calc), tiltDeg * 0.9) << "real tilt must be converged to, not projected away";
	EXPECT_LT(AppliedTiltDeg(calc), tiltDeg * 1.05);
	// With the tilt converged the applied transform matches the truth -- no
	// residual fit penalty from the damper.
	EXPECT_LT((calc.Transformation().translation() - cTrue.translation()).norm(), 1e-6);
	EXPECT_LT(
	    Eigen::Quaterniond(calc.Transformation().rotation()).angularDistance(Eigen::Quaterniond(cTrue.rotation())),
	    0.005);
}

// An 8-deg banked tilt seeded into the solver (stale profile / recorded seed)
// is capped on the first damped accept and decays toward level candidates.
TEST(TiltDampingTest, EightDegreeSeedIsCappedThenConverges)
{
	auto calc = MakeLockedCalc(/*tiltDamping=*/true);
	calc.SeedEstimatedTransformation(MakeCal(0.5, 8.0 * EIGEN_PI / 180.0, Eigen::Vector3d(0.3, 0.1, 0.0)),
	                                 /*annotate=*/false);
	const Eigen::AffineCompact3d cLevel = MakeCal(0.5, 0.0, Eigen::Vector3d(0.3, 0.1, 0.0));
	const double capDeg = gravity::kMaxTiltRad * 180.0 / EIGEN_PI;

	double t = 0.0;
	ASSERT_TRUE(AcceptOnce(calc, cLevel, t));
	EXPECT_LE(AppliedTiltDeg(calc), capDeg + 1e-6) << "first damped accept must cap the banked tilt";

	double prev = AppliedTiltDeg(calc);
	for (int k = 0; k < 60; ++k) {
		ASSERT_TRUE(AcceptOnce(calc, cLevel, t)) << k;
		const double cur = AppliedTiltDeg(calc);
		EXPECT_LE(cur, prev + 1e-9) << "tilt must decay monotonically toward level candidates at step " << k;
		prev = cur;
	}
	EXPECT_LT(prev, 1.0) << "tilt should be mostly gone after ~2 time constants";
}

// Flag off runs the raw path bit-identically (parity escape hatch), and the
// filter is inert on a tilt-free candidate stream even when on. Constant
// streams throughout: the solver averages its whole sample window, so a
// varying stream would make the candidate diverge from the last input.
TEST(TiltDampingTest, DisabledFlagMatchesBaselineExactly)
{
	// Off vs off over an identical tilted stream: deterministic, bit-identical.
	auto offA = MakeLockedCalc(/*tiltDamping=*/false);
	auto offB = MakeLockedCalc(/*tiltDamping=*/false);
	// On over a PURE-YAW stream: the swing is identity throughout, so damping
	// must change nothing beyond fp noise.
	auto onYaw = MakeLockedCalc(/*tiltDamping=*/true);
	const Eigen::AffineCompact3d tilted = MakeCal(0.7, 1.7 * EIGEN_PI / 180.0, Eigen::Vector3d(0.25, 0.1, -0.05));
	const Eigen::AffineCompact3d yawOnly = MakeCal(0.7, 0.0, Eigen::Vector3d(0.25, 0.1, -0.05));
	double t1 = 0.0, t2 = 0.0, t3 = 0.0;
	for (int k = 0; k < 10; ++k) {
		ASSERT_TRUE(AcceptOnce(offA, tilted, t1)) << k;
		ASSERT_TRUE(AcceptOnce(offB, tilted, t2)) << k;
		EXPECT_EQ((offA.Transformation().matrix() - offB.Transformation().matrix()).norm(), 0.0) << k;
		ASSERT_TRUE(AcceptOnce(onYaw, yawOnly, t3)) << k;
		EXPECT_LT(Eigen::Quaterniond(onYaw.Transformation().rotation())
		              .angularDistance(Eigen::Quaterniond(yawOnly.rotation())),
		          1e-9)
		    << k;
		EXPECT_LT((onYaw.Transformation().translation() - yawOnly.translation()).norm(), 1e-9) << k;
	}
	// The undamped pair really carried the tilt through (the flag was off).
	EXPECT_NEAR(AppliedTiltDeg(offA), 1.7, 0.01);
}
