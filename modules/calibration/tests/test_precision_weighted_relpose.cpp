// Precision-weighted relative-pose solve tests.
//
// CalibrateByRelPose computes the absolute calibration C as an average of the
// per-sample estimate R * S * T^-1. Each estimate's translation error scales
// with the lever arm (distance of ref/target from their tracking origins), so a
// far-from-origin sample is far noisier. The weighted solve down-weights those
// by 1/lever-arm^2; the uniform solve trusts them equally. This pins the
// contract that a handful of far, wrong readings can't drag the weighted result
// off the truth the near readings agree on -- the root-cause fix for the
// head-mounted-tracker "flies off far from origin" report.

#include "CalibrationCalc.h"

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <vector>

namespace {

Eigen::AffineCompact3d Translate(double x, double y, double z)
{
	Eigen::AffineCompact3d t = Eigen::AffineCompact3d::Identity();
	t.translation() = Eigen::Vector3d(x, y, z);
	return t;
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

// Sample whose per-sample estimate (with relpose S = identity) is exactly `cx`:
// target = cx^-1 * ref  =>  R * I * T^-1 = ref * (cx^-1 * ref)^-1 = cx.
Sample ExactSample(const Eigen::AffineCompact3d& ref, const Eigen::AffineCompact3d& cx, double t)
{
	Eigen::AffineCompact3d target = cx.inverse() * ref;
	return Sample(AffineToPose(ref), AffineToPose(target), t);
}

// Solve the relpose calibration for the given sample set with weighting on/off.
// Returns the resulting translation; sets `ok`.
Eigen::Vector3d SolveTranslation(const std::vector<Sample>& samples, bool weighted, bool& ok)
{
	CalibrationCalc calc;
	calc.lockRelativePosition = true;
	calc.setRelativeTransformation(Eigen::AffineCompact3d::Identity(), true);
	calc.SetPrecisionWeightedRelPose(weighted);
	for (const auto& s : samples)
		calc.PushSample(s);
	bool lerp = false;
	ok = calc.ComputeIncremental(lerp, /*threshold=*/1.5, /*relPoseMaxError=*/1.0, /*ignoreOutliers=*/false);
	return calc.Transformation().translation();
}

} // namespace

// The true calibration is a small offset; a majority of near-origin samples
// agree on it, while a few far-from-origin samples report a different (wrong)
// calibration. The weighted solve must land far closer to the truth than the
// uniform average, which the far readings pull off by their full 1/N share.
TEST(PrecisionWeightedRelPoseTest, FarReadingsDoNotDragWeightedResult)
{
	const Eigen::AffineCompact3d cTrue = Translate(0.2, 0.0, 0.0);
	const Eigen::AffineCompact3d cWrong = Translate(0.2, 0.2, 0.0); // differs by 20 cm in Y

	std::vector<Sample> samples;
	double t = 0.0;
	// 55 near-origin readings that agree on the truth (small lever arm).
	for (int i = 0; i < 55; ++i) {
		const double yaw = 0.03 * i;
		const Eigen::Vector3d trans(0.05 * std::sin(0.7 * i), 0.05 * std::cos(0.5 * i), 0.05 * std::sin(0.3 * i));
		samples.push_back(ExactSample(MakeRef(yaw, trans), cTrue, t));
		t += 0.01;
	}
	// 5 far readings reporting the wrong calibration (large lever arm ~3 m).
	for (int i = 0; i < 5; ++i) {
		const Eigen::Vector3d trans(3.0 + 0.1 * i, 0.3, -0.2 * i);
		samples.push_back(ExactSample(MakeRef(0.2 * i, trans), cWrong, t));
		t += 0.01;
	}

	bool okUniform = false, okWeighted = false;
	const Eigen::Vector3d uniform = SolveTranslation(samples, /*weighted=*/false, okUniform);
	const Eigen::Vector3d weighted = SolveTranslation(samples, /*weighted=*/true, okWeighted);
	ASSERT_TRUE(okUniform) << "uniform relpose solve should still produce a candidate";
	ASSERT_TRUE(okWeighted) << "weighted relpose solve should still produce a candidate";

	const double uniformErr = (uniform - cTrue.translation()).norm();
	const double weightedErr = (weighted - cTrue.translation()).norm();

	EXPECT_LT(weightedErr, uniformErr * 0.25) << "weighted err=" << weightedErr << "  uniform err=" << uniformErr;
	EXPECT_LT(weightedErr, 0.01) << "weighted solve should sit within 1 cm of truth";
}

// When every reading is near-origin and consistent, weighting must not change
// the result -- weights are ~equal, so the weighted mean equals the uniform one.
// Guards against a near-origin regression on the 9/10 working case.
TEST(PrecisionWeightedRelPoseTest, NearOriginMatchesUniform)
{
	const Eigen::AffineCompact3d cTrue = Translate(0.15, -0.1, 0.05);

	std::vector<Sample> samples;
	double t = 0.0;
	for (int i = 0; i < 60; ++i) {
		const double yaw = 0.05 * i;
		const Eigen::Vector3d trans(0.1 * std::sin(0.6 * i), 0.1 * std::cos(0.4 * i), 0.1 * std::sin(0.2 * i));
		samples.push_back(ExactSample(MakeRef(yaw, trans), cTrue, t));
		t += 0.01;
	}

	bool okUniform = false, okWeighted = false;
	const Eigen::Vector3d uniform = SolveTranslation(samples, /*weighted=*/false, okUniform);
	const Eigen::Vector3d weighted = SolveTranslation(samples, /*weighted=*/true, okWeighted);
	ASSERT_TRUE(okUniform);
	ASSERT_TRUE(okWeighted);

	// Both must recover the truth, and match each other to sub-mm.
	EXPECT_LT((weighted - cTrue.translation()).norm(), 1e-3);
	EXPECT_LT((weighted - uniform).norm(), 1e-3);
}
