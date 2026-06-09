#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <random>
#include <vector>

#include "CalibrationCalc.h"

namespace {

Eigen::AffineCompact3d MakeTransform(double yawRad, double pitchRad, double rollRad, const Eigen::Vector3d& translation)
{
	Eigen::Quaterniond q = Eigen::AngleAxisd(yawRad, Eigen::Vector3d::UnitY()) *
	                       Eigen::AngleAxisd(pitchRad, Eigen::Vector3d::UnitX()) *
	                       Eigen::AngleAxisd(rollRad, Eigen::Vector3d::UnitZ());
	Eigen::AffineCompact3d t;
	t.linear() = q.toRotationMatrix();
	t.translation() = translation;
	return t;
}

Pose AffineToPose(const Eigen::AffineCompact3d& a)
{
	Pose p;
	p.rot = a.rotation();
	p.trans = a.translation();
	return p;
}

Sample MakeSample(const Eigen::AffineCompact3d& refPose, const Eigen::AffineCompact3d& calibration, double timestamp)
{
	Eigen::AffineCompact3d targetPose = calibration.inverse() * refPose;
	return Sample(AffineToPose(refPose), AffineToPose(targetPose), timestamp);
}

std::vector<Sample> MakeSamplePairs(const Eigen::AffineCompact3d& calibration, int numSamples, uint32_t seed = 42)
{
	std::mt19937 rng(seed);
	std::uniform_real_distribution<double> angleDist(-EIGEN_PI, EIGEN_PI);
	std::uniform_real_distribution<double> transDist(-1.0, 1.0);

	std::vector<Sample> samples;
	samples.reserve(numSamples);
	for (int i = 0; i < numSamples; i++) {
		Eigen::AffineCompact3d refPose = MakeTransform(angleDist(rng), angleDist(rng) * 0.5, angleDist(rng) * 0.5,
		                                               Eigen::Vector3d(transDist(rng), transDist(rng), transDist(rng)));
		samples.push_back(MakeSample(refPose, calibration, i * 0.01));
	}
	return samples;
}

Sample MakePoseSample(const Eigen::Vector3d& refTrans, const Eigen::Vector3d& targetTrans, double yawRad,
                      double timestamp)
{
	Pose ref;
	ref.rot = Eigen::AngleAxisd(yawRad, Eigen::Vector3d::UnitY()).toRotationMatrix();
	ref.trans = refTrans;
	Pose target;
	target.rot = ref.rot;
	target.trans = targetTrans;
	return Sample(ref, target, timestamp);
}

double RotationErrorDegrees(const Eigen::AffineCompact3d& a, const Eigen::AffineCompact3d& b)
{
	Eigen::Matrix3d delta = a.rotation() * b.rotation().transpose();
	double trace = delta.trace();
	double cosAngle = std::max(-1.0, std::min(1.0, (trace - 1.0) / 2.0));
	return std::acos(cosAngle) * 180.0 / EIGEN_PI;
}

constexpr int kSampleCount = 60;

} // namespace

TEST(CalibrationCalcTest, RecoversIdentity)
{
	Eigen::AffineCompact3d expected = Eigen::AffineCompact3d::Identity();
	CalibrationCalc calc;
	for (auto& s : MakeSamplePairs(expected, kSampleCount)) {
		calc.PushSample(s);
	}

	ASSERT_TRUE(calc.ComputeOneshot(false));
	EXPECT_LT((calc.Transformation().matrix() - expected.matrix()).norm(), 1e-6);
}

TEST(CalibrationCalcTest, RecoversPureYaw)
{
	const double yawRad = 20.0 * EIGEN_PI / 180.0;
	Eigen::AffineCompact3d expected = MakeTransform(yawRad, 0.0, 0.0, Eigen::Vector3d::Zero());

	CalibrationCalc calc;
	for (auto& s : MakeSamplePairs(expected, kSampleCount)) {
		calc.PushSample(s);
	}

	ASSERT_TRUE(calc.ComputeOneshot(false));
	EXPECT_LT(RotationErrorDegrees(calc.Transformation(), expected), 0.5);
}

TEST(CalibrationCalcTest, RecoversPureTranslation)
{
	Eigen::Vector3d trans(0.25, -0.10, 0.40);
	Eigen::AffineCompact3d expected = MakeTransform(0.0, 0.0, 0.0, trans);

	CalibrationCalc calc;
	for (auto& s : MakeSamplePairs(expected, kSampleCount)) {
		calc.PushSample(s);
	}

	ASSERT_TRUE(calc.ComputeOneshot(false));
	EXPECT_LT((calc.Transformation().translation() - trans).norm(), 5e-3);
}

TEST(CalibrationCalcTest, RecoversCombinedOffset)
{
	const double yawRad = 10.0 * EIGEN_PI / 180.0;
	Eigen::Vector3d trans(0.5, 0.1, -0.3);
	Eigen::AffineCompact3d expected = MakeTransform(yawRad, 0.0, 0.0, trans);

	CalibrationCalc calc;
	for (auto& s : MakeSamplePairs(expected, kSampleCount)) {
		calc.PushSample(s);
	}

	ASSERT_TRUE(calc.ComputeOneshot(false));
	EXPECT_LT((calc.Transformation().translation() - trans).norm(), 1e-2);
	EXPECT_LT(RotationErrorDegrees(calc.Transformation(), expected), 1.0);
}

TEST(CalibrationCalcTest, SeedEstimatedTransformationStartsIncrementalFromProfile)
{
	const Eigen::Vector3d trans(-1.06882, 2.47276, 0.50086);
	Eigen::AffineCompact3d profile = MakeTransform(20.0 * EIGEN_PI / 180.0, 0.0, 0.0, trans);

	CalibrationCalc calc;
	calc.Clear();
	calc.SeedEstimatedTransformation(profile, false);

	ASSERT_TRUE(calc.isValid());
	EXPECT_LT((calc.Transformation().translation() - trans).norm(), 1e-9);

	for (auto& s : MakeSamplePairs(profile, kSampleCount)) {
		calc.PushSample(s);
	}

	bool lerp = false;
	(void)calc.ComputeIncremental(lerp, 1.5, 0.005, false);
	ASSERT_TRUE(calc.isValid());
	ASSERT_TRUE(std::isfinite(calc.LastPriorErrorM()));
	EXPECT_LT(calc.LastPriorErrorM(), 1e-3);
}

TEST(CalibrationCalcTest, SeededTransformSurvivesTooFewIncrementalSamples)
{
	const Eigen::Vector3d trans(-1.06882, 2.47276, 0.50086);
	Eigen::AffineCompact3d profile = MakeTransform(20.0 * EIGEN_PI / 180.0, 0.0, 0.0, trans);

	CalibrationCalc calc;
	calc.Clear();
	calc.SeedEstimatedTransformation(profile, false);

	for (auto& s : MakeSamplePairs(profile, 5)) {
		calc.PushSample(s);
	}

	bool lerp = false;
	(void)calc.ComputeIncremental(lerp, 1.5, 0.005, false);

	EXPECT_TRUE(calc.isValid());
	EXPECT_LT((calc.Transformation().translation() - trans).norm(), 1e-9);
}

TEST(CalibrationCalcTest, IgnoreOutliersKeepsCleanFitNearTruth)
{
	const double yawRad = 20.0 * EIGEN_PI / 180.0;
	Eigen::Vector3d trans(0.2, 0.0, 0.4);
	Eigen::AffineCompact3d expected = MakeTransform(yawRad, 0.0, 0.0, trans);
	auto samples = MakeSamplePairs(expected, kSampleCount);

	std::mt19937 rng(1234);
	std::uniform_real_distribution<double> angleDist(-EIGEN_PI, EIGEN_PI);
	for (size_t i = 0; i < samples.size(); i += 20) {
		Eigen::Quaterniond randomRot = Eigen::AngleAxisd(angleDist(rng), Eigen::Vector3d::UnitX()) *
		                               Eigen::AngleAxisd(angleDist(rng), Eigen::Vector3d::UnitY()) *
		                               Eigen::AngleAxisd(angleDist(rng), Eigen::Vector3d::UnitZ());
		samples[i].target.rot = randomRot.toRotationMatrix();
	}

	CalibrationCalc calc;
	for (auto& s : samples) {
		calc.PushSample(s);
	}

	ASSERT_TRUE(calc.ComputeOneshot(true));
	EXPECT_LT((calc.Transformation().translation() - trans).norm(), 5e-2);
	EXPECT_LT(RotationErrorDegrees(calc.Transformation(), expected), 2.0);
}

TEST(CalibrationCalcTest, QualityReportPassesWellCoveredLegacyFit)
{
	const double yawRad = 15.0 * EIGEN_PI / 180.0;
	Eigen::Vector3d trans(0.35, -0.08, 0.22);
	Eigen::AffineCompact3d expected = MakeTransform(yawRad, 0.0, 0.0, trans);

	CalibrationCalc calc;
	for (auto& s : MakeSamplePairs(expected, kSampleCount, 0x5150)) {
		calc.PushSample(s);
	}

	ASSERT_TRUE(calc.ComputeOneshot(false));
	const CalibrationQualityReport report = calc.EvaluateCalibrationQuality(calc.Transformation(), true, false);

	EXPECT_EQ(report.validSampleCount, kSampleCount);
	EXPECT_TRUE(report.legacyRmsPass);
	EXPECT_TRUE(report.geometryPass);
	EXPECT_TRUE(report.robustResidualPass);
	EXPECT_TRUE(report.holdoutPass);
	EXPECT_TRUE(report.trackingHealthPass);
	EXPECT_TRUE(report.shadowDynamicPass);
	const CalibrationQualityVerdict verdict = EvaluateCalibrationQualityVerdict(report);
	EXPECT_TRUE(verdict.wouldAccept);
	EXPECT_STREQ(verdict.reason, "pass");
	EXPECT_GT(report.validRotationPairCount, 10);
	EXPECT_GE(report.translationRank, 2);
	EXPECT_TRUE(std::isfinite(report.dynamicLimitM));
	EXPECT_LT(report.residuals.rmsM, 1e-4);
	EXPECT_LT(report.holdoutResiduals.rmsM, 1e-3);
}

TEST(CalibrationCalcTest, QualityReportRejectsStaleTrackingInShadowVerdict)
{
	const double yawRad = 15.0 * EIGEN_PI / 180.0;
	Eigen::Vector3d trans(0.35, -0.08, 0.22);
	Eigen::AffineCompact3d expected = MakeTransform(yawRad, 0.0, 0.0, trans);

	CalibrationCalc calc;
	for (auto& s : MakeSamplePairs(expected, kSampleCount, 0x5150)) {
		s.refPoseAgeMs = 130.0;
		s.targetPoseAgeMs = 131.0;
		s.trackingPoseStale = true;
		calc.PushSample(s);
	}

	ASSERT_TRUE(calc.ComputeOneshot(false));
	const CalibrationQualityReport report = calc.EvaluateCalibrationQuality(calc.Transformation(), true, false);

	EXPECT_TRUE(report.legacyRmsPass);
	EXPECT_TRUE(report.geometryPass);
	EXPECT_TRUE(report.robustResidualPass);
	EXPECT_FALSE(report.trackingHealthPass);
	EXPECT_FALSE(report.shadowDynamicPass);
	EXPECT_EQ(report.trackingStaleSampleCount, kSampleCount);
	EXPECT_GT(report.maxPoseAgeMs, 120.0);

	const CalibrationQualityVerdict verdict = EvaluateCalibrationQualityVerdict(report);
	EXPECT_FALSE(verdict.wouldAccept);
	EXPECT_STREQ(verdict.reason, "tracking_health");
	const CalibrationQualityShadowSignals signals = EvaluateCalibrationQualityShadowSignals(report);
	EXPECT_TRUE(signals.trackingContaminated);
	EXPECT_TRUE(signals.legacyAcceptedButShadowRejected);
}

TEST(CalibrationCalcTest, QualityReportRejectsLowGeometryEvenWhenRmsIsSmall)
{
	CalibrationCalc calc;
	for (int i = 0; i < 30; i++) {
		const Eigen::Vector3d pos(i * 0.01, 0.0, 0.0);
		calc.PushSample(MakePoseSample(pos, pos, 0.0, i * 0.01));
	}

	const CalibrationQualityReport report =
	    calc.EvaluateCalibrationQuality(Eigen::AffineCompact3d::Identity(), true, false);

	EXPECT_TRUE(report.legacyRmsPass);
	EXPECT_FALSE(report.geometryPass);
	EXPECT_FALSE(report.shadowDynamicPass);
	const CalibrationQualityVerdict verdict = EvaluateCalibrationQualityVerdict(report);
	EXPECT_FALSE(verdict.wouldAccept);
	EXPECT_STREQ(verdict.reason, "geometry");
	EXPECT_EQ(report.validRotationPairCount, 0);
	EXPECT_LT(report.residuals.rmsM, 1e-6);

	const CalibrationQualityShadowSignals signals = EvaluateCalibrationQualityShadowSignals(report);
	EXPECT_TRUE(signals.lowResidualGeometryReject);
	EXPECT_TRUE(signals.novaWouldRejectForDeltaPairs);
	EXPECT_FALSE(signals.novaDeltaPairsPass);
}

TEST(CalibrationCalcTest, QualityVerdictReportsFirstFailedGate)
{
	CalibrationQualityReport report;

	CalibrationQualityVerdict verdict = EvaluateCalibrationQualityVerdict(report);
	EXPECT_FALSE(verdict.wouldAccept);
	EXPECT_STREQ(verdict.reason, "no_valid_samples");

	report.validSampleCount = 30;
	report.legacyRmsPass = false;
	report.geometryPass = false;
	report.robustResidualPass = false;
	report.holdoutPass = false;
	verdict = EvaluateCalibrationQualityVerdict(report);
	EXPECT_FALSE(verdict.wouldAccept);
	EXPECT_STREQ(verdict.reason, "legacy_rms");

	report.legacyRmsPass = true;
	verdict = EvaluateCalibrationQualityVerdict(report);
	EXPECT_FALSE(verdict.wouldAccept);
	EXPECT_STREQ(verdict.reason, "geometry");

	report.geometryPass = true;
	verdict = EvaluateCalibrationQualityVerdict(report);
	EXPECT_FALSE(verdict.wouldAccept);
	EXPECT_STREQ(verdict.reason, "robust_residual");

	report.robustResidualPass = true;
	verdict = EvaluateCalibrationQualityVerdict(report);
	EXPECT_FALSE(verdict.wouldAccept);
	EXPECT_STREQ(verdict.reason, "holdout");

	report.holdoutPass = true;
	report.trackingHealthPass = false;
	verdict = EvaluateCalibrationQualityVerdict(report);
	EXPECT_FALSE(verdict.wouldAccept);
	EXPECT_STREQ(verdict.reason, "tracking_health");

	report.trackingHealthPass = true;
	verdict = EvaluateCalibrationQualityVerdict(report);
	EXPECT_TRUE(verdict.wouldAccept);
	EXPECT_STREQ(verdict.reason, "pass");
}

TEST(CalibrationCalcTest, DoesNotCrashOnSmallSampleBuffer)
{
	Eigen::AffineCompact3d expected = MakeTransform(0.2, 0.0, 0.0, Eigen::Vector3d(0.1, 0.0, -0.2));

	for (int n : {1, 2, 3, 4, 5}) {
		CalibrationCalc calc;
		auto samples = MakeSamplePairs(expected, n, 0xC0FFEE);
		for (auto& s : samples)
			calc.PushSample(s);

		EXPECT_NO_THROW({ (void)calc.ComputeOneshot(true); }) << "n=" << n;
		EXPECT_NO_THROW({ (void)calc.ComputeOneshot(false); }) << "n=" << n;
	}
}

TEST(CalibrationCalcTest, JitterReflectsPositionSpread)
{
	CalibrationCalc calc;
	EXPECT_DOUBLE_EQ(calc.ReferenceJitter(), 0.0);
	EXPECT_DOUBLE_EQ(calc.TargetJitter(), 0.0);

	Pose stationaryRef;
	stationaryRef.rot = Eigen::Matrix3d::Identity();
	stationaryRef.trans = Eigen::Vector3d(1.0, 2.0, 3.0);
	Pose stationaryTgt;
	stationaryTgt.rot = Eigen::Matrix3d::Identity();
	stationaryTgt.trans = Eigen::Vector3d(0.0, 0.0, 0.0);
	for (int i = 0; i < 100; i++) {
		calc.PushSample(Sample(stationaryRef, stationaryTgt, i * 0.01));
	}
	EXPECT_LT(calc.ReferenceJitter(), 1e-9);
	EXPECT_LT(calc.TargetJitter(), 1e-9);

	CalibrationCalc noisy;
	std::mt19937 rng(123);
	std::normal_distribution<double> noise(0.0, 0.005);
	for (int i = 0; i < 100; i++) {
		Pose noisyTarget = stationaryTgt;
		noisyTarget.trans += Eigen::Vector3d(noise(rng), noise(rng), noise(rng));
		noisy.PushSample(Sample(stationaryRef, noisyTarget, i * 0.01));
	}

	EXPECT_GT(noisy.TargetJitter(), 0.003);
	EXPECT_LT(noisy.TargetJitter(), 0.020);
}

TEST(CalibrationCalcTest, TranslationDiversityBoundaryCases)
{
	CalibrationCalc empty;
	EXPECT_DOUBLE_EQ(empty.TranslationDiversity(), 0.0);

	CalibrationCalc oneAxis;
	for (int i = 0; i < 10; i++) {
		oneAxis.PushSample(MakePoseSample(Eigen::Vector3d::Zero(), Eigen::Vector3d(i * 0.03, 0.0, 0.0), 0.0, i * 0.01));
	}
	EXPECT_DOUBLE_EQ(oneAxis.TranslationDiversity(), 0.0);

	CalibrationCalc full;
	full.PushSample(MakePoseSample(Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, 0.0, 0.0), 0.0, 0.0));
	full.PushSample(MakePoseSample(Eigen::Vector3d::Zero(), Eigen::Vector3d(0.25, 0.22, 0.21), 0.0, 0.01));
	EXPECT_DOUBLE_EQ(full.TranslationDiversity(), 1.0);
	EXPECT_TRUE(full.TranslationAxisRangesCm().isApprox(Eigen::Vector3d(25.0, 22.0, 21.0)));
}

TEST(CalibrationCalcTest, RotationDiversityBoundaryCases)
{
	CalibrationCalc empty;
	EXPECT_DOUBLE_EQ(empty.RotationDiversity(), 0.0);

	CalibrationCalc same;
	for (int i = 0; i < 10; i++) {
		same.PushSample(MakePoseSample(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 0.0, i * 0.01));
	}
	EXPECT_DOUBLE_EQ(same.RotationDiversity(), 0.0);

	CalibrationCalc wide;
	wide.PushSample(MakePoseSample(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 0.0, 0.0));
	wide.PushSample(MakePoseSample(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), EIGEN_PI / 2.0, 0.01));
	EXPECT_DOUBLE_EQ(wide.RotationDiversity(), 1.0);
}

TEST(CalibrationCalcTest, RotationDiversityUsesBestPairNotFirstSample)
{
	CalibrationCalc calc;
	calc.PushSample(MakePoseSample(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 0.0, 0.0));
	calc.PushSample(MakePoseSample(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 0.05, 0.01));
	calc.PushSample(MakePoseSample(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), EIGEN_PI / 2.0, 0.02));

	EXPECT_DOUBLE_EQ(calc.RotationDiversity(), 1.0);
}
