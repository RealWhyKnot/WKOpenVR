// Unit tests for wkopenvr::headmount::Solver.
//
// The solver finds T such that hmd_i ~= tracker_i * T given a stream of
// paired (hmd_world, tracker_world) poses.

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <algorithm>
#include <random>
#include <cmath>

#include "HeadFromTrackerSolve.h"

using namespace wkopenvr::headmount;

namespace {

// Build an Affine3d from RPY (in radians) and a translation.
Eigen::Affine3d MakePose(double yaw, double pitch, double roll, const Eigen::Vector3d& t)
{
	Eigen::Quaterniond q = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitY()) *
	                       Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitX()) *
	                       Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitZ());
	Eigen::Affine3d a;
	a.linear() = q.toRotationMatrix();
	a.translation() = t;
	return a;
}

// Generate `n` synthetic pose pairs satisfying hmd_i = tracker_i * T.
// Tracker poses are random full-rotation samples; HMD poses are derived.
std::vector<std::pair<Eigen::Affine3d, Eigen::Affine3d>> MakePairs(const Eigen::AffineCompact3d& T, int n,
                                                                   uint32_t seed = 42)
{
	std::mt19937 rng(seed);
	std::uniform_real_distribution<double> ang(-EIGEN_PI, EIGEN_PI);
	std::uniform_real_distribution<double> tr(-1.0, 1.0);

	std::vector<std::pair<Eigen::Affine3d, Eigen::Affine3d>> out;
	out.reserve(n);
	for (int i = 0; i < n; i++) {
		Eigen::Affine3d tracker =
		    MakePose(ang(rng), ang(rng) * 0.5, ang(rng) * 0.5, Eigen::Vector3d(tr(rng), tr(rng), tr(rng)));
		Eigen::Affine3d hmd = tracker * Eigen::Affine3d(T);
		out.emplace_back(hmd, tracker);
	}
	return out;
}

std::vector<std::pair<Eigen::Affine3d, Eigen::Affine3d>> MakeYawOnlyPairs(const Eigen::AffineCompact3d& T, int n)
{
	std::vector<std::pair<Eigen::Affine3d, Eigen::Affine3d>> out;
	out.reserve(n);
	for (int i = 0; i < n; i++) {
		const double yaw =
		    (-25.0 + 50.0 * static_cast<double>(i) / static_cast<double>(std::max(1, n - 1))) * EIGEN_PI / 180.0;
		Eigen::Affine3d tracker = MakePose(yaw, 0.0, 0.0, Eigen::Vector3d(0.01 * i, 1.6, 0.05));
		Eigen::Affine3d hmd = tracker * Eigen::Affine3d(T);
		out.emplace_back(hmd, tracker);
	}
	return out;
}

} // namespace

// --- Basic state machine tests -------------------------------------------

TEST(Solver, IdleAtStart)
{
	Solver s;
	EXPECT_EQ(s.state(), SolveState::Idle);
	EXPECT_EQ(s.sampleCount(), 0u);
}

TEST(Solver, CollectsWithMotion)
{
	Solver s;
	s.Start();
	EXPECT_EQ(s.state(), SolveState::Collecting);

	Eigen::Affine3d hmd = MakePose(0.1, 0.0, 0.0, Eigen::Vector3d(0, 1.6, 0));
	Eigen::Affine3d tracker = MakePose(0.1, 0.0, 0.0, Eigen::Vector3d(0, 1.6, -0.05));

	bool accepted = s.Tick(hmd, tracker, Solver::kMinHmdSpeedMps + 0.1);
	EXPECT_TRUE(accepted);
	EXPECT_EQ(s.sampleCount(), 1u);
}

TEST(Solver, RejectsWithoutMotion)
{
	Solver s;
	s.Start();

	Eigen::Affine3d hmd = MakePose(0.0, 0.0, 0.0, Eigen::Vector3d(0, 1.6, 0));
	Eigen::Affine3d tracker = hmd;

	// Speed below threshold: sample must be rejected.
	bool accepted = s.Tick(hmd, tracker, Solver::kMinHmdSpeedMps - 0.01);
	EXPECT_FALSE(accepted);
	EXPECT_EQ(s.sampleCount(), 0u);
}

TEST(Solver, CancelClearsBuffer)
{
	Solver s;
	s.Start();

	Eigen::Affine3d p = MakePose(0.0, 0.0, 0.0, Eigen::Vector3d::Zero());
	s.Tick(p, p, 1.0);
	EXPECT_GT(s.sampleCount(), 0u);

	s.Cancel();
	EXPECT_EQ(s.state(), SolveState::Idle);
	EXPECT_EQ(s.sampleCount(), 0u);
}

// --- Failure-path tests --------------------------------------------------

TEST(Solver, FinishFailsFewSamples)
{
	Solver s;
	s.Start();

	// Feed fewer samples than the readiness floor.
	auto pairs = MakePairs(Eigen::AffineCompact3d::Identity(), 50);
	for (const auto& [hmd, tracker] : pairs) {
		s.Tick(hmd, tracker, 1.0);
	}
	EXPECT_LT(s.sampleCount(), Solver::kMinReadySampleCount);
	EXPECT_FALSE(s.readiness().ready);

	s.Finish();
	EXPECT_EQ(s.state(), SolveState::Failed);
	EXPECT_EQ(s.result().failReason, "not enough motion");
	EXPECT_EQ(s.result().samplesUsed, 50);
}

TEST(Solver, FinishFailsHighResidual)
{
	// Build pairs with large random jitter (> kResidualThresholdMm).
	std::mt19937 rng(99);
	std::uniform_real_distribution<double> jitter(-0.05, 0.05); // +/-5 cm noise

	Eigen::AffineCompact3d T;
	T.linear() = Eigen::Matrix3d::Identity();
	T.translation() = Eigen::Vector3d(0.0, 0.05, -0.03);

	auto cleanPairs = MakePairs(T, static_cast<int>(Solver::kTargetSampleCount) + 50);

	Solver s;
	s.Start();
	for (auto& [hmd, tracker] : cleanPairs) {
		// Add big jitter to the HMD position only -- simulates a slipping mount.
		Eigen::Affine3d hmdNoisy = hmd;
		hmdNoisy.translation() += Eigen::Vector3d(jitter(rng), jitter(rng), jitter(rng));
		s.Tick(hmdNoisy, tracker, 1.0);
	}

	s.Finish();
	EXPECT_EQ(s.state(), SolveState::Failed);
	EXPECT_EQ(s.result().failReason, "mount may be slipping");
	EXPECT_GT(s.result().samplesUsed, 0);
}

TEST(Solver, ReadinessRequiresPitchYawAndRollCoverage)
{
	Eigen::AffineCompact3d T = Eigen::AffineCompact3d::Identity();

	Solver s;
	s.Start();
	auto pairs = MakeYawOnlyPairs(T, static_cast<int>(Solver::kTargetSampleCount));
	for (const auto& [hmd, tracker] : pairs) {
		s.Tick(hmd, tracker, 1.0);
	}

	const CollectionReadiness r = s.readiness();
	EXPECT_TRUE(r.enoughSamples);
	EXPECT_FALSE(r.ready);
	EXPECT_GT(r.axisScore[1], 0.9);
	EXPECT_LT(r.axisScore[0], 0.1);
	EXPECT_LT(r.axisScore[2], 0.1);

	s.Finish();
	EXPECT_EQ(s.state(), SolveState::Failed);
	EXPECT_EQ(s.result().failReason, "not enough motion");
}

TEST(Solver, ReadinessCanPassBeforeOldTargetCount)
{
	const double kYaw = 5.0 * EIGEN_PI / 180.0;
	Eigen::AffineCompact3d T;
	T.linear() = (Eigen::AngleAxisd(kYaw, Eigen::Vector3d::UnitY())).toRotationMatrix();
	T.translation() = Eigen::Vector3d(0.0, 0.01, -0.03);

	auto pairs = MakePairs(T, static_cast<int>(Solver::kMinReadySampleCount) + 20);
	ASSERT_LT(pairs.size(), Solver::kTargetSampleCount);

	Solver s;
	s.Start();
	for (const auto& [hmd, tracker] : pairs) {
		s.Tick(hmd, tracker, 1.0);
	}

	const CollectionReadiness r = s.readiness();
	EXPECT_TRUE(r.ready);
	EXPECT_TRUE(r.enoughSamples);
	EXPECT_TRUE(r.enoughMotion);
	EXPECT_TRUE(r.residualGood);

	s.Finish();
	EXPECT_EQ(s.state(), SolveState::Done) << "failReason: " << s.result().failReason;
}

// --- Happy-path test: reconstruct known T --------------------------------

TEST(Solver, FinishSucceedsOnKnownTransform)
{
	// Small but non-trivial offset: 3 cm forward, 1 cm up, 5 deg yaw.
	const double kYaw = 5.0 * EIGEN_PI / 180.0;
	Eigen::AffineCompact3d T;
	T.linear() = (Eigen::AngleAxisd(kYaw, Eigen::Vector3d::UnitY())).toRotationMatrix();
	T.translation() = Eigen::Vector3d(0.0, 0.01, -0.03); // 1 cm up, 3 cm back

	// 300 samples, well above the 200-sample gate.
	auto pairs = MakePairs(T, 300);

	Solver s;
	s.Start();
	for (const auto& [hmd, tracker] : pairs) {
		s.Tick(hmd, tracker, 1.0);
	}

	s.Finish();
	ASSERT_EQ(s.state(), SolveState::Done) << "failReason: " << s.result().failReason;

	const SolveResult& r = s.result();
	EXPECT_EQ(r.samplesUsed, 300);
	EXPECT_LT(r.residualMm, 1.0); // sub-mm on clean synthetic data

	// Rotation error: angle between recovered R and expected R.
	Eigen::Matrix3d dR = T.rotation().transpose() * r.headFromTracker.rotation();
	double angleErr = std::acos(std::max(-1.0, std::min(1.0, (dR.trace() - 1.0) * 0.5))) * 180.0 / EIGEN_PI;
	EXPECT_LT(angleErr, 0.5); // within 0.5 degrees

	// Translation error (mm).
	double transErrMm = (T.translation() - r.headFromTracker.translation()).norm() * 1000.0;
	EXPECT_LT(transErrMm, 1.0); // sub-mm
}

TEST(Solver, CommonFrameConversionRecoversOffsetAcrossCalibrationTransform)
{
	const double kYaw = 7.0 * EIGEN_PI / 180.0;
	Eigen::AffineCompact3d headFromTracker;
	headFromTracker.linear() = (Eigen::AngleAxisd(kYaw, Eigen::Vector3d::UnitY())).toRotationMatrix();
	headFromTracker.translation() = Eigen::Vector3d(0.02, 0.03, -0.04);

	auto targetFramePairs = MakePairs(headFromTracker, static_cast<int>(Solver::kTargetSampleCount) + 25, 77);

	Eigen::Affine3d targetToReference =
	    Eigen::Translation3d(Eigen::Vector3d(1.2, -0.4, 0.8)) * Eigen::AngleAxisd(0.35, Eigen::Vector3d::UnitZ());

	Solver s;
	s.Start();
	for (const auto& [hmdTargetFrame, trackerTargetFrame] : targetFramePairs) {
		const Eigen::Affine3d hmdReferenceFrame = targetToReference * hmdTargetFrame;
		const Eigen::Affine3d hmdConvertedBack = targetToReference.inverse() * hmdReferenceFrame;
		s.Tick(hmdConvertedBack, trackerTargetFrame, 1.0);
	}

	s.Finish();
	ASSERT_EQ(s.state(), SolveState::Done) << "failReason: " << s.result().failReason;

	EXPECT_LT(s.result().residualMm, 1.0);
	EXPECT_LT((s.result().headFromTracker.translation() - headFromTracker.translation()).norm() * 1000.0, 1.0);
}

TEST(Solver, MixedCalibrationFramesKeepConsistencyAtZero)
{
	const double kYaw = 7.0 * EIGEN_PI / 180.0;
	Eigen::AffineCompact3d headFromTracker;
	headFromTracker.linear() = (Eigen::AngleAxisd(kYaw, Eigen::Vector3d::UnitY())).toRotationMatrix();
	headFromTracker.translation() = Eigen::Vector3d(0.02, 0.03, -0.04);

	auto targetFramePairs = MakePairs(headFromTracker, static_cast<int>(Solver::kMinReadySampleCount) + 40, 91);

	Eigen::Affine3d targetToReferenceAtStart =
	    Eigen::Translation3d(Eigen::Vector3d(1.2, -0.4, 0.8)) * Eigen::AngleAxisd(0.35, Eigen::Vector3d::UnitZ());
	const Eigen::Affine3d frozenTargetFromReference = targetToReferenceAtStart.inverse();

	Solver frozen;
	Solver mixed;
	frozen.Start();
	mixed.Start();

	for (size_t i = 0; i < targetFramePairs.size(); ++i) {
		const auto& hmdTargetFrame = targetFramePairs[i].first;
		const auto& trackerTargetFrame = targetFramePairs[i].second;
		const Eigen::Affine3d hmdReferenceFrame = targetToReferenceAtStart * hmdTargetFrame;

		frozen.Tick(frozenTargetFromReference * hmdReferenceFrame, trackerTargetFrame, 1.0);

		const double drift = 0.0015 * static_cast<double>(i);
		const Eigen::Affine3d liveTargetToReference = targetToReferenceAtStart *
		                                              Eigen::Translation3d(Eigen::Vector3d(drift, 0.0, 0.0)) *
		                                              Eigen::AngleAxisd(drift, Eigen::Vector3d::UnitZ());
		mixed.Tick(liveTargetToReference.inverse() * hmdReferenceFrame, trackerTargetFrame, 1.0);
	}

	const CollectionReadiness frozenReady = frozen.readiness();
	ASSERT_TRUE(frozenReady.ready);
	EXPECT_LT(frozenReady.residualMm, 1.0);

	const CollectionReadiness mixedReady = mixed.readiness();
	EXPECT_FALSE(mixedReady.ready);
	EXPECT_GT(mixedReady.residualMm, Solver::kResidualThresholdMm * 2.0);
	EXPECT_DOUBLE_EQ(mixedReady.residualScore, 0.0);
}
