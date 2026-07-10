#include <gtest/gtest.h>

#include "CalibrationCalc.h"
#include "LeverArmCovariance.h"
#include "MotionRecording.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <random>
#include <string>
#include <vector>

namespace replay = spacecal::replay;

namespace {

bool EnvFlagLocal(const char* name, bool fallback)
{
	const char* raw = std::getenv(name);
	if (!raw) return fallback;
	std::string value = raw;
	for (char& c : value) {
		c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
	}
	if (value == "1" || value == "true" || value == "yes" || value == "on") return true;
	if (value == "0" || value == "false" || value == "no" || value == "off") return false;
	return fallback;
}

double MedianOf(std::vector<double>& v)
{
	if (v.empty()) return 0.0;
	const std::size_t mid = v.size() / 2;
	std::nth_element(v.begin(), v.begin() + mid, v.end());
	return v[mid];
}

// Angle of the delta rotation between two consecutive orientation samples.
double DeltaAngleRad(const Eigen::Matrix3d& a, const Eigen::Matrix3d& b)
{
	const Eigen::Matrix3d d = b * a.transpose();
	const double c = std::clamp((d.trace() - 1.0) * 0.5, -1.0, 1.0);
	return std::acos(c);
}

// Per-device pose-noise accumulator over stationary consecutive row pairs.
// Consecutive deltas difference away any slow drift, leaving the
// high-frequency jitter the lever-arm covariance model needs:
//   delta rotation vector ~ N(0, 2*sigma_theta^2 I3)  =>  |phi| is chi(3)
//   scaled by sigma_theta*sqrt(2); the chi(3) median is 1.538172, so
//   sigma_theta = median(|phi|) / (1.538172 * sqrt(2)).
//   per-axis translation delta ~ N(0, 2*sigma_jit^2)  =>
//   sigma_jit = 1.4826 * median(|d_axis|) / sqrt(2).
// The reported translation jitter still contains the tracking system's own
// angular error times the device's distance from its origin, so the median
// lever-arm distance is printed alongside for interpretation.
struct DeviceNoise
{
	std::vector<double> rotDeltaRad;
	std::vector<double> transDeltaAbs; // per-axis, meters
	std::vector<double> leverM;

	void Push(const Pose& prev, const Pose& cur)
	{
		rotDeltaRad.push_back(DeltaAngleRad(prev.rot, cur.rot));
		const Eigen::Vector3d d = cur.trans - prev.trans;
		transDeltaAbs.push_back(std::abs(d.x()));
		transDeltaAbs.push_back(std::abs(d.y()));
		transDeltaAbs.push_back(std::abs(d.z()));
		leverM.push_back(cur.trans.norm());
	}

	double SigmaThetaRad()
	{
		constexpr double kChi3Median = 1.538172;
		return MedianOf(rotDeltaRad) / (kChi3Median * std::sqrt(2.0));
	}
	double SigmaJitM() { return 1.4826 * MedianOf(transDeltaAbs) / std::sqrt(2.0); }
	double MedianLeverM() { return MedianOf(leverM); }
};

} // namespace

// Env-driven noise estimation over retained/pinned recordings. Measures the
// per-device angular and translational pose jitter that parameterize the
// lever-arm covariance model (sigma_theta, sigma_jit). Prints one
// [noise-estimate] line per recording; tools/Measure-TrackerNoise.ps1 wraps
// this test.
TEST(LeverArmNoiseTest, EstimateFromRecordingsWhenRequested)
{
	if (!EnvFlagLocal("WKOPENVR_NOISE_ESTIMATE", false)) {
		GTEST_SKIP() << "Set WKOPENVR_NOISE_ESTIMATE=1 (with WKOPENVR_REPLAY_PATHS) to estimate tracker noise.";
	}
	const char* rawPaths = std::getenv("WKOPENVR_REPLAY_PATHS");
	if (!rawPaths || !*rawPaths) {
		GTEST_SKIP() << "WKOPENVR_REPLAY_PATHS not set.";
	}

	// Matches spacecal::autolock::kStationaryHmdMps; kept literal so this
	// analysis does not move if the auto-lock gate is retuned.
	constexpr double kStationaryMps = 0.05;
	constexpr double kMaxPairDtSec = 0.5;

	std::string paths = rawPaths;
	std::size_t start = 0;
	int estimated = 0;
	while (start <= paths.size()) {
		std::size_t end = paths.find(';', start);
		if (end == std::string::npos) end = paths.size();
		const std::string path = paths.substr(start, end - start);
		start = end + 1;
		if (path.empty()) continue;

		const auto rec = replay::LoadRecording(path);
		ASSERT_TRUE(rec.error.empty()) << rec.error;

		DeviceNoise ref, target;
		std::size_t stationaryPairs = 0;
		for (std::size_t i = 1; i < rec.rows.size(); ++i) {
			const auto& prev = rec.rows[i - 1];
			const auto& cur = rec.rows[i];
			const double dt = cur.timestamp - prev.timestamp;
			if (dt <= 0.0 || dt > kMaxPairDtSec) continue;
			const double refSpeed = (cur.ref.trans - prev.ref.trans).norm() / dt;
			const double tgtSpeed = (cur.target.trans - prev.target.trans).norm() / dt;
			if (refSpeed >= kStationaryMps || tgtSpeed >= kStationaryMps) continue;
			ref.Push(prev.ref, cur.ref);
			target.Push(prev.target, cur.target);
			++stationaryPairs;
		}

		const std::string name = std::filesystem::path(path).filename().string();
		if (stationaryPairs < 100) {
			std::cout << "[noise-estimate] file=" << name << " rows=" << rec.rows.size()
			          << " stationary_pairs=" << stationaryPairs << " skipped=too_few_stationary_pairs\n";
			continue;
		}
		std::cout << "[noise-estimate] file=" << name << " rows=" << rec.rows.size()
		          << " stationary_pairs=" << stationaryPairs << " ref_sigma_theta_rad=" << ref.SigmaThetaRad()
		          << " ref_sigma_theta_deg=" << ref.SigmaThetaRad() * 180.0 / EIGEN_PI
		          << " ref_sigma_jit_mm=" << ref.SigmaJitM() * 1000.0 << " ref_median_lever_m=" << ref.MedianLeverM()
		          << " tgt_sigma_theta_rad=" << target.SigmaThetaRad()
		          << " tgt_sigma_theta_deg=" << target.SigmaThetaRad() * 180.0 / EIGEN_PI
		          << " tgt_sigma_jit_mm=" << target.SigmaJitM() * 1000.0
		          << " tgt_median_lever_m=" << target.MedianLeverM() << "\n";
		++estimated;
	}
	EXPECT_GT(estimated, 0);
}

// ---------------------------------------------------------------------------
// Covariance model.

namespace {
namespace levercov = spacecal::levercov;

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

// Sample whose per-sample estimate (with relpose S = identity) is exactly `cx`.
Sample ExactSample(const Eigen::AffineCompact3d& ref, const Eigen::AffineCompact3d& cx, double t)
{
	Eigen::AffineCompact3d target = cx.inverse() * ref;
	return Sample(AffineToPose(ref), AffineToPose(target), t);
}

Eigen::Vector3d SolveTranslation(const std::vector<Sample>& samples, CalibrationCalc::RelPoseWeightMode mode, bool& ok)
{
	CalibrationCalc calc;
	calc.lockRelativePosition = true;
	calc.setRelativeTransformation(Eigen::AffineCompact3d::Identity(), true);
	calc.SetRelPoseWeightMode(mode);
	for (const auto& s : samples)
		calc.PushSample(s);
	bool lerp = false;
	ok = calc.ComputeIncremental(lerp, /*threshold=*/1.5, /*relPoseMaxError=*/1.0, /*ignoreOutliers=*/false);
	return calc.Transformation().translation();
}

} // namespace

// Eigenstructure of the single-device covariance: one radial eigenvalue at the
// jitter floor (angular error cannot move a point along its own radius) and
// two tangential eigenvalues at sigma_theta^2 |t|^2 above it.
TEST(LeverArmCovarianceTest, EigenstructureMatchesClosedForm)
{
	const Eigen::Vector3d t(2.0, 0.0, 0.0);
	const double sTheta = 0.003, sJit = 0.001;
	const Eigen::Matrix3d sigma = levercov::TranslationCovariance(t, sTheta, sJit);

	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(sigma);
	const Eigen::Vector3d ev = es.eigenvalues(); // ascending
	EXPECT_NEAR(ev(0), sJit * sJit, 1e-12);
	EXPECT_NEAR(ev(1), sJit * sJit + sTheta * sTheta * t.squaredNorm(), 1e-12);
	EXPECT_NEAR(ev(2), sJit * sJit + sTheta * sTheta * t.squaredNorm(), 1e-12);
	// Smallest-eigenvalue direction is the radius.
	EXPECT_NEAR(std::abs(es.eigenvectors().col(0).dot(t.normalized())), 1.0, 1e-9);
}

// Positive definite at the origin and at an extreme lever arm (LLT succeeds).
TEST(LeverArmCovarianceTest, PositiveDefiniteAcrossLeverArms)
{
	const double sTheta = 0.003, sJit = 0.0007;
	for (const Eigen::Vector3d& t : {Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(10, 0, 0), Eigen::Vector3d(-3, 7, -2)}) {
		const Eigen::Matrix3d sigma = levercov::TranslationCovariance(t, sTheta, sJit);
		Eigen::LLT<Eigen::Matrix3d> llt(sigma);
		EXPECT_EQ(llt.info(), Eigen::Success) << "t=" << t.transpose();
	}
}

// Rotation-equivariance pin: the covariance of a rotated lever arm is the
// rotated covariance. Catches any transposition/block-placement mistake in
// the |t|^2 I - t t^T form.
TEST(LeverArmCovarianceTest, RotationConjugationPin)
{
	const double sTheta = 0.004, sJit = 0.0009;
	const Eigen::Vector3d t(1.3, -0.4, 2.6);
	const Eigen::Matrix3d R = (Eigen::AngleAxisd(0.7, Eigen::Vector3d(0.2, 1.0, -0.5).normalized())).toRotationMatrix();

	const Eigen::Matrix3d lhs = levercov::TranslationCovariance(R * t, sTheta, sJit);
	const Eigen::Matrix3d rhs = R * levercov::TranslationCovariance(t, sTheta, sJit) * R.transpose();
	EXPECT_LT((lhs - rhs).norm(), 1e-12);
}

// Monte-Carlo pin (fixed seed): small random rotations of the tracking frame
// about its origin displace a device at position t with empirical covariance
// matching sigma_theta^2 (|t|^2 I - t t^T).
TEST(LeverArmCovarianceTest, MonteCarloInducedCovarianceMatchesModel)
{
	const Eigen::Vector3d t(3.0, 1.0, -2.0);
	const double sTheta = 0.005;
	std::mt19937 rng(20260710);
	std::normal_distribution<double> gauss(0.0, sTheta);

	Eigen::Matrix3d empirical = Eigen::Matrix3d::Zero();
	const int n = 20000;
	for (int i = 0; i < n; ++i) {
		const Eigen::Vector3d w(gauss(rng), gauss(rng), gauss(rng));
		const double angle = w.norm();
		Eigen::Matrix3d dR = Eigen::Matrix3d::Identity();
		if (angle > 1e-12) dR = Eigen::AngleAxisd(angle, w / angle).toRotationMatrix();
		const Eigen::Vector3d e = dR * t - t;
		empirical += e * e.transpose();
	}
	empirical /= static_cast<double>(n);

	const Eigen::Matrix3d predicted = levercov::TranslationCovariance(t, sTheta, /*sigmaJitM=*/0.0);
	EXPECT_LT((empirical - predicted).norm() / predicted.norm(), 0.15) << "empirical=\n"
	                                                                   << empirical << "\npredicted=\n"
	                                                                   << predicted;
}

// The scalar fallback is exactly the legacy lever weight shape with the
// regularizer expressed through the measured noise ratio: 3/trace(Sigma) is
// proportional to 1/(3*(sJit/sTheta)^2 + r_ref^2 + r_tgt^2).
TEST(LeverArmCovarianceTest, ScalarFallbackMatchesLegacyWeightShape)
{
	const double sTheta = 0.003, sJit = 0.0007;
	const double reg = 3.0 * (sJit / sTheta) * (sJit / sTheta);
	const Eigen::Vector3d refA(0.2, 0.1, 0.0), tgtA(0.3, -0.1, 0.2);
	const Eigen::Vector3d refB(3.0, 0.5, -1.0), tgtB(2.0, 1.5, 0.7);

	const double wA = levercov::ScalarPrecision(levercov::SampleTranslationCovariance(refA, tgtA, sTheta, sJit));
	const double wB = levercov::ScalarPrecision(levercov::SampleTranslationCovariance(refB, tgtB, sTheta, sJit));
	const double legacyA = 1.0 / (reg + refA.squaredNorm() + tgtA.squaredNorm());
	const double legacyB = 1.0 / (reg + refB.squaredNorm() + tgtB.squaredNorm());
	EXPECT_NEAR(wA / wB, legacyA / legacyB, 1e-9);
}

// Mahalanobis normalization: at a long lever arm, the same-magnitude residual
// is far more surprising along the radius (only jitter can produce it) than
// tangentially (angular error routinely produces it).
TEST(LeverArmCovarianceTest, MahalanobisDiscriminatesRadialVsTangential)
{
	const Eigen::Vector3d refT(3.5, 0.0, 0.0);
	const Eigen::Vector3d tgtT(0.1, 0.0, 0.0);
	const double sTheta = 0.003, sJit = 0.0007;

	Eigen::Matrix<double, 6, 1> radial = Eigen::Matrix<double, 6, 1>::Zero();
	radial(0) = 0.01; // 1 cm along the lever arm
	Eigen::Matrix<double, 6, 1> tangential = Eigen::Matrix<double, 6, 1>::Zero();
	tangential(1) = 0.01; // 1 cm perpendicular

	const double d2Radial = levercov::MahalanobisSq(radial, refT, tgtT, sTheta, sJit);
	const double d2Tangential = levercov::MahalanobisSq(tangential, refT, tgtT, sTheta, sJit);
	EXPECT_GT(d2Radial, d2Tangential * 10.0);
}

// Far samples wrong purely tangentially (the shape a fixed angular bias
// produces) cannot drag the covariance-weighted solve; the uniform mean takes
// their full 1/N share. Mirrors PrecisionWeightedRelPoseTest for the
// anisotropic weight.
TEST(LeverArmCovarianceTest, TangentialFarErrorDoesNotDragCovarianceSolve)
{
	const Eigen::AffineCompact3d cTrue = Translate(0.2, 0.0, 0.0);
	// Far samples at +X report a calibration wrong in Y -- tangential to their
	// lever arm.
	const Eigen::AffineCompact3d cWrong = Translate(0.2, 0.2, 0.0);

	std::vector<Sample> samples;
	double t = 0.0;
	for (int i = 0; i < 55; ++i) {
		const double yaw = 0.03 * i;
		const Eigen::Vector3d trans(0.05 * std::sin(0.7 * i), 0.05 * std::cos(0.5 * i), 0.05 * std::sin(0.3 * i));
		samples.push_back(ExactSample(MakeRef(yaw, trans), cTrue, t));
		t += 0.01;
	}
	for (int i = 0; i < 5; ++i) {
		const Eigen::Vector3d trans(3.0 + 0.1 * i, 0.0, 0.0);
		samples.push_back(ExactSample(MakeRef(0.2 * i, trans), cWrong, t));
		t += 0.01;
	}

	bool okUniform = false, okCov = false;
	const Eigen::Vector3d uniform = SolveTranslation(samples, CalibrationCalc::RelPoseWeightMode::Uniform, okUniform);
	const Eigen::Vector3d cov = SolveTranslation(samples, CalibrationCalc::RelPoseWeightMode::Covariance, okCov);
	ASSERT_TRUE(okUniform);
	ASSERT_TRUE(okCov);

	const double uniformErr = (uniform - cTrue.translation()).norm();
	const double covErr = (cov - cTrue.translation()).norm();
	EXPECT_LT(covErr, uniformErr * 0.25) << "cov err=" << covErr << "  uniform err=" << uniformErr;
	EXPECT_LT(covErr, 0.01);
}

// With only exact far samples available, the covariance weight must still
// recover the truth -- far readings are reweighted, never zeroed out.
TEST(LeverArmCovarianceTest, ExactFarSamplesStillRecoverTruth)
{
	const Eigen::AffineCompact3d cTrue = Translate(0.15, -0.1, 0.05);
	std::vector<Sample> samples;
	double t = 0.0;
	for (int i = 0; i < 60; ++i) {
		const Eigen::Vector3d trans(3.0 + 0.05 * std::sin(0.6 * i), 0.5 + 0.05 * std::cos(0.4 * i), -1.0);
		samples.push_back(ExactSample(MakeRef(0.05 * i, trans), cTrue, t));
		t += 0.01;
	}

	bool ok = false;
	const Eigen::Vector3d cov = SolveTranslation(samples, CalibrationCalc::RelPoseWeightMode::Covariance, ok);
	ASSERT_TRUE(ok);
	EXPECT_LT((cov - cTrue.translation()).norm(), 1e-3);
}

// Monte-Carlo estimator comparison (fixed seed, 25 trials): rotational noise
// at mixed lever arms. The covariance weight is the exact inverse-variance
// weight for this noise model, so its RMS error across trials must beat both
// the uniform mean and the isotropic scalar weight. (Per-trial errors can go
// either way on a lucky draw; the RMS comparison is the estimator claim.)
TEST(LeverArmCovarianceTest, CovarianceWeightBeatsUniformAndScalarUnderRotationalNoise)
{
	const Eigen::AffineCompact3d cTrue = Translate(0.2, 0.05, -0.1);
	std::mt19937 rng(42);
	std::normal_distribution<double> gauss(0.0, 0.005); // sigma_theta = 5 mrad

	double sumSqU = 0.0, sumSqS = 0.0, sumSqC = 0.0;
	const int trials = 25;
	for (int trial = 0; trial < trials; ++trial) {
		std::vector<Sample> samples;
		double t = 0.0;
		auto pushNoisy = [&](const Eigen::Vector3d& trans, double yaw) {
			// Angular jitter on the reference device's reported ORIENTATION
			// only (its position stays true). In C = R * S * T^-1 that error
			// multiplies the rest of the chain, displacing the estimate's
			// translation by ~sigma_theta * lever arm, tangentially -- the
			// exact error the covariance weight models.
			const Eigen::AffineCompact3d refTrue = MakeRef(yaw, trans);
			const Eigen::Vector3d w(gauss(rng), gauss(rng), gauss(rng));
			const double angle = w.norm();
			Eigen::Matrix3d dR = Eigen::Matrix3d::Identity();
			if (angle > 1e-12) dR = Eigen::AngleAxisd(angle, w / angle).toRotationMatrix();
			Eigen::AffineCompact3d refObserved = refTrue;
			refObserved.linear() = dR * refTrue.linear();
			const Eigen::AffineCompact3d target = cTrue.inverse() * refTrue;
			samples.push_back(Sample(AffineToPose(refObserved), AffineToPose(target), t));
			t += 0.01;
		};
		for (int i = 0; i < 30; ++i) {
			pushNoisy(Eigen::Vector3d(0.3 * std::sin(0.7 * i), 0.3 * std::cos(0.5 * i), 0.3 * std::sin(0.3 * i)),
			          0.03 * i);
		}
		for (int i = 0; i < 30; ++i) {
			pushNoisy(Eigen::Vector3d(3.5, 0.4 * std::cos(0.4 * i), 0.4 * std::sin(0.6 * i)), 0.05 * i);
		}

		bool okU = false, okS = false, okC = false;
		const Eigen::Vector3d uni = SolveTranslation(samples, CalibrationCalc::RelPoseWeightMode::Uniform, okU);
		const Eigen::Vector3d sca = SolveTranslation(samples, CalibrationCalc::RelPoseWeightMode::ScalarLever, okS);
		const Eigen::Vector3d cov = SolveTranslation(samples, CalibrationCalc::RelPoseWeightMode::Covariance, okC);
		ASSERT_TRUE(okU);
		ASSERT_TRUE(okS);
		ASSERT_TRUE(okC);
		sumSqU += (uni - cTrue.translation()).squaredNorm();
		sumSqS += (sca - cTrue.translation()).squaredNorm();
		sumSqC += (cov - cTrue.translation()).squaredNorm();
	}

	const double rmsU = std::sqrt(sumSqU / trials);
	const double rmsS = std::sqrt(sumSqS / trials);
	const double rmsC = std::sqrt(sumSqC / trials);
	EXPECT_LT(rmsC, rmsU) << "cov=" << rmsC << " uniform=" << rmsU;
	EXPECT_LE(rmsC, rmsS * 1.05) << "cov=" << rmsC << " scalar=" << rmsS;
}
