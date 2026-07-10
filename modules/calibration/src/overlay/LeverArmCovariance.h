#pragma once

// Lever-arm covariance weighting for the relative-pose solve.
//
// Each per-sample calibration estimate C_i = R_i * S * T_i^-1 inherits the
// devices' angular jitter scaled by their distance from the tracking origin:
// a pure angular error sigma_theta on a device at distance r displaces the
// estimate's translation by ~r * sigma_theta, and only in the directions
// perpendicular to the radius. The translation covariance of that error is
//
//   Sigma(t) = sigma_theta^2 * (|t|^2 * I - t * t^T) + sigma_jit^2 * I
//
// -- a rank-2 rotational term (angular error cannot displace a point along
// its own radius) plus an isotropic positional-jitter floor that keeps the
// matrix positive definite. Summing the reference- and target-device terms
// gives the per-sample translation covariance; its inverse is the
// anisotropic weight for the solve's translation average, replacing the
// scalar 1/(0.04 + r_ref^2 + r_tgt^2) approximation (which is the isotropic
// trace of the same model -- kLeverRegM2 = 0.04 m^2 ~ (sigma_jit /
// sigma_theta)^2).
//
// Defaults measured 2026-07-10 with tools/Measure-TrackerNoise.ps1 over the
// stationary stretches of seven retained session recordings
// (spacecal_log.2026-07-02T15-20-28 .. 2026-07-08T00-15-28): median
// sigma_theta 0.0030/0.0031 rad and sigma_jit 0.56/0.85 mm across the
// reference/target devices. Both devices co-move on a head-mounted rig, so
// the medians include real micro head-motion and are upper bounds -- fine
// for weighting, where only the relative scale across lever arms matters.
// Measured ratio check: (0.0007/0.003)^2 = 0.054 m^2, the same order as the
// legacy kLeverRegM2 = 0.04 m^2 regularizer.

#include <Eigen/Dense>

namespace spacecal::levercov {

constexpr double kDefaultSigmaThetaRad = 0.003; // per-axis angular jitter
constexpr double kDefaultSigmaJitterM = 0.0007; // positional jitter floor

// Config-knob clamps: a zero or runaway sigma would degenerate the weights.
constexpr double kSigmaThetaMinRad = 1e-4;
constexpr double kSigmaThetaMaxRad = 0.1;
constexpr double kSigmaJitterMinM = 1e-5;
constexpr double kSigmaJitterMaxM = 0.02;

inline double ClampSigmaTheta(double v)
{
	return v < kSigmaThetaMinRad ? kSigmaThetaMinRad : (v > kSigmaThetaMaxRad ? kSigmaThetaMaxRad : v);
}
inline double ClampSigmaJitter(double v)
{
	return v < kSigmaJitterMinM ? kSigmaJitterMinM : (v > kSigmaJitterMaxM ? kSigmaJitterMaxM : v);
}

// Translation covariance of one device's contribution to the per-sample
// estimate. Symmetric positive definite for sigmaJit > 0.
inline Eigen::Matrix3d TranslationCovariance(const Eigen::Vector3d& t, double sigmaThetaRad, double sigmaJitM)
{
	return sigmaThetaRad * sigmaThetaRad * (t.squaredNorm() * Eigen::Matrix3d::Identity() - t * t.transpose()) +
	       sigmaJitM * sigmaJitM * Eigen::Matrix3d::Identity();
}

// Per-sample translation covariance: reference and target device errors are
// independent, so their covariances add.
inline Eigen::Matrix3d SampleTranslationCovariance(const Eigen::Vector3d& refTrans, const Eigen::Vector3d& tgtTrans,
                                                   double sigmaThetaRad, double sigmaJitM)
{
	return TranslationCovariance(refTrans, sigmaThetaRad, sigmaJitM) +
	       TranslationCovariance(tgtTrans, sigmaThetaRad, sigmaJitM);
}

// Scalar precision for the parts of the solve that cannot consume a matrix
// weight (the quaternion average): the isotropic equivalent 3/trace(Sigma).
inline double ScalarPrecision(const Eigen::Matrix3d& sigma)
{
	const double tr = sigma.trace();
	return tr > 0.0 ? 3.0 / tr : 0.0;
}

// Mahalanobis-squared norm of an SE(3) residual [rho(3); phi(3)] (translation
// first) under the lever-arm noise model. The rotation residual sees both
// devices' angular jitter directly (no lever scaling): Sigma_phi ~
// 2*sigma_theta^2 * I.
inline double MahalanobisSq(const Eigen::Matrix<double, 6, 1>& residual, const Eigen::Vector3d& refTrans,
                            const Eigen::Vector3d& tgtTrans, double sigmaThetaRad, double sigmaJitM)
{
	const Eigen::Vector3d rho = residual.head<3>();
	const Eigen::Vector3d phi = residual.tail<3>();
	const Eigen::Matrix3d sigmaT = SampleTranslationCovariance(refTrans, tgtTrans, sigmaThetaRad, sigmaJitM);
	// Fixed-size closed-form inverse; sigmaT is SPD with a jitter floor, so
	// it is always well conditioned for it.
	const Eigen::Vector3d whitened = sigmaT.inverse() * rho;
	const double rotVar = 2.0 * sigmaThetaRad * sigmaThetaRad;
	return rho.dot(whitened) + phi.squaredNorm() / rotVar;
}

} // namespace spacecal::levercov
