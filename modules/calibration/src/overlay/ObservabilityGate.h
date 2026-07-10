#pragma once

// Observability gate for the continuous relative-pose solve.
//
// A rigid ref/target pair constrains the calibration only along the rotation
// axes it has actually exercised: two poses related by rotations about a
// single axis leave the components about the orthogonal axes unobservable
// (the classic two-non-parallel-motions requirement of hand-eye calibration).
// A head-mounted pair spends most of its time turning about gravity, so the
// solve's yaw-coupled components are well constrained while pitch/roll-coupled
// ones float on noise -- and the fit residual cannot tell the difference.
//
// The gate accumulates the excitation matrix M = sum_i a_i a_i^T over the
// unit rotation axes a_i of consecutive reference-pose deltas in the sample
// window. Its eigenvalues count (softly) how many delta-axes excite each
// direction; an eigenvalue below kLambdaMin marks a direction the window has
// not observed. Candidate updates are projected onto the observed subspace
// and the unobserved component of the delta is held at the prior.
//
// Recomputed fresh from the sample window each solve (n <= a few hundred, one
// quaternion product + rank-1 update per pair), so there is no rolling state
// to reset.

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <cmath>

namespace spacecal::observability {

// Deltas below this angle are jitter, not excitation.
constexpr double kAxisMinDeltaRad = 0.2 * EIGEN_PI / 180.0;

// Minimum eigenvalue of M (~ number of window deltas exciting a direction)
// for that direction to count as observed. Replay-tuned starting point.
constexpr double kLambdaMin = 3.0;

struct Excitation
{
	Eigen::Matrix3d M = Eigen::Matrix3d::Zero();
	// Ascending, from SelfAdjointEigenSolver.
	Eigen::Vector3d eigenvalues = Eigen::Vector3d::Zero();
	Eigen::Matrix3d eigenvectors = Eigen::Matrix3d::Identity();
	int deltasCounted = 0;

	double LambdaMin() const { return eigenvalues(0); }
};

// Accumulate M over consecutive valid samples' reference-rotation deltas.
// `Samples` is any iterable of objects with `.valid` and `.ref.rot`
// (Eigen::Matrix3d), i.e. the solver's sample deque.
template <typename Samples> inline Excitation AccumulateExcitation(const Samples& samples)
{
	Excitation ex;
	bool havePrev = false;
	Eigen::Quaterniond prev = Eigen::Quaterniond::Identity();
	for (const auto& sample : samples) {
		if (!sample.valid) continue;
		const Eigen::Quaterniond cur(sample.ref.rot);
		if (havePrev) {
			Eigen::Quaterniond dq = cur * prev.conjugate();
			if (dq.w() < 0.0) dq.coeffs() = -dq.coeffs();
			const double angle = 2.0 * std::acos(std::min(1.0, dq.w()));
			const double vecNorm = dq.vec().norm();
			if (angle >= kAxisMinDeltaRad && vecNorm > 1e-12) {
				const Eigen::Vector3d a = dq.vec() / vecNorm;
				ex.M += a * a.transpose();
				++ex.deltasCounted;
			}
		}
		prev = cur;
		havePrev = true;
	}
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(ex.M);
	ex.eigenvalues = es.eigenvalues();
	ex.eigenvectors = es.eigenvectors();
	return ex;
}

// Project a delta vector onto the observed subspace: components along
// eigenvectors whose eigenvalue is below `lambdaMin` are dropped (held at
// the prior by the caller).
inline Eigen::Vector3d ProjectToStrongSubspace(const Eigen::Vector3d& v, const Excitation& ex, double lambdaMin)
{
	Eigen::Vector3d out = Eigen::Vector3d::Zero();
	for (int i = 0; i < 3; ++i) {
		if (ex.eigenvalues(i) < lambdaMin) continue;
		const Eigen::Vector3d axis = ex.eigenvectors.col(i);
		out += axis * axis.dot(v);
	}
	return out;
}

// True when at least one direction is below the threshold (some component of
// a candidate delta would be held).
inline bool HasWeakDirection(const Excitation& ex, double lambdaMin)
{
	return ex.eigenvalues(0) < lambdaMin;
}

} // namespace spacecal::observability
