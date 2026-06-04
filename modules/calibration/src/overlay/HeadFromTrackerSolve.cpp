#include "HeadFromTrackerSolve.h"

#include <Eigen/SVD>
#include <algorithm>
#include <cmath>
#include <limits>

namespace wkopenvr::headmount {

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

namespace {

struct ComputedSolve
{
	Eigen::AffineCompact3d headFromTracker = Eigen::AffineCompact3d::Identity();
	double residualMm = std::numeric_limits<double>::infinity();
};

// Is every component of the matrix finite?
bool IsFinite(const Eigen::Affine3d& t)
{
	return t.matrix().allFinite();
}

// Average an array of unit quaternions via the eigenvector method (Markley
// et al., "Averaging Quaternions"). For small counts and well-clustered
// quaternions this is numerically equivalent to the iterative geodesic mean
// but is closed-form and O(N) in samples + O(1) eigen solve on a 4x4 matrix.
//
// Reference: Markley, Cheng, Crassidis, Oshman (2007), Journal of Guidance,
// Control, and Dynamics 30(4).
Eigen::Quaterniond AverageQuaternions(const std::vector<Eigen::Quaterniond>& qs)
{
	// Accumulate the symmetric 4x4 outer-product matrix M = sum(q_i q_i^T).
	Eigen::Matrix4d M = Eigen::Matrix4d::Zero();
	for (const auto& q : qs) {
		Eigen::Vector4d v(q.w(), q.x(), q.y(), q.z());
		M += v * v.transpose();
	}
	M /= static_cast<double>(qs.size());

	// The average rotation is the eigenvector of M with the largest eigenvalue.
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> eig(M);
	// SelfAdjointEigenSolver sorts eigenvalues ascending; take the last column.
	Eigen::Vector4d v = eig.eigenvectors().col(3);
	// Convention: (w, x, y, z).
	return Eigen::Quaterniond(v(0), v(1), v(2), v(3)).normalized();
}

ComputedSolve ComputeSolve(const std::vector<std::pair<Eigen::Affine3d, Eigen::Affine3d>>& pairs)
{
	ComputedSolve out;
	if (pairs.empty()) {
		return out;
	}

	std::vector<Eigen::Quaterniond> relRotations;
	relRotations.reserve(pairs.size());

	for (const auto& pair : pairs) {
		const Eigen::Affine3d& hmd = pair.first;
		const Eigen::Affine3d& tracker = pair.second;
		Eigen::Affine3d rel = tracker.inverse() * hmd;
		Eigen::Quaterniond q(rel.rotation());
		// Enforce consistent hemisphere to avoid sign-flip averaging artefacts.
		if (q.w() < 0.0) q.coeffs() = -q.coeffs();
		relRotations.push_back(q.normalized());
	}

	Eigen::Quaterniond avgRot = AverageQuaternions(relRotations);
	Eigen::Matrix3d R = avgRot.toRotationMatrix();

	Eigen::Vector3d sumT = Eigen::Vector3d::Zero();
	for (const auto& pair : pairs) {
		const Eigen::Affine3d& hmd = pair.first;
		const Eigen::Affine3d& tracker = pair.second;
		sumT += tracker.rotation().transpose() * (hmd.translation() - tracker.translation());
	}
	Eigen::Vector3d t = sumT / static_cast<double>(pairs.size());

	Eigen::AffineCompact3d T;
	T.linear() = R;
	T.translation() = t;

	double sumResidualSq = 0.0;
	for (const auto& pair : pairs) {
		const Eigen::Affine3d& hmd = pair.first;
		const Eigen::Affine3d& tracker = pair.second;
		Eigen::Affine3d trackA(tracker);
		Eigen::Vector3d predicted = (trackA * T).translation();
		sumResidualSq += (hmd.translation() - predicted).squaredNorm();
	}

	out.headFromTracker = T;
	out.residualMm = std::sqrt(sumResidualSq / static_cast<double>(pairs.size())) * 1000.0;
	return out;
}

std::array<double, 3> RotationAxisRangesDeg(const std::vector<std::pair<Eigen::Affine3d, Eigen::Affine3d>>& pairs)
{
	std::array<double, 3> ranges{0.0, 0.0, 0.0};
	if (pairs.size() < 2) {
		return ranges;
	}

	const Eigen::Matrix3d firstInv = pairs.front().second.rotation().transpose();
	std::array<double, 3> minV{0.0, 0.0, 0.0};
	std::array<double, 3> maxV{0.0, 0.0, 0.0};
	bool initialized = false;

	for (const auto& pair : pairs) {
		const Eigen::Matrix3d rel = firstInv * pair.second.rotation();
		Eigen::AngleAxisd aa(rel);
		Eigen::Vector3d rotVecDeg = aa.axis() * aa.angle() * (180.0 / EIGEN_PI);

		if (!rotVecDeg.allFinite()) {
			continue;
		}

		if (!initialized) {
			for (int i = 0; i < 3; ++i) {
				minV[i] = rotVecDeg(i);
				maxV[i] = rotVecDeg(i);
			}
			initialized = true;
			continue;
		}

		for (int i = 0; i < 3; ++i) {
			minV[i] = std::min(minV[i], rotVecDeg(i));
			maxV[i] = std::max(maxV[i], rotVecDeg(i));
		}
	}

	if (!initialized) {
		return ranges;
	}

	// Solver UI labels these as pitch/yaw/roll. The coordinate convention here
	// matches the synthetic poses and the in-app fine-adjustment sliders.
	ranges[0] = maxV[0] - minV[0];
	ranges[1] = maxV[1] - minV[1];
	ranges[2] = maxV[2] - minV[2];
	return ranges;
}

} // namespace

// ---------------------------------------------------------------------------
// Solver
// ---------------------------------------------------------------------------

void Solver::Start()
{
	m_pairs.clear();
	m_result = {};
	m_state = SolveState::Collecting;
}

void Solver::Cancel()
{
	m_pairs.clear();
	m_result = {};
	m_state = SolveState::Idle;
}

bool Solver::Tick(const Eigen::Affine3d& hmdPose, const Eigen::Affine3d& trackerPose, double hmdSpeedMps)
{
	if (m_state != SolveState::Collecting) return false;

	// Require motion so the rotational component of T is constrained.
	if (hmdSpeedMps < kMinHmdSpeedMps) return false;

	// Discard degenerate / NaN poses from tracking glitches.
	if (!IsFinite(hmdPose) || !IsFinite(trackerPose)) return false;

	m_pairs.emplace_back(hmdPose, trackerPose);
	return true;
}

void Solver::Finish()
{
	if (m_state != SolveState::Collecting) return;

	m_state = SolveState::Solving;

	const CollectionReadiness r = readiness();
	if (!r.enoughSamples || !r.enoughMotion) {
		m_result.failReason = "not enough motion";
		m_result.samplesUsed = static_cast<int>(m_pairs.size());
		m_state = SolveState::Failed;
		return;
	}

	const ComputedSolve solve = ComputeSolve(m_pairs);
	const double residualMm = solve.residualMm;

	if (residualMm > kResidualThresholdMm) {
		m_result.failReason = "mount may be slipping";
		m_result.residualMm = residualMm;
		m_result.samplesUsed = static_cast<int>(m_pairs.size());
		m_state = SolveState::Failed;
		return;
	}

	m_result.headFromTracker = solve.headFromTracker;
	m_result.residualMm = residualMm;
	m_result.samplesUsed = static_cast<int>(m_pairs.size());
	m_result.failReason = {};
	m_state = SolveState::Done;
}

CollectionReadiness Solver::readiness() const
{
	CollectionReadiness r;
	r.samplesUsed = m_pairs.size();
	r.sampleScore = std::min(1.0, static_cast<double>(m_pairs.size()) / static_cast<double>(kMinReadySampleCount));
	r.enoughSamples = m_pairs.size() >= kMinReadySampleCount;

	r.axisRangeDeg = RotationAxisRangesDeg(m_pairs);
	for (int i = 0; i < 3; ++i) {
		r.axisScore[i] = std::min(1.0, r.axisRangeDeg[i] / kAxisRangeTargetDeg);
	}
	r.motionScore = std::min(r.axisScore[0], std::min(r.axisScore[1], r.axisScore[2]));
	r.enoughMotion = r.motionScore >= 1.0;

	if (m_pairs.size() >= 2) {
		const ComputedSolve solve = ComputeSolve(m_pairs);
		r.residualMm = solve.residualMm;
		if (std::isfinite(r.residualMm)) {
			r.residualScore = std::clamp((kResidualThresholdMm * 2.0 - r.residualMm) / kResidualThresholdMm, 0.0, 1.0);
			r.residualGood = r.residualMm <= kResidualThresholdMm;
		}
	}

	r.overallScore = std::min(r.sampleScore, std::min(r.motionScore, r.residualScore));
	r.ready = r.enoughSamples && r.enoughMotion && r.residualGood;
	return r;
}

} // namespace wkopenvr::headmount
