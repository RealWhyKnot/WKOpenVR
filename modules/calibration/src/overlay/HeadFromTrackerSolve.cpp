#include "HeadFromTrackerSolve.h"

#include <Eigen/SVD>
#include <cmath>
#include <limits>

namespace wkopenvr::headmount {

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

namespace {

// Is every component of the matrix finite?
bool IsFinite(const Eigen::Affine3d& t) {
    return t.matrix().allFinite();
}

// Average an array of unit quaternions via the eigenvector method (Markley
// et al., "Averaging Quaternions"). For small counts and well-clustered
// quaternions this is numerically equivalent to the iterative geodesic mean
// but is closed-form and O(N) in samples + O(1) eigen solve on a 4x4 matrix.
//
// Reference: Markley, Cheng, Crassidis, Oshman (2007), Journal of Guidance,
// Control, and Dynamics 30(4).
Eigen::Quaterniond AverageQuaternions(
    const std::vector<Eigen::Quaterniond>& qs)
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

} // namespace

// ---------------------------------------------------------------------------
// Solver
// ---------------------------------------------------------------------------

void Solver::Start() {
    m_pairs.clear();
    m_result  = {};
    m_state   = SolveState::Collecting;
}

void Solver::Cancel() {
    m_pairs.clear();
    m_result  = {};
    m_state   = SolveState::Idle;
}

bool Solver::Tick(const Eigen::Affine3d& hmdPose,
                  const Eigen::Affine3d& trackerPose,
                  double hmdSpeedMps)
{
    if (m_state != SolveState::Collecting) return false;

    // Require motion so the rotational component of T is constrained.
    if (hmdSpeedMps < kMinHmdSpeedMps) return false;

    // Discard degenerate / NaN poses from tracking glitches.
    if (!IsFinite(hmdPose) || !IsFinite(trackerPose)) return false;

    m_pairs.emplace_back(hmdPose, trackerPose);
    return true;
}

void Solver::Finish() {
    if (m_state != SolveState::Collecting) return;

    m_state = SolveState::Solving;

    if (m_pairs.size() < kTargetSampleCount) {
        m_result.failReason = "not enough motion";
        m_result.samplesUsed = static_cast<int>(m_pairs.size());
        m_state = SolveState::Failed;
        return;
    }

    // --- Rotation solve ---------------------------------------------------
    //
    // For each pair compute the relative transform T_i = tracker_i^-1 * hmd_i.
    // The rotation components should all be the same rigid T; average them to
    // suppress per-sample noise.

    std::vector<Eigen::Quaterniond> relRotations;
    relRotations.reserve(m_pairs.size());

    for (const auto& [hmd, tracker] : m_pairs) {
        Eigen::Affine3d rel = tracker.inverse() * hmd;
        Eigen::Quaterniond q(rel.rotation());
        // Enforce consistent hemisphere to avoid sign-flip averaging artefacts.
        if (q.w() < 0.0) q.coeffs() = -q.coeffs();
        relRotations.push_back(q.normalized());
    }

    Eigen::Quaterniond avgRot = AverageQuaternions(relRotations);
    Eigen::Matrix3d R = avgRot.toRotationMatrix();

    // --- Translation solve -----------------------------------------------
    //
    // From hmd_i = tracker_i * T:
    //
    //   hmd_i.trans = tracker_i.rot * T.trans + tracker_i.trans
    //
    // => T.trans = tracker_i.rot^T * (hmd_i.trans - tracker_i.trans)
    //
    // This holds per-sample; average across all pairs to suppress noise.
    // Note: T.trans is entirely determined by the tracker rotation and the
    // position residual; the averaged T.rot (R) does NOT enter this equation.

    Eigen::Vector3d sumT = Eigen::Vector3d::Zero();
    for (const auto& [hmd, tracker] : m_pairs) {
        sumT += tracker.rotation().transpose() * (hmd.translation() - tracker.translation());
    }
    Eigen::Vector3d t = sumT / static_cast<double>(m_pairs.size());

    // Assemble candidate T.
    Eigen::AffineCompact3d T;
    T.linear()      = R;
    T.translation() = t;

    // --- Residual check ---------------------------------------------------
    double sumResidualSq = 0.0;
    for (const auto& [hmd, tracker] : m_pairs) {
        Eigen::Affine3d trackA(tracker);
        Eigen::Vector3d predicted = (trackA * T).translation();
        sumResidualSq += (hmd.translation() - predicted).squaredNorm();
    }
    const double residualMm = std::sqrt(sumResidualSq / static_cast<double>(m_pairs.size())) * 1000.0;

    if (residualMm > kResidualThresholdMm) {
        m_result.failReason = "mount may be slipping";
        m_result.residualMm = residualMm;
        m_result.samplesUsed = static_cast<int>(m_pairs.size());
        m_state = SolveState::Failed;
        return;
    }

    m_result.headFromTracker = T;
    m_result.residualMm      = residualMm;
    m_result.samplesUsed     = static_cast<int>(m_pairs.size());
    m_result.failReason      = {};
    m_state = SolveState::Done;
}

} // namespace wkopenvr::headmount
