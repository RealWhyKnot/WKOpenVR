// Observability-gate tests (ObservabilityGate.h): the excitation matrix over
// window rotation deltas, the weak-direction detection, and the strong-
// subspace projection that holds unobserved components of a candidate update.

#include <gtest/gtest.h>

#include "ObservabilityGate.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <vector>

namespace obs = spacecal::observability;

namespace {

// Minimal window row: the accumulator template only touches .valid and .ref.rot.
struct MiniSample
{
	bool valid = true;
	struct
	{
		Eigen::Matrix3d rot;
	} ref;
};

std::vector<MiniSample> RotationsAbout(const Eigen::Vector3d& axis, int count, double stepRad)
{
	std::vector<MiniSample> window;
	for (int i = 0; i < count; ++i) {
		MiniSample s;
		s.ref.rot = Eigen::AngleAxisd(stepRad * i, axis.normalized()).toRotationMatrix();
		window.push_back(s);
	}
	return window;
}

} // namespace

// Motion about a single axis produces a rank-1 excitation matrix: the excited
// direction carries all the weight, the two orthogonal directions read ~0.
TEST(ObservabilityGateTest, SingleAxisMotionIsRankOne)
{
	const auto window = RotationsAbout(Eigen::Vector3d::UnitY(), 40, 1.0 * EIGEN_PI / 180.0);
	const obs::Excitation ex = obs::AccumulateExcitation(window);

	EXPECT_EQ(ex.deltasCounted, 39);
	EXPECT_NEAR(ex.eigenvalues(0), 0.0, 1e-9);
	EXPECT_NEAR(ex.eigenvalues(1), 0.0, 1e-9);
	EXPECT_NEAR(ex.eigenvalues(2), 39.0, 1e-9);
	EXPECT_TRUE(obs::HasWeakDirection(ex, obs::kLambdaMin));
	// Strongest eigenvector is the excited axis.
	EXPECT_NEAR(std::abs(ex.eigenvectors.col(2).dot(Eigen::Vector3d::UnitY())), 1.0, 1e-9);
}

// Under single-axis excitation, a delta along the excited axis passes and a
// delta orthogonal to it is held.
TEST(ObservabilityGateTest, ProjectionHoldsOrthogonalDelta)
{
	const auto window = RotationsAbout(Eigen::Vector3d::UnitY(), 40, 1.0 * EIGEN_PI / 180.0);
	const obs::Excitation ex = obs::AccumulateExcitation(window);

	const Eigen::Vector3d alongY(0.0, 0.02, 0.0);
	const Eigen::Vector3d alongX(0.02, 0.0, 0.0);
	EXPECT_LT((obs::ProjectToStrongSubspace(alongY, ex, obs::kLambdaMin) - alongY).norm(), 1e-9);
	EXPECT_LT(obs::ProjectToStrongSubspace(alongX, ex, obs::kLambdaMin).norm(), 1e-9);
}

// Excitation about all three axes makes the projection the identity.
TEST(ObservabilityGateTest, FullExcitationPassesEverything)
{
	// Three blocks of pure incremental rotation, one per axis, so the
	// consecutive deltas excite X, then Y, then Z (~19 deltas each).
	std::vector<MiniSample> window;
	for (int axis = 0; axis < 3; ++axis) {
		const Eigen::Vector3d a =
		    axis == 0 ? Eigen::Vector3d::UnitX() : (axis == 1 ? Eigen::Vector3d::UnitY() : Eigen::Vector3d::UnitZ());
		for (int i = 0; i < 20; ++i) {
			MiniSample s;
			s.ref.rot = Eigen::AngleAxisd(0.02 * i, a).toRotationMatrix();
			window.push_back(s);
		}
	}
	const obs::Excitation ex = obs::AccumulateExcitation(window);
	ASSERT_FALSE(obs::HasWeakDirection(ex, obs::kLambdaMin)) << "lambda_min=" << ex.LambdaMin();

	const Eigen::Vector3d v(0.013, -0.007, 0.021);
	EXPECT_LT((obs::ProjectToStrongSubspace(v, ex, obs::kLambdaMin) - v).norm(), 1e-9);
}

// Sub-threshold deltas are jitter, not excitation: a window of rotations
// smaller than kAxisMinDeltaRad counts zero deltas.
TEST(ObservabilityGateTest, SubThresholdDeltasIgnored)
{
	const auto window = RotationsAbout(Eigen::Vector3d::UnitY(), 40, 0.05 * EIGEN_PI / 180.0);
	const obs::Excitation ex = obs::AccumulateExcitation(window);
	EXPECT_EQ(ex.deltasCounted, 0);
	EXPECT_TRUE(obs::HasWeakDirection(ex, obs::kLambdaMin));
}

// Invalid samples do not contribute deltas (and do not bridge a delta across
// the gap either -- the previous-quaternion carry skips them entirely).
TEST(ObservabilityGateTest, InvalidSamplesSkipped)
{
	auto window = RotationsAbout(Eigen::Vector3d::UnitY(), 10, 1.0 * EIGEN_PI / 180.0);
	for (auto& s : window)
		s.valid = false;
	const obs::Excitation ex = obs::AccumulateExcitation(window);
	EXPECT_EQ(ex.deltasCounted, 0);
}

// Determinism: the same window accumulates the same eigen-decomposition.
TEST(ObservabilityGateTest, Deterministic)
{
	const auto window = RotationsAbout(Eigen::Vector3d(1.0, 1.0, 0.0), 30, 0.8 * EIGEN_PI / 180.0);
	const obs::Excitation a = obs::AccumulateExcitation(window);
	const obs::Excitation b = obs::AccumulateExcitation(window);
	EXPECT_EQ((a.M - b.M).norm(), 0.0);
	EXPECT_EQ((a.eigenvalues - b.eigenvalues).norm(), 0.0);
}
