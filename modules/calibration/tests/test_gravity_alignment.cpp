// Gravity-constrained calibration rotation (GravityAlignment.h): the
// yaw-about-+Y projection must strip roll/pitch exactly, preserve yaw, and
// leave translation untouched.

#include "GravityAlignment.h"

#include <gtest/gtest.h>

#include <Eigen/Geometry>

namespace gravity = spacecal::gravity;

namespace {
Eigen::Quaterniond Yaw(double rad)
{
	return Eigen::Quaterniond(Eigen::AngleAxisd(rad, Eigen::Vector3d::UnitY()));
}
} // namespace

TEST(GravityAlignmentTest, PureYawIsUnchanged)
{
	for (double yaw : {-2.5, -0.7, 0.0, 0.4, 3.0}) {
		const Eigen::Quaterniond q = Yaw(yaw);
		EXPECT_LT(gravity::YawTwist(q).angularDistance(q), 1e-12) << yaw;
	}
}

TEST(GravityAlignmentTest, RollPitchIsRemovedYawPreserved)
{
	const double yaw = 0.8;
	const Eigen::Quaterniond noisy = Yaw(yaw) * Eigen::AngleAxisd(0.05, Eigen::Vector3d::UnitX()) *
	                                 Eigen::AngleAxisd(-0.03, Eigen::Vector3d::UnitZ());
	const Eigen::Quaterniond projected = gravity::YawTwist(noisy);
	// Result rotates only about +Y...
	EXPECT_NEAR(projected.x(), 0.0, 1e-12);
	EXPECT_NEAR(projected.z(), 0.0, 1e-12);
	// ...and stays close to the true yaw (swing-twist keeps the axis
	// component; small roll/pitch perturbs the recovered yaw only at second
	// order).
	EXPECT_LT(projected.angularDistance(Yaw(yaw)), 0.01);
}

TEST(GravityAlignmentTest, ProjectionKeepsTranslationAndHandlesDegenerate)
{
	Eigen::AffineCompact3d c(Yaw(1.1) * Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitX()));
	c.translation() = Eigen::Vector3d(1.0, -2.0, 3.5);
	const Eigen::AffineCompact3d out = gravity::ProjectRotationToYaw(c);
	EXPECT_LT((out.translation() - c.translation()).norm(), 1e-15);
	EXPECT_NEAR(Eigen::Quaterniond(out.rotation()).x(), 0.0, 1e-12);
	EXPECT_NEAR(Eigen::Quaterniond(out.rotation()).z(), 0.0, 1e-12);

	// Degenerate: a pure 180-degree roll has no yaw component; projection
	// falls back to identity instead of dividing by zero.
	const Eigen::Quaterniond roll180(Eigen::AngleAxisd(EIGEN_PI, Eigen::Vector3d::UnitX()));
	EXPECT_LT(gravity::YawTwist(roll180).angularDistance(Eigen::Quaterniond::Identity()), 1e-12);
}
