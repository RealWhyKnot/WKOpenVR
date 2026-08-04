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

TEST(GravityAlignmentTest, SwingTwistRecomposes)
{
	const Eigen::Quaterniond q = Yaw(1.3) * Eigen::AngleAxisd(0.07, Eigen::Vector3d::UnitX()) *
	                             Eigen::AngleAxisd(-0.04, Eigen::Vector3d::UnitZ());
	const Eigen::Quaterniond recomposed = gravity::SwingOf(q) * gravity::YawTwist(q);
	EXPECT_LT(recomposed.angularDistance(q), 1e-12);
	// The swing axis is horizontal (no yaw content).
	const Eigen::Quaterniond swing = gravity::SwingOf(q);
	EXPECT_NEAR(gravity::YawTwist(swing).angularDistance(Eigen::Quaterniond::Identity()), 0.0, 1e-9);
}

TEST(GravityAlignmentTest, TiltAngleMatchesInjectedTilt)
{
	for (double tiltRad : {0.0, 0.01, 0.05, 0.14}) {
		const Eigen::Quaterniond q = Yaw(-0.9) * Eigen::AngleAxisd(tiltRad, Eigen::Vector3d::UnitX());
		EXPECT_NEAR(gravity::TiltAngleRad(q), tiltRad, 2e-4) << tiltRad;
	}
	EXPECT_NEAR(gravity::TiltAngleDeg(Yaw(0.5)), 0.0, 1e-9);
}

TEST(GravityAlignmentTest, ClampTiltCapsAndPreservesYawTranslation)
{
	const double yaw = 0.6;
	const double bigTilt = 8.0 * EIGEN_PI / 180.0;
	Eigen::AffineCompact3d c(Yaw(yaw) * Eigen::AngleAxisd(bigTilt, Eigen::Vector3d::UnitZ()));
	c.translation() = Eigen::Vector3d(0.4, 2.0, -1.1);

	const Eigen::AffineCompact3d out = gravity::ClampTilt(c, gravity::kMaxTiltRad);
	const Eigen::Quaterniond outQ(out.rotation());
	EXPECT_NEAR(gravity::TiltAngleRad(outQ), gravity::kMaxTiltRad, 1e-6);
	// Yaw component unchanged; translation untouched.
	EXPECT_LT(gravity::YawTwist(outQ).angularDistance(gravity::YawTwist(Eigen::Quaterniond(c.rotation()))), 1e-6);
	EXPECT_LT((out.translation() - c.translation()).norm(), 1e-15);

	// Under-cap input passes through bit-identically.
	Eigen::AffineCompact3d small(Yaw(yaw) * Eigen::AngleAxisd(0.01, Eigen::Vector3d::UnitX()));
	small.translation() = Eigen::Vector3d(1, 2, 3);
	const Eigen::AffineCompact3d same = gravity::ClampTilt(small, gravity::kMaxTiltRad);
	EXPECT_LT(Eigen::Quaterniond(same.rotation()).angularDistance(Eigen::Quaterniond(small.rotation())), 1e-15);
}

TEST(GravityAlignmentTest, DampTiltMovesSwingByAlphaOnly)
{
	// Prior: level. Candidate: same yaw path but tilted 2 deg, moved 10 cm.
	const double tilt = 2.0 * EIGEN_PI / 180.0;
	Eigen::AffineCompact3d prior(Yaw(0.3));
	prior.translation() = Eigen::Vector3d(0, 0, 0);
	Eigen::AffineCompact3d cand(Yaw(0.9) * Eigen::AngleAxisd(tilt, Eigen::Vector3d::UnitX()));
	cand.translation() = Eigen::Vector3d(0.1, 0.0, 0.0);

	const double alpha = 0.25;
	const Eigen::AffineCompact3d out = gravity::DampTilt(prior, cand, alpha, gravity::kMaxTiltRad);
	const Eigen::Quaterniond outQ(out.rotation());
	// Yaw + translation come from the candidate at full speed...
	EXPECT_LT(gravity::YawTwist(outQ).angularDistance(gravity::YawTwist(Eigen::Quaterniond(cand.rotation()))), 1e-9);
	EXPECT_LT((out.translation() - cand.translation()).norm(), 1e-15);
	// ...while the tilt only moved a quarter of the way.
	EXPECT_NEAR(gravity::TiltAngleRad(outQ), alpha * tilt, 2e-4);

	// alpha=1 adopts the candidate outright; alpha=0 keeps the prior swing.
	EXPECT_NEAR(
	    gravity::TiltAngleRad(Eigen::Quaterniond(gravity::DampTilt(prior, cand, 1.0, gravity::kMaxTiltRad).rotation())),
	    tilt, 2e-4);
	EXPECT_NEAR(
	    gravity::TiltAngleRad(Eigen::Quaterniond(gravity::DampTilt(prior, cand, 0.0, gravity::kMaxTiltRad).rotation())),
	    0.0, 1e-9);
}
