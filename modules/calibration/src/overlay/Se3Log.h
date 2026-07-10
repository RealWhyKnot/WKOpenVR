#pragma once

// SE(3) logarithm / exponential maps. Vector ordering is [rho(3); phi(3)] --
// TRANSLATION FIRST -- pinned by unit test; every consumer (the sequential
// validation residual, its Mahalanobis normalization) assumes it.
//
//   Exp([rho; phi]) = ( R = exp([phi]x),  t = V(phi) * rho )
//   V(phi) = I + (1-cos th)/th^2 [phi]x + (th - sin th)/th^3 [phi]x^2
//   Log(T) = [ V^-1(phi) * t ; phi ],  phi = angle*axis of R
//
// Taylor branches below th < 1e-5 keep the small-angle limit exact.

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <cmath>

namespace spacecal::se3 {

inline Eigen::Matrix3d Skew(const Eigen::Vector3d& v)
{
	Eigen::Matrix3d m;
	m << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
	return m;
}

inline Eigen::AffineCompact3d ExpSE3(const Eigen::Matrix<double, 6, 1>& xi)
{
	const Eigen::Vector3d rho = xi.head<3>();
	const Eigen::Vector3d phi = xi.tail<3>();
	const double theta = phi.norm();
	const Eigen::Matrix3d W = Skew(phi);

	Eigen::Matrix3d R, V;
	if (theta < 1e-5) {
		R = Eigen::Matrix3d::Identity() + W + 0.5 * W * W;
		V = Eigen::Matrix3d::Identity() + 0.5 * W + (1.0 / 6.0) * W * W;
	}
	else {
		R = Eigen::AngleAxisd(theta, phi / theta).toRotationMatrix();
		V = Eigen::Matrix3d::Identity() + (1.0 - std::cos(theta)) / (theta * theta) * W +
		    (theta - std::sin(theta)) / (theta * theta * theta) * W * W;
	}

	Eigen::AffineCompact3d out;
	out.linear() = R;
	out.translation() = V * rho;
	return out;
}

inline Eigen::Matrix<double, 6, 1> LogSE3(const Eigen::AffineCompact3d& T)
{
	const Eigen::AngleAxisd aa(Eigen::Matrix3d(T.rotation()));
	const double theta = aa.angle();
	const Eigen::Vector3d phi = theta * aa.axis();
	const Eigen::Matrix3d W = Skew(phi);

	Eigen::Matrix3d Vinv;
	if (theta < 1e-5) {
		Vinv = Eigen::Matrix3d::Identity() - 0.5 * W + (1.0 / 12.0) * W * W;
	}
	else {
		// (theta/2) * cot(theta/2)
		const double halfCot = 0.5 * theta / std::tan(0.5 * theta);
		Vinv = Eigen::Matrix3d::Identity() - 0.5 * W + (1.0 - halfCot) / (theta * theta) * W * W;
	}

	Eigen::Matrix<double, 6, 1> out;
	out.head<3>() = Vinv * T.translation();
	out.tail<3>() = phi;
	return out;
}

} // namespace spacecal::se3
