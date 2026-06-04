#pragma once

#include "Boundary.h"
#include "BoundaryPreview.h"

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace boundary_test {

constexpr double kDoubleTolerance = 1e-9;
constexpr float kFloatTolerance = 1e-6f;

inline Eigen::Affine3d MakePose(double x, double y, double z)
{
	Eigen::Affine3d pose = Eigen::Affine3d::Identity();
	pose.translation() = Eigen::Vector3d(x, y, z);
	return pose;
}

inline Eigen::Affine3d MakeFloorPointerPose(double x, double z, double y = 1.0)
{
	Eigen::Affine3d pose = MakePose(x, y, z);
	pose.linear() = Eigen::AngleAxisd(-EIGEN_PI / 2.0, Eigen::Vector3d::UnitX()).toRotationMatrix();
	return pose;
}

inline vr::HmdMatrix34_t MakeIdentityMatrix34()
{
	vr::HmdMatrix34_t m{};
	m.m[0][0] = 1.0f;
	m.m[1][1] = 1.0f;
	m.m[2][2] = 1.0f;
	return m;
}

inline void SetTranslation(vr::HmdMatrix34_t& m, float x, float y, float z)
{
	m.m[0][3] = x;
	m.m[1][3] = y;
	m.m[2][3] = z;
}

inline Eigen::Affine3d Matrix34ToAffine(const vr::HmdMatrix34_t& m)
{
	Eigen::Affine3d affine = Eigen::Affine3d::Identity();
	affine.linear() << m.m[0][0], m.m[0][1], m.m[0][2], m.m[1][0], m.m[1][1], m.m[1][2], m.m[2][0], m.m[2][1],
	    m.m[2][2];
	affine.translation() = Eigen::Vector3d(m.m[0][3], m.m[1][3], m.m[2][3]);
	return affine;
}

inline Eigen::Vector3d TransformStandingPointToRaw(const vr::HmdMatrix34_t& standingToRaw, double x, double y, double z)
{
	return Matrix34ToAffine(standingToRaw) * Eigen::Vector3d(x, y, z);
}

inline void ExpectVertexNear(const BoundaryVertex& actual, double x, double y, double z,
                             double tolerance = kDoubleTolerance)
{
	EXPECT_NEAR(actual.x, x, tolerance);
	EXPECT_NEAR(actual.y, y, tolerance);
	EXPECT_NEAR(actual.z, z, tolerance);
}

inline void ExpectMatrixTranslationNear(const vr::HmdMatrix34_t& actual, float x, float y, float z,
                                        float tolerance = kFloatTolerance)
{
	EXPECT_NEAR(actual.m[0][3], x, tolerance);
	EXPECT_NEAR(actual.m[1][3], y, tolerance);
	EXPECT_NEAR(actual.m[2][3], z, tolerance);
}

inline void ExpectMatrixRotationUnchanged(const vr::HmdMatrix34_t& actual, const vr::HmdMatrix34_t& expected,
                                          float tolerance = kFloatTolerance)
{
	for (int r = 0; r < 3; ++r) {
		for (int c = 0; c < 3; ++c) {
			EXPECT_NEAR(actual.m[r][c], expected.m[r][c], tolerance) << "matrix cell [" << r << "][" << c << "]";
		}
	}
}

inline std::vector<BoundaryVertex> SquareBoundary(double minX, double minZ, double maxX, double maxZ, double y = 0.0)
{
	return {
	    {minX, y, minZ},
	    {maxX, y, minZ},
	    {maxX, y, maxZ},
	    {minX, y, maxZ},
	};
}

inline void ExpectBoundsNear(const wkopenvr::boundary::PolygonBounds& actual, double xMin, double xMax, double zMin,
                             double zMax, double tolerance = kDoubleTolerance)
{
	EXPECT_NEAR(actual.xMin, xMin, tolerance);
	EXPECT_NEAR(actual.xMax, xMax, tolerance);
	EXPECT_NEAR(actual.zMin, zMin, tolerance);
	EXPECT_NEAR(actual.zMax, zMax, tolerance);
}

inline size_t CountAlphaPixels(const wkopenvr::boundary::BoundaryPreviewRaster& raster)
{
	size_t count = 0;
	for (size_t i = 3; i < raster.rgba.size(); i += 4) {
		if (raster.rgba[i] != 0) ++count;
	}
	return count;
}

} // namespace boundary_test
