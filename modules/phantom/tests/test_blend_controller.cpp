#include <gtest/gtest.h>

#include "BlendController.h"

#include <openvr_driver.h>

namespace {

vr::DriverPose_t MakePose(double x, double qw)
{
	vr::DriverPose_t p{};
	p.qWorldFromDriverRotation = {1, 0, 0, 0};
	p.qDriverFromHeadRotation = {1, 0, 0, 0};
	p.vecPosition[0] = x;
	p.qRotation = {qw, 0, 0, 0};
	p.result = vr::TrackingResult_Running_OK;
	return p;
}

} // namespace

TEST(BlendControllerTest, EndpointsAreClean)
{
	auto a = MakePose(0.0, 1.0);
	auto b = MakePose(1.0, 1.0);
	vr::DriverPose_t out{};
	phantom::BlendController::Lerp(a, b, 0.0, out);
	EXPECT_DOUBLE_EQ(out.vecPosition[0], 0.0);
	phantom::BlendController::Lerp(a, b, 1.0, out);
	EXPECT_DOUBLE_EQ(out.vecPosition[0], 1.0);
}

TEST(BlendControllerTest, MidpointIsHalfwayInPosition)
{
	auto a = MakePose(0.0, 1.0);
	auto b = MakePose(1.0, 1.0);
	vr::DriverPose_t out{};
	phantom::BlendController::Lerp(a, b, 0.5, out);
	// smoothstep(0.5) = 0.5, so midpoint position is 0.5 too.
	EXPECT_NEAR(out.vecPosition[0], 0.5, 1e-9);
}

TEST(BlendControllerTest, ClampsAlphaOutOfRange)
{
	auto a = MakePose(0.0, 1.0);
	auto b = MakePose(1.0, 1.0);
	vr::DriverPose_t out{};
	phantom::BlendController::Lerp(a, b, -0.5, out);
	EXPECT_DOUBLE_EQ(out.vecPosition[0], 0.0);
	phantom::BlendController::Lerp(a, b, 1.5, out);
	EXPECT_DOUBLE_EQ(out.vecPosition[0], 1.0);
}

TEST(BlendControllerTest, RotationStaysUnit)
{
	auto a = MakePose(0.0, 1.0);
	auto b = MakePose(0.0, -1.0); // opposite-sign quaternion; same rotation
	vr::DriverPose_t out{};
	phantom::BlendController::Lerp(a, b, 0.5, out);
	const double n = out.qRotation.w * out.qRotation.w + out.qRotation.x * out.qRotation.x +
	                 out.qRotation.y * out.qRotation.y + out.qRotation.z * out.qRotation.z;
	EXPECT_NEAR(n, 1.0, 1e-6);
}
