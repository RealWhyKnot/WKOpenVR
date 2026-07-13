#include <gtest/gtest.h>

#include "IkFallback.h"
#include "RoleCatalog.h"

#include <openvr_driver.h>

#include <cmath>

namespace {

vr::DriverPose_t MakeHmdPose(double x, double y, double z, const vr::HmdQuaternion_t& rot)
{
	vr::DriverPose_t p{};
	p.qWorldFromDriverRotation = {1, 0, 0, 0};
	p.qDriverFromHeadRotation = {1, 0, 0, 0};
	p.vecPosition[0] = x;
	p.vecPosition[1] = y;
	p.vecPosition[2] = z;
	p.qRotation = rot;
	p.poseIsValid = true;
	p.deviceIsConnected = true;
	p.result = vr::TrackingResult_Running_OK;
	return p;
}

} // namespace

TEST(IkFallbackTest, EmptyTableSolvesNothing)
{
	phantom::IkFallback f;
	EXPECT_FALSE(f.AnyOffsetAvailable());
	vr::DriverPose_t out{};
	vr::HmdQuaternion_t I{1, 0, 0, 0};
	auto hmd = MakeHmdPose(0, 1.7, 0, I);
	EXPECT_FALSE(f.Solve(phantom::BodyRole::Waist, hmd, out));
}

TEST(IkFallbackTest, IdentityHmdReproducesOffsetAsWorldPosition)
{
	phantom::IkFallback f;
	const double waist_offset[3] = {0.0, -0.55, 0.0};
	vr::HmdQuaternion_t I{1, 0, 0, 0};
	f.SetOffset(phantom::BodyRole::Waist, waist_offset, I);

	auto hmd = MakeHmdPose(0, 1.7, 0, I);
	vr::DriverPose_t out{};
	ASSERT_TRUE(f.Solve(phantom::BodyRole::Waist, hmd, out));
	EXPECT_NEAR(out.vecPosition[0], 0.0, 1e-9);
	EXPECT_NEAR(out.vecPosition[1], 1.15, 1e-9);
	EXPECT_NEAR(out.vecPosition[2], 0.0, 1e-9);
}

TEST(IkFallbackTest, RotatedHmdRotatesOffset)
{
	phantom::IkFallback f;
	// Waist offset is "1m in front of HMD" in HMD-local +z (which is forward).
	const double front_offset[3] = {0.0, 0.0, 1.0};
	vr::HmdQuaternion_t I{1, 0, 0, 0};
	f.SetOffset(phantom::BodyRole::Waist, front_offset, I);

	// HMD rotated 90 degrees around Y axis: forward (HMD-local +z) maps to
	// world +x. MSVC's <cmath> does not expose M_PI by default, so use the
	// constant directly to keep the test self-contained.
	constexpr double kPi = 3.14159265358979323846;
	vr::HmdQuaternion_t yawL{std::cos(kPi / 4), 0.0, std::sin(kPi / 4), 0.0};
	auto hmd = MakeHmdPose(0, 1.7, 0, yawL);
	vr::DriverPose_t out{};
	ASSERT_TRUE(f.Solve(phantom::BodyRole::Waist, hmd, out));
	EXPECT_NEAR(out.vecPosition[0], 1.0, 1e-9);
	EXPECT_NEAR(out.vecPosition[1], 1.7, 1e-9);
	EXPECT_NEAR(out.vecPosition[2], 0.0, 1e-9);
}

// The solved pose starts as a copy of the HMD pose; the HMD's head-model
// transform must not ride along on a synthesized tracker.
TEST(IkFallbackTest, SolvedPoseHasIdentityDriverFromHead)
{
	phantom::IkFallback f;
	const double off[3] = {0.0, -0.55, 0.0};
	vr::HmdQuaternion_t I{1, 0, 0, 0};
	f.SetOffset(phantom::BodyRole::Waist, off, I);

	auto hmd = MakeHmdPose(0, 1.7, 0, I);
	hmd.shouldApplyHeadModel = true;
	hmd.qDriverFromHeadRotation = {0.9848, 0.0, 0.1736, 0.0};
	hmd.vecDriverFromHeadTranslation[2] = -0.08;

	vr::DriverPose_t out{};
	ASSERT_TRUE(f.Solve(phantom::BodyRole::Waist, hmd, out));
	EXPECT_FALSE(out.shouldApplyHeadModel);
	EXPECT_NEAR(out.qDriverFromHeadRotation.w, 1.0, 1e-12);
	EXPECT_NEAR(out.qDriverFromHeadRotation.y, 0.0, 1e-12);
	EXPECT_NEAR(out.vecDriverFromHeadTranslation[2], 0.0, 1e-12);
}

TEST(IkFallbackTest, ClearOffsetRemoves)
{
	phantom::IkFallback f;
	const double off[3] = {0, 0, 0};
	vr::HmdQuaternion_t I{1, 0, 0, 0};
	f.SetOffset(phantom::BodyRole::LeftFoot, off, I);
	EXPECT_TRUE(f.HasOffset(phantom::BodyRole::LeftFoot));
	f.ClearOffset(phantom::BodyRole::LeftFoot);
	EXPECT_FALSE(f.HasOffset(phantom::BodyRole::LeftFoot));
}

TEST(IkFallbackTest, ClearAllZerosTable)
{
	phantom::IkFallback f;
	const double off[3] = {0, 0, 0};
	vr::HmdQuaternion_t I{1, 0, 0, 0};
	f.SetOffset(phantom::BodyRole::LeftFoot, off, I);
	f.SetOffset(phantom::BodyRole::RightFoot, off, I);
	EXPECT_TRUE(f.AnyOffsetAvailable());
	f.ClearAll();
	EXPECT_FALSE(f.AnyOffsetAvailable());
}
