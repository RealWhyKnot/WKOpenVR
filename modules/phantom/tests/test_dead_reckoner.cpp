#include <gtest/gtest.h>

#include "DeadReckoner.h"
#include "PoseHistory.h"

#include <openvr_driver.h>

#include <cmath>

namespace {

vr::DriverPose_t MakePose(double x, double vx)
{
	vr::DriverPose_t p{};
	p.qWorldFromDriverRotation = {1, 0, 0, 0};
	p.qDriverFromHeadRotation = {1, 0, 0, 0};
	p.qRotation = {1, 0, 0, 0};
	p.vecPosition[0] = x;
	p.vecVelocity[0] = vx;
	p.poseIsValid = true;
	p.deviceIsConnected = true;
	p.result = vr::TrackingResult_Running_OK;
	return p;
}

} // namespace

TEST(DeadReckonerTest, RefusesEmptyHistory)
{
	phantom::PoseHistory hist;
	phantom::DeadReckoner r;
	vr::DriverPose_t out{};
	EXPECT_FALSE(r.Project(hist, 10000000, 1, out));
}

TEST(DeadReckonerTest, ProjectsByVelocity)
{
	phantom::PoseHistory hist;
	const int64_t qpc_freq = 10000000; // 10 MHz
	// Push a real pose at t=0 with velocity 1 m/s along +X.
	hist.Push(0, MakePose(/*x=*/0.0, /*vx=*/1.0));

	phantom::DeadReckoner r;
	vr::DriverPose_t out{};
	// Project to t = 50 ms (qpc = 0.05 * 10MHz = 500000)
	ASSERT_TRUE(r.Project(hist, qpc_freq, 500000, out));
	// Damping ramps linearly from 1.0 at t=0 down to 0.0 at kFullDampMs=250.
	// At t=50 ms damping is 0.8, so projected x = v*damp*dt = 1.0*0.8*0.05 = 0.04.
	EXPECT_NEAR(out.vecPosition[0], 0.04, 1e-6);
	EXPECT_TRUE(out.poseIsValid);
	EXPECT_TRUE(out.deviceIsConnected);
	EXPECT_EQ(out.result, vr::TrackingResult_Running_OK);
}

TEST(DeadReckonerTest, DampsBeyondFullDampWindow)
{
	phantom::PoseHistory hist;
	const int64_t qpc_freq = 10000000;
	hist.Push(0, MakePose(/*x=*/0.0, /*vx=*/10.0));

	phantom::DeadReckoner r;
	vr::DriverPose_t out{};
	// Project to t = kFullDampMs + a bit; damping should zero out velocity.
	const int64_t target = (phantom::DeadReckoner::kFullDampMs + 100) * qpc_freq / 1000;
	ASSERT_TRUE(r.Project(hist, qpc_freq, target, out));
	EXPECT_EQ(out.vecVelocity[0], 0.0);
	EXPECT_EQ(out.vecAcceleration[0], 0.0);
}

TEST(DeadReckonerTest, RejectsNonFiniteInput)
{
	phantom::PoseHistory hist;
	const int64_t qpc_freq = 10000000;
	auto p = MakePose(0.0, 0.0);
	p.vecVelocity[0] = std::numeric_limits<double>::infinity();
	hist.Push(0, p);

	phantom::DeadReckoner r;
	vr::DriverPose_t out{};
	EXPECT_FALSE(r.Project(hist, qpc_freq, 500000, out));
}
