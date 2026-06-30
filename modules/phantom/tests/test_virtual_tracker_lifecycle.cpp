#include <gtest/gtest.h>

#include "FakeDriverHost.h"
#include "VirtualTrackerManager.h"

#include <openvr_driver.h>

#include <chrono>
#include <utility>
#include <vector>

// Lifecycle invariants for absent-mode virtual trackers: disabling a role (or the
// whole module) must publish a *disconnected* pose so SteamVR hides the device
// promptly instead of floating its last pose, GetPose must report disconnected
// while disabled, disable->re-enable must work within a session, and none of this
// may call TrackedDevicePoseUpdated directly (that re-enters the pose hook and
// deadlocks -- the manager only caches poses for the out-of-lock drain).

namespace {

using Update = std::pair<uint32_t, vr::DriverPose_t>;
constexpr auto kWaist = phantom::BodyRole::Waist;
constexpr auto kLeftFoot = phantom::BodyRole::LeftFoot;

vr::DriverPose_t MakeHmdPose()
{
	vr::DriverPose_t p{};
	p.qWorldFromDriverRotation = {1, 0, 0, 0};
	p.qDriverFromHeadRotation = {1, 0, 0, 0};
	p.qRotation = {1, 0, 0, 0};
	p.poseIsValid = true;
	p.deviceIsConnected = true;
	p.result = vr::TrackingResult_Running_OK;
	return p;
}

phantom::BodyCompletionResult MakeBody(phantom::BodyRole role, double x, double y, double z)
{
	phantom::BodyCompletionResult body{};
	auto& r = body.roles[static_cast<size_t>(role)];
	r.valid = true;
	r.confidence = 1.0f;
	r.pose.position[0] = x;
	r.pose.position[1] = y;
	r.pose.position[2] = z;
	r.pose.rotation[0] = 1.0;
	return body;
}

class VirtualTrackerLifecycle : public ::testing::Test
{
protected:
	void SetUp() override
	{
		phantom_test::InstallFakeDriverContext(ctx_);
		mgr_.OnDriverInit();
		mgr_.SetInitSettleDelay(std::chrono::milliseconds(0)); // activate without waiting
		mgr_.SetMasterEnabled(true);
	}
	void TearDown() override { vr::CleanupDriverContext(); }

	phantom_test::FakeDriverContext ctx_;
	phantom::VirtualTrackerManager mgr_;
};

} // namespace

TEST_F(VirtualTrackerLifecycle, DisableRolePublishesDisconnectAndGetPoseReportsDisconnected)
{
	const auto hmd = MakeHmdPose();
	const auto body = MakeBody(kWaist, 0.1, 0.95, 0.2);

	mgr_.SetEnabled(kWaist, true);
	mgr_.Tick(hmd, body, 0.0);
	EXPECT_EQ(mgr_.ActiveCount(), 1);
	EXPECT_EQ(ctx_.host.add_count, 1);

	std::vector<Update> out;
	mgr_.CollectPoseUpdates(out);
	ASSERT_EQ(out.size(), 1u);
	EXPECT_TRUE(out[0].second.poseIsValid);
	EXPECT_TRUE(out[0].second.deviceIsConnected);

	vr::ITrackedDeviceServerDriver* dev = ctx_.host.last_driver;
	ASSERT_NE(dev, nullptr);
	EXPECT_TRUE(dev->GetPose().poseIsValid);

	// Disable: GetPose reports disconnected immediately (gate closed synchronously).
	mgr_.SetEnabled(kWaist, false);
	EXPECT_FALSE(dev->GetPose().poseIsValid);

	// Next tick pushes a disconnected pose through the drain.
	out.clear();
	mgr_.Tick(hmd, body, 0.0);
	mgr_.CollectPoseUpdates(out);
	ASSERT_EQ(out.size(), 1u);
	EXPECT_FALSE(out[0].second.poseIsValid);
	EXPECT_FALSE(out[0].second.deviceIsConnected);
	EXPECT_EQ(out[0].second.result, vr::TrackingResult_Running_OutOfRange);
	EXPECT_FALSE(dev->GetPose().poseIsValid);

	// The manager must never publish directly.
	EXPECT_EQ(ctx_.host.pose_update_count, 0);
}

TEST_F(VirtualTrackerLifecycle, ModuleDisableFlushesDisconnectsForAllActiveVirtuals)
{
	const auto hmd = MakeHmdPose();
	mgr_.SetEnabled(kWaist, true);
	mgr_.SetEnabled(kLeftFoot, true);

	// One tick with both roles valid activates and publishes both.
	auto body = MakeBody(kWaist, 0.1, 0.95, 0.2);
	body.roles[static_cast<size_t>(kLeftFoot)] =
	    MakeBody(kLeftFoot, -0.1, 0.05, 0.1).roles[static_cast<size_t>(kLeftFoot)];
	mgr_.Tick(hmd, body, 0.0);
	EXPECT_EQ(mgr_.ActiveCount(), 2);
	EXPECT_EQ(ctx_.host.add_count, 2);

	std::vector<Update> disc;
	mgr_.CollectDisconnects(disc);
	ASSERT_EQ(disc.size(), 2u);
	for (const auto& u : disc) {
		EXPECT_FALSE(u.second.poseIsValid);
		EXPECT_FALSE(u.second.deviceIsConnected);
		EXPECT_EQ(u.second.result, vr::TrackingResult_Running_OutOfRange);
	}
	// CollectDisconnects returns poses for the umbrella to forward; it does not
	// publish them itself.
	EXPECT_EQ(ctx_.host.pose_update_count, 0);
}

TEST_F(VirtualTrackerLifecycle, DisableThenReEnableWithinSession)
{
	const auto hmd = MakeHmdPose();
	const auto body = MakeBody(kWaist, 0.1, 0.95, 0.2);

	mgr_.SetEnabled(kWaist, true);
	mgr_.Tick(hmd, body, 0.0);
	std::vector<Update> out;
	mgr_.CollectPoseUpdates(out);
	ASSERT_EQ(out.size(), 1u);

	mgr_.SetEnabled(kWaist, false);
	out.clear();
	mgr_.Tick(hmd, body, 0.0);
	mgr_.CollectPoseUpdates(out);
	ASSERT_EQ(out.size(), 1u);
	EXPECT_FALSE(out[0].second.poseIsValid);

	// Re-enable: the device object is still alive, so it just resumes publishing.
	mgr_.SetEnabled(kWaist, true);
	out.clear();
	mgr_.Tick(hmd, body, 0.0);
	mgr_.CollectPoseUpdates(out);
	ASSERT_EQ(out.size(), 1u);
	EXPECT_TRUE(out[0].second.poseIsValid);
	EXPECT_TRUE(out[0].second.deviceIsConnected);
	EXPECT_EQ(ctx_.host.add_count, 1); // reused, not re-registered
	vr::ITrackedDeviceServerDriver* dev = ctx_.host.last_driver;
	ASSERT_NE(dev, nullptr);
	EXPECT_TRUE(dev->GetPose().poseIsValid);
}

TEST_F(VirtualTrackerLifecycle, ReEnableReRegistersAfterSteamVrDeactivate)
{
	const auto hmd = MakeHmdPose();
	const auto body = MakeBody(kWaist, 0.1, 0.95, 0.2);

	mgr_.SetEnabled(kWaist, true);
	mgr_.Tick(hmd, body, 0.0);
	ASSERT_EQ(ctx_.host.add_count, 1);
	vr::ITrackedDeviceServerDriver* dev = ctx_.host.last_driver;
	ASSERT_NE(dev, nullptr);

	// Simulate SteamVR dropping the device after the disconnect (it called
	// Deactivate). The wedge fix must re-register it on the next tick.
	dev->Deactivate();
	EXPECT_EQ(mgr_.ActiveCount(), 0);

	mgr_.Tick(hmd, body, 0.0);
	EXPECT_EQ(ctx_.host.add_count, 2); // re-registered
	EXPECT_EQ(mgr_.ActiveCount(), 1);

	std::vector<Update> out;
	mgr_.CollectPoseUpdates(out);
	ASSERT_EQ(out.size(), 1u);
	EXPECT_TRUE(out[0].second.poseIsValid);
}
