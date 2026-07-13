#include "PoseSpace.h"
#include "SnapCalibrate.h"

#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <vector>

using namespace phantom;

namespace {

vr::HmdQuaternion_t QConj(const vr::HmdQuaternion_t& q)
{
	return {q.w, -q.x, -q.y, -q.z};
}

void QRotateRef(const vr::HmdQuaternion_t& q, const double v[3], double out[3])
{
	// Reference rotation via the full quaternion sandwich, independent of the
	// cross-product form the production helper uses.
	const vr::HmdQuaternion_t p{0.0, v[0], v[1], v[2]};
	const vr::HmdQuaternion_t r = pose_space::QMul(pose_space::QMul(q, p), QConj(q));
	out[0] = r.x;
	out[1] = r.y;
	out[2] = r.z;
}

vr::DriverPose_t MakePose(const double pos[3], const vr::HmdQuaternion_t& rot)
{
	vr::DriverPose_t p{};
	p.qWorldFromDriverRotation = {1, 0, 0, 0};
	p.qDriverFromHeadRotation = {1, 0, 0, 0};
	p.vecPosition[0] = pos[0];
	p.vecPosition[1] = pos[1];
	p.vecPosition[2] = pos[2];
	p.qRotation = rot;
	p.poseIsValid = true;
	p.deviceIsConnected = true;
	p.result = vr::TrackingResult_Running_OK;
	return p;
}

// Express a desired world-space position in a driver frame defined by the
// world-from-driver transform {q, t}: driver_pos = q^-1 * (world_pos - t).
void WorldToDriver(const vr::HmdQuaternion_t& q, const double t[3], const double world[3], double driver[3])
{
	const double delta[3] = {world[0] - t[0], world[1] - t[1], world[2] - t[2]};
	QRotateRef(QConj(q), delta, driver);
}

// Yaw rotation about +Y.
vr::HmdQuaternion_t YawQuat(double radians)
{
	return {std::cos(radians / 2.0), 0.0, std::sin(radians / 2.0), 0.0};
}

} // namespace

// A pose whose world-from-driver is identity passes through bit-exact.
TEST(PoseSpace, IdentityIsPassthrough)
{
	const double pos[3] = {0.4, 1.2, -0.7};
	vr::DriverPose_t p = MakePose(pos, YawQuat(0.3));
	p.vecVelocity[0] = 1.0;
	p.vecVelocity[2] = -2.0;
	const vr::DriverPose_t w = ToWorldSpacePose(p);
	EXPECT_EQ(std::memcmp(&w, &p, sizeof(w)), 0);
}

// Position, rotation, and all four velocity/acceleration vectors fold through
// the world-from-driver transform; driver-from-head passes through untouched.
TEST(PoseSpace, FoldsWorldFromDriverIntoPose)
{
	const double pos[3] = {1.0, 0.5, 0.0};
	vr::DriverPose_t p = MakePose(pos, YawQuat(0.25));
	const vr::HmdQuaternion_t q = YawQuat(3.14159265358979 / 2.0); // +90 deg yaw
	p.qWorldFromDriverRotation = q;
	p.vecWorldFromDriverTranslation[0] = 10.0;
	p.vecWorldFromDriverTranslation[1] = -1.0;
	p.vecWorldFromDriverTranslation[2] = 2.0;
	p.vecVelocity[0] = 1.0; // +X in driver space -> -Z after +90 deg yaw
	p.vecAngularVelocity[2] = 3.0;
	p.qDriverFromHeadRotation = YawQuat(0.1);
	p.vecDriverFromHeadTranslation[1] = 0.05;

	const vr::DriverPose_t w = ToWorldSpacePose(p);

	// +90 deg yaw maps (x, y, z) -> (z, y, -x).
	EXPECT_NEAR(w.vecPosition[0], 10.0 + 0.0, 1e-9);
	EXPECT_NEAR(w.vecPosition[1], -1.0 + 0.5, 1e-9);
	EXPECT_NEAR(w.vecPosition[2], 2.0 - 1.0, 1e-9);
	EXPECT_NEAR(w.vecVelocity[0], 0.0, 1e-9);
	EXPECT_NEAR(w.vecVelocity[2], -1.0, 1e-9);
	EXPECT_NEAR(w.vecAngularVelocity[0], 3.0, 1e-9);
	EXPECT_NEAR(w.vecAngularVelocity[2], 0.0, 1e-9);

	// Rotation composes on the left; verify by rotating a probe vector.
	double probe[3] = {0.0, 0.0, -1.0};
	double via_world[3];
	double via_parts[3];
	QRotateRef(w.qRotation, probe, via_world);
	double tmp[3];
	QRotateRef(p.qRotation, probe, tmp);
	QRotateRef(q, tmp, via_parts);
	EXPECT_NEAR(via_world[0], via_parts[0], 1e-9);
	EXPECT_NEAR(via_world[1], via_parts[1], 1e-9);
	EXPECT_NEAR(via_world[2], via_parts[2], 1e-9);

	// Output world-from-driver is identity; driver-from-head untouched.
	EXPECT_NEAR(w.qWorldFromDriverRotation.w, 1.0, 1e-12);
	EXPECT_NEAR(w.vecWorldFromDriverTranslation[0], 0.0, 1e-12);
	EXPECT_NEAR(w.qDriverFromHeadRotation.y, p.qDriverFromHeadRotation.y, 1e-12);
	EXPECT_NEAR(w.vecDriverFromHeadTranslation[1], 0.05, 1e-12);
}

// Round trip: express a world pose in an arbitrary driver frame, fold it back,
// and land on the original world coordinates.
TEST(PoseSpace, DriverFrameRoundTripsToWorld)
{
	const vr::HmdQuaternion_t q = YawQuat(1.1);
	const double t[3] = {-3.2, 0.4, 7.7};
	const double world[3] = {0.3, 0.06, -0.4};

	double driver[3];
	WorldToDriver(q, t, world, driver);
	vr::DriverPose_t p = MakePose(driver, pose_space::QMul(QConj(q), YawQuat(0.6)));
	p.qWorldFromDriverRotation = q;
	p.vecWorldFromDriverTranslation[0] = t[0];
	p.vecWorldFromDriverTranslation[1] = t[1];
	p.vecWorldFromDriverTranslation[2] = t[2];

	const vr::DriverPose_t w = ToWorldSpacePose(p);
	EXPECT_NEAR(w.vecPosition[0], world[0], 1e-9);
	EXPECT_NEAR(w.vecPosition[1], world[1], 1e-9);
	EXPECT_NEAR(w.vecPosition[2], world[2], 1e-9);
	// q * (q^-1 * yaw(0.6)) == yaw(0.6).
	const vr::HmdQuaternion_t expect_rot = YawQuat(0.6);
	EXPECT_NEAR(std::fabs(w.qRotation.w * expect_rot.w + w.qRotation.x * expect_rot.x + w.qRotation.y * expect_rot.y +
	                      w.qRotation.z * expect_rot.z),
	            1.0, 1e-9);
}

// Calibration invariance: a clean six-point stand expressed in a rotated and
// translated driver frame (with the matching world-from-driver transform, as
// the calibration applies it) snaps to the same roles and confidences as the
// same layout expressed directly in world space.
TEST(PoseSpace, SnapAssignmentsInvariantUnderCalibratedFrame)
{
	const double head_y = 1.70;
	const double hmd_world[3] = {0.0, head_y, 0.0};
	const double right[2] = {1.0, 0.0};
	const double fwd[2] = {0.0, -1.0};

	struct P
	{
		double height_ratio;
		double lateral;
	};
	const std::vector<P> layout = {
	    {0.53, 0.00}, {0.74, 0.00}, {0.06, -0.12}, {0.06, 0.12}, {0.28, -0.10}, {0.28, 0.10},
	};

	std::vector<SnapTrackerInput> direct;
	for (uint32_t i = 0; i < layout.size(); ++i) {
		SnapTrackerInput t;
		t.id = i;
		t.pos[0] = layout[i].lateral * head_y;
		t.pos[1] = layout[i].height_ratio * head_y;
		t.pos[2] = 0.0;
		direct.push_back(t);
	}
	const SnapResult expect = SnapCalibrate(hmd_world, true, right, fwd, 0.0, direct);
	ASSERT_TRUE(expect.ok);

	// Same layout, but every pose arrives from a driver whose frame is yawed
	// 137 degrees and shifted metres away -- the shape of a lighthouse stack
	// beside a standalone HMD stack once the calibration transform is applied.
	const vr::HmdQuaternion_t q = YawQuat(137.0 * 3.14159265358979 / 180.0);
	const double t[3] = {4.2, -0.6, -2.9};
	std::vector<SnapTrackerInput> folded;
	for (uint32_t i = 0; i < direct.size(); ++i) {
		double driver[3];
		WorldToDriver(q, t, direct[i].pos, driver);
		vr::DriverPose_t p = MakePose(driver, QConj(q));
		p.qWorldFromDriverRotation = q;
		p.vecWorldFromDriverTranslation[0] = t[0];
		p.vecWorldFromDriverTranslation[1] = t[1];
		p.vecWorldFromDriverTranslation[2] = t[2];
		const vr::DriverPose_t w = ToWorldSpacePose(p);
		SnapTrackerInput in;
		in.id = i;
		in.pos[0] = w.vecPosition[0];
		in.pos[1] = w.vecPosition[1];
		in.pos[2] = w.vecPosition[2];
		folded.push_back(in);
	}
	const SnapResult got = SnapCalibrate(hmd_world, true, right, fwd, 0.0, folded);

	ASSERT_TRUE(got.ok);
	EXPECT_EQ(got.assigned_count, expect.assigned_count);
	for (uint32_t i = 0; i < direct.size(); ++i) {
		BodyRole want = BodyRole::None;
		BodyRole have = BodyRole::None;
		float want_conf = 0.0f;
		float have_conf = 0.0f;
		for (const auto& a : expect.assignments) {
			if (a.id == i) {
				want = a.role;
				want_conf = a.confidence;
			}
		}
		for (const auto& a : got.assignments) {
			if (a.id == i) {
				have = a.role;
				have_conf = a.confidence;
			}
		}
		EXPECT_EQ(have, want) << "tracker " << i;
		EXPECT_NEAR(have_conf, want_conf, 1e-4f) << "tracker " << i;
	}
}
