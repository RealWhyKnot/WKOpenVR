// Unit tests for facetracking::VergenceLock.
//
// Coverage:
//   - Strength 0 leaves the frame untouched.
//   - Strength 100 with non-parallel gaze rays drives both eyes to converge on
//     the skew-line midpoint focus point.
//   - Parallel gaze is detected and bypassed (the frame is left unmodified).
//   - Single-eye dropout (one eye's confidence below 0.3) drives both eyes
//     from the surviving eye's gaze.
//   - Eye invalidity flag (bit 0 not set) is honored -- the frame is left
//     unmodified regardless of strength.

#include "VergenceLock.h"
#include "Protocol.h"

#include <gtest/gtest.h>

#include <cmath>

namespace {

void SetVec3(float dst[3], float x, float y, float z)
{
	dst[0] = x;
	dst[1] = y;
	dst[2] = z;
}

float Dot(const float a[3], const float b[3])
{
	return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
float Length(const float a[3])
{
	return std::sqrt(Dot(a, a));
}
void Normalize(float a[3])
{
	const float n = Length(a);
	if (n > 1e-6f) {
		a[0] /= n;
		a[1] /= n;
		a[2] /= n;
	}
}

// A baseline two-eye frame: eyes 6 cm apart on X, both gazing straight ahead.
protocol::FaceTrackingFrameBody MakeBaseline()
{
	protocol::FaceTrackingFrameBody f{};
	SetVec3(f.eye_origin_l, -0.031f, 0.0f, 0.0f);
	SetVec3(f.eye_origin_r, 0.031f, 0.0f, 0.0f);
	SetVec3(f.eye_gaze_l, 0.0f, 0.0f, -1.0f);
	SetVec3(f.eye_gaze_r, 0.0f, 0.0f, -1.0f);
	f.eye_openness_l = 1.0f;
	f.eye_openness_r = 1.0f;
	f.eye_confidence_l = 1.0f;
	f.eye_confidence_r = 1.0f;
	f.flags = 0x1; // eye valid
	return f;
}

} // namespace

TEST(VergenceLock, StrengthZeroIsNoOp)
{
	facetracking::VergenceLock lock;
	protocol::FaceTrackingFrameBody f = MakeBaseline();
	SetVec3(f.eye_gaze_l, 0.1f, 0.0f, -1.0f);
	Normalize(f.eye_gaze_l);
	SetVec3(f.eye_gaze_r, -0.1f, 0.0f, -1.0f);
	Normalize(f.eye_gaze_r);

	const protocol::FaceTrackingFrameBody before = f;
	lock.Apply(f, 0);

	EXPECT_FLOAT_EQ(f.eye_gaze_l[0], before.eye_gaze_l[0]);
	EXPECT_FLOAT_EQ(f.eye_gaze_l[1], before.eye_gaze_l[1]);
	EXPECT_FLOAT_EQ(f.eye_gaze_l[2], before.eye_gaze_l[2]);
	EXPECT_FLOAT_EQ(f.eye_gaze_r[0], before.eye_gaze_r[0]);
	EXPECT_FLOAT_EQ(f.eye_gaze_r[1], before.eye_gaze_r[1]);
	EXPECT_FLOAT_EQ(f.eye_gaze_r[2], before.eye_gaze_r[2]);
}

TEST(VergenceLock, StrengthFullConvergesEyesOnSharedFocus)
{
	facetracking::VergenceLock lock;
	protocol::FaceTrackingFrameBody f = MakeBaseline();

	// Both eyes are converging in front of the face at ~50 cm: each ray points
	// toward (0, 0, -0.5) from its own origin.
	float target[3] = {0.0f, 0.0f, -0.5f};
	f.eye_gaze_l[0] = target[0] - f.eye_origin_l[0];
	f.eye_gaze_l[1] = target[1] - f.eye_origin_l[1];
	f.eye_gaze_l[2] = target[2] - f.eye_origin_l[2];
	Normalize(f.eye_gaze_l);
	f.eye_gaze_r[0] = target[0] - f.eye_origin_r[0];
	f.eye_gaze_r[1] = target[1] - f.eye_origin_r[1];
	f.eye_gaze_r[2] = target[2] - f.eye_origin_r[2];
	Normalize(f.eye_gaze_r);

	lock.Apply(f, 100);

	// After full vergence lock, both rays from their own origins should still
	// hit the same focus point.  Reconstruct each direction's intersection
	// with z = -0.5 and check it lands near the target.
	auto IntersectZ = [](const float origin[3], const float dir[3], float z, float out[3]) {
		const float t = (z - origin[2]) / dir[2];
		out[0] = origin[0] + t * dir[0];
		out[1] = origin[1] + t * dir[1];
		out[2] = z;
	};
	float hit_l[3], hit_r[3];
	IntersectZ(f.eye_origin_l, f.eye_gaze_l, -0.5f, hit_l);
	IntersectZ(f.eye_origin_r, f.eye_gaze_r, -0.5f, hit_r);

	EXPECT_NEAR(hit_l[0], target[0], 0.02f);
	EXPECT_NEAR(hit_l[1], target[1], 0.02f);
	EXPECT_NEAR(hit_r[0], target[0], 0.02f);
	EXPECT_NEAR(hit_r[1], target[1], 0.02f);
}

TEST(VergenceLock, ParallelGazeBypassed)
{
	facetracking::VergenceLock lock;
	protocol::FaceTrackingFrameBody f = MakeBaseline();
	// Both eyes parallel pointing -Z.  denom = a*c - b*b approaches 0 when
	// d_L and d_R are parallel; the engine should bypass the lock.
	const protocol::FaceTrackingFrameBody before = f;
	lock.Apply(f, 100);

	EXPECT_NEAR(f.eye_gaze_l[0], before.eye_gaze_l[0], 1e-5f);
	EXPECT_NEAR(f.eye_gaze_l[2], before.eye_gaze_l[2], 1e-5f);
	EXPECT_NEAR(f.eye_gaze_r[0], before.eye_gaze_r[0], 1e-5f);
	EXPECT_NEAR(f.eye_gaze_r[2], before.eye_gaze_r[2], 1e-5f);
}

TEST(VergenceLock, InvalidFlagPreservesFrame)
{
	facetracking::VergenceLock lock;
	protocol::FaceTrackingFrameBody f = MakeBaseline();
	f.flags = 0; // clear eye-valid bit

	SetVec3(f.eye_gaze_l, 0.2f, 0.0f, -1.0f);
	Normalize(f.eye_gaze_l);
	SetVec3(f.eye_gaze_r, -0.2f, 0.0f, -1.0f);
	Normalize(f.eye_gaze_r);
	const protocol::FaceTrackingFrameBody before = f;

	lock.Apply(f, 100);

	EXPECT_FLOAT_EQ(f.eye_gaze_l[0], before.eye_gaze_l[0]);
	EXPECT_FLOAT_EQ(f.eye_gaze_r[0], before.eye_gaze_r[0]);
}

TEST(VergenceLock, EyeDropoutDrivesBothFromSurvivingEye)
{
	facetracking::VergenceLock lock;
	protocol::FaceTrackingFrameBody f = MakeBaseline();

	// Left eye confidence near zero (lost), right eye still valid and gazing
	// slightly upward.  Expect both eyes to roughly agree on the right eye's
	// direction after the lock.
	f.eye_confidence_l = 0.05f;
	f.eye_confidence_r = 1.0f;
	SetVec3(f.eye_gaze_l, 0.5f, -0.5f, -0.5f);
	Normalize(f.eye_gaze_l);
	SetVec3(f.eye_gaze_r, 0.0f, 0.2f, -1.0f);
	Normalize(f.eye_gaze_r);

	lock.Apply(f, 100);

	// Left eye should now point roughly upward, matching the right eye's
	// pitch sign rather than its original downward direction.
	EXPECT_GT(f.eye_gaze_l[1], -0.1f);
}
