// Tests for the per-device transform deadband (spacecal::apply::TransformPayloadNearEqual).
// The overlay caches the last transform it sent each device and only re-sends
// (a blocking IPC round-trip) when the new payload differs. Continuous cal
// updates its offset every tick, so an exact compare re-sent ~13x/sec of
// sub-millimetre solver jitter. The deadband suppresses imperceptible transform
// changes while sending any flag change immediately; the cache holds the last
// SENT payload, so the driver's offset is never more than one deadband stale.

#include <gtest/gtest.h>

#include "TransformPayloadCompare.h"

#include <cmath>

using protocol::SetDeviceTransform;
using spacecal::apply::kTransformNearEqualMeters;
using spacecal::apply::kTransformNearEqualRotDotEps;
using spacecal::apply::TransformPayloadNearEqual;

namespace {
constexpr double kPi = 3.14159265358979323846;

SetDeviceTransform MakePayload()
{
	vr::HmdVector3d_t t{};
	t.v[0] = 0.10;
	t.v[1] = 2.00;
	t.v[2] = 1.50; // metres
	vr::HmdQuaternion_t r{1.0, 0.0, 0.0, 0.0};
	return SetDeviceTransform(1u, /*enabled=*/true, t, r, /*scale=*/1.0);
}

// Set b's rotation to a pure rotation of `deg` degrees about the X axis.
void SetRotationDegAboutX(SetDeviceTransform& p, double deg)
{
	const double half = (deg * kPi / 180.0) / 2.0;
	p.rotation.w = std::cos(half);
	p.rotation.x = std::sin(half);
	p.rotation.y = 0.0;
	p.rotation.z = 0.0;
}
} // namespace

TEST(TransformPayloadNearEqual, IdenticalAreEqual)
{
	const auto a = MakePayload();
	const auto b = a;
	EXPECT_TRUE(TransformPayloadNearEqual(a, b));
}

TEST(TransformPayloadNearEqual, SubDeadbandTranslationEqual)
{
	const auto a = MakePayload();
	auto b = a;
	b.translation.v[0] += kTransformNearEqualMeters * 0.5; // half the deadband
	EXPECT_TRUE(TransformPayloadNearEqual(a, b));
}

TEST(TransformPayloadNearEqual, AboveDeadbandTranslationNotEqual)
{
	const auto a = MakePayload();
	auto b = a;
	b.translation.v[2] += kTransformNearEqualMeters * 2.0; // twice the deadband
	EXPECT_FALSE(TransformPayloadNearEqual(a, b));
}

// Every non-transform field is exact: a flip on any of them must re-send even
// when the transform itself is byte-identical. This is the carve-out that keeps
// motion-gate `lerp` and head-mount `quash` toggles propagating immediately.
TEST(TransformPayloadNearEqual, FlagChangesAlwaysSend)
{
	const auto base = MakePayload();

	auto lerp = base;
	lerp.lerp = !base.lerp;
	EXPECT_FALSE(TransformPayloadNearEqual(base, lerp));

	auto quash = base;
	quash.quash = !base.quash;
	EXPECT_FALSE(TransformPayloadNearEqual(base, quash));

	auto updateQuash = base;
	updateQuash.updateQuash = !base.updateQuash;
	EXPECT_FALSE(TransformPayloadNearEqual(base, updateQuash));

	auto enabled = base;
	enabled.enabled = !base.enabled;
	EXPECT_FALSE(TransformPayloadNearEqual(base, enabled));

	auto recal = base;
	recal.recalibrateOnMovement = !base.recalibrateOnMovement;
	EXPECT_FALSE(TransformPayloadNearEqual(base, recal));

	auto pred = base;
	pred.predictionSmoothness = static_cast<decltype(pred.predictionSmoothness)>(base.predictionSmoothness + 1);
	EXPECT_FALSE(TransformPayloadNearEqual(base, pred));

	auto scale = base;
	scale.scale = base.scale + 0.01;
	EXPECT_FALSE(TransformPayloadNearEqual(base, scale));
}

TEST(TransformPayloadNearEqual, SubDeadbandRotationEqual)
{
	const auto a = MakePayload();
	auto b = a;
	SetRotationDegAboutX(b, 0.01); // below the ~0.02 deg deadband
	EXPECT_TRUE(TransformPayloadNearEqual(a, b));
}

TEST(TransformPayloadNearEqual, AboveDeadbandRotationNotEqual)
{
	const auto a = MakePayload();
	auto b = a;
	SetRotationDegAboutX(b, 0.1); // above the deadband
	EXPECT_FALSE(TransformPayloadNearEqual(a, b));
}

// q and -q are the same orientation; the |dot| in the deadband must treat them
// as equal so a quaternion sign flip from the solver doesn't force a re-send.
TEST(TransformPayloadNearEqual, NegatedQuaternionSameOrientation)
{
	const auto a = MakePayload();
	auto b = a;
	b.rotation.w = -a.rotation.w;
	b.rotation.x = -a.rotation.x;
	b.rotation.y = -a.rotation.y;
	b.rotation.z = -a.rotation.z;
	EXPECT_TRUE(TransformPayloadNearEqual(a, b));
}

// Bounded staleness: the anchor (last sent) stays fixed while the live payload
// drifts in sub-deadband steps. Suppressed until cumulative drift exceeds the
// deadband, then a "send" re-anchors -- so the driver is never more than one
// deadband behind, and total sends are far below per-tick.
TEST(TransformPayloadNearEqual, BoundedStalenessAgainstFixedAnchor)
{
	auto anchor = MakePayload();
	auto live = anchor;
	const double step = kTransformNearEqualMeters * 0.3; // 0.3 deadband per tick
	const int ticks = 30;
	int sends = 0;
	for (int i = 0; i < ticks; ++i) {
		live.translation.v[0] += step;
		if (!TransformPayloadNearEqual(anchor, live)) {
			++sends;
			anchor = live; // re-anchor on send
		}
	}
	EXPECT_GE(sends, 5);
	EXPECT_LT(sends, ticks); // far fewer than one send per tick
	const double residual = std::fabs(live.translation.v[0] - anchor.translation.v[0]);
	EXPECT_LE(residual, kTransformNearEqualMeters);
}
