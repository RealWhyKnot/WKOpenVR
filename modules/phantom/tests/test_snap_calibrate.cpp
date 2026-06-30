#include "SnapCalibrate.h"

#include <gtest/gtest.h>

#include <vector>

using namespace phantom;

namespace {

// HMD at origin facing -Z so right = +X, forward = -Z. Floor at 0, head at 1.70.
constexpr double kHeadY = 1.70;
const double kHmd[3] = {0.0, kHeadY, 0.0};
const double kRight[2] = {1.0, 0.0};
const double kFwd[2] = {0.0, -1.0};

// Build a tracker world position for a role at normalized {height_ratio, lateral}.
SnapTrackerInput At(uint32_t id, double height_ratio, double lateral)
{
	SnapTrackerInput t;
	t.id = id;
	t.pos[0] = lateral * kHeadY; // along +X (right)
	t.pos[1] = height_ratio * kHeadY;
	t.pos[2] = 0.0;
	return t;
}

BodyRole RoleFor(const SnapResult& r, uint32_t id)
{
	for (const auto& a : r.assignments)
		if (a.id == id) return a.role;
	return BodyRole::None;
}

float ConfFor(const SnapResult& r, uint32_t id)
{
	for (const auto& a : r.assignments)
		if (a.id == id) return a.confidence;
	return 0.0f;
}

} // namespace

// A clean six-point stand maps every tracker in a single static read, with
// confidence above the auto-accept bar -- no motion required.
TEST(SnapCalibrate, CleanSixPointMapsInstantly)
{
	std::vector<SnapTrackerInput> trackers = {
	    At(0, 0.53, 0.00),  // waist
	    At(1, 0.74, 0.00),  // chest
	    At(2, 0.06, -0.12), // left foot
	    At(3, 0.06, 0.12),  // right foot
	    At(4, 0.28, -0.10), // left knee
	    At(5, 0.28, 0.10),  // right knee
	};

	const SnapResult r = SnapCalibrate(kHmd, /*axes_valid=*/true, kRight, kFwd, /*floor_y=*/0.0, trackers);

	EXPECT_TRUE(r.ok);
	EXPECT_EQ(r.status, SnapStatus::Ok);
	EXPECT_EQ(r.assigned_count, 6u);
	EXPECT_EQ(RoleFor(r, 0), BodyRole::Waist);
	EXPECT_EQ(RoleFor(r, 1), BodyRole::Chest);
	EXPECT_EQ(RoleFor(r, 2), BodyRole::LeftFoot);
	EXPECT_EQ(RoleFor(r, 3), BodyRole::RightFoot);
	EXPECT_EQ(RoleFor(r, 4), BodyRole::LeftKnee);
	EXPECT_EQ(RoleFor(r, 5), BodyRole::RightKnee);
	for (uint32_t i = 0; i < 6; ++i) {
		EXPECT_GE(ConfFor(r, i), 0.70f) << "tracker " << i;
	}
	EXPECT_NEAR(r.measured_height_m, kHeadY, 1e-6);
}

// The snap honours the learned floor: the same geometry shifted up by a 1 m floor
// resolves identically.
TEST(SnapCalibrate, FloorRelativeIsInvariant)
{
	const double floor = 1.0;
	const double hmd[3] = {0.0, kHeadY + floor, 0.0};
	std::vector<SnapTrackerInput> trackers = {
	    {0, {0.0, 0.53 * kHeadY + floor, 0.0}},
	    {1, {-0.12 * kHeadY, 0.06 * kHeadY + floor, 0.0}},
	    {2, {0.12 * kHeadY, 0.06 * kHeadY + floor, 0.0}},
	};
	const SnapResult r = SnapCalibrate(hmd, true, kRight, kFwd, floor, trackers);
	EXPECT_EQ(RoleFor(r, 0), BodyRole::Waist);
	EXPECT_EQ(RoleFor(r, 1), BodyRole::LeftFoot);
	EXPECT_EQ(RoleFor(r, 2), BodyRole::RightFoot);
	EXPECT_NEAR(r.measured_height_m, kHeadY, 1e-6);
}

// A foot held on the centreline is ambiguous between left and right -> None.
TEST(SnapCalibrate, MidlineFootIsUnassigned)
{
	std::vector<SnapTrackerInput> trackers = {At(0, 0.06, 0.0)};
	const SnapResult r = SnapCalibrate(kHmd, true, kRight, kFwd, 0.0, trackers);
	EXPECT_EQ(RoleFor(r, 0), BodyRole::None);
}

// A tilted head (no usable horizontal axis) refuses to snap.
TEST(SnapCalibrate, HeadTiltedRefuses)
{
	std::vector<SnapTrackerInput> trackers = {At(0, 0.53, 0.0)};
	const SnapResult r = SnapCalibrate(kHmd, /*axes_valid=*/false, kRight, kFwd, 0.0, trackers);
	EXPECT_FALSE(r.ok);
	EXPECT_EQ(r.status, SnapStatus::HeadTilted);
}

// Trackers in a different tracking space than the HMD (metres away, below the
// floor) -- the classic un-run Space Calibrator on a mixed setup -- report
// NotCalibrated rather than silently failing to assign.
TEST(SnapCalibrate, UncalibratedTrackersReportNotCalibrated)
{
	std::vector<SnapTrackerInput> trackers = {
	    {0, {1.0, -0.80, -2.60}},
	    {1, {0.3, -0.60, -2.40}},
	    {2, {0.6, -0.30, -2.35}},
	};
	const SnapResult r = SnapCalibrate(kHmd, /*axes_valid=*/true, kRight, kFwd, /*floor_y=*/0.0, trackers);
	EXPECT_FALSE(r.ok);
	EXPECT_EQ(r.status, SnapStatus::NotCalibrated);
}

// No trackers -> nothing to assign.
TEST(SnapCalibrate, NoTrackers)
{
	std::vector<SnapTrackerInput> trackers;
	const SnapResult r = SnapCalibrate(kHmd, true, kRight, kFwd, 0.0, trackers);
	EXPECT_FALSE(r.ok);
	EXPECT_EQ(r.status, SnapStatus::NoTrackers);
}
