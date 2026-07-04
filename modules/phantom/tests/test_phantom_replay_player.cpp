#define _CRT_SECURE_NO_WARNINGS
#include "PhantomReplayPlayer.h"

#include <gtest/gtest.h>

#include <cstdlib>
#include <fstream>
#include <string>
#include <vector>

using namespace phantom;

namespace {

constexpr double kHeadY = 1.70;

ReplaySample Hmd(double t)
{
	ReplaySample s;
	s.time_ms = t;
	s.device_id = 0;
	s.serial = "PHR-HMD";
	s.device_class = "hmd";
	s.ground_truth_role = BodyRole::Hmd;
	s.pose_valid = true;
	s.pos[0] = 0.0;
	s.pos[1] = kHeadY;
	s.pos[2] = 0.0;
	s.quat[0] = 1.0; // identity -> facing -Z
	return s;
}

ReplaySample Trk(double t, uint32_t id, const char* serial, BodyRole role, double h, double lat)
{
	ReplaySample s;
	s.time_ms = t;
	s.device_id = id;
	s.serial = serial;
	s.device_class = "tracker";
	s.ground_truth_role = role;
	s.pose_valid = true;
	s.pos[0] = lat * kHeadY;
	s.pos[1] = h * kHeadY;
	s.pos[2] = 0.0;
	s.quat[0] = 1.0;
	return s;
}

} // namespace

// CSV parsing skips comments + the header row and reads the data fields.
TEST(PhantomReplayPlayer, ParsesV1Csv)
{
	const std::vector<std::string> lines = {
	    "# phantom_replay_v1",
	    "# build_stamp=test",
	    "time_ms,device_id,serial,class,controller_role,body_role,dropout_enabled,pose_valid,connected,result,x,y,z,qw,"
	    "qx,qy,qz,vx,vy,vz",
	    "0,0,PHR-HMD,hmd,invalid,hmd,0,1,1,ok,0,1.70,0,1,0,0,0,0,0,0",
	    "0,5,PHR-WAIST,tracker,opt_out,waist,1,1,1,ok,0,0.90,0,1,0,0,0,0,0,0",
	};
	const ParsedReplay parsed = ParseReplay(lines);
	ASSERT_TRUE(parsed.ok);
	ASSERT_EQ(parsed.samples.size(), 2u);
	EXPECT_EQ(parsed.samples[0].device_class, "hmd");
	EXPECT_EQ(parsed.samples[1].serial, "PHR-WAIST");
	EXPECT_EQ(parsed.samples[1].ground_truth_role, BodyRole::Waist);
	EXPECT_NEAR(parsed.samples[1].pos[1], 0.90, 1e-9);
}

// Annotation lines interleaved with data rows are comments and must not
// change what parses out -- recordings carry budget/decision markers between
// samples.
TEST(PhantomReplayPlayer, AnnotationLinesBetweenRowsAreIgnored)
{
	const std::vector<std::string> plain = {
	    "# phantom_replay_v1",
	    "time_ms,device_id,serial,class,controller_role,body_role,dropout_enabled,pose_valid,connected,result,x,y,z,qw,"
	    "qx,qy,qz,vx,vy,vz",
	    "0,0,PHR-HMD,hmd,invalid,hmd,0,1,1,ok,0,1.70,0,1,0,0,0,0,0,0",
	    "100,5,PHR-WAIST,tracker,opt_out,waist,1,1,1,ok,0,0.90,0,1,0,0,0,0,0,0",
	};
	std::vector<std::string> annotated = plain;
	annotated.insert(annotated.begin() + 1, "# build_channel=dev");
	annotated.insert(annotated.begin() + 2, "# [0.000] budget: max_hz_hmd=5 max_hz_dev=2");
	annotated.insert(annotated.begin() + 5, "# [0.050] budget_counters: written=1 suppressed=12");

	const ParsedReplay a = ParseReplay(plain);
	const ParsedReplay b = ParseReplay(annotated);
	ASSERT_TRUE(a.ok);
	ASSERT_TRUE(b.ok);
	ASSERT_EQ(a.samples.size(), b.samples.size());
	for (size_t i = 0; i < a.samples.size(); ++i) {
		EXPECT_EQ(a.samples[i].serial, b.samples[i].serial);
		EXPECT_EQ(a.samples[i].time_ms, b.samples[i].time_ms);
		EXPECT_EQ(a.samples[i].pos[1], b.samples[i].pos[1]);
	}
}

// A clean six-point replay is fully recovered by the snap; passive recovers at
// least the torso (feet need motion the static replay doesn't carry).
TEST(PhantomReplayPlayer, ScoresCleanSixPoint)
{
	std::vector<ReplaySample> samples;
	for (int frame = 0; frame <= 12; ++frame) {
		const double t = frame * 100.0;
		samples.push_back(Hmd(t));
		samples.push_back(Trk(t, 1, "PHR-WAIST", BodyRole::Waist, 0.53, 0.00));
		samples.push_back(Trk(t, 2, "PHR-CHEST", BodyRole::Chest, 0.74, 0.00));
		samples.push_back(Trk(t, 3, "PHR-LFOOT", BodyRole::LeftFoot, 0.06, -0.12));
		samples.push_back(Trk(t, 4, "PHR-RFOOT", BodyRole::RightFoot, 0.06, 0.12));
		samples.push_back(Trk(t, 5, "PHR-LKNEE", BodyRole::LeftKnee, 0.28, -0.10));
		samples.push_back(Trk(t, 6, "PHR-RKNEE", BodyRole::RightKnee, 0.28, 0.10));
	}

	const ReplayScore score = ScoreReplay(samples);
	EXPECT_EQ(score.total, 6u);
	EXPECT_EQ(score.snap_correct, 6u) << "snap should recover every role from static geometry";
	EXPECT_GE(score.passive_correct, 2u) << "passive should at least recover the torso";
}

// Tuning hook: point WKOPENVR_PHANTOM_REPLAY_FILE at a real phantom_replay capture
// to score it. Skipped when unset so CI stays hermetic.
TEST(PhantomReplayPlayer, ScoresRealCaptureFromEnv)
{
	const char* path = std::getenv("WKOPENVR_PHANTOM_REPLAY_FILE");
	if (!path || !*path) {
		GTEST_SKIP() << "set WKOPENVR_PHANTOM_REPLAY_FILE to score a real capture";
	}
	std::ifstream f(path);
	ASSERT_TRUE(f.good()) << "cannot open " << path;
	std::vector<std::string> lines;
	std::string line;
	while (std::getline(f, line))
		lines.push_back(line);
	const ParsedReplay parsed = ParseReplay(lines);
	ASSERT_TRUE(parsed.ok) << parsed.error;
	const ReplayScore score = ScoreReplay(parsed.samples);
	for (const auto& t : score.trackers) {
		std::printf("  %-24s truth=%-11s passive=%-11s(%.2f) snap=%-11s(%.2f) detect=%.0fms\n", t.serial.c_str(),
		            BodyRoleToKey(t.ground_truth), BodyRoleToKey(t.passive_predicted), t.passive_confidence,
		            BodyRoleToKey(t.snap_predicted), t.snap_confidence, t.passive_detect_ms);
	}
	std::printf("[replay] passive=%u/%u snap=%u/%u over %.0f ms\n", score.passive_correct, score.total,
	            score.snap_correct, score.total, score.duration_ms);
	SUCCEED();
}
