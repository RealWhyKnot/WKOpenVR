#define _CRT_SECURE_NO_WARNINGS
#include "SmoothingRecordingSchema.h"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <map>
#include <string>
#include <utility>
#include <vector>

using namespace skeletal;
using namespace skeletal::recording;

namespace {

constexpr double kFrameMs = 1000.0 / 90.0;
constexpr uint16_t kAllFingers = 0x3FF;

vr::VRBoneTransform_t IdentityBone()
{
	vr::VRBoneTransform_t bone{};
	bone.orientation.w = 1.0f;
	return bone;
}

// Finger bones (2..25) carry the signal; the pass-through bones stay fixed so
// jerk comparisons only see smoothed output.
SmoothingFrameRow MakeFrame(int frameIdx, float fingerX)
{
	SmoothingFrameRow row;
	row.timeMs = frameIdx * kFrameMs;
	row.frameIdx = frameIdx;
	row.windowId = 1;
	for (uint32_t b = 0; b < math::kFingerBoneCount; ++b) {
		row.bones[b] = IdentityBone();
		if (math::FingerIndexForBone(b) >= 0) {
			row.bones[b].position.v[0] = fingerX;
			row.bones[b].position.v[1] = 0.1f * static_cast<float>(b);
		}
	}
	return row;
}

std::vector<SmoothingFrameRow> NoisyWindow(int frames)
{
	std::vector<SmoothingFrameRow> out;
	out.reserve(frames);
	for (int i = 0; i < frames; ++i) {
		// Deterministic 4 mm alternating jitter.
		const float x = ((i % 2) == 0) ? 0.004f : -0.004f;
		out.push_back(MakeFrame(i, x));
	}
	return out;
}

std::vector<SmoothingFrameRow> StepWindow(int frames)
{
	std::vector<SmoothingFrameRow> out;
	out.reserve(frames);
	for (int i = 0; i < frames; ++i) {
		out.push_back(MakeFrame(i, i < frames / 2 ? 0.0f : 0.05f));
	}
	return out;
}

} // namespace

TEST(SmoothingRecordingSchema, FrameRowRoundTrips)
{
	SmoothingFrameRow row = MakeFrame(7, 0.1234f);
	row.hand = 1;
	row.motionRange = 1;
	row.windowId = 3;

	const std::vector<std::string> lines = {
	    std::string("# ") + kSchemaBanner,
	    "# build_stamp=test",
	    BuildColumnHeader(),
	    "# [0.500] window: frames=90 period_ms=60000",
	    FormatFrameRow(row),
	};
	const LoadedSmoothingRecording parsed = ParseSmoothingRecording(lines);
	ASSERT_TRUE(parsed.ok) << parsed.error;
	ASSERT_EQ(1u, parsed.frames.size());
	EXPECT_TRUE(parsed.summaries.empty());

	const SmoothingFrameRow& r = parsed.frames[0];
	EXPECT_EQ(1, r.hand);
	EXPECT_EQ(1, r.motionRange);
	EXPECT_EQ(3u, r.windowId);
	EXPECT_EQ(7, r.frameIdx);
	for (uint32_t b = 0; b < math::kFingerBoneCount; ++b) {
		EXPECT_NEAR(row.bones[b].position.v[0], r.bones[b].position.v[0], 1e-4f);
		EXPECT_NEAR(row.bones[b].position.v[1], r.bones[b].position.v[1], 1e-4f);
		EXPECT_NEAR(row.bones[b].orientation.w, r.bones[b].orientation.w, 1e-4f);
	}
}

TEST(SmoothingRecordingSchema, SummaryRowRoundTrips)
{
	SmoothingSummaryRow row;
	row.timeMs = 1000.0;
	row.hand = 0;
	for (int f = 0; f < math::kFingersPerHand; ++f)
		row.alpha[f] = 0.1f * static_cast<float>(f + 1);
	row.fingerMask = kAllFingers;
	row.frames = 180;
	row.smoothedFrames = 90;
	row.winMaxPosDelta = 0.0123f;
	row.winMaxPosBone = 7;
	row.winMinQuatDot = 0.9876f;
	row.winMinQuatBone = 11;

	const std::vector<std::string> lines = {
	    std::string("# ") + kSchemaBanner,
	    BuildColumnHeader(),
	    FormatSummaryRow(row),
	};
	const LoadedSmoothingRecording parsed = ParseSmoothingRecording(lines);
	ASSERT_TRUE(parsed.ok) << parsed.error;
	ASSERT_EQ(1u, parsed.summaries.size());
	EXPECT_TRUE(parsed.frames.empty());

	const SmoothingSummaryRow& r = parsed.summaries[0];
	EXPECT_EQ(0, r.hand);
	for (int f = 0; f < math::kFingersPerHand; ++f) {
		EXPECT_NEAR(row.alpha[f], r.alpha[f], 1e-4f);
	}
	EXPECT_EQ(kAllFingers, r.fingerMask);
	EXPECT_EQ(180u, r.frames);
	EXPECT_EQ(90u, r.smoothedFrames);
	EXPECT_NEAR(0.0123f, r.winMaxPosDelta, 1e-4f);
	EXPECT_EQ(7, r.winMaxPosBone);
	EXPECT_NEAR(0.9876f, r.winMinQuatDot, 1e-4f);
	EXPECT_EQ(11, r.winMinQuatBone);
}

TEST(SmoothingRecordingSchema, WindowSchedulerRecordsPeriodicBursts)
{
	WindowScheduler scheduler;
	uint64_t recorded = 0;
	uint64_t lastWindowId = 0;
	int lastFrameIdx = -1;
	for (int frame = 0; frame * kFrameMs < 3.0 * 60000.0; ++frame) {
		const auto d = scheduler.OnFrame(frame * kFrameMs);
		if (!d.record) continue;
		++recorded;
		if (d.windowId != lastWindowId) {
			EXPECT_EQ(0, d.frameIdx) << "window must start at frame 0";
			lastWindowId = d.windowId;
		}
		else {
			EXPECT_EQ(lastFrameIdx + 1, d.frameIdx) << "frames within a window are contiguous";
		}
		lastFrameIdx = d.frameIdx;
	}
	EXPECT_EQ(3u, lastWindowId);
	EXPECT_EQ(3u * kWindowFrames, recorded);
}

TEST(SmoothingRecordingSchema, HourOfWindowsFitsTheVolumeBudget)
{
	// Both hands, 90 Hz, one hour. Frame rows dominate; summaries add ~1 MB/h.
	constexpr double kFrameRowBytes = 1650.0;
	WindowScheduler left, right;
	uint64_t rows = 0;
	for (int frame = 0; frame * kFrameMs < 3600.0 * 1000.0; ++frame) {
		const double t = frame * kFrameMs;
		if (left.OnFrame(t).record) ++rows;
		if (right.OnFrame(t).record) ++rows;
	}
	const double bytesPerHour = static_cast<double>(rows) * kFrameRowBytes + 1024.0 * 1024.0;
	EXPECT_LE(bytesPerHour, 20.0 * 1024.0 * 1024.0)
	    << "rows=" << rows << " estimated_bytes=" << static_cast<uint64_t>(bytesPerHour);
}

TEST(SmoothingRecordingReplay, StrongerSmoothingLowersJerkOnNoisyInput)
{
	const auto window = NoisyWindow(90);
	const float weak[math::kFingersPerHand] = {0.9f, 0.9f, 0.9f, 0.9f, 0.9f};
	const float strong[math::kFingersPerHand] = {0.2f, 0.2f, 0.2f, 0.2f, 0.2f};

	const auto weakMetrics = ScoreWindow(window, weak, kAllFingers, 0);
	const auto strongMetrics = ScoreWindow(window, strong, kAllFingers, 0);
	ASSERT_EQ(90, weakMetrics.framesScored);
	EXPECT_LT(strongMetrics.maxInterFrameOutPosDelta, weakMetrics.maxInterFrameOutPosDelta);
	EXPECT_LT(strongMetrics.meanInterFrameOutPosDelta, weakMetrics.meanInterFrameOutPosDelta);
}

TEST(SmoothingRecordingReplay, StrongerSmoothingLagsBehindSteps)
{
	const auto window = StepWindow(90);
	const float weak[math::kFingersPerHand] = {0.9f, 0.9f, 0.9f, 0.9f, 0.9f};
	const float strong[math::kFingersPerHand] = {0.2f, 0.2f, 0.2f, 0.2f, 0.2f};

	const auto weakMetrics = ScoreWindow(window, weak, kAllFingers, 0);
	const auto strongMetrics = ScoreWindow(window, strong, kAllFingers, 0);
	EXPECT_GT(strongMetrics.meanLagM, weakMetrics.meanLagM);
}

// Tuning hook: replays a real smoothing_rec capture with candidate parameters
// and prints jerk/lag per window against the recorded parameters. Skipped
// unless WKOPENVR_REPLAY_RECORDINGS=1 and WKOPENVR_SMOOTHING_REPLAY_FILE point
// at a capture, so CI stays hermetic.
TEST(SmoothingRecordingReplay, ReplayLocalRecordingsWhenRequested)
{
	const char* master = std::getenv("WKOPENVR_REPLAY_RECORDINGS");
	if (!master || std::string(master) != "1") {
		GTEST_SKIP() << "set WKOPENVR_REPLAY_RECORDINGS=1 to replay local recordings";
	}
	const char* path = std::getenv("WKOPENVR_SMOOTHING_REPLAY_FILE");
	if (!path || !*path) {
		GTEST_SKIP() << "set WKOPENVR_SMOOTHING_REPLAY_FILE to a smoothing_rec capture";
	}
	uint8_t candidateSmoothness = 80;
	if (const char* s = std::getenv("WKOPENVR_SMOOTHING_REPLAY_SMOOTHNESS")) {
		const long v = std::strtol(s, nullptr, 10);
		if (v >= 0 && v <= 100) candidateSmoothness = static_cast<uint8_t>(v);
	}

	std::ifstream f(path);
	ASSERT_TRUE(f.good()) << "cannot open " << path;
	std::vector<std::string> lines;
	std::string line;
	while (std::getline(f, line))
		lines.push_back(line);
	const LoadedSmoothingRecording rec = ParseSmoothingRecording(lines);
	ASSERT_TRUE(rec.ok) << rec.error;
	ASSERT_FALSE(rec.frames.empty()) << "capture has no frame windows";

	std::map<std::pair<int, uint64_t>, std::vector<SmoothingFrameRow>> windows;
	for (const auto& frame : rec.frames) {
		windows[{frame.hand, frame.windowId}].push_back(frame);
	}

	const float candidateAlpha = math::SmoothnessToAlpha(candidateSmoothness);
	float candidate[math::kFingersPerHand];
	for (int fi = 0; fi < math::kFingersPerHand; ++fi)
		candidate[fi] = candidateAlpha;

	for (const auto& [key, frames] : windows) {
		const int hand = key.first;
		// Recorded parameters: the summary for this hand closest in time to
		// the window start; sessions without summaries score against
		// passthrough (alpha=1).
		float recorded[math::kFingersPerHand] = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f};
		uint16_t mask = kAllFingers;
		double bestDist = 1e300;
		for (const auto& summary : rec.summaries) {
			if (summary.hand != hand) continue;
			const double dist = std::fabs(summary.timeMs - frames.front().timeMs);
			if (dist < bestDist) {
				bestDist = dist;
				for (int fi = 0; fi < math::kFingersPerHand; ++fi)
					recorded[fi] = summary.alpha[fi];
				mask = summary.fingerMask;
			}
		}

		const int handBase = hand * math::kFingersPerHand;
		const auto base = ScoreWindow(frames, recorded, mask, handBase);
		const auto cand = ScoreWindow(frames, candidate, mask, handBase);
		std::printf("[smoothing-replay] hand=%d window=%llu frames=%d | recorded jerk max=%.4f mean=%.5f lag=%.5f | "
		            "candidate(s=%u) jerk max=%.4f mean=%.5f lag=%.5f\n",
		            hand, (unsigned long long)key.second, base.framesScored, base.maxInterFrameOutPosDelta,
		            base.meanInterFrameOutPosDelta, base.meanLagM, (unsigned)candidateSmoothness,
		            cand.maxInterFrameOutPosDelta, cand.meanInterFrameOutPosDelta, cand.meanLagM);
	}
	SUCCEED();
}
