#include "PhantomReplayBudget.h"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>

using namespace phantom;

namespace {

constexpr double kFrameMs = 1000.0 / 90.0;

struct SimDevice
{
	DeviceRecordState state;
	uint64_t writes = 0;
	uint64_t keyframes = 0;
};

// Advances one observation through the gate and commits accepted writes,
// mirroring the recorder's RecordPose flow.
RecordDecision Observe(SimDevice& dev, const ReplayBudgetConfig& cfg, bool isHmd, double nowMs, const double pos[3],
                       const double quat[4], uint32_t bits)
{
	const RecordDecision d = ShouldWriteRow(dev.state, cfg, isHmd, nowMs, pos, quat, bits);
	if (d.write) {
		CommitWrite(dev.state, nowMs, pos, quat, bits);
		++dev.writes;
		if (d.keyframe) ++dev.keyframes;
	}
	return d;
}

const double kIdentityQuat[4] = {1.0, 0.0, 0.0, 0.0};

} // namespace

TEST(PhantomReplayBudget, FirstObservationIsAKeyframe)
{
	SimDevice dev;
	ReplayBudgetConfig cfg;
	const double pos[3] = {0.0, 1.7, 0.0};
	const RecordDecision d = Observe(dev, cfg, true, 0.0, pos, kIdentityQuat, 0);
	EXPECT_TRUE(d.write);
	EXPECT_TRUE(d.keyframe);
}

TEST(PhantomReplayBudget, StaticDeviceReducesToKeyframes)
{
	SimDevice dev;
	ReplayBudgetConfig cfg;
	const double pos[3] = {1.0, 2.0, -0.5};
	for (int frame = 0; frame * kFrameMs < 20000.0; ++frame) {
		Observe(dev, cfg, false, frame * kFrameMs, pos, kIdentityQuat, 0);
	}
	// First write + one keyframe per 5 s interval over 20 s.
	EXPECT_GE(dev.writes, 4u);
	EXPECT_LE(dev.writes, 5u);
	EXPECT_EQ(dev.writes, dev.keyframes);
}

TEST(PhantomReplayBudget, JitterBelowEpsilonIsSuppressed)
{
	SimDevice dev;
	ReplayBudgetConfig cfg;
	cfg.keyframeIntervalMs = 0.0; // isolate the change test
	uint64_t decisions = 0;
	for (int frame = 0; frame * kFrameMs < 5000.0; ++frame) {
		const double t = frame * kFrameMs;
		// 2 mm wobble, well under the 5 mm epsilon.
		const double pos[3] = {0.002 * std::sin(t), 1.7, 0.0};
		const RecordDecision d = Observe(dev, cfg, false, t, pos, kIdentityQuat, 0);
		if (d.write) ++decisions;
	}
	EXPECT_EQ(1u, decisions); // only the initial keyframe
}

TEST(PhantomReplayBudget, ContinuousMotionIsCappedAtDeviceHz)
{
	SimDevice dev;
	ReplayBudgetConfig cfg;
	const double durationMs = 10000.0;
	for (int frame = 0; frame * kFrameMs < durationMs; ++frame) {
		const double t = frame * kFrameMs;
		// 1 m/s: ~11 mm per frame, always above epsilon.
		const double pos[3] = {t / 1000.0, 1.0, 0.0};
		Observe(dev, cfg, false, t, pos, kIdentityQuat, 0);
	}
	// 2 Hz cap over 10 s -> ~20 rows plus the initial keyframe; frame quantization allows slack.
	EXPECT_GE(dev.writes, 18u);
	EXPECT_LE(dev.writes, 23u);
}

TEST(PhantomReplayBudget, HmdUsesItsOwnCap)
{
	SimDevice dev;
	ReplayBudgetConfig cfg;
	const double durationMs = 10000.0;
	for (int frame = 0; frame * kFrameMs < durationMs; ++frame) {
		const double t = frame * kFrameMs;
		const double pos[3] = {t / 1000.0, 1.7, 0.0};
		Observe(dev, cfg, true, t, pos, kIdentityQuat, 0);
	}
	// 5 Hz cap over 10 s -> ~50 rows.
	EXPECT_GE(dev.writes, 45u);
	EXPECT_LE(dev.writes, 55u);
}

TEST(PhantomReplayBudget, DiscreteChangeBypassesRateCap)
{
	SimDevice dev;
	ReplayBudgetConfig cfg;
	const double pos[3] = {0.0, 1.0, 0.0};
	Observe(dev, cfg, false, 0.0, pos, kIdentityQuat, PackDiscreteBits(false, true, true, 1, 0, 0));
	// One frame later -- far inside the 2 Hz interval -- validity flips.
	const RecordDecision d =
	    Observe(dev, cfg, false, kFrameMs, pos, kIdentityQuat, PackDiscreteBits(false, false, true, 1, 0, 0));
	EXPECT_TRUE(d.write);
	EXPECT_TRUE(d.discreteChange);
}

TEST(PhantomReplayBudget, RotationAboveEpsilonCountsAsMotion)
{
	SimDevice dev;
	ReplayBudgetConfig cfg;
	cfg.keyframeIntervalMs = 0.0;
	const double pos[3] = {0.0, 1.0, 0.0};
	Observe(dev, cfg, false, 0.0, pos, kIdentityQuat, 0);

	// ~2 degree rotation about Y: 1-|dot| well above the 0.5 degree epsilon.
	const double halfAngle = 2.0 * 3.14159265358979 / 180.0 / 2.0;
	const double rotated[4] = {std::cos(halfAngle), 0.0, std::sin(halfAngle), 0.0};
	const RecordDecision d = Observe(dev, cfg, false, 600.0, pos, rotated, 0);
	EXPECT_TRUE(d.write);
}

TEST(PhantomReplayBudget, FullRateBypassesEverything)
{
	SimDevice dev;
	ReplayBudgetConfig cfg;
	cfg.fullRate = true;
	const double pos[3] = {0.0, 1.0, 0.0};
	uint64_t frames = 0;
	for (int frame = 0; frame * kFrameMs < 1000.0; ++frame) {
		Observe(dev, cfg, false, frame * kFrameMs, pos, kIdentityQuat, 0);
		++frames;
	}
	EXPECT_EQ(frames, dev.writes);
}

TEST(PhantomReplayBudget, WorstCaseHourFitsTheVolumeBudget)
{
	// 1 HMD + 10 devices, every one in constant motion for an hour at 90 Hz.
	constexpr int kDeviceCount = 11;
	constexpr double kDurationMs = 3600.0 * 1000.0;
	constexpr double kRowBytes = 200.0;

	SimDevice devices[kDeviceCount];
	ReplayBudgetConfig cfg;
	uint64_t totalWrites = 0;
	for (int frame = 0; frame * kFrameMs < kDurationMs; ++frame) {
		const double t = frame * kFrameMs;
		for (int i = 0; i < kDeviceCount; ++i) {
			const double pos[3] = {t / 1000.0 + i, 1.0, 0.0};
			Observe(devices[i], cfg, i == 0, t, pos, kIdentityQuat, 0);
		}
	}
	for (const auto& dev : devices) {
		totalWrites += dev.writes;
	}

	const double bytesPerHour = static_cast<double>(totalWrites) * kRowBytes;
	EXPECT_LE(bytesPerHour, 20.0 * 1024.0 * 1024.0)
	    << "rows=" << totalWrites << " estimated_bytes=" << static_cast<uint64_t>(bytesPerHour);
}
