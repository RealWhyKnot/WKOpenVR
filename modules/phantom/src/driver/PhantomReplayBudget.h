#pragma once

#include <cstdint>

// Volume budget for the phantom replay recording. The raw pose stream is one
// row per device per update (~90 Hz x every tracked device), which fills a
// disk in a few dev sessions. Replays mostly need decision points, not dense
// motion, so rows are gated three ways:
//   - discrete state changes (validity, connection, roles) write immediately;
//   - continuous motion writes at a capped per-device rate;
//   - a periodic keyframe writes even when nothing changes, so every device
//     stays enumerable and truncated captures keep ground truth.
// Worst case (every device in constant motion) this holds ~25 rows/sec at 11
// devices (~18 MB/h); an idle rig writes keyframes only (~1.6 MB/h).

namespace phantom {

struct ReplayBudgetConfig
{
	double maxHzHmd = 5.0;
	double maxHzDevice = 2.0;
	double keyframeIntervalMs = 5000.0;
	double posEpsilonM = 0.005;     // per-axis; above tracking noise
	double quatDotEpsilon = 3.8e-5; // 1-|q.q'| for ~0.5 degrees
	bool fullRate = false;          // record every row (dense capture)
};

struct DeviceRecordState
{
	double lastWrittenMs = 0.0;
	double pos[3] = {};
	double quat[4] = {};
	uint32_t discreteBits = 0;
	bool hasWritten = false;
};

struct RecordDecision
{
	bool write = false;
	bool keyframe = false;
	bool discreteChange = false;
};

// Discrete row fields packed for a single equality test. Any change here is a
// decision point and bypasses the rate cap.
inline uint32_t PackDiscreteBits(bool dropoutEnabled, bool poseValid, bool connected, int result, int controllerRole,
                                 int bodyRole)
{
	return (dropoutEnabled ? 1u : 0u) | (poseValid ? 2u : 0u) | (connected ? 4u : 0u) |
	       ((static_cast<uint32_t>(result) & 0xFu) << 3) | ((static_cast<uint32_t>(controllerRole) & 0xFu) << 7) |
	       ((static_cast<uint32_t>(bodyRole) & 0xFFu) << 11);
}

inline RecordDecision ShouldWriteRow(const DeviceRecordState& state, const ReplayBudgetConfig& cfg, bool isHmd,
                                     double nowMs, const double pos[3], const double quat[4], uint32_t discreteBits)
{
	RecordDecision decision;
	if (cfg.fullRate) {
		decision.write = true;
		return decision;
	}
	if (!state.hasWritten) {
		decision.write = true;
		decision.keyframe = true;
		return decision;
	}
	if (discreteBits != state.discreteBits) {
		decision.write = true;
		decision.discreteChange = true;
		return decision;
	}

	const double sinceMs = nowMs - state.lastWrittenMs;
	if (cfg.keyframeIntervalMs > 0.0 && sinceMs >= cfg.keyframeIntervalMs) {
		decision.write = true;
		decision.keyframe = true;
		return decision;
	}

	// Velocity is intentionally not part of the change test: it is noisy on
	// every real device and would defeat the suppression. It still lands in
	// whatever rows do get written.
	bool moved = false;
	for (int i = 0; i < 3 && !moved; ++i) {
		const double delta = pos[i] - state.pos[i];
		moved = (delta > cfg.posEpsilonM) || (delta < -cfg.posEpsilonM);
	}
	if (!moved) {
		double dot =
		    state.quat[0] * quat[0] + state.quat[1] * quat[1] + state.quat[2] * quat[2] + state.quat[3] * quat[3];
		if (dot < 0.0) dot = -dot;
		moved = (1.0 - dot) > cfg.quatDotEpsilon;
	}
	if (!moved) return decision;

	const double maxHz = isHmd ? cfg.maxHzHmd : cfg.maxHzDevice;
	if (maxHz <= 0.0) return decision;
	decision.write = sinceMs >= (1000.0 / maxHz);
	return decision;
}

inline void CommitWrite(DeviceRecordState& state, double nowMs, const double pos[3], const double quat[4],
                        uint32_t discreteBits)
{
	state.lastWrittenMs = nowMs;
	for (int i = 0; i < 3; ++i)
		state.pos[i] = pos[i];
	for (int i = 0; i < 4; ++i)
		state.quat[i] = quat[i];
	state.discreteBits = discreteBits;
	state.hasWritten = true;
}

} // namespace phantom
