#pragma once

#include <cstdint>

namespace openvr_pair::overlay {

struct FrameHitchSample
{
	double frameGapMs = 0.0;
	double totalMs = 0.0;
	double slowStageMs = 0.0;
};

struct FrameHitchDecision
{
	bool shouldLog = false;
	uint32_t suppressedSinceLastLog = 0;
};

inline bool IsFrameHitchWarning(const FrameHitchSample& sample)
{
	return sample.frameGapMs >= 250.0 || sample.totalMs >= 250.0 || sample.slowStageMs >= 75.0;
}

class FrameHitchGate
{
public:
	double BeginFrame(double nowSeconds)
	{
		double frameGapMs = 0.0;
		if (lastFrameStartSeconds_ > 0.0 && nowSeconds >= lastFrameStartSeconds_) {
			frameGapMs = (nowSeconds - lastFrameStartSeconds_) * 1000.0;
		}
		lastFrameStartSeconds_ = nowSeconds;
		return frameGapMs;
	}

	FrameHitchDecision Evaluate(double nowSeconds, const FrameHitchSample& sample)
	{
		if (!IsFrameHitchWarning(sample)) return {};

		if (lastLogSeconds_ > 0.0 && nowSeconds >= lastLogSeconds_ &&
		    nowSeconds - lastLogSeconds_ < kLogThrottleSeconds) {
			++suppressedSinceLastLog_;
			return {};
		}

		FrameHitchDecision decision;
		decision.shouldLog = true;
		decision.suppressedSinceLastLog = suppressedSinceLastLog_;
		suppressedSinceLastLog_ = 0;
		lastLogSeconds_ = nowSeconds;
		return decision;
	}

private:
	static constexpr double kLogThrottleSeconds = 1.0;

	double lastFrameStartSeconds_ = 0.0;
	double lastLogSeconds_ = 0.0;
	uint32_t suppressedSinceLastLog_ = 0;
};

} // namespace openvr_pair::overlay
