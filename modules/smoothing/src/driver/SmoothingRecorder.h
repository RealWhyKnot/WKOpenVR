#pragma once

#include "RecordingEnvelope.h"
#include "SkeletalSmoothingMath.h"
#include "SmoothingRecordingSchema.h"

#include <openvr_driver.h>

#include <chrono>
#include <cstdint>
#include <string>

namespace skeletal::recording {

// Envelope-backed writer for smoothing_rec captures. Not thread-safe: the
// caller invokes it from inside the skeletal hand-state lock, which already
// serializes every smoothing call.
class SmoothingRecorder
{
public:
	void OnSmoothedFrame(int hand, int motionRange, const vr::VRBoneTransform_t* input, uint32_t count,
	                     const float alphaPerFinger[math::kFingersPerHand], uint16_t fingerMask,
	                     const math::FingerFrameResult& result);
	void Annotate(const std::string& event);
	void Close();

private:
	bool ShouldRecord();
	bool OpenIfNeeded();
	double NowMs();

	struct HandAccumulator
	{
		WindowScheduler scheduler;
		double lastSummaryMs = -1e18;
		uint64_t frames = 0;
		uint64_t smoothedFrames = 0;
		float winMaxPosDelta = 0.0f;
		int winMaxPosBone = -1;
		float winMinQuatDot = 1.0f;
		int winMinQuatBone = -1;
	};

	bool checked_enabled_ = false;
	bool enabled_ = false;
	bool open_attempted_ = false;
	bool clock_started_ = false;
	std::chrono::steady_clock::time_point clock_base_{};
	openvr_pair::common::recording::RecordingEnvelope envelope_;
	HandAccumulator hands_[2];
};

} // namespace skeletal::recording
