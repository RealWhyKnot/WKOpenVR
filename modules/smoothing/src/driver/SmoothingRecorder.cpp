#include "SmoothingRecorder.h"

#include "DebugLogging.h"
#include "Logging.h"

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include <iterator>
#include <sstream>

namespace skeletal::recording {

namespace {

constexpr std::size_t kRetentionMaxFiles = 5;
constexpr uint64_t kRetentionMaxBytes = 256ull * 1024ull * 1024ull;

bool EnvFlagEnabled(const wchar_t* name)
{
	wchar_t buf[32]{};
	const DWORD n = GetEnvironmentVariableW(name, buf, (DWORD)std::size(buf));
	if (n == 0 || n >= std::size(buf)) return false;
	return buf[0] == L'1' || buf[0] == L'y' || buf[0] == L'Y' || buf[0] == L't' || buf[0] == L'T';
}

} // namespace

bool SmoothingRecorder::ShouldRecord()
{
	if (!checked_enabled_) {
		checked_enabled_ = true;
		if (EnvFlagEnabled(L"WKOPENVR_NO_SMOOTHING_REC_RECORD")) {
			enabled_ = false;
		}
		else if (EnvFlagEnabled(L"WKOPENVR_SMOOTHING_REC_RECORD")) {
			enabled_ = true;
		}
		else {
			enabled_ = openvr_pair::common::IsDebugLoggingEnabled();
		}
	}
	return enabled_;
}

double SmoothingRecorder::NowMs()
{
	const auto now = std::chrono::steady_clock::now();
	if (!clock_started_) {
		clock_started_ = true;
		clock_base_ = now;
	}
	return std::chrono::duration<double, std::milli>(now - clock_base_).count();
}

bool SmoothingRecorder::OpenIfNeeded()
{
	if (envelope_.IsOpen()) return true;
	if (open_attempted_) return false;
	open_attempted_ = true;

	namespace recording = openvr_pair::common::recording;
	recording::EnvelopeOptions options;
	options.prefix = L"smoothing_rec";
	options.extension = L"csv";
	options.schemaBanner = kSchemaBanner;
	options.headerKVs = {{"columns", BuildColumnHeader()}};
	options.retention.maxFiles = kRetentionMaxFiles;
	options.retention.maxTotalBytes = kRetentionMaxBytes;

	if (!envelope_.Open(options)) {
		LOG("[smoothing][rec] failed to open recording");
		return false;
	}
	envelope_.WriteRow(BuildColumnHeader());

	std::ostringstream params;
	params << "window: frames=" << kWindowFrames << " period_ms=" << kWindowPeriodMs
	       << " summary_ms=" << kSummaryIntervalMs;
	envelope_.WriteAnnotation(0.0, params.str());

	LOG("[smoothing][rec] recording to %ls", envelope_.Path().c_str());
	return true;
}

void SmoothingRecorder::OnSmoothedFrame(int hand, int motionRange, const vr::VRBoneTransform_t* input, uint32_t count,
                                        const float alphaPerFinger[math::kFingersPerHand], uint16_t fingerMask,
                                        const math::FingerFrameResult& result)
{
	if (hand < 0 || hand > 1 || !input || count < math::kFingerBoneCount) return;
	if (!ShouldRecord()) return;
	if (!OpenIfNeeded()) return;

	const double nowMs = NowMs();
	HandAccumulator& acc = hands_[hand];

	++acc.frames;
	if (result.appliedSmoothing) ++acc.smoothedFrames;
	if (result.maxPosDelta > acc.winMaxPosDelta) {
		acc.winMaxPosDelta = result.maxPosDelta;
		acc.winMaxPosBone = result.maxPosDeltaBone;
	}
	if (result.minQuatDot < acc.winMinQuatDot) {
		acc.winMinQuatDot = result.minQuatDot;
		acc.winMinQuatBone = result.minQuatDotBone;
	}

	const WindowScheduler::Decision window = acc.scheduler.OnFrame(nowMs);
	if (window.record) {
		SmoothingFrameRow row;
		row.timeMs = nowMs;
		row.hand = hand;
		row.motionRange = motionRange;
		row.windowId = window.windowId;
		row.frameIdx = window.frameIdx;
		for (uint32_t b = 0; b < math::kFingerBoneCount; ++b) {
			row.bones[b] = input[b];
		}
		envelope_.WriteRow(FormatFrameRow(row));
	}

	if (nowMs - acc.lastSummaryMs >= kSummaryIntervalMs) {
		acc.lastSummaryMs = nowMs;
		SmoothingSummaryRow summary;
		summary.timeMs = nowMs;
		summary.hand = hand;
		for (int f = 0; f < math::kFingersPerHand; ++f) {
			summary.alpha[f] = alphaPerFinger[f];
		}
		summary.fingerMask = fingerMask;
		summary.frames = acc.frames;
		summary.smoothedFrames = acc.smoothedFrames;
		summary.winMaxPosDelta = acc.winMaxPosDelta;
		summary.winMaxPosBone = acc.winMaxPosBone;
		summary.winMinQuatDot = acc.winMinQuatDot;
		summary.winMinQuatBone = acc.winMinQuatBone;
		envelope_.WriteRow(FormatSummaryRow(summary));
		acc.winMaxPosDelta = 0.0f;
		acc.winMaxPosBone = -1;
		acc.winMinQuatDot = 1.0f;
		acc.winMinQuatBone = -1;
	}
}

void SmoothingRecorder::Annotate(const std::string& event)
{
	if (!envelope_.IsOpen()) return;
	envelope_.WriteAnnotation(NowMs() / 1000.0, event);
}

void SmoothingRecorder::Close()
{
	envelope_.Close();
}

} // namespace skeletal::recording
