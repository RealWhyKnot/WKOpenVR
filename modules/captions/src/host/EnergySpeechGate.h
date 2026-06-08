#pragma once

#include <algorithm>
#include <cstddef>
#include <deque>
#include <vector>

namespace captions {

inline int AlwaysOnPrerollFrames()
{
	return 32; // ~1024 ms at 512 samples / 16 kHz
}

inline int AlwaysOnSilenceFrames()
{
	return 32; // ~1024 ms at 512 samples / 16 kHz
}

inline int AlwaysOnActivationWindowFrames()
{
	return 5; // ~160 ms activation window
}

inline int AlwaysOnActivationSpeechFrames()
{
	return 2;
}

inline int AlwaysOnActivationPossibleFrames()
{
	return 3;
}

inline size_t AlwaysOnMinSpeechSamples()
{
	return 3200; // 200 ms of gated speech
}

inline size_t AlwaysOnShortSpeechSamples()
{
	return 1600; // 100 ms of confident gated speech
}

inline size_t AlwaysOnMaxSpeechSamples()
{
	return 80000; // 5 s max before flushing a continuous segment
}

inline size_t AlwaysOnContinuationOverlapSamples()
{
	return 9600; // 600 ms repeated into the next continuous segment
}

inline size_t PushToTalkMinSpeechSamples()
{
	return 1600; // 100 ms of held PTT audio
}

inline bool SpeechBufferLongEnough(size_t gatedSamples, bool alwaysOn)
{
	return gatedSamples >= (alwaysOn ? AlwaysOnMinSpeechSamples() : PushToTalkMinSpeechSamples());
}

inline bool SpeechSegmentShouldTranscribe(size_t gatedSamples, bool alwaysOn, float maxVadProbability,
                                          float maxFramePeak, float speechPeakThreshold)
{
	if (!alwaysOn) return gatedSamples >= PushToTalkMinSpeechSamples();
	if (gatedSamples >= AlwaysOnMinSpeechSamples()) return true;
	if (gatedSamples < AlwaysOnShortSpeechSamples()) return false;

	const float strongPeakThreshold = std::max(0.07f, speechPeakThreshold * 2.0f);
	return maxVadProbability >= 0.70f || maxFramePeak >= strongPeakThreshold;
}

inline bool SpeechGateIsSpeech(float vadProbability, float framePeak)
{
	constexpr float kVadSpeechThreshold = 0.5f;
	constexpr float kEnergySpeechThreshold = 0.08f;
	return vadProbability >= kVadSpeechThreshold || framePeak >= kEnergySpeechThreshold;
}

inline bool SpeechGateIsSilence(float vadProbability, float framePeak)
{
	constexpr float kVadSilenceThreshold = 0.35f;
	constexpr float kEnergySilenceThreshold = 0.025f;
	return vadProbability < kVadSilenceThreshold && framePeak <= kEnergySilenceThreshold;
}

class AdaptiveSpeechGate
{
public:
	bool IsSpeech(float vadProbability, float framePeak) const
	{
		if (vadProbability >= 0.5f) return true;
		if (framePeak >= SpeechPeakThreshold()) return true;
		return vadProbability >= 0.32f && framePeak >= SoftSpeechPeakThreshold();
	}

	bool IsSilence(float vadProbability, float framePeak) const
	{
		return vadProbability < 0.35f && framePeak <= SilencePeakThreshold();
	}

	bool IsPossibleSpeech(float vadProbability, float framePeak) const
	{
		if (IsSpeech(vadProbability, framePeak)) return true;
		if (vadProbability >= 0.20f) return true;
		return framePeak >= PossibleSpeechPeakThreshold();
	}

	void ObserveAmbient(float framePeak)
	{
		framePeak = Clamp(framePeak, 0.0f, 1.0f);
		const float alpha = (framePeak > ambient_peak_) ? 0.015f : 0.08f;
		ambient_peak_ += (framePeak - ambient_peak_) * alpha;
		ambient_peak_ = Clamp(ambient_peak_, 0.001f, 0.25f);
	}

	float AmbientPeak() const { return ambient_peak_; }

	float SpeechPeakThreshold() const { return Clamp((ambient_peak_ * 2.5f) + 0.012f, 0.022f, 0.10f); }

	float SoftSpeechPeakThreshold() const { return SpeechPeakThreshold() * 0.72f; }

	float PossibleSpeechPeakThreshold() const
	{
		return std::max(SilencePeakThreshold() * 1.25f, SpeechPeakThreshold() * 0.55f);
	}

	float SilencePeakThreshold() const
	{
		const float threshold = (ambient_peak_ * 1.6f) + 0.006f;
		return Clamp(threshold, 0.014f, SpeechPeakThreshold() * 0.70f);
	}

private:
	static float Clamp(float value, float lo, float hi) { return std::max(lo, std::min(value, hi)); }

	float ambient_peak_ = 0.004f;
};

class SpeechActivationWindow
{
public:
	void Reset() { frames_.clear(); }

	void Push(bool speechFrame, bool possibleSpeechFrame)
	{
		frames_.push_back({speechFrame, speechFrame || possibleSpeechFrame});
		while (frames_.size() > static_cast<size_t>(AlwaysOnActivationWindowFrames())) {
			frames_.pop_front();
		}
	}

	bool ShouldOpen() const
	{
		const int speech = SpeechFrames();
		if (speech >= AlwaysOnActivationSpeechFrames()) return true;
		return speech >= 1 && PossibleFrames() >= AlwaysOnActivationPossibleFrames();
	}

	int SpeechFrames() const
	{
		int count = 0;
		for (const Frame& frame : frames_) {
			if (frame.speech) ++count;
		}
		return count;
	}

	int PossibleFrames() const
	{
		int count = 0;
		for (const Frame& frame : frames_) {
			if (frame.possible) ++count;
		}
		return count;
	}

	size_t Size() const { return frames_.size(); }

private:
	struct Frame
	{
		bool speech = false;
		bool possible = false;
	};

	std::deque<Frame> frames_;
};

inline std::vector<float> CopyTrailingSamples(const std::vector<float>& pcm, size_t keepSamples)
{
	if (keepSamples == 0 || pcm.empty()) return {};
	const size_t start = pcm.size() > keepSamples ? pcm.size() - keepSamples : 0;
	return std::vector<float>(pcm.begin() + static_cast<std::ptrdiff_t>(start), pcm.end());
}

} // namespace captions
