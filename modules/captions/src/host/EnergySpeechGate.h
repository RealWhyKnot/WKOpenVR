#pragma once

#include <algorithm>
#include <cstddef>
#include <deque>
#include <vector>

namespace captions {

inline int AlwaysOnPrerollFrames(bool extendedTiming = true)
{
	return extendedTiming ? 32 : 20; // ~1024 ms or ~640 ms at 512 samples / 16 kHz
}

inline int AlwaysOnSilenceFrames(bool extendedTiming = true)
{
	return extendedTiming ? 32 : 30; // ~1024 ms or ~960 ms at 512 samples / 16 kHz
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

inline size_t AlwaysOnWeakEvidenceSamples()
{
	return 8000; // 500 ms of gated speech
}

inline size_t AlwaysOnMaxSpeechSamples(bool extendedTiming = true)
{
	return extendedTiming ? 80000 : 128000; // 5 s or 8 s max before flushing a continuous segment
}

inline size_t AlwaysOnContinuationOverlapSamples(bool extendedTiming = true)
{
	return extendedTiming ? 9600 : 6400; // 600 ms or 400 ms repeated into the next continuous segment
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
                                          float maxFramePeak, float speechPeakThreshold, float maxFrameRms = 1.0f,
                                          float speechRmsThreshold = 0.0f)
{
	if (!alwaysOn) return gatedSamples >= PushToTalkMinSpeechSamples();
	if (gatedSamples < AlwaysOnShortSpeechSamples()) return false;

	const bool strongVadEvidence = maxVadProbability >= 0.68f;
	const bool usableVadEvidence = maxVadProbability >= 0.50f;
	const bool strongEnergyEvidence = maxFramePeak >= std::max(0.08f, speechPeakThreshold * 2.0f) &&
	                                  maxFrameRms >= std::max(0.022f, speechRmsThreshold * 1.45f);
	const bool sustainedEnergyEvidence = maxFramePeak >= std::max(0.050f, speechPeakThreshold * 1.35f) &&
	                                     maxFrameRms >= std::max(0.014f, speechRmsThreshold);

	if (gatedSamples < AlwaysOnWeakEvidenceSamples()) return strongVadEvidence || strongEnergyEvidence;

	return usableVadEvidence || sustainedEnergyEvidence;
}

inline float SpeechGateRatio(size_t numerator, size_t denominator)
{
	return denominator == 0 ? 0.0f : static_cast<float>(numerator) / static_cast<float>(denominator);
}

inline bool SpeechSegmentHasUsableShape(size_t totalFrames, size_t speechFrames, size_t possibleFrames,
                                        size_t gatedSamples, bool alwaysOn, float maxVadProbability, float maxFramePeak,
                                        float speechPeakThreshold, float maxFrameRms = 1.0f,
                                        float speechRmsThreshold = 0.0f)
{
	if (!alwaysOn || totalFrames == 0) return true;

	const bool strongVadEvidence = maxVadProbability >= 0.68f;
	const bool strongEnergyEvidence = maxFramePeak >= std::max(0.08f, speechPeakThreshold * 2.0f) &&
	                                  maxFrameRms >= std::max(0.022f, speechRmsThreshold * 1.45f);
	if (strongVadEvidence || strongEnergyEvidence) return true;

	const float speechFrameRatio = SpeechGateRatio(speechFrames, totalFrames);
	const float possibleFrameRatio = SpeechGateRatio(possibleFrames, totalFrames);
	if (gatedSamples < AlwaysOnWeakEvidenceSamples() && speechFrameRatio < 0.50f && possibleFrameRatio < 0.70f) {
		return false;
	}
	if (gatedSamples < 16000 && speechFrameRatio < 0.25f && possibleFrameRatio < 0.45f) {
		return false;
	}

	return true;
}

inline bool SpeechGateIsSpeech(float vadProbability, float framePeak, float frameRms)
{
	constexpr float kVadSpeechThreshold = 0.5f;
	constexpr float kEnergySpeechThreshold = 0.08f;
	constexpr float kEnergySpeechRmsThreshold = 0.018f;
	return vadProbability >= kVadSpeechThreshold ||
	       (framePeak >= kEnergySpeechThreshold && frameRms >= kEnergySpeechRmsThreshold);
}

inline bool SpeechGateIsSpeech(float vadProbability, float framePeak)
{
	constexpr float kVadSpeechThreshold = 0.5f;
	constexpr float kEnergySpeechThreshold = 0.08f;
	return vadProbability >= kVadSpeechThreshold || framePeak >= kEnergySpeechThreshold;
}

inline bool SpeechGateIsSilence(float vadProbability, float framePeak, float frameRms)
{
	constexpr float kVadSilenceThreshold = 0.35f;
	constexpr float kEnergySilenceThreshold = 0.025f;
	constexpr float kEnergySilenceRmsThreshold = 0.012f;
	return vadProbability < kVadSilenceThreshold && framePeak <= kEnergySilenceThreshold &&
	       frameRms <= kEnergySilenceRmsThreshold;
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
		return IsSpeech(vadProbability, framePeak, framePeak);
	}

	bool IsSpeech(float vadProbability, float framePeak, float frameRms) const
	{
		if (vadProbability >= 0.5f) return true;
		if (HasSpeechEnergy(framePeak, frameRms)) return true;
		return vadProbability >= 0.32f && HasPossibleSpeechEnergy(framePeak, frameRms);
	}

	bool IsSilence(float vadProbability, float framePeak) const
	{
		return IsSilence(vadProbability, framePeak, framePeak);
	}

	bool IsSilence(float vadProbability, float framePeak, float frameRms) const
	{
		return vadProbability < 0.35f && framePeak <= SilencePeakThreshold() && frameRms <= SilenceRmsThreshold();
	}

	bool IsPossibleSpeech(float vadProbability, float framePeak) const
	{
		return IsPossibleSpeech(vadProbability, framePeak, framePeak);
	}

	bool IsPossibleSpeech(float vadProbability, float framePeak, float frameRms) const
	{
		if (IsSpeech(vadProbability, framePeak, frameRms)) return true;
		if (vadProbability >= 0.20f && HasPossibleSpeechEnergy(framePeak, frameRms)) return true;
		return HasPossibleSpeechEnergy(framePeak, frameRms);
	}

	void ObserveAmbient(float framePeak) { ObserveAmbient(framePeak, framePeak * 0.5f); }

	void ObserveAmbient(float framePeak, float frameRms)
	{
		framePeak = Clamp(framePeak, 0.0f, 1.0f);
		frameRms = Clamp(frameRms, 0.0f, 1.0f);
		const float alpha = (framePeak > ambient_peak_) ? 0.015f : 0.08f;
		ambient_peak_ += (framePeak - ambient_peak_) * alpha;
		ambient_peak_ = Clamp(ambient_peak_, 0.001f, 0.25f);
		const float rmsAlpha = (frameRms > ambient_rms_) ? 0.020f : 0.10f;
		ambient_rms_ += (frameRms - ambient_rms_) * rmsAlpha;
		ambient_rms_ = Clamp(ambient_rms_, 0.0005f, 0.18f);
	}

	bool ObserveRejectedSegment(float maxVadProbability, float maxFramePeak, float maxFrameRms)
	{
		if (maxVadProbability >= 0.45f) return false;
		ObserveAmbient(maxFramePeak, maxFrameRms);
		return true;
	}

	float AmbientPeak() const { return ambient_peak_; }
	float AmbientRms() const { return ambient_rms_; }

	float SpeechPeakThreshold() const { return Clamp((ambient_peak_ * 3.6f) + 0.018f, 0.028f, 0.14f); }

	float SpeechRmsThreshold() const { return Clamp((ambient_rms_ * 3.2f) + 0.006f, 0.010f, 0.065f); }

	float SoftSpeechPeakThreshold() const { return SpeechPeakThreshold() * 0.70f; }

	float SoftSpeechRmsThreshold() const { return SpeechRmsThreshold() * 0.72f; }

	float PossibleSpeechPeakThreshold() const
	{
		return std::max(SilencePeakThreshold() * 1.40f, SpeechPeakThreshold() * 0.62f);
	}

	float PossibleSpeechRmsThreshold() const
	{
		return std::max(SilenceRmsThreshold() * 1.35f, SpeechRmsThreshold() * 0.62f);
	}

	float SilencePeakThreshold() const
	{
		const float threshold = (ambient_peak_ * 1.6f) + 0.006f;
		return Clamp(threshold, 0.014f, SpeechPeakThreshold() * 0.70f);
	}

	float SilenceRmsThreshold() const
	{
		const float threshold = (ambient_rms_ * 1.7f) + 0.003f;
		return Clamp(threshold, 0.006f, SpeechRmsThreshold() * 0.72f);
	}

private:
	static float Clamp(float value, float lo, float hi) { return std::max(lo, std::min(value, hi)); }

	bool HasSpeechEnergy(float framePeak, float frameRms) const
	{
		const bool absolute = framePeak >= SpeechPeakThreshold() && frameRms >= SpeechRmsThreshold();
		const bool relative =
		    framePeak >= (ambient_peak_ * 4.0f + 0.012f) && frameRms >= (ambient_rms_ * 3.0f + 0.004f);
		const bool strongPeakWithBody =
		    framePeak >= SpeechPeakThreshold() * 1.6f && frameRms >= SpeechRmsThreshold() * 0.82f;
		return absolute || relative || strongPeakWithBody;
	}

	bool HasPossibleSpeechEnergy(float framePeak, float frameRms) const
	{
		return framePeak >= PossibleSpeechPeakThreshold() && frameRms >= PossibleSpeechRmsThreshold();
	}

	float ambient_peak_ = 0.004f;
	float ambient_rms_ = 0.0015f;
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
