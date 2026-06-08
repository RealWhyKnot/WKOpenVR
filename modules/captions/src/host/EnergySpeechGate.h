#pragma once

#include <cstddef>

namespace captions {

inline int AlwaysOnPrerollFrames()
{
	return 8; // ~256 ms at 512 samples / 16 kHz
}

inline int AlwaysOnSilenceFrames()
{
	return 24; // ~768 ms at 512 samples / 16 kHz
}

inline size_t AlwaysOnMinSpeechSamples()
{
	return 4800; // 300 ms of gated speech
}

inline size_t PushToTalkMinSpeechSamples()
{
	return 1600; // 100 ms of held PTT audio
}

inline bool SpeechBufferLongEnough(size_t gatedSamples, bool alwaysOn)
{
	return gatedSamples >= (alwaysOn ? AlwaysOnMinSpeechSamples() : PushToTalkMinSpeechSamples());
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

} // namespace captions
