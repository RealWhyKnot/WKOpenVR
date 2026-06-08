#pragma once

namespace captions {

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
