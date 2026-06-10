#pragma once

#include <cstdint>

namespace captions {

static constexpr uint8_t kCaptionsSpeechModelBalanced = 0;
static constexpr uint8_t kCaptionsSpeechModelHighAccuracy = 1;

inline uint8_t NormalizeCaptionsSpeechModel(int value)
{
	return value == kCaptionsSpeechModelHighAccuracy ? kCaptionsSpeechModelHighAccuracy : kCaptionsSpeechModelBalanced;
}

inline const char* CaptionsSpeechModelFileName(uint8_t model)
{
	return NormalizeCaptionsSpeechModel(model) == kCaptionsSpeechModelHighAccuracy ? "ggml-large-v3-turbo-q5_0.bin"
	                                                                               : "ggml-base.bin";
}

inline const char* CaptionsSpeechModelPackId(uint8_t model)
{
	return NormalizeCaptionsSpeechModel(model) == kCaptionsSpeechModelHighAccuracy ? "speech-large-v3-turbo"
	                                                                               : "speech-base";
}

inline const char* CaptionsSpeechModelName(uint8_t model)
{
	return NormalizeCaptionsSpeechModel(model) == kCaptionsSpeechModelHighAccuracy ? "high-accuracy" : "balanced";
}

} // namespace captions
