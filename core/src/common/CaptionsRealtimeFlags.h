#pragma once

#include <cstdint>

namespace captions {

static constexpr uint8_t kCaptionsRealtimeExtendedTiming = 1u << 0;
static constexpr uint8_t kCaptionsRealtimeSpeechEvidenceGate = 1u << 1;
static constexpr uint8_t kCaptionsRealtimeConfidenceFilter = 1u << 2;
static constexpr uint8_t kCaptionsRealtimeOverlapCleanup = 1u << 3;
static constexpr uint8_t kCaptionsRealtimeChatboxSplitting = 1u << 4;
static constexpr uint8_t kCaptionsRealtimeWhisperNoSpeechGate = 1u << 5;
static constexpr uint8_t kCaptionsRealtimePromptContext = 1u << 6;
static constexpr uint8_t kCaptionsRealtimeTypingIndicator = 1u << 7;

static constexpr uint8_t kCaptionsRealtimeSpeechPickupMask =
    kCaptionsRealtimeExtendedTiming | kCaptionsRealtimeSpeechEvidenceGate | kCaptionsRealtimeOverlapCleanup;
static constexpr uint8_t kCaptionsRealtimeRandomCaptionMask =
    kCaptionsRealtimeConfidenceFilter | kCaptionsRealtimeWhisperNoSpeechGate;

static constexpr uint8_t kCaptionsRealtimeDefaultFlags =
    kCaptionsRealtimeExtendedTiming | kCaptionsRealtimeSpeechEvidenceGate | kCaptionsRealtimeConfidenceFilter |
    kCaptionsRealtimeOverlapCleanup | kCaptionsRealtimeChatboxSplitting | kCaptionsRealtimeWhisperNoSpeechGate |
    kCaptionsRealtimePromptContext | kCaptionsRealtimeTypingIndicator;

inline bool CaptionsRealtimeFlagEnabled(uint8_t flags, uint8_t flag)
{
	return (flags & flag) != 0;
}

inline uint8_t SetCaptionsRealtimeFlag(uint8_t flags, uint8_t flag, bool enabled)
{
	return enabled ? static_cast<uint8_t>(flags | flag) : static_cast<uint8_t>(flags & ~flag);
}

inline bool CaptionsRealtimeMaskEnabled(uint8_t flags, uint8_t mask)
{
	return (flags & mask) == mask;
}

inline uint8_t SetCaptionsRealtimeMask(uint8_t flags, uint8_t mask, bool enabled)
{
	return enabled ? static_cast<uint8_t>(flags | mask) : static_cast<uint8_t>(flags & ~mask);
}

} // namespace captions
