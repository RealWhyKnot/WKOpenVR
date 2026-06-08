#pragma once

#include <cstdint>

namespace captions {

static constexpr uint8_t kCaptionsRealtimeExtendedTiming = 1u << 0;
static constexpr uint8_t kCaptionsRealtimeSpeechEvidenceGate = 1u << 1;
static constexpr uint8_t kCaptionsRealtimeConfidenceFilter = 1u << 2;
static constexpr uint8_t kCaptionsRealtimeOverlapCleanup = 1u << 3;
static constexpr uint8_t kCaptionsRealtimeChatboxSplitting = 1u << 4;

static constexpr uint8_t kCaptionsRealtimeDefaultFlags =
    kCaptionsRealtimeExtendedTiming | kCaptionsRealtimeSpeechEvidenceGate | kCaptionsRealtimeConfidenceFilter |
    kCaptionsRealtimeOverlapCleanup | kCaptionsRealtimeChatboxSplitting;

inline bool CaptionsRealtimeFlagEnabled(uint8_t flags, uint8_t flag)
{
	return (flags & flag) != 0;
}

inline uint8_t SetCaptionsRealtimeFlag(uint8_t flags, uint8_t flag, bool enabled)
{
	return enabled ? static_cast<uint8_t>(flags | flag) : static_cast<uint8_t>(flags & ~flag);
}

} // namespace captions
