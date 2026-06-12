#pragma once

namespace captions {

static constexpr int kCaptionsChatboxSplitDelayDefaultMs = 1200;
static constexpr int kCaptionsChatboxSplitDelayMinMs = 0;
static constexpr int kCaptionsChatboxSplitDelayMaxMs = 10000;

inline int NormalizeCaptionsChatboxSplitDelayMs(int ms)
{
	if (ms < kCaptionsChatboxSplitDelayMinMs) return kCaptionsChatboxSplitDelayMinMs;
	if (ms > kCaptionsChatboxSplitDelayMaxMs) return kCaptionsChatboxSplitDelayMaxMs;
	return ms;
}

} // namespace captions
