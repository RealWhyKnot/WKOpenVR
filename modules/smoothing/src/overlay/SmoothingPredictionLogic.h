#pragma once

#include "Config.h"

#include <string>

namespace wkopenvr::smoothing_prediction {

inline int ClampSmoothness(int value)
{
	if (value < 0) return 0;
	if (value > 100) return 100;
	return value;
}

inline bool RemoveHeadsetSynthesisTrackerSmoothness(SmoothingConfig& cfg, const std::string& serial)
{
	if (serial.empty()) return false;
	return cfg.trackerSmoothness.erase(serial) > 0;
}

inline int TrackerPredictionSmoothness(const SmoothingConfig& cfg, const std::string& serial, bool isLocked)
{
	if (isLocked) return 0;
	auto it = cfg.trackerSmoothness.find(serial);
	return it == cfg.trackerSmoothness.end() ? 0 : ClampSmoothness(it->second);
}

inline int VisiblePredictionRowSmoothness(const SmoothingConfig& cfg, const std::string& serial, bool isLocked,
                                          bool isHeadsetSynthesisTracker, bool haveLockedHeadsetSmoothing,
                                          int lockedHeadsetSmoothing)
{
	if (isHeadsetSynthesisTracker && haveLockedHeadsetSmoothing) {
		return ClampSmoothness(lockedHeadsetSmoothing);
	}
	return TrackerPredictionSmoothness(cfg, serial, isLocked);
}

} // namespace wkopenvr::smoothing_prediction
