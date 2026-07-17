#pragma once

#include "Calibration.h"

#include <cstdint>
#include <string>

void ResetAndDisableOffsets(uint32_t id, const std::string& trackingSystem = "");
void ScanAndApplyProfile(CalibrationContext& ctx, bool forceSnapThisCycle = false,
                         const char* forceSnapReason = nullptr);
void InvalidateAllTransformCaches();

// Join the off-thread apply sender. Called at umbrella shutdown; queued
// republish traffic is discarded (the next session re-sends it).
void StopProfileApplyWorker();
