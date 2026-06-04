#pragma once

#include "Calibration.h"

#include <cstdint>
#include <string>

void ResetAndDisableOffsets(uint32_t id, const std::string& trackingSystem = "");
void ScanAndApplyProfile(CalibrationContext& ctx, bool forceSnapThisCycle = false,
                         const char* forceSnapReason = nullptr);
void InvalidateAllTransformCaches();
