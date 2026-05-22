#pragma once

#include "Calibration.h"

#include <cstdint>
#include <string>

void ResetAndDisableOffsets(uint32_t id, const std::string& trackingSystem = "");
void ScanAndApplyProfile(CalibrationContext& ctx);
void InvalidateAllTransformCaches();
