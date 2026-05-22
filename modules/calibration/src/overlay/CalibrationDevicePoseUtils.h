#pragma once

#include <cstdint>

bool IsHmdDevice(int32_t id);
uint64_t HashPositionLow64(const double v[3]);
