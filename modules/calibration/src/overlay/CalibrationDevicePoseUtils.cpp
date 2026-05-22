#include "CalibrationDevicePoseUtils.h"

#include <openvr.h>

#include <cstring>

bool IsHmdDevice(int32_t id)
{
	return id == static_cast<int32_t>(vr::k_unTrackedDeviceIndex_Hmd);
}

uint64_t HashPositionLow64(const double v[3])
{
	// Detect pose freshness, not identity; deterministic bit folding is enough.
	uint64_t h0 = 0;
	uint64_t h1 = 0;
	uint64_t h2 = 0;
	std::memcpy(&h0, &v[0], sizeof h0);
	std::memcpy(&h1, &v[1], sizeof h1);
	std::memcpy(&h2, &v[2], sizeof h2);
	return h0 ^ (h1 * 0x9E3779B97F4A7C15ull) ^ (h2 * 0xC2B2AE3D27D4EB4Full);
}
