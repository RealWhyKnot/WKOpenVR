#pragma once

#include "Protocol.h"

#include <cstdint>

namespace pairdriver {

uint64_t PackFingerHeader(const protocol::FingerSmoothingConfig& cfg);
uint64_t PackFingerLow(const protocol::FingerSmoothingConfig& cfg);
protocol::FingerSmoothingConfig UnpackFingerSmoothing(uint64_t header, uint64_t low);
uint16_t ComputeFingerSmoothingReseedBits(const protocol::FingerSmoothingConfig& prev,
                                          const protocol::FingerSmoothingConfig& next);

struct DashboardHandTrackingSnapshot
{
	bool enabled = false;
	bool dashboardVisible = false;
	bool stale = false;
	bool active = false;
	uint8_t primaryHand = protocol::DashboardHandTrackingHandUnknown;
	uint64_t updateMonoMs = 0;
	uint64_t ageMs = 0;
};

uint64_t PackDashboardHandTrackingState(const protocol::DashboardHandTrackingState& state);
protocol::DashboardHandTrackingState UnpackDashboardHandTrackingState(uint64_t packed);
DashboardHandTrackingSnapshot DecodeDashboardHandTrackingState(uint64_t packed, uint64_t nowMonoMs,
                                                               uint64_t staleAfterMs);

uint64_t PackInputHealthConfig(const protocol::InputHealthConfig& cfg);
protocol::InputHealthConfig UnpackInputHealthConfig(uint64_t packed);

} // namespace pairdriver
