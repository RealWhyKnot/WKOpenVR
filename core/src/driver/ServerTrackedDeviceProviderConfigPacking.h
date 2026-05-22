#pragma once

#include "Protocol.h"

#include <cstdint>

namespace pairdriver {

uint64_t PackFingerHeader(const protocol::FingerSmoothingConfig& cfg);
uint64_t PackFingerLow(const protocol::FingerSmoothingConfig& cfg);
protocol::FingerSmoothingConfig UnpackFingerSmoothing(uint64_t header, uint64_t low);
uint16_t ComputeFingerSmoothingReseedBits(
	const protocol::FingerSmoothingConfig& prev,
	const protocol::FingerSmoothingConfig& next);

uint64_t PackInputHealthConfig(const protocol::InputHealthConfig& cfg);
protocol::InputHealthConfig UnpackInputHealthConfig(uint64_t packed);

} // namespace pairdriver
