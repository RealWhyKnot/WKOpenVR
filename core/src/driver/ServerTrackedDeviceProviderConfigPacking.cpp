#include "ServerTrackedDeviceProviderConfigPacking.h"

#include <cstring>

namespace pairdriver {

uint64_t PackFingerHeader(const protocol::FingerSmoothingConfig& cfg)
{
	uint64_t packed = 0;
	uint8_t* b = reinterpret_cast<uint8_t*>(&packed);
	b[0] = cfg.master_enabled ? 1 : 0;
	b[1] = cfg.smoothness;
	b[2] = static_cast<uint8_t>(cfg.finger_mask & 0xFF);
	b[3] = static_cast<uint8_t>((cfg.finger_mask >> 8) & 0xFF);
	b[4] = 0;
	b[5] = cfg.per_finger_smoothness[8];
	b[6] = cfg.per_finger_smoothness[9];
	b[7] = 0;
	return packed;
}

uint64_t PackFingerLow(const protocol::FingerSmoothingConfig& cfg)
{
	uint64_t packed = 0;
	std::memcpy(&packed, cfg.per_finger_smoothness, 8);
	return packed;
}

protocol::FingerSmoothingConfig UnpackFingerSmoothing(uint64_t header, uint64_t low)
{
	protocol::FingerSmoothingConfig cfg{};
	const uint8_t* b = reinterpret_cast<const uint8_t*>(&header);
	cfg.master_enabled = b[0] != 0;
	cfg.smoothness = b[1];
	cfg.finger_mask = static_cast<uint16_t>(b[2] | (static_cast<uint16_t>(b[3]) << 8));
	cfg._reserved = 0;
	std::memcpy(cfg.per_finger_smoothness, &low, 8);
	cfg.per_finger_smoothness[8] = b[5];
	cfg.per_finger_smoothness[9] = b[6];
	cfg._reserved2[0] = 0;
	cfg._reserved2[1] = 0;
	return cfg;
}

namespace {

bool IsFingerSmoothed(const protocol::FingerSmoothingConfig& cfg, int idx)
{
	if (!cfg.master_enabled) return false;
	if (((cfg.finger_mask >> idx) & 1u) == 0) return false;
	const uint8_t perFinger = cfg.per_finger_smoothness[idx];
	const uint8_t effective = perFinger != 0 ? perFinger : cfg.smoothness;
	return effective != 0;
}

} // namespace

uint16_t ComputeFingerSmoothingReseedBits(const protocol::FingerSmoothingConfig& prev,
                                          const protocol::FingerSmoothingConfig& next)
{
	uint16_t reseedBits = 0;
	for (int i = 0; i < 10; ++i) {
		const bool wasOn = IsFingerSmoothed(prev, i);
		const bool isOn = IsFingerSmoothed(next, i);
		if (!wasOn && isOn) reseedBits |= static_cast<uint16_t>(1u << i);
	}
	return reseedBits;
}

uint64_t PackInputHealthConfig(const protocol::InputHealthConfig& cfg)
{
	static_assert(sizeof(protocol::InputHealthConfig) <= sizeof(uint64_t),
	              "InputHealthConfig must fit inside atomic<uint64_t>");
	uint64_t packed = 0;
	std::memcpy(&packed, &cfg, sizeof(cfg));
	return packed;
}

protocol::InputHealthConfig UnpackInputHealthConfig(uint64_t packed)
{
	protocol::InputHealthConfig cfg{};
	std::memcpy(&cfg, &packed, sizeof(cfg));
	return cfg;
}

} // namespace pairdriver
