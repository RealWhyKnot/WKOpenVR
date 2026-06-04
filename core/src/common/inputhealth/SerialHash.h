#pragma once

#include <cstdint>
#include <string>

// FNV-1a 64-bit hash of a device serial string. Both the driver and the
// InputHealth overlay must produce the same uint64_t for the same serial so
// that RequestResetInputHealthStats.device_serial_hash matches the entries
// the driver has cached against their own component containers. The hash is
// not a security primitive; collisions on real-world serials are vanishingly
// unlikely (the input domain is short ASCII strings).

namespace inputhealth {

inline uint64_t Fnv1a64(const char* bytes, size_t len)
{
	uint64_t h = 0xcbf29ce484222325ULL;
	constexpr uint64_t kPrime = 0x100000001b3ULL;
	for (size_t i = 0; i < len; ++i) {
		h ^= static_cast<uint8_t>(bytes[i]);
		h *= kPrime;
	}
	return h;
}

inline uint64_t Fnv1a64(const std::string& s)
{
	return Fnv1a64(s.data(), s.size());
}

// Sentinel used in InputHealthResetStats.device_serial_hash to mean
// "every device" -- the wizard's nuclear "wipe everything" path. Real
// serial hashes will never collide with 0 in practice (a serial that
// hashes to 0 under FNV-1a 64-bit would be extraordinary), but we still
// reserve the value rather than computing it from a real input.
constexpr uint64_t kSerialHashAllDevices = 0;

} // namespace inputhealth
