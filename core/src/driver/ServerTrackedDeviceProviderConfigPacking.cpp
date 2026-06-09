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

namespace {

constexpr uint64_t kDashboardFlagsMask = 0xFFFFull;
constexpr uint64_t kDashboardTimestampMask = 0x0000FFFFFFFFFFFFull;

uint8_t NormalizeDashboardHand(uint8_t hand)
{
	if (hand == protocol::DashboardHandTrackingHandLeft || hand == protocol::DashboardHandTrackingHandRight) {
		return hand;
	}
	return protocol::DashboardHandTrackingHandUnknown;
}

} // namespace

uint64_t PackDashboardHandTrackingState(const protocol::DashboardHandTrackingState& state)
{
	uint64_t flags = 0;
	if (state.enabled) flags |= 0x1ull;
	if (state.dashboard_visible) flags |= 0x2ull;
	flags |= (static_cast<uint64_t>(NormalizeDashboardHand(state.primary_hand)) & 0x3ull) << 2;

	const uint64_t timestamp = state.update_mono_ms & kDashboardTimestampMask;
	return (timestamp << 16) | (flags & kDashboardFlagsMask);
}

protocol::DashboardHandTrackingState UnpackDashboardHandTrackingState(uint64_t packed)
{
	protocol::DashboardHandTrackingState state{};
	const uint64_t flags = packed & kDashboardFlagsMask;
	state.enabled = (flags & 0x1ull) != 0 ? 1 : 0;
	state.dashboard_visible = (flags & 0x2ull) != 0 ? 1 : 0;
	state.primary_hand = NormalizeDashboardHand(static_cast<uint8_t>((flags >> 2) & 0x3ull));
	state._reserved = 0;
	state.update_mono_ms = (packed >> 16) & kDashboardTimestampMask;
	return state;
}

DashboardHandTrackingSnapshot DecodeDashboardHandTrackingState(uint64_t packed, uint64_t nowMonoMs,
                                                               uint64_t staleAfterMs)
{
	const protocol::DashboardHandTrackingState state = UnpackDashboardHandTrackingState(packed);

	DashboardHandTrackingSnapshot snapshot{};
	snapshot.enabled = state.enabled != 0;
	snapshot.dashboardVisible = state.dashboard_visible != 0;
	snapshot.primaryHand = state.primary_hand;
	snapshot.updateMonoMs = state.update_mono_ms;
	if (nowMonoMs >= state.update_mono_ms) {
		snapshot.ageMs = nowMonoMs - state.update_mono_ms;
	}
	else {
		snapshot.ageMs = 0;
	}
	snapshot.stale = snapshot.enabled && snapshot.dashboardVisible && snapshot.ageMs > staleAfterMs;
	snapshot.active = snapshot.enabled && snapshot.dashboardVisible && !snapshot.stale;
	return snapshot;
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
