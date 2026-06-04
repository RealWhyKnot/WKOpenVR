#include "InputHealthSnapshotStaging.h"

#include "InputHealthState.h"
#include "Logging.h"
#include "ServerTrackedDeviceProvider.h"
#include "inputhealth/SerialHash.h"

#include <algorithm>
#include <atomic>
#include <cstring>
#include <mutex>
#include <string>

namespace inputhealth {

namespace {

// Helper: copy one ComponentStats into the wire-format snapshot body. The
// caller owns the body; this function only translates fields. Path is
// truncated to fit INPUTHEALTH_PATH_LEN-1 bytes; OpenVR component paths are
// well under that in practice.
void FillSnapshotBody(vr::VRInputComponentHandle_t handle, const ComponentStats& s,
                      protocol::InputHealthSnapshotBody& out)
{
	std::memset(&out, 0, sizeof(out));

	out.handle = static_cast<uint64_t>(handle);
	out.container_handle = static_cast<uint64_t>(s.container_handle);
	out.device_serial_hash = s.device_serial_hash;
	out.partner_handle = static_cast<uint64_t>(s.partner_handle);

	const size_t plen = std::min<size_t>(s.path.size(), protocol::INPUTHEALTH_PATH_LEN - 1);
	if (plen > 0) std::memcpy(out.path, s.path.data(), plen);
	out.path[plen] = '\0';

	out.is_scalar = s.is_scalar ? 1 : 0;
	out.is_boolean = s.is_boolean ? 1 : 0;
	out.axis_role = static_cast<uint8_t>(s.axis_role);
	out.scalar_type = s.scalar_type;
	out.scalar_units = s.scalar_units;
	out.ph_initialized = s.ph_drift.initialized ? 1 : 0;
	out.ph_triggered = s.ph_drift.triggered ? 1 : 0;
	out.ph_triggered_positive = s.ph_drift.triggered_positive ? 1 : 0;
	out.rest_min_initialized = s.rest_min.initialized ? 1 : 0;
	out.last_boolean = s.last_boolean ? 1 : 0;

	out.welford_count = s.welford.count;
	out.welford_mean = s.welford.mean;
	out.welford_m2 = s.welford.m2;

	out.ph_mean = s.ph_drift.mean;
	out.ph_pos = s.ph_drift.ph_pos;
	out.ph_neg = s.ph_drift.ph_neg;

	out.rest_min = s.rest_min.value;

	out.last_value = s.last_value;
	out.last_update_us = s.last_update_us;
	out.press_count = s.press_count;
	out.bounce_transition_count = s.bounce_transition_count;
	out.bounce_max_interval_us = s.bounce_max_interval_us;
	out.scalar_range_initialized = s.scalar_range_initialized ? 1 : 0;
	out.observed_min = s.observed_min;
	out.observed_max = s.observed_max;

	for (int i = 0; i < protocol::INPUTHEALTH_POLAR_BIN_COUNT && i < kBinCount; ++i) {
		out.polar_max_r[i] = s.polar.bins[i].max_r;
		out.polar_count[i] = s.polar.bins[i].count;
		out.polar_last_update_us[i] = s.polar.bins[i].last_update_us;
	}
	out.polar_global_max_r = s.polar.global_max_r;
}

} // namespace

void StageSnapshots(std::vector<StagedSnapshot>& out)
{
	// Fill snapshot bodies directly under one lock. The previous
	// move-out / fill / merge-back pattern took the mutex twice and the
	// merge-back was O(N) try_emplace under the second lock, during which
	// detour threads either blocked or (when they used try_lock) silently
	// dropped observations. FillSnapshotBody is a fixed-size field copy
	// per entry -- microseconds of work for the ~50-100 components a
	// dual-controller session produces -- so a single short blocking lock
	// is cheaper end-to-end and no longer loses state.
	std::lock_guard<std::mutex> lk(g_componentMutex);
	out.reserve(out.size() + g_componentStats.size());
	for (const auto& kv : g_componentStats) {
		out.emplace_back();
		auto& rec = out.back();
		rec.handle = static_cast<uint64_t>(kv.first);
		FillSnapshotBody(kv.first, kv.second, rec.body);
	}
}

void ApplyResetRequest(const protocol::InputHealthResetStats& req)
{
	// Pass 1: snapshot (handle, container, cached_hash) without holding the
	// mutex during the VRProperties query. The detour path is a hot path so
	// we keep the critical section short.
	struct Snapshot
	{
		vr::VRInputComponentHandle_t handle;
		vr::PropertyContainerHandle_t container;
		uint64_t hash;
	};
	std::vector<Snapshot> snap;
	{
		std::lock_guard<std::mutex> lk(g_componentMutex);
		snap.reserve(g_componentStats.size());
		for (const auto& kv : g_componentStats) {
			snap.push_back({kv.first, kv.second.container_handle, kv.second.device_serial_hash});
		}
	}

	// Pass 2: lazily resolve any unresolved hashes via VRProperties.
	auto* helpers = vr::VRProperties();
	for (auto& s : snap) {
		if (s.hash != 0) continue;
		if (s.container == vr::k_ulInvalidPropertyContainer) continue;
		if (!helpers) continue;
		vr::ETrackedPropertyError err = vr::TrackedProp_Success;
		std::string serial = helpers->GetStringProperty(s.container, vr::Prop_SerialNumber_String, &err);
		if (err == vr::TrackedProp_Success && !serial.empty()) {
			s.hash = Fnv1a64(serial);
		}
	}

	// Pass 3: re-take the mutex, fold resolved hashes back in, and reset
	// matching entries. Entries that were added or removed between passes
	// are handled correctly: we only act on handles still present in the
	// map.
	const bool match_all = (req.device_serial_hash == kSerialHashAllDevices);
	int matched = 0;
	int reset_passive_count = 0;
	int reset_curves_count = 0;
	{
		std::lock_guard<std::mutex> lk(g_componentMutex);
		for (auto& s : snap) {
			auto it = g_componentStats.find(s.handle);
			if (it == g_componentStats.end()) continue;
			if (s.hash != 0 && it->second.device_serial_hash == 0) {
				it->second.device_serial_hash = s.hash;
			}
			if (!match_all && it->second.device_serial_hash != req.device_serial_hash) continue;
			++matched;
			if (req.reset_passive) {
				ComponentStatsResetPassive(it->second);
				++reset_passive_count;
			}
		}
	}

	if (req.reset_curves) {
		if (auto* driver = g_driver.load(std::memory_order_acquire)) {
			driver->ClearInputHealthCompensation(req.device_serial_hash);
			reset_curves_count = 1;
		}
	}

	LOG("[inputhealth] HandleResetInputHealthStats: serial_hash=0x%016llx passive=%d active=%d curves=%d -> matched=%d "
	    "passive_reset=%d curves_reset=%d total_components=%zu",
	    (unsigned long long)req.device_serial_hash, (int)req.reset_passive, (int)req.reset_active,
	    (int)req.reset_curves, matched, reset_passive_count, reset_curves_count, snap.size());
}

} // namespace inputhealth
