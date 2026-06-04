#include "InputHealthComponentRegistry.h"

#include "InputHealthState.h"
#include "Logging.h"
#include "inputhealth/SerialHash.h"

#include <mutex>
#include <string>
#include <utility>

namespace {

// Find an existing scalar handle whose path stem and complementary axis role
// match the new handle. Caller must hold g_componentMutex. Returns 0 (the
// invalid component handle) if no partner exists yet.
vr::VRInputComponentHandle_t FindStickPartner_locked(const std::string& stem, inputhealth::AxisRole this_role,
                                                     vr::PropertyContainerHandle_t container)
{
	const inputhealth::AxisRole want =
	    (this_role == inputhealth::AxisRole::StickX) ? inputhealth::AxisRole::StickY : inputhealth::AxisRole::StickX;
	for (auto& kv : g_componentStats) {
		auto& peer = kv.second;
		if (!peer.is_scalar) continue;
		if (peer.axis_role != want) continue;
		if (peer.container_handle != container) continue;
		// Match on stem: peer's path with its trailing /x or /y stripped.
		std::string peerStem;
		(void)inputhealth::ClassifyAxisRole(peer.path, peerStem);
		if (peerStem == stem) return kv.first;
	}
	return 0;
}

} // namespace

namespace inputhealth {

uint64_t ResolveSerialHash(vr::PropertyContainerHandle_t container)
{
	if (container == vr::k_ulInvalidPropertyContainer) return 0;
	auto* helpers = vr::VRProperties();
	if (!helpers) return 0;

	vr::ETrackedPropertyError err = vr::TrackedProp_Success;
	std::string serial = helpers->GetStringProperty(container, vr::Prop_SerialNumber_String, &err);
	if (err != vr::TrackedProp_Success || serial.empty()) return 0;
	return inputhealth::Fnv1a64(serial);
}

void RegisterBooleanComponent(vr::VRInputComponentHandle_t handle, vr::PropertyContainerHandle_t container,
                              const char* path)
{
	std::lock_guard<std::mutex> lk(g_componentMutex);
	auto& stats = g_componentStats[handle];
	stats.path = path;
	stats.is_boolean = true;
	stats.is_scalar = false;
	stats.first_update_logged = false;
	stats.container_handle = container;
	stats.device_serial_hash = ResolveSerialHash(container);
}

void RegisterScalarComponent(vr::VRInputComponentHandle_t handle, vr::PropertyContainerHandle_t container,
                             const char* path, vr::EVRScalarType scalar_type, vr::EVRScalarUnits scalar_units)
{
	std::string componentPath = path;
	std::string stem;
	const auto role = inputhealth::ClassifyAxisRole(componentPath, stem);

	std::lock_guard<std::mutex> lk(g_componentMutex);
	auto& stats = g_componentStats[handle];
	stats.path = std::move(componentPath);
	stats.is_boolean = false;
	stats.is_scalar = true;
	stats.scalar_type = static_cast<uint8_t>(scalar_type);
	stats.scalar_units = static_cast<uint8_t>(scalar_units);
	stats.first_update_logged = false;
	stats.axis_role = role;
	stats.partner_handle = 0;
	stats.container_handle = container;
	stats.device_serial_hash = ResolveSerialHash(container);

	if (role == inputhealth::AxisRole::StickX || role == inputhealth::AxisRole::StickY) {
		vr::VRInputComponentHandle_t partner = FindStickPartner_locked(stem, role, container);
		if (partner != 0) {
			stats.partner_handle = partner;
			g_componentStats[partner].partner_handle = handle;
			LOG("[inputhealth] paired stick axes: stem='%s' xHandle=%llu yHandle=%llu", stem.c_str(),
			    role == inputhealth::AxisRole::StickX ? (unsigned long long)handle : (unsigned long long)partner,
			    role == inputhealth::AxisRole::StickY ? (unsigned long long)handle : (unsigned long long)partner);
		}
	}
}

} // namespace inputhealth
