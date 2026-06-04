#include "InputHealthCompensation.h"

#include "ServerTrackedDeviceProvider.h"
#include "inputhealth/CompensationRules.h"
#include "inputhealth/LearningRules.h"

#include <algorithm>

namespace inputhealth {

float ApplyScalarCompensation(ServerTrackedDeviceProvider* driver, const protocol::InputHealthCompensationEntry& entry,
                              const ComponentStats& stats, float rawValue, float partnerValue, bool hasPartner,
                              const std::string& partnerPath)
{
	float partnerRestOffset = 0.0f;
	bool hasPartnerRestOffset = false;
	if (entry.kind == protocol::InputHealthCompStickX || entry.kind == protocol::InputHealthCompStickY) {
		if (hasPartner && stats.device_serial_hash != 0 && !partnerPath.empty()) {
			protocol::InputHealthCompensationEntry partnerEntry{};
			if (driver->LookupInputHealthCompensation(stats.device_serial_hash, partnerPath, partnerEntry)) {
				partnerRestOffset = partnerEntry.learned_rest_offset;
				hasPartnerRestOffset = true;
			}
		}
	}

	return inputhealth::ApplyScalarCompensationValue(entry.kind, stats.path, stats.scalar_type, stats.scalar_units,
	                                                 rawValue, entry.learned_rest_offset, entry.learned_trigger_min,
	                                                 entry.learned_trigger_max, entry.learned_deadzone_radius,
	                                                 partnerValue, hasPartner, partnerRestOffset, hasPartnerRestOffset);
}

bool ShouldSwallowBooleanUpdate(const ComponentStats& stats, const protocol::InputHealthCompensationEntry& entry,
                                bool newValue, uint64_t nowUs)
{
	return entry.kind == protocol::InputHealthCompBoolean && !inputhealth::IsSystemButtonPath(stats.path) &&
	       entry.learned_debounce_us != 0 && newValue != stats.last_boolean && stats.last_committed_us != 0 &&
	       nowUs - stats.last_committed_us < entry.learned_debounce_us;
}

} // namespace inputhealth
