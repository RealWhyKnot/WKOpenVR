#pragma once

#include "Protocol.h"
#include "inputhealth/PerComponentStats.h"

#include <cstdint>
#include <string>

class ServerTrackedDeviceProvider;

namespace inputhealth {

float ApplyScalarCompensation(ServerTrackedDeviceProvider* driver, const protocol::InputHealthCompensationEntry& entry,
                              const ComponentStats& stats, float rawValue, float partnerValue, bool hasPartner,
                              const std::string& partnerPath);

bool ShouldSwallowBooleanUpdate(const ComponentStats& stats, const protocol::InputHealthCompensationEntry& entry,
                                bool newValue, uint64_t nowUs);

} // namespace inputhealth
