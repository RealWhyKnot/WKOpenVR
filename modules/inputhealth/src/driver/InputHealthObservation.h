#pragma once

#include "inputhealth/PerComponentStats.h"

#include <cstdint>

namespace inputhealth {

void ObserveBooleanSample(ComponentStats& stats, bool newValue, uint64_t nowUs);
void ObserveScalarSample(ComponentStats& stats, float newValue, uint64_t nowUs, ComponentStats* partnerStats);

} // namespace inputhealth
