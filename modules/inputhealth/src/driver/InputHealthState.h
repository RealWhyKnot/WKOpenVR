#pragma once

#include "inputhealth/PerComponentStats.h"

#include <atomic>
#include <mutex>
#include <unordered_map>

class ServerTrackedDeviceProvider;

extern std::atomic<ServerTrackedDeviceProvider*> g_driver;
extern std::unordered_map<vr::VRInputComponentHandle_t, inputhealth::ComponentStats> g_componentStats;
extern std::mutex g_componentMutex;
