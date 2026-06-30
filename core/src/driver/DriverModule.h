#pragma once

#include "Protocol.h"

#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

#include <openvr_driver.h>

class ServerTrackedDeviceProvider;

struct DriverModuleContext
{
	ServerTrackedDeviceProvider* provider = nullptr;
	vr::IVRDriverContext* driverContext = nullptr;
	uint32_t featureFlags = 0;
};

class DriverModule
{
public:
	virtual ~DriverModule() = default;

	virtual const char* Name() const = 0;
	virtual uint32_t FeatureMask() const = 0;
	virtual const char* PipeName() const = 0;

	virtual bool Init(DriverModuleContext& context) = 0;
	virtual void Shutdown() {}
	virtual void OnGetGenericInterface(const char* pchInterface, void* iface) {}
	virtual bool HandleRequest(const protocol::Request& request, protocol::Response& response) = 0;
};

namespace calibration {
std::unique_ptr<DriverModule> CreateDriverModule();
}

namespace smoothing {
std::unique_ptr<DriverModule> CreateDriverModule();
}

namespace inputhealth {
std::unique_ptr<DriverModule> CreateDriverModule();
}

namespace facetracking {
std::unique_ptr<DriverModule> CreateDriverModule();
}

namespace oscrouter {
std::unique_ptr<DriverModule> CreateDriverModule();
}

namespace captions {
std::unique_ptr<DriverModule> CreateDriverModule();
}

namespace phantom {
std::unique_ptr<DriverModule> CreateDriverModule();

// Hot-path entry points called from
// ServerTrackedDeviceProvider::HandleDevicePoseUpdated. Both are safe no-ops
// when the phantom module has not been initialised (feature disabled, or
// CreateDriverModule never instantiated).
void OnRealPoseObserved(uint32_t openVRID, int64_t qpc_ns, const vr::DriverPose_t& pose);
bool MaybeOverridePose(uint32_t openVRID, int64_t qpc_ns, int64_t qpc_freq, vr::DriverPose_t& pose);
void CollectSilentPoseUpdates(uint32_t triggeringOpenVRID, int64_t qpc_ns, int64_t qpc_freq,
                              std::vector<std::pair<uint32_t, vr::DriverPose_t>>& out);

// Teardown helpers used by ServerTrackedDeviceProvider when the module is being
// disabled: stop virtual publishing, then drain a disconnected pose for every
// active virtual tracker so they can be forwarded (out-of-lock) before the
// module is destroyed. Safe no-ops when the module is not initialised.
void SetVirtualMasterEnabled(bool enabled);
void CollectVirtualDisconnects(std::vector<std::pair<uint32_t, vr::DriverPose_t>>& out);
} // namespace phantom
