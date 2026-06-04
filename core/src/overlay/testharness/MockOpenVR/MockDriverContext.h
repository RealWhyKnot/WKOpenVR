#pragma once

#include "BuildChannel.h"

#if WKOPENVR_BUILD_IS_DEV

#include <openvr_driver.h>

#include <string>
#include <unordered_map>

namespace openvr_pair::overlay::testharness {

class MockOpenVRRuntime;

// Implements vr::IVRDriverContext. The driver fetches every other interface
// (IVRServerDriverHost, IVRDriverInput, IVRProperties, IVRSettings,
// IVRDriverLog, IVRResources) via GetGenericInterface. Our context routes
// each requested version string to the matching mock instance owned by
// MockOpenVRRuntime.
class MockDriverContext : public vr::IVRDriverContext
{
public:
	explicit MockDriverContext(MockOpenVRRuntime& owner);

	void RegisterInterface(std::string version, void* iface);

	// vr::IVRDriverContext
	void* GetGenericInterface(const char* pchInterfaceVersion, vr::EVRInitError* peError = nullptr) override;
	vr::DriverHandle_t GetDriverHandle() override;

private:
	MockOpenVRRuntime& owner_;
	std::unordered_map<std::string, void*> interfaces_;
};

} // namespace openvr_pair::overlay::testharness

#endif // WKOPENVR_BUILD_IS_DEV
