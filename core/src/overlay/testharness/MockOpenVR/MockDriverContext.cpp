#include "MockDriverContext.h"

#if WKOPENVR_BUILD_IS_DEV

#include "../HarnessScenario.h"

#include <cstdio>
#include <string>

namespace openvr_pair::overlay::testharness {

MockDriverContext::MockDriverContext(MockOpenVRRuntime& owner) : owner_(owner) {}

void MockDriverContext::RegisterInterface(std::string version, void* iface)
{
	interfaces_[std::move(version)] = iface;
}

void* MockDriverContext::GetGenericInterface(const char* pchInterfaceVersion, vr::EVRInitError* peError)
{
	if (peError) *peError = vr::VRInitError_None;
	if (pchInterfaceVersion == nullptr) {
		if (peError) *peError = vr::VRInitError_Init_InterfaceNotFound;
		return nullptr;
	}
	auto it = interfaces_.find(std::string(pchInterfaceVersion));
	if (it == interfaces_.end()) {
		// Unknown interface request from the driver. The hook injector
		// returns the underlying pointer, so we still log + return null so
		// the request is visible during debugging without crashing.
		std::fprintf(stderr, "[testharness][MockDriverContext] unhandled GetGenericInterface(%s)\n",
		             pchInterfaceVersion);
		if (peError) *peError = vr::VRInitError_Init_InterfaceNotFound;
		return nullptr;
	}
	return it->second;
}

vr::DriverHandle_t MockDriverContext::GetDriverHandle()
{
	// Any non-zero handle satisfies the driver's internal book-keeping.
	return (vr::DriverHandle_t)0x10000001ULL;
}

} // namespace openvr_pair::overlay::testharness

#endif // WKOPENVR_BUILD_IS_DEV
