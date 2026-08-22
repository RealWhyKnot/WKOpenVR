#pragma once

#include <cstdint>
#include <memory>
#include <string>

// Feature bits the module registry marks desktop-capable.
uint32_t DesktopFeatureMask();

// Feature bits this build actually has a driver factory for. Equal to
// DesktopFeatureMask() unless a module was gated out of the build.
uint32_t DesktopHostableMask();

// Hosts the desktop-capable driver modules inside the app while SteamVR is
// down. Same module objects, same pipe names, same protocol as the SteamVR
// driver, so overlay plugins cannot tell the two apart.
//
// The implementation is hidden because it pulls in openvr_driver.h, which
// cannot share a translation unit with the openvr.h the app uses.
class DesktopDriverHost
{
public:
	DesktopDriverHost();
	~DesktopDriverHost();

	DesktopDriverHost(const DesktopDriverHost&) = delete;
	DesktopDriverHost& operator=(const DesktopDriverHost&) = delete;

	// Returns the mask that came up. resourcesDir is the driver's own
	// resources directory; modules that spawn a sidecar resolve it from there.
	uint32_t Start(uint32_t featureMask, const std::wstring& resourcesDir);
	void Stop();

	bool Running() const;
	uint32_t ActiveMask() const;

private:
	struct Impl;
	std::unique_ptr<Impl> impl_;
};
