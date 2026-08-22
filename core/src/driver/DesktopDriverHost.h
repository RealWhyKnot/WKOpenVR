#pragma once

#include "IPCServer.h"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

class DriverModule;

// Feature bits the module registry marks desktop-capable.
uint32_t DesktopFeatureMask();

// Feature bits this build actually has a driver factory for. Equal to
// DesktopFeatureMask() unless a module was gated out of the build.
uint32_t DesktopHostableMask();

// Hosts the desktop-capable driver modules inside the app while SteamVR is
// down. Same module objects, same pipe names, same protocol as the SteamVR
// driver, so overlay plugins cannot tell the two apart.
class DesktopDriverHost final : public IpcRequestSink
{
public:
	~DesktopDriverHost() override;

	// Returns the mask that came up. resourcesDir is the driver's own
	// resources directory; modules that spawn a sidecar resolve it from there.
	uint32_t Start(uint32_t featureMask, const std::wstring& resourcesDir);
	void Stop();

	bool Running() const { return running_; }
	uint32_t ActiveMask() const { return activeMask_; }

	bool HandleIpcRequest(uint32_t featureMask, const protocol::Request& request,
	                      protocol::Response& response) override;

private:
	struct Hosted
	{
		std::unique_ptr<DriverModule> module;
		std::unique_ptr<IPCServer> server;
	};

	std::vector<Hosted> hosted_;
	uint32_t activeMask_ = 0;
	bool running_ = false;
};
