#pragma once

#include "BuildChannel.h"

#if WKOPENVR_BUILD_IS_DEV

#include <openvr_driver.h>

#include <atomic>
#include <filesystem>
#include <memory>
#include <string>
#include <thread>

namespace openvr_pair::overlay::testharness {

class MockOpenVRRuntime;

// Loads driver_wkopenvr.dll out of a staged sandbox and drives it in-process
// using a mock OpenVR runtime. The loader owns the RunFrame ticker thread
// that mimics SteamVR's main-thread cadence.
class InProcessDriverLoader
{
public:
	InProcessDriverLoader() = default;
	~InProcessDriverLoader();

	InProcessDriverLoader(const InProcessDriverLoader&) = delete;
	InProcessDriverLoader& operator=(const InProcessDriverLoader&) = delete;

	// Load the DLL, resolve the factory, init the driver against the mock.
	// Throws std::runtime_error on failure.
	void Load(const std::filesystem::path& driver_dll, MockOpenVRRuntime& runtime);

	// Start the RunFrame ticker thread (~90 Hz). Safe to call once.
	void StartFrameTicker();

	// Stop the ticker, call provider->Cleanup(), FreeLibrary.
	void Stop();

	vr::IServerTrackedDeviceProvider* provider() noexcept { return provider_; }

private:
	void FrameLoop();

	void* hModule_ = nullptr; // HMODULE; void* to keep windows.h out of the header
	vr::IServerTrackedDeviceProvider* provider_ = nullptr;
	std::atomic<bool> stop_requested_{false};
	std::thread ticker_;
	bool started_ticker_ = false;
	bool cleaned_up_ = false;
};

} // namespace openvr_pair::overlay::testharness

#endif // WKOPENVR_BUILD_IS_DEV
