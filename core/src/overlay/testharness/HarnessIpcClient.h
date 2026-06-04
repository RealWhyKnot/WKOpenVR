#pragma once

#include "BuildChannel.h"

#if WKOPENVR_BUILD_IS_DEV

#include "IpcClientBase.h"
#include "Protocol.h"

#include <chrono>
#include <stdexcept>
#include <string>

namespace openvr_pair::overlay::testharness {

// Concrete IpcClientBase subclass used by scenarios. Provides default
// formatters and a Connect(pipe) wrapper that retries briefly while the
// driver is still spinning up its IPC threads.
class HarnessIpcClient : public openvr_pair::overlay::IpcClientBase
{
public:
	HarnessIpcClient() = default;

	// Connect with a small retry budget (driver takes a few RunFrame ticks to
	// finish opening pipes). Throws std::runtime_error on persistent failure.
	void OpenWithRetries(const char* pipe_name,
	                     std::chrono::milliseconds total_budget = std::chrono::milliseconds(3000));
};

} // namespace openvr_pair::overlay::testharness

#endif // WKOPENVR_BUILD_IS_DEV
