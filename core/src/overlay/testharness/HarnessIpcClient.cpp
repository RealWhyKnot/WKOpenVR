#include "HarnessIpcClient.h"

#if WKOPENVR_BUILD_IS_DEV

#include <chrono>
#include <stdexcept>
#include <thread>

namespace openvr_pair::overlay::testharness {

namespace {

std::string DescribeWin32(DWORD err, const std::string& details)
{
	char buf[64];
	std::snprintf(buf, sizeof(buf), " (err=%lu)", (unsigned long)err);
	return details + buf;
}

openvr_pair::overlay::IpcClientConnectOptions DefaultOptions()
{
	openvr_pair::overlay::IpcClientConnectOptions o;
	o.pipeUnavailable = [](DWORD err, const std::string& details) {
		return "harness pipe open failed: " + DescribeWin32(err, details);
	};
	o.pipeModeFailed = [](DWORD err, const std::string& details) {
		return "harness pipe set-mode failed: " + DescribeWin32(err, details);
	};
	o.versionMismatch = [](uint32_t expected, uint32_t got) {
		return "harness protocol version mismatch: expected " + std::to_string(expected) + " got " +
		       std::to_string(got);
	};
	o.reconnectFailurePrefix = "harness IPC reconnect failed after broken pipe: ";
	o.writeFailurePrefix = "harness IPC write failed";
	o.readFailurePrefix = "harness IPC read failed";
	o.oversizedResponseMessage = "harness IPC response too large: ";
	o.sizeMismatchMessagePrefix = "harness IPC response size mismatch";
	return o;
}

} // namespace

void HarnessIpcClient::OpenWithRetries(const char* pipe_name, std::chrono::milliseconds total_budget)
{
	const auto deadline = std::chrono::steady_clock::now() + total_budget;
	std::string last_error;
	for (;;) {
		try {
			Connect(pipe_name, DefaultOptions());
			return;
		}
		catch (const std::exception& ex) {
			last_error = ex.what();
			if (std::chrono::steady_clock::now() >= deadline) {
				throw std::runtime_error(std::string("HarnessIpcClient::OpenWithRetries(") + pipe_name +
				                         "): " + last_error);
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(80));
		}
	}
}

} // namespace openvr_pair::overlay::testharness

#endif // WKOPENVR_BUILD_IS_DEV
