#include "IPCClient.h"

#include <stdexcept>
#include <string>

namespace {
openvr_pair::overlay::IpcClientConnectOptions Options()
{
	openvr_pair::overlay::IpcClientConnectOptions options;
	options.pipeUnavailable = [](DWORD error, const std::string& details) {
		return "WKOpenVR smoothing pipe unavailable. Make sure SteamVR is running and the WKOpenVR-Smoothing addon is "
		       "installed. Error " +
		       std::to_string(error) + ": " + details;
	};
	options.pipeModeFailed = [](DWORD error, const std::string& details) {
		return "Couldn't set pipe mode. Error " + std::to_string(error) + ": " + details;
	};
	options.versionMismatch = [](uint32_t expected, uint32_t driver) {
		return "Driver protocol version mismatch. Reinstall WKOpenVR-Smoothing and WKOpenVR-SpaceCalibrator at "
		       "compatible versions. (Client: " +
		       std::to_string(expected) + ", Driver: " + std::to_string(driver) + ")";
	};
	options.writeFailurePrefix = "Error writing IPC request";
	options.readFailurePrefix = "Error reading IPC response";
	options.oversizedResponseMessage = "Invalid IPC response. Message larger than expected ";
	options.sizeMismatchMessagePrefix = "Invalid IPC response";
	return options;
}
} // namespace

void SmoothingIPCClient::Connect()
{
	openvr_pair::overlay::IpcClientBase::Connect(OPENVR_PAIRDRIVER_SMOOTHING_PIPE_NAME, Options());
}
