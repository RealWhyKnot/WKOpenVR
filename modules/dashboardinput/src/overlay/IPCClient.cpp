#include "IPCClient.h"

#include "ProtocolNames.h"

#include <stdexcept>
#include <string>

namespace {

openvr_pair::overlay::IpcClientConnectOptions Options()
{
	openvr_pair::overlay::IpcClientConnectOptions options;
	options.pipeUnavailable = [](DWORD error, const std::string& details) {
		return "WKOpenVR Dashboard Input pipe unavailable. Make sure SteamVR is running and the Dashboard Input "
		       "module is enabled. Error " +
		       std::to_string(error) + ": " + details;
	};
	options.pipeModeFailed = [](DWORD error, const std::string& details) {
		return "Couldn't set pipe mode. Error " + std::to_string(error) + ": " + details;
	};
	options.versionMismatch = [](uint32_t expected, uint32_t driver) {
		return "Driver protocol version mismatch. Reinstall WKOpenVR at a compatible version. (Client: " +
		       std::to_string(expected) + ", Driver: " + std::to_string(driver) + ")";
	};
	options.writeFailurePrefix = "Error writing Dashboard Input IPC request";
	options.readFailurePrefix = "Error reading Dashboard Input IPC response";
	options.oversizedResponseMessage = "Invalid Dashboard Input IPC response. Message larger than expected ";
	options.sizeMismatchMessagePrefix = "Invalid Dashboard Input IPC response";
	return options;
}

} // namespace

void DashboardInputIPCClient::Connect()
{
	openvr_pair::overlay::IpcClientBase::Connect(OPENVR_PAIRDRIVER_DASHBOARDINPUT_PIPE_NAME, Options());
}
