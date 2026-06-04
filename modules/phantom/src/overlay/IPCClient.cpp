#include "IPCClient.h"

#include <string>

namespace {

openvr_pair::overlay::IpcClientConnectOptions Options()
{
	openvr_pair::overlay::IpcClientConnectOptions options;
	options.pipeUnavailable = [](DWORD error, const std::string& details) {
		return "WKOpenVR phantom pipe unavailable. Make sure SteamVR is running and the WKOpenVR-Phantom module is "
		       "installed. Error " +
		       std::to_string(error) + ": " + details;
	};
	options.pipeModeFailed = [](DWORD error, const std::string& details) {
		return "Couldn't set pipe mode. Error " + std::to_string(error) + ": " + details;
	};
	options.versionMismatch = [](uint32_t expected, uint32_t driver) {
		return "Driver protocol version mismatch. Reinstall WKOpenVR at a matching driver/overlay version. (Client: " +
		       std::to_string(expected) + ", Driver: " + std::to_string(driver) + ")";
	};
	options.writeFailurePrefix = "Error writing IPC request";
	options.readFailurePrefix = "Error reading IPC response";
	options.oversizedResponseMessage = "Invalid IPC response. Message larger than expected ";
	options.sizeMismatchMessagePrefix = "Invalid IPC response";
	return options;
}

} // namespace

void PhantomIPCClient::Connect()
{
	openvr_pair::overlay::IpcClientBase::Connect(OPENVR_PAIRDRIVER_PHANTOM_PIPE_NAME, Options());
}
