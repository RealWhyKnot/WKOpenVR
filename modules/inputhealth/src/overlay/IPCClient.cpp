#include "IPCClient.h"

#include <string>

namespace {

openvr_pair::overlay::IpcClientConnectOptions Options()
{
	openvr_pair::overlay::IpcClientConnectOptions options;
	options.pipeUnavailable = [](DWORD, const std::string&) {
		return "InputHealth driver unavailable. SteamVR is not running, the WKOpenVR shared driver is not installed, "
		       "or the InputHealth feature is not enabled (enable_inputhealth.flag missing in the driver's resources "
		       "folder).";
	};
	options.pipeModeFailed = [](DWORD error, const std::string& details) {
		return "Could not set pipe mode. Error " + std::to_string(error) + ": " + details;
	};
	options.versionMismatch = [](uint32_t expected, uint32_t driver) {
		return "Driver protocol version mismatch. Reinstall InputHealth so the overlay and shared driver are paired. "
		       "(Overlay: " +
		       std::to_string(expected) + ", driver: " + std::to_string(driver) + ")";
	};
	options.reconnectFailurePrefix = "InputHealth IPC reconnect failed: ";
	options.writeFailurePrefix = "InputHealth IPC write error";
	options.readFailurePrefix = "InputHealth IPC read error";
	options.oversizedResponseMessage = "Invalid IPC response: message exceeds expected size ";
	options.sizeMismatchMessagePrefix = "Invalid IPC response";
	return options;
}
} // namespace

void IPCClient::Connect()
{
	openvr_pair::overlay::IpcClientBase::Connect(OPENVR_PAIRDRIVER_INPUTHEALTH_PIPE_NAME, Options());
}

protocol::Response IPCClient::SendCompensationEntry(const protocol::InputHealthCompensationEntry& entry)
{
	protocol::Request req(protocol::RequestSetInputHealthCompensation);
	req.setInputHealthCompensation = entry;
	return SendBlocking(req);
}
