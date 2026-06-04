#include "CaptionsIpcClient.h"
#include "Protocol.h"

#include <string>

namespace {

openvr_pair::overlay::IpcClientConnectOptions Options()
{
	openvr_pair::overlay::IpcClientConnectOptions options;
	options.pipeUnavailable = [](DWORD, const std::string&) {
		return "Captions driver unavailable. SteamVR is not running, "
		       "the WKOpenVR shared driver is not installed, or "
		       "the Captions feature is not enabled "
		       "(enable_captions.flag missing from the driver's resources folder).";
	};
	options.pipeModeFailed = [](DWORD error, const std::string& details) {
		return "Could not set Captions pipe mode. Error " + std::to_string(error) + ": " + details;
	};
	options.versionMismatch = [](uint32_t expected, uint32_t driver) {
		return "Captions driver protocol version mismatch. "
		       "Reinstall WKOpenVR so the overlay and driver are paired. "
		       "(Overlay: " +
		       std::to_string(expected) + ", driver: " + std::to_string(driver) + ")";
	};
	options.reconnectFailurePrefix = "Captions IPC reconnect failed: ";
	options.writeFailurePrefix = "Captions IPC write error";
	options.readFailurePrefix = "Captions IPC read error";
	options.oversizedResponseMessage = "Invalid Captions IPC response: message exceeds expected size ";
	options.sizeMismatchMessagePrefix = "Invalid Captions IPC response";
	return options;
}

} // namespace

void CaptionsIpcClient::Connect()
{
	openvr_pair::overlay::IpcClientBase::Connect(OPENVR_PAIRDRIVER_CAPTIONS_PIPE_NAME, Options());
}
