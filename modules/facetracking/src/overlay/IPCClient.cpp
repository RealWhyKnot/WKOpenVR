#include "IPCClient.h"

#include <string>

namespace {

openvr_pair::overlay::IpcClientConnectOptions Options()
{
	openvr_pair::overlay::IpcClientConnectOptions options;
	options.pipeUnavailable = [](DWORD, const std::string&) {
		return "FaceTracking driver unavailable. SteamVR is not running, "
		       "the WKOpenVR shared driver is not installed, or "
		       "the FaceTracking feature is not enabled "
		       "(enable_facetracking.flag missing from the driver's resources folder).";
	};
	options.pipeModeFailed = [](DWORD error, const std::string& details) {
		return "Could not set FaceTracking pipe mode. Error " + std::to_string(error) + ": " + details;
	};
	options.versionMismatch = [](uint32_t expected, uint32_t driver) {
		return "FaceTracking driver protocol version mismatch. "
		       "Reinstall WKOpenVR so the overlay and driver are paired. "
		       "(Overlay: " +
		       std::to_string(expected) + ", driver: " + std::to_string(driver) + ")";
	};
	options.reconnectFailurePrefix = "FaceTracking IPC reconnect failed: ";
	options.writeFailurePrefix = "FaceTracking IPC write error";
	options.readFailurePrefix = "FaceTracking IPC read error";
	options.oversizedResponseMessage = "Invalid FaceTracking IPC response: message exceeds expected size ";
	options.sizeMismatchMessagePrefix = "Invalid FaceTracking IPC response";
	return options;
}
} // namespace

void FtIPCClient::Connect()
{
	openvr_pair::overlay::IpcClientBase::Connect(OPENVR_PAIRDRIVER_FACETRACKING_PIPE_NAME, Options());
}
