#include "IPCClient.h"
#include "CalibrationMetrics.h" // WriteLogAnnotation -- see Connect() for why
                                // we trace the IPC handshake outcomes.

#include <cstdio>
#include <string>

// Forward-declared rather than #include "Calibration.h" because Calibration.h
// pulls in <openvr.h> while SCIPCClient.cpp's translation unit (via Protocol.h)
// already includes <openvr_driver.h>; the two openvr headers redefine common
// constants and conflict at compile time. The single function we need is
// signature-stable, so a local forward declaration is the safe minimum coupling.
void ReopenShmem();

namespace {
openvr_pair::overlay::IpcClientConnectOptions Options()
{
	openvr_pair::overlay::IpcClientConnectOptions options;
	options.pipeUnavailable = [](DWORD, const std::string&) {
		return "Space Calibrator driver unavailable. Make sure SteamVR is running, and the Space Calibrator addon is "
		       "enabled in SteamVR settings.";
	};
	options.pipeModeFailed = [](DWORD error, const std::string& details) {
		return "Couldn't set pipe mode. Error " + std::to_string(error) + ": " + details;
	};
	options.versionMismatch = [](uint32_t expected, uint32_t driver) {
		return "Incorrect driver version installed, try reinstalling Space Calibrator. (Client: " +
		       std::to_string(expected) + ", Driver: " + std::to_string(driver) + ")";
	};
	options.writeFailurePrefix = "Error writing IPC request";
	options.readFailurePrefix = "Error reading IPC response";
	options.oversizedResponseMessage = "Invalid IPC response. Error MESSAGE_TOO_LARGE, expected ";
	options.sizeMismatchMessagePrefix = "Invalid IPC response. Error SIZE_MISMATCH";
	return options;
}
} // namespace

void SCIPCClient::Connect()
{
	openvr_pair::overlay::IpcClientBase::Connect(OPENVR_PAIRDRIVER_CALIBRATION_PIPE_NAME, Options());
}

void SCIPCClient::OnPipeOpenAttempt(HANDLE pipe, DWORD lastError)
{
	char annot[256];
	snprintf(annot, sizeof annot, "ipc_pipe_open: handle=%p err=%lu", pipe, lastError);
	Metrics::WriteLogAnnotation(annot);
}

void SCIPCClient::OnHandshakeResponse(const protocol::Response& response)
{
	char annot[160];
	snprintf(annot, sizeof annot, "ipc_handshake: response_type=%d server_version=%u client_version=%u",
	         (int)response.type, (unsigned)response.protocol.version, (unsigned)protocol::Version);
	Metrics::WriteLogAnnotation(annot);
}

void SCIPCClient::OnBrokenPipe(DWORD error)
{
	fprintf(stderr, "[SCIPCClient] Broken pipe (error %lu) during request; attempting reconnect...\n",
	        (unsigned long)error);
}

void SCIPCClient::OnReconnectSucceeded()
{
	ReopenShmem();
}
