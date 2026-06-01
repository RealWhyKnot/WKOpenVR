#include "RouterIpcClient.h"

#include <string>

namespace {

openvr_pair::overlay::IpcClientConnectOptions Options()
{
    openvr_pair::overlay::IpcClientConnectOptions opts;
    opts.pipeUnavailable = [](DWORD, const std::string &) {
        return "OSC Router driver unavailable. SteamVR is not running, "
               "the WKOpenVR shared driver is not installed, or "
               "no enabled module requested OSC routing.";
    };
    opts.pipeModeFailed = [](DWORD error, const std::string &details) {
        return "Could not set OSC Router pipe mode. Error "
               + std::to_string(error) + ": " + details;
    };
    opts.versionMismatch = [](uint32_t expected, uint32_t driver) {
        return "OSC Router driver protocol version mismatch. "
               "Reinstall WKOpenVR so the overlay and driver are paired. "
               "(Overlay: " + std::to_string(expected) +
               ", driver: "  + std::to_string(driver) + ")";
    };
    opts.reconnectFailurePrefix  = "OSC Router IPC reconnect failed: ";
    opts.writeFailurePrefix      = "OSC Router IPC write error";
    opts.readFailurePrefix       = "OSC Router IPC read error";
    opts.oversizedResponseMessage = "Invalid OSC Router IPC response: message exceeds expected size ";
    opts.sizeMismatchMessagePrefix = "Invalid OSC Router IPC response";
    return opts;
}

} // namespace

void RouterIpcClient::Connect()
{
    openvr_pair::overlay::IpcClientBase::Connect(
        OPENVR_PAIRDRIVER_OSCROUTER_PIPE_NAME,
        Options());
}
