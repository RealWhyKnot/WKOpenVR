#pragma once

#include "IpcClientBase.h"
#include "Protocol.h"

// Named-pipe client for the WKOpenVR smoothing pipe.
// Connects to OPENVR_PAIRDRIVER_SMOOTHING_PIPE_NAME, performs the protocol
// handshake, and sends RequestSetFingerSmoothing payloads. Mirrors the SC
// SmoothingIPCClient pattern but only has to deal with the smoothing-feature subset
// of the protocol -- everything else is rejected by the driver's per-pipe
// feature mask.
class SmoothingIPCClient : public openvr_pair::overlay::IpcClientBase
{
public:
	void Connect();
};
