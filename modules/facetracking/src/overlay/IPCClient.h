#pragma once

#include "IpcClientBase.h"
#include "Protocol.h"

// IPC client for the FaceTracking overlay. Talks to the WKOpenVR
// shared driver over OPENVR_PAIRDRIVER_FACETRACKING_PIPE_NAME. Same
// Request / Response protocol as the other feature pipes; this client sends
// RequestHandshake, RequestSetFaceTrackingConfig,
// RequestSetFaceCalibrationCommand, and RequestSetFaceActiveModule.

class FtIPCClient : public openvr_pair::overlay::IpcClientBase
{
public:
	// Open the pipe, set message mode, handshake, and verify protocol version.
	// Throws std::runtime_error on failure with a UI-ready message.
	void Connect();

	// Send a request and read the response, with one transparent reconnect on
	// broken-pipe errors (covers vrserver restart mid-request).
	using openvr_pair::overlay::IpcClientBase::Close;
	using openvr_pair::overlay::IpcClientBase::ConnectionGeneration;
	using openvr_pair::overlay::IpcClientBase::IsConnected;
	using openvr_pair::overlay::IpcClientBase::Receive;
	using openvr_pair::overlay::IpcClientBase::Send;
	using openvr_pair::overlay::IpcClientBase::SendBlocking;
};
