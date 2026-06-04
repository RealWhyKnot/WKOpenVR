#pragma once

#include "IpcClientBase.h"
#include "Protocol.h"

// IPC client for the Captions overlay plugin. Connects to
// OPENVR_PAIRDRIVER_CAPTIONS_PIPE_NAME. Sends
// RequestHandshake, RequestSetCaptionsConfig, and
// RequestCaptionsRestartHost.

class CaptionsIpcClient : public openvr_pair::overlay::IpcClientBase
{
public:
	// Open the pipe, set message mode, handshake, verify protocol version.
	// Throws std::runtime_error on failure.
	void Connect();

	using openvr_pair::overlay::IpcClientBase::Close;
	using openvr_pair::overlay::IpcClientBase::ConnectionGeneration;
	using openvr_pair::overlay::IpcClientBase::IsConnected;
	using openvr_pair::overlay::IpcClientBase::Receive;
	using openvr_pair::overlay::IpcClientBase::Send;
	using openvr_pair::overlay::IpcClientBase::SendBlocking;
};
