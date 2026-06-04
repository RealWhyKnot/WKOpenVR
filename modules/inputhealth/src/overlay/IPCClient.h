#pragma once

#include "IpcClientBase.h"
#include "Protocol.h"

// IPC client for the InputHealth overlay. Talks to the WKOpenVR
// shared driver over `\\.\pipe\WKOpenVR-InputHealth` (the third feature pipe
// alongside calibration and smoothing). Same Request/Response protocol on
// the wire; this overlay only sends RequestHandshake,
// RequestSetInputHealthConfig, RequestSetInputHealthCompensation, and
// RequestResetInputHealthStats.

class IPCClient : public openvr_pair::overlay::IpcClientBase
{
public:
	// Open the pipe, switch to message mode, send a handshake, and verify
	// the protocol version matches. Throws std::runtime_error on any
	// failure with a human-readable message suitable for surfacing in the
	// UI's status bar.
	void Connect();

	// Send a request and read the response in one call. Transparently
	// reconnects (once) if the pipe is broken between SteamVR restarts.
	protocol::Response SendCompensationEntry(const protocol::InputHealthCompensationEntry& entry);

	using openvr_pair::overlay::IpcClientBase::Close;
	using openvr_pair::overlay::IpcClientBase::ConnectionGeneration;
	using openvr_pair::overlay::IpcClientBase::IsConnected;
	using openvr_pair::overlay::IpcClientBase::Receive;
	using openvr_pair::overlay::IpcClientBase::Send;
	using openvr_pair::overlay::IpcClientBase::SendBlocking;
};
