#pragma once

#include "IpcClientBase.h"
#include "Protocol.h"

// IPC client for the OSC Router overlay. Talks to the WKOpenVR shared driver
// over OPENVR_PAIRDRIVER_OSCROUTER_PIPE_NAME. Sends route subscribe/
// unsubscribe, test publish, and stats-query requests.

class RouterIpcClient : public openvr_pair::overlay::IpcClientBase
{
public:
	void Connect();

	using openvr_pair::overlay::IpcClientBase::Close;
	using openvr_pair::overlay::IpcClientBase::ConnectionGeneration;
	using openvr_pair::overlay::IpcClientBase::IsConnected;
	using openvr_pair::overlay::IpcClientBase::Receive;
	using openvr_pair::overlay::IpcClientBase::Send;
	using openvr_pair::overlay::IpcClientBase::SendBlocking;
};
