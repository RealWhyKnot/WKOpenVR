#pragma once

#include "IpcClientBase.h"
#include "Protocol.h"

class SCIPCClient : public openvr_pair::overlay::IpcClientBase
{
public:
	void Connect();

	// True once Connect() has completed and the pipe handle is still alive.
	// Goes back to false if a broken-pipe error closed the handle and the
	// transparent reconnect attempt failed. The overlay UI uses this to
	// show a connection-status dot in the version line -- when false, the
	// user has likely uninstalled or disabled the SteamVR driver.
	using openvr_pair::overlay::IpcClientBase::IsConnected;
	using openvr_pair::overlay::IpcClientBase::Receive;
	using openvr_pair::overlay::IpcClientBase::Send;
	using openvr_pair::overlay::IpcClientBase::SendBlocking;

protected:
	void OnPipeOpenAttempt(HANDLE pipe, DWORD lastError) override;
	void OnHandshakeResponse(const protocol::Response& response) override;
	void OnBrokenPipe(DWORD error) override;
	void OnReconnectSucceeded() override;
};
