#pragma once

#include "Protocol.h"

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

#include <chrono>
#include <cstdint>
#include <functional>
#include <string>

namespace openvr_pair::overlay {

struct IpcClientConnectOptions
{
	using Win32ErrorFormatter = std::function<std::string(DWORD error, const std::string& details)>;
	using VersionMismatchFormatter = std::function<std::string(uint32_t expectedVersion, uint32_t driverVersion)>;

	Win32ErrorFormatter pipeUnavailable;
	Win32ErrorFormatter pipeModeFailed;
	VersionMismatchFormatter versionMismatch;
	std::string reconnectFailurePrefix = "IPC reconnect failed after broken pipe: ";
	std::string writeFailurePrefix = "Error writing IPC request";
	std::string readFailurePrefix = "Error reading IPC response";
	std::string oversizedResponseMessage = "Invalid IPC response. Message larger than expected ";
	std::string sizeMismatchMessagePrefix = "Invalid IPC response";
	DWORD waitTimeoutMs = 50;
};

class IpcClientBase
{
public:
	enum class MismatchState
	{
		Matching,
		OverlayNewer,
		DriverNewer
	};

	virtual ~IpcClientBase();

	void Connect(const char* pipeName, IpcClientConnectOptions options = {});
	protocol::Response SendBlocking(const protocol::Request& request);
	void Send(const protocol::Request& request);
	protocol::Response Receive();
	bool IsConnected() const { return pipe_ != INVALID_HANDLE_VALUE; }
	void Close();
	uint64_t ConnectionGeneration() const { return connectionGeneration_; }

	MismatchState GetMismatchState() const { return mismatchState_; }
	uint32_t GetDriverVersion() const { return driverVersion_; }
	uint32_t GetExpectedVersion() const { return protocol::Version; }

protected:
	virtual void OnPipeOpenAttempt(HANDLE pipe, DWORD lastError)
	{
		(void)pipe;
		(void)lastError;
	}
	virtual void OnHandshakeResponse(const protocol::Response& response) { (void)response; }
	virtual void OnBrokenPipe(DWORD error) { (void)error; }
	virtual void OnReconnectSucceeded() {}

	HANDLE pipe_ = INVALID_HANDLE_VALUE;
	std::string pipeName_;
	IpcClientConnectOptions options_;
	uint64_t connectionGeneration_ = 0;
	bool reconnecting_ = false;

	MismatchState mismatchState_ = MismatchState::Matching;
	uint32_t driverVersion_ = 0;
};

} // namespace openvr_pair::overlay
