#pragma once

#include "Protocol.h"

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

#include <chrono>
#include <cstdint>
#include <functional>
#include <stdexcept>
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

	// Hard deadline for each pipe read/write. The overlay runs every module
	// on its render thread, so a driver that accepts the connection but
	// stalls mid-request must produce a bounded failure, never an indefinite
	// hang. On expiry the transfer is cancelled, the pipe closes (a
	// half-transferred message stream is unusable), and the caller gets an
	// IpcTimeoutException.
	DWORD ioTimeoutMs = 2000;
};

// Thrown when a pipe operation exceeds IpcClientConnectOptions::ioTimeoutMs.
// Derives from runtime_error so existing catch sites degrade to their
// driver-disconnected handling; distinct so callers may special-case it.
class IpcTimeoutException : public std::runtime_error
{
public:
	explicit IpcTimeoutException(const std::string& message) : std::runtime_error(message) {}
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

private:
	// Failed connect attempts arm a backoff window; further attempts inside
	// it fail immediately without touching the pipe. Several modules retry
	// on 1 s heartbeats -- without this, each disconnected module pays the
	// full WaitNamedPipe timeout on the render thread every second.
	static constexpr std::chrono::milliseconds kConnectBackoff{1000};

	struct IoResult
	{
		bool ok = false;
		DWORD bytes = 0;
		DWORD error = ERROR_SUCCESS;
	};

	// One overlapped read/write with the ioTimeoutMs deadline. Throws
	// IpcTimeoutException (after cancelling the transfer and closing the
	// pipe) on expiry; otherwise returns the completion result for the
	// caller's own error handling.
	IoResult OverlappedTransfer(bool isWrite, void* buffer, DWORD length);

	void ConnectImpl(const char* pipeName, IpcClientConnectOptions options);

	HANDLE ioEvent_ = nullptr;
	std::chrono::steady_clock::time_point nextConnectAttempt_{};
};

} // namespace openvr_pair::overlay
