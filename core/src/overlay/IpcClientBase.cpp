#include "IpcClientBase.h"

#include "Win32Errors.h"

#include <cstdio>
#include <stdexcept>
#include <utility>

namespace openvr_pair::overlay {
namespace {

class BrokenPipeException : public std::runtime_error
{
public:
	BrokenPipeException(const std::string &message, DWORD code)
		: std::runtime_error(message), errorCode(code) {}

	DWORD errorCode = ERROR_SUCCESS;
};

bool IsBrokenPipeError(DWORD code)
{
	return code == ERROR_BROKEN_PIPE || code == ERROR_PIPE_NOT_CONNECTED || code == ERROR_NO_DATA;
}

std::string GenericWin32Message(const char *prefix, DWORD error)
{
	return std::string(prefix) + ". Error " + std::to_string(error) + ": "
		+ openvr_pair::common::FormatWin32Error(error);
}

std::string FormatUnavailable(const IpcClientConnectOptions &options, DWORD error)
{
	std::string details = openvr_pair::common::FormatWin32Error(error);
	if (options.pipeUnavailable) return options.pipeUnavailable(error, details);
	return GenericWin32Message("IPC pipe unavailable", error);
}

std::string FormatModeFailure(const IpcClientConnectOptions &options, DWORD error)
{
	std::string details = openvr_pair::common::FormatWin32Error(error);
	if (options.pipeModeFailed) return options.pipeModeFailed(error, details);
	return GenericWin32Message("IPC pipe mode failed", error);
}

std::string FormatVersionMismatch(const IpcClientConnectOptions &options, uint32_t expected, uint32_t driver)
{
	if (options.versionMismatch) return options.versionMismatch(expected, driver);
	return "Driver protocol version mismatch. (Overlay: " + std::to_string(expected)
		+ ", driver: " + std::to_string(driver) + ")";
}

} // namespace

IpcClientBase::~IpcClientBase()
{
	Close();
}

void IpcClientBase::Close()
{
	if (pipe_ != INVALID_HANDLE_VALUE) {
		CloseHandle(pipe_);
		pipe_ = INVALID_HANDLE_VALUE;
	}
}

void IpcClientBase::Connect(const char *pipeName, IpcClientConnectOptions options)
{
	Close();
	options_ = std::move(options);
	pipeName_ = pipeName ? pipeName : "";
	WaitNamedPipeA(pipeName_.c_str(), 1000);
	pipe_ = CreateFileA(pipeName_.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, 0, nullptr);
	DWORD openError = (pipe_ == INVALID_HANDLE_VALUE) ? GetLastError() : ERROR_SUCCESS;
	OnPipeOpenAttempt(pipe_, openError);
	if (pipe_ == INVALID_HANDLE_VALUE) {
		throw std::runtime_error(FormatUnavailable(options_, openError));
	}

	DWORD mode = PIPE_READMODE_MESSAGE;
	if (!SetNamedPipeHandleState(pipe_, &mode, nullptr, nullptr)) {
		DWORD err = GetLastError();
		Close();
		throw std::runtime_error(FormatModeFailure(options_, err));
	}

	auto response = SendBlocking(protocol::Request(protocol::RequestHandshake));
	OnHandshakeResponse(response);
	if (response.type != protocol::ResponseHandshake || response.protocol.version != protocol::Version) {
		driverVersion_ = response.protocol.version;
		mismatchState_ = (response.protocol.version < protocol::Version)
			? MismatchState::OverlayNewer
			: MismatchState::DriverNewer;
		Close();
		throw std::runtime_error(FormatVersionMismatch(options_, protocol::Version, response.protocol.version));
	}
	mismatchState_ = MismatchState::Matching;
	fprintf(stderr, "[IPC] %s handshake ok: our_protocol=%u driver_protocol=%u\n",
		pipeName_.c_str(), (unsigned)protocol::Version, (unsigned)response.protocol.version);
	++connectionGeneration_;
}

protocol::Response IpcClientBase::SendBlocking(const protocol::Request &request)
{
	if (pipe_ == INVALID_HANDLE_VALUE) {
		if (pipeName_.empty()) throw std::runtime_error("IPC pipe is not connected.");
		Connect(pipeName_.c_str(), options_);
	}

	try {
		Send(request);
		return Receive();
	} catch (const BrokenPipeException &e) {
		if (reconnecting_ || pipeName_.empty()) throw;

		OnBrokenPipe(e.errorCode);
		Close();

		reconnecting_ = true;
		try {
			Connect(pipeName_.c_str(), options_);
			reconnecting_ = false;
			OnReconnectSucceeded();
		} catch (const std::exception &reconnectError) {
			reconnecting_ = false;
			throw std::runtime_error(options_.reconnectFailurePrefix + reconnectError.what());
		}

		Send(request);
		return Receive();
	}
}

void IpcClientBase::Send(const protocol::Request &request)
{
	if (pipe_ == INVALID_HANDLE_VALUE) {
		throw std::runtime_error("IPC pipe is not connected.");
	}

	DWORD bytesWritten = 0;
	if (!WriteFile(pipe_, &request, sizeof request, &bytesWritten, nullptr)) {
		DWORD err = GetLastError();
		Close();
		std::string msg = options_.writeFailurePrefix + ". Error " + std::to_string(err)
			+ ": " + openvr_pair::common::FormatWin32Error(err);
		if (IsBrokenPipeError(err)) {
			throw BrokenPipeException(msg, err);
		}
		throw std::runtime_error(msg);
	}
	if (bytesWritten != sizeof request) {
		Close();
		throw std::runtime_error("IPC write truncated: wrote " + std::to_string(bytesWritten)
			+ " of " + std::to_string(sizeof request) + " bytes");
	}
}

protocol::Response IpcClientBase::Receive()
{
	if (pipe_ == INVALID_HANDLE_VALUE) {
		throw std::runtime_error("IPC pipe is not connected.");
	}

	protocol::Response response(protocol::ResponseInvalid);
	DWORD bytesRead = 0;
	if (!ReadFile(pipe_, &response, sizeof response, &bytesRead, nullptr)) {
		DWORD err = GetLastError();
		if (err == ERROR_MORE_DATA) {
			char drainBuffer[1024];
			for (;;) {
				DWORD drained = 0;
				BOOL drainOk = ReadFile(pipe_, drainBuffer, sizeof drainBuffer, &drained, nullptr);
				if (drainOk) break;

				DWORD drainErr = GetLastError();
				if (drainErr == ERROR_MORE_DATA) continue;
				if (IsBrokenPipeError(drainErr)) {
					Close();
					throw BrokenPipeException("Pipe broken while draining oversized IPC response", drainErr);
				}
				break;
			}
			throw std::runtime_error(options_.oversizedResponseMessage + std::to_string(sizeof response) + " bytes.");
		}

		Close();
		std::string msg = options_.readFailurePrefix + ". Error " + std::to_string(err)
			+ ": " + openvr_pair::common::FormatWin32Error(err);
		if (IsBrokenPipeError(err)) {
			throw BrokenPipeException(msg, err);
		}
		throw std::runtime_error(msg);
	}
	if (bytesRead != sizeof response) {
		Close();
		throw std::runtime_error(options_.sizeMismatchMessagePrefix + ": got "
			+ std::to_string(bytesRead) + " bytes, expected "
			+ std::to_string(sizeof response));
	}

	return response;
}

} // namespace openvr_pair::overlay
