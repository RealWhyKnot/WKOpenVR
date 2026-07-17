#include "IpcClientBase.h"

#include "DiagnosticsLog.h"
#include "Win32Errors.h"

#include <cstdio>
#include <stdexcept>
#include <utility>

namespace openvr_pair::overlay {
namespace {

class BrokenPipeException : public std::runtime_error
{
public:
	BrokenPipeException(const std::string& message, DWORD code) : std::runtime_error(message), errorCode(code) {}

	DWORD errorCode = ERROR_SUCCESS;
};

bool IsBrokenPipeError(DWORD code)
{
	return code == ERROR_BROKEN_PIPE || code == ERROR_PIPE_NOT_CONNECTED || code == ERROR_NO_DATA;
}

std::string GenericWin32Message(const char* prefix, DWORD error)
{
	return std::string(prefix) + ". Error " + std::to_string(error) + ": " +
	       openvr_pair::common::FormatWin32Error(error);
}

std::string FormatUnavailable(const IpcClientConnectOptions& options, DWORD error)
{
	std::string details = openvr_pair::common::FormatWin32Error(error);
	if (options.pipeUnavailable) return options.pipeUnavailable(error, details);
	return GenericWin32Message("IPC pipe unavailable", error);
}

std::string FormatModeFailure(const IpcClientConnectOptions& options, DWORD error)
{
	std::string details = openvr_pair::common::FormatWin32Error(error);
	if (options.pipeModeFailed) return options.pipeModeFailed(error, details);
	return GenericWin32Message("IPC pipe mode failed", error);
}

std::string FormatVersionMismatch(const IpcClientConnectOptions& options, uint32_t expected, uint32_t driver)
{
	if (options.versionMismatch) return options.versionMismatch(expected, driver);
	return "Driver protocol version mismatch. (Overlay: " + std::to_string(expected) +
	       ", driver: " + std::to_string(driver) + ")";
}

} // namespace

IpcClientBase::~IpcClientBase()
{
	Close();
	if (ioEvent_) {
		CloseHandle(ioEvent_);
		ioEvent_ = nullptr;
	}
}

void IpcClientBase::Close()
{
	if (pipe_ != INVALID_HANDLE_VALUE) {
		openvr_pair::common::DiagnosticLog("ipc-client", "close pipe='%s' generation=%llu", pipeName_.c_str(),
		                                   static_cast<unsigned long long>(connectionGeneration_));
		CloseHandle(pipe_);
		pipe_ = INVALID_HANDLE_VALUE;
	}
}

void IpcClientBase::Connect(const char* pipeName, IpcClientConnectOptions options)
{
	const auto now = std::chrono::steady_clock::now();
	if (now < nextConnectAttempt_) {
		throw std::runtime_error("IPC connect attempt deferred after a recent failure.");
	}
	try {
		ConnectImpl(pipeName, std::move(options));
		nextConnectAttempt_ = {};
	}
	catch (...) {
		nextConnectAttempt_ = now + kConnectBackoff;
		throw;
	}
}

void IpcClientBase::ConnectImpl(const char* pipeName, IpcClientConnectOptions options)
{
	Close();
	options_ = std::move(options);
	pipeName_ = pipeName ? pipeName : "";
	const BOOL waitOk = WaitNamedPipeA(pipeName_.c_str(), options_.waitTimeoutMs);
	const DWORD waitError = waitOk ? ERROR_SUCCESS : GetLastError();
	openvr_pair::common::DiagnosticLog(
	    "ipc-client", "connect_start pipe='%s' wait_ok=%d wait_error=%lu wait_timeout_ms=%lu", pipeName_.c_str(),
	    waitOk ? 1 : 0, waitError, static_cast<unsigned long>(options_.waitTimeoutMs));
	pipe_ = CreateFileA(pipeName_.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING,
	                    FILE_FLAG_OVERLAPPED, nullptr);
	DWORD openError = (pipe_ == INVALID_HANDLE_VALUE) ? GetLastError() : ERROR_SUCCESS;
	OnPipeOpenAttempt(pipe_, openError);
	if (pipe_ == INVALID_HANDLE_VALUE) {
		openvr_pair::common::DiagnosticLog("ipc-client", "connect_open_failed pipe='%s' error=%lu", pipeName_.c_str(),
		                                   openError);
		throw std::runtime_error(FormatUnavailable(options_, openError));
	}

	DWORD mode = PIPE_READMODE_MESSAGE;
	if (!SetNamedPipeHandleState(pipe_, &mode, nullptr, nullptr)) {
		DWORD err = GetLastError();
		openvr_pair::common::DiagnosticLog("ipc-client", "connect_mode_failed pipe='%s' error=%lu", pipeName_.c_str(),
		                                   err);
		Close();
		throw std::runtime_error(FormatModeFailure(options_, err));
	}

	auto response = SendBlocking(protocol::Request(protocol::RequestHandshake));
	OnHandshakeResponse(response);
	openvr_pair::common::DiagnosticLog(
	    "ipc-client", "handshake_response pipe='%s' response_type=%d driver_protocol=%u expected_protocol=%u",
	    pipeName_.c_str(), response.type, (unsigned)response.protocol.version, (unsigned)protocol::Version);
	if (response.type != protocol::ResponseHandshake || response.protocol.version != protocol::Version) {
		driverVersion_ = response.protocol.version;
		mismatchState_ =
		    (response.protocol.version < protocol::Version) ? MismatchState::OverlayNewer : MismatchState::DriverNewer;
		openvr_pair::common::DiagnosticLog(
		    "ipc-client",
		    "handshake_mismatch pipe='%s' response_type=%d driver_protocol=%u expected_protocol=%u state=%d",
		    pipeName_.c_str(), response.type, (unsigned)response.protocol.version, (unsigned)protocol::Version,
		    (int)mismatchState_);
		Close();
		throw std::runtime_error(FormatVersionMismatch(options_, protocol::Version, response.protocol.version));
	}
	mismatchState_ = MismatchState::Matching;
	fprintf(stderr, "[IPC] %s handshake ok: our_protocol=%u driver_protocol=%u\n", pipeName_.c_str(),
	        (unsigned)protocol::Version, (unsigned)response.protocol.version);
	++connectionGeneration_;
	openvr_pair::common::DiagnosticLog("ipc-client", "connect_ok pipe='%s' generation=%llu", pipeName_.c_str(),
	                                   static_cast<unsigned long long>(connectionGeneration_));
}

protocol::Response IpcClientBase::SendBlocking(const protocol::Request& request)
{
	if (pipe_ == INVALID_HANDLE_VALUE) {
		if (pipeName_.empty()) throw std::runtime_error("IPC pipe is not connected.");
		Connect(pipeName_.c_str(), options_);
	}

	try {
		Send(request);
		return Receive();
	}
	catch (const BrokenPipeException& e) {
		if (reconnecting_ || pipeName_.empty()) throw;

		openvr_pair::common::DiagnosticLog(
		    "ipc-client", "broken_pipe pipe='%s' request_type=%d error=%lu generation=%llu", pipeName_.c_str(),
		    request.type, e.errorCode, static_cast<unsigned long long>(connectionGeneration_));
		OnBrokenPipe(e.errorCode);
		Close();

		reconnecting_ = true;
		try {
			Connect(pipeName_.c_str(), options_);
			reconnecting_ = false;
			OnReconnectSucceeded();
			openvr_pair::common::DiagnosticLog("ipc-client", "reconnect_ok pipe='%s' request_type=%d generation=%llu",
			                                   pipeName_.c_str(), request.type,
			                                   static_cast<unsigned long long>(connectionGeneration_));
		}
		catch (const std::exception& reconnectError) {
			reconnecting_ = false;
			openvr_pair::common::DiagnosticLog("ipc-client", "reconnect_failed pipe='%s' request_type=%d error='%s'",
			                                   pipeName_.c_str(), request.type, reconnectError.what());
			throw std::runtime_error(options_.reconnectFailurePrefix + reconnectError.what());
		}

		Send(request);
		return Receive();
	}
}

IpcClientBase::IoResult IpcClientBase::OverlappedTransfer(bool isWrite, void* buffer, DWORD length)
{
	if (!ioEvent_) {
		ioEvent_ = CreateEventW(nullptr, /*manualReset=*/TRUE, /*initial=*/FALSE, nullptr);
		if (!ioEvent_) {
			throw std::runtime_error("IPC event creation failed. Error " + std::to_string(GetLastError()));
		}
	}
	ResetEvent(ioEvent_);
	OVERLAPPED ov{};
	ov.hEvent = ioEvent_;

	const BOOL started =
	    isWrite ? WriteFile(pipe_, buffer, length, nullptr, &ov) : ReadFile(pipe_, buffer, length, nullptr, &ov);
	if (!started) {
		const DWORD startError = GetLastError();
		if (startError == ERROR_IO_PENDING) {
			const DWORD wait = WaitForSingleObject(ioEvent_, options_.ioTimeoutMs);
			if (wait != WAIT_OBJECT_0) {
				// Deadline expired (or the wait itself failed): cancel the
				// transfer, reap the cancellation so the OVERLAPPED can leave
				// scope safely, and drop the pipe -- a half-transferred
				// message stream cannot be resynchronized.
				CancelIoEx(pipe_, &ov);
				DWORD ignored = 0;
				GetOverlappedResult(pipe_, &ov, &ignored, /*bWait=*/TRUE);
				openvr_pair::common::DiagnosticLog("ipc-client", "io_timeout pipe='%s' op=%s timeout_ms=%lu",
				                                   pipeName_.c_str(), isWrite ? "write" : "read",
				                                   static_cast<unsigned long>(options_.ioTimeoutMs));
				Close();
				throw IpcTimeoutException(std::string("IPC ") + (isWrite ? "write" : "read") + " timed out after " +
				                          std::to_string(options_.ioTimeoutMs) + " ms.");
			}
		}
		else {
			// Immediate failure (broken pipe, oversized message, ...). Fetch
			// whatever partial byte count exists and hand the caller the
			// original error for its own handling.
			IoResult result;
			result.ok = false;
			result.error = startError;
			GetOverlappedResult(pipe_, &ov, &result.bytes, /*bWait=*/FALSE);
			return result;
		}
	}

	IoResult result;
	result.ok = GetOverlappedResult(pipe_, &ov, &result.bytes, /*bWait=*/FALSE) != FALSE;
	result.error = result.ok ? ERROR_SUCCESS : GetLastError();
	return result;
}

void IpcClientBase::Send(const protocol::Request& request)
{
	if (pipe_ == INVALID_HANDLE_VALUE) {
		throw std::runtime_error("IPC pipe is not connected.");
	}

	const IoResult io = OverlappedTransfer(/*isWrite=*/true, const_cast<protocol::Request*>(&request), sizeof request);
	if (!io.ok) {
		DWORD err = io.error;
		openvr_pair::common::DiagnosticLog("ipc-client", "write_failed pipe='%s' request_type=%d error=%lu",
		                                   pipeName_.c_str(), request.type, err);
		Close();
		std::string msg = options_.writeFailurePrefix + ". Error " + std::to_string(err) + ": " +
		                  openvr_pair::common::FormatWin32Error(err);
		if (IsBrokenPipeError(err)) {
			throw BrokenPipeException(msg, err);
		}
		throw std::runtime_error(msg);
	}
	if (io.bytes != sizeof request) {
		openvr_pair::common::DiagnosticLog("ipc-client",
		                                   "write_truncated pipe='%s' request_type=%d wrote=%lu expected=%zu",
		                                   pipeName_.c_str(), request.type, io.bytes, sizeof request);
		Close();
		throw std::runtime_error("IPC write truncated: wrote " + std::to_string(io.bytes) + " of " +
		                         std::to_string(sizeof request) + " bytes");
	}
}

protocol::Response IpcClientBase::Receive()
{
	if (pipe_ == INVALID_HANDLE_VALUE) {
		throw std::runtime_error("IPC pipe is not connected.");
	}

	protocol::Response response(protocol::ResponseInvalid);
	const IoResult io = OverlappedTransfer(/*isWrite=*/false, &response, sizeof response);
	if (!io.ok) {
		DWORD err = io.error;
		if (err == ERROR_MORE_DATA) {
			openvr_pair::common::DiagnosticLog("ipc-client", "oversized_response pipe='%s' error=%lu expected_max=%zu",
			                                   pipeName_.c_str(), err, sizeof response);
			char drainBuffer[1024];
			for (;;) {
				const IoResult drain = OverlappedTransfer(/*isWrite=*/false, drainBuffer, sizeof drainBuffer);
				if (drain.ok) break;

				if (drain.error == ERROR_MORE_DATA) continue;
				if (IsBrokenPipeError(drain.error)) {
					Close();
					throw BrokenPipeException("Pipe broken while draining oversized IPC response", drain.error);
				}
				break;
			}
			throw std::runtime_error(options_.oversizedResponseMessage + std::to_string(sizeof response) + " bytes.");
		}

		Close();
		openvr_pair::common::DiagnosticLog("ipc-client", "read_failed pipe='%s' error=%lu", pipeName_.c_str(), err);
		std::string msg = options_.readFailurePrefix + ". Error " + std::to_string(err) + ": " +
		                  openvr_pair::common::FormatWin32Error(err);
		if (IsBrokenPipeError(err)) {
			throw BrokenPipeException(msg, err);
		}
		throw std::runtime_error(msg);
	}
	if (io.bytes != sizeof response) {
		openvr_pair::common::DiagnosticLog("ipc-client", "read_truncated pipe='%s' read=%lu expected=%zu",
		                                   pipeName_.c_str(), io.bytes, sizeof response);
		Close();
		throw std::runtime_error(options_.sizeMismatchMessagePrefix + ": got " + std::to_string(io.bytes) +
		                         " bytes, expected " + std::to_string(sizeof response));
	}

	return response;
}

} // namespace openvr_pair::overlay
