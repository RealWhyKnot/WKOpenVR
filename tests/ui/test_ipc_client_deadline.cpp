// Deadline + reconnect-backoff behavior of the overlay IPC client, exercised
// against a live in-process named-pipe server. The overlay runs every module
// on its render thread, so the client contract under test is: no pipe
// operation may block past its deadline, a timed-out pipe is dropped (and
// reconnectable), and repeated connect attempts against a dead driver fail
// fast inside the backoff window instead of re-paying the pipe wait each try.

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <string>
#include <thread>

#include "IpcClientBase.h"
#include "Protocol.h"

using openvr_pair::overlay::IpcClientBase;
using openvr_pair::overlay::IpcClientConnectOptions;
using openvr_pair::overlay::IpcTimeoutException;

namespace {

class PipeServer
{
public:
	enum class Mode
	{
		RespondAll,            // answer every request
		SwallowAfterHandshake, // answer handshakes, never anything else
	};

	explicit PipeServer(Mode mode) : mode_(mode)
	{
		static std::atomic<int> s_counter{0};
		name_ = "\\\\.\\pipe\\wkopenvr_ipc_test_" + std::to_string(GetCurrentProcessId()) + "_" +
		        std::to_string(s_counter.fetch_add(1));
		thread_ = std::thread([this] { Run(); });
	}

	~PipeServer()
	{
		stop_.store(true);
		// Unblock a server parked in ConnectNamedPipe by completing one
		// connection ourselves.
		HANDLE nudge = CreateFileA(name_.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, 0, nullptr);
		if (nudge != INVALID_HANDLE_VALUE) CloseHandle(nudge);
		if (thread_.joinable()) thread_.join();
	}

	const char* Name() const { return name_.c_str(); }

private:
	void Run()
	{
		while (!stop_.load()) {
			HANDLE pipe = CreateNamedPipeA(name_.c_str(), PIPE_ACCESS_DUPLEX,
			                               PIPE_TYPE_MESSAGE | PIPE_READMODE_MESSAGE | PIPE_WAIT, 1,
			                               sizeof(protocol::Response), sizeof(protocol::Request), 0, nullptr);
			if (pipe == INVALID_HANDLE_VALUE) return;
			const BOOL connected =
			    ConnectNamedPipe(pipe, nullptr) ? TRUE : (GetLastError() == ERROR_PIPE_CONNECTED ? TRUE : FALSE);
			if (!connected || stop_.load()) {
				CloseHandle(pipe);
				continue;
			}
			ServeOne(pipe);
			DisconnectNamedPipe(pipe);
			CloseHandle(pipe);
		}
	}

	void ServeOne(HANDLE pipe)
	{
		bool handshakeDone = false;
		for (;;) {
			protocol::Request request;
			DWORD bytes = 0;
			if (!ReadFile(pipe, &request, sizeof request, &bytes, nullptr) || bytes != sizeof request) {
				return; // client hung up
			}
			if (request.type == protocol::RequestHandshake) {
				protocol::Response resp(protocol::ResponseHandshake);
				resp.protocol.version = protocol::Version;
				DWORD written = 0;
				if (!WriteFile(pipe, &resp, sizeof resp, &written, nullptr)) return;
				handshakeDone = true;
				continue;
			}
			if (mode_ == Mode::SwallowAfterHandshake && handshakeDone) {
				continue; // never reply: the client's deadline must fire
			}
			protocol::Response resp(protocol::ResponseSuccess);
			DWORD written = 0;
			if (!WriteFile(pipe, &resp, sizeof resp, &written, nullptr)) return;
		}
	}

	Mode mode_;
	std::string name_;
	std::atomic<bool> stop_{false};
	std::thread thread_;
};

IpcClientConnectOptions ShortDeadline()
{
	IpcClientConnectOptions options;
	options.ioTimeoutMs = 250;
	return options;
}

} // namespace

TEST(IpcClientDeadline, RoundTripSucceeds)
{
	PipeServer server(PipeServer::Mode::RespondAll);
	IpcClientBase client;
	client.Connect(server.Name(), ShortDeadline());
	ASSERT_TRUE(client.IsConnected());
	const protocol::Response resp = client.SendBlocking(protocol::Request(protocol::RequestSetFreezeAllTracking));
	EXPECT_EQ(resp.type, protocol::ResponseSuccess);
}

TEST(IpcClientDeadline, StalledDriverTimesOutAndDropsThePipe)
{
	PipeServer server(PipeServer::Mode::SwallowAfterHandshake);
	IpcClientBase client;
	client.Connect(server.Name(), ShortDeadline());
	ASSERT_TRUE(client.IsConnected());

	const auto start = std::chrono::steady_clock::now();
	EXPECT_THROW(client.SendBlocking(protocol::Request(protocol::RequestSetFreezeAllTracking)), IpcTimeoutException);
	const auto elapsed =
	    std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start);

	EXPECT_GE(elapsed.count(), 200) << "the deadline should be allowed to run";
	EXPECT_LT(elapsed.count(), 3000) << "a stalled driver must produce a bounded failure, not a hang";
	EXPECT_FALSE(client.IsConnected()) << "a half-transferred message stream is unusable and must be dropped";
}

TEST(IpcClientDeadline, ReconnectsAfterATimeout)
{
	PipeServer server(PipeServer::Mode::SwallowAfterHandshake);
	IpcClientBase client;
	client.Connect(server.Name(), ShortDeadline());
	EXPECT_THROW(client.SendBlocking(protocol::Request(protocol::RequestSetFreezeAllTracking)), IpcTimeoutException);
	ASSERT_FALSE(client.IsConnected());

	// Timeouts do not arm the connect backoff; the next call reconnects
	// (the server answers handshakes even in swallow mode).
	const protocol::Response resp = client.SendBlocking(protocol::Request(protocol::RequestHandshake));
	EXPECT_EQ(resp.type, protocol::ResponseHandshake);
	EXPECT_TRUE(client.IsConnected());
}

TEST(IpcClientDeadline, FailedConnectArmsTheBackoffWindow)
{
	const std::string missing = "\\\\.\\pipe\\wkopenvr_ipc_test_missing_" + std::to_string(GetCurrentProcessId());
	IpcClientBase client;
	EXPECT_THROW(client.Connect(missing.c_str(), ShortDeadline()), std::runtime_error);

	// Inside the backoff window the client must fail fast without touching
	// the pipe again.
	try {
		client.Connect(missing.c_str(), ShortDeadline());
		FAIL() << "second connect inside the backoff window should throw";
	}
	catch (const std::runtime_error& e) {
		EXPECT_NE(std::string(e.what()).find("deferred"), std::string::npos)
		    << "expected the fast backoff failure, got: " << e.what();
	}
}
