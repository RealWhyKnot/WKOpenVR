// Behavior pins for the off-thread IPC send queue: FIFO delivery through the
// worker's own pipe connection, latest-wins coalescing while the worker is
// busy, and a bounded Stop() even when the driver side has stalled.

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "IpcSendQueue.h"
#include "Protocol.h"

using openvr_pair::overlay::IpcClientConnectOptions;
using openvr_pair::overlay::IpcSendQueue;

namespace {

// Minimal recording pipe server. Always answers handshakes; records every
// non-handshake request type in arrival order; in Stall mode it records but
// never answers them.
class RecordingPipeServer
{
public:
	enum class Mode
	{
		RespondAll,
		StallAfterHandshake,
	};

	explicit RecordingPipeServer(Mode mode) : mode_(mode)
	{
		static std::atomic<int> s_counter{0};
		name_ = "\\\\.\\pipe\\wkopenvr_sendqueue_test_" + std::to_string(GetCurrentProcessId()) + "_" +
		        std::to_string(s_counter.fetch_add(1));
		thread_ = std::thread([this] { Run(); });
	}

	~RecordingPipeServer()
	{
		stop_.store(true);
		HANDLE nudge = CreateFileA(name_.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, 0, nullptr);
		if (nudge != INVALID_HANDLE_VALUE) CloseHandle(nudge);
		if (thread_.joinable()) thread_.join();
	}

	const char* Name() const { return name_.c_str(); }

	std::vector<int> RecordedTypes() const
	{
		std::lock_guard<std::mutex> lock(mutex_);
		return recorded_;
	}

	bool WaitForRecordedCount(size_t count, std::chrono::milliseconds deadline) const
	{
		const auto until = std::chrono::steady_clock::now() + deadline;
		while (std::chrono::steady_clock::now() < until) {
			{
				std::lock_guard<std::mutex> lock(mutex_);
				if (recorded_.size() >= count) return true;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(5));
		}
		std::lock_guard<std::mutex> lock(mutex_);
		return recorded_.size() >= count;
	}

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
		for (;;) {
			protocol::Request request;
			DWORD bytes = 0;
			if (!ReadFile(pipe, &request, sizeof request, &bytes, nullptr) || bytes != sizeof request) {
				return;
			}
			if (request.type == protocol::RequestHandshake) {
				protocol::Response resp(protocol::ResponseHandshake);
				resp.protocol.version = protocol::Version;
				DWORD written = 0;
				if (!WriteFile(pipe, &resp, sizeof resp, &written, nullptr)) return;
				continue;
			}
			{
				std::lock_guard<std::mutex> lock(mutex_);
				recorded_.push_back(static_cast<int>(request.type));
			}
			if (mode_ == Mode::StallAfterHandshake) {
				continue; // recorded but never answered
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
	mutable std::mutex mutex_;
	std::vector<int> recorded_;
};

IpcClientConnectOptions ShortDeadline()
{
	IpcClientConnectOptions options;
	options.ioTimeoutMs = 250;
	return options;
}

// The server thread creates its pipe instance asynchronously; connecting
// before it exists would arm the client's backoff and burn the first
// entries. Production self-heals through the periodic republish; the test
// must not depend on that.
bool WaitForPipe(const char* name, std::chrono::milliseconds deadline)
{
	const auto until = std::chrono::steady_clock::now() + deadline;
	while (std::chrono::steady_clock::now() < until) {
		if (WaitNamedPipeA(name, 50)) return true;
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
	return false;
}

bool WaitForSentCount(const IpcSendQueue& queue, uint64_t count, std::chrono::milliseconds deadline)
{
	const auto until = std::chrono::steady_clock::now() + deadline;
	while (std::chrono::steady_clock::now() < until) {
		if (queue.GetStatus().sent >= count) return true;
		std::this_thread::sleep_for(std::chrono::milliseconds(5));
	}
	return queue.GetStatus().sent >= count;
}

} // namespace

TEST(IpcSendQueue, DeliversInEnqueueOrder)
{
	RecordingPipeServer server(RecordingPipeServer::Mode::RespondAll);
	ASSERT_TRUE(WaitForPipe(server.Name(), std::chrono::milliseconds(3000)));
	IpcSendQueue queue;
	queue.Start(server.Name(), ShortDeadline());

	queue.Enqueue(protocol::Request(protocol::RequestSetDeviceTransform), "d1");
	queue.Enqueue(protocol::Request(protocol::RequestSetTrackingSystemFallback), "fA");
	queue.Enqueue(protocol::Request(protocol::RequestSetAlignmentSpeedParams), "c-align");

	ASSERT_TRUE(server.WaitForRecordedCount(3, std::chrono::milliseconds(5000)));
	const std::vector<int> types = server.RecordedTypes();
	ASSERT_EQ(types.size(), 3u);
	EXPECT_EQ(types[0], (int)protocol::RequestSetDeviceTransform);
	EXPECT_EQ(types[1], (int)protocol::RequestSetTrackingSystemFallback);
	EXPECT_EQ(types[2], (int)protocol::RequestSetAlignmentSpeedParams);

	EXPECT_TRUE(WaitForSentCount(queue, 3, std::chrono::milliseconds(3000)));
	EXPECT_TRUE(queue.GetStatus().connected);
	queue.Stop();
}

TEST(IpcSendQueue, CoalescesLatestWinsWhileWorkerIsBusy)
{
	// The stalled server pins the worker inside one send deadline, so
	// same-key enqueues pile up against the queued entry, not the wire.
	RecordingPipeServer server(RecordingPipeServer::Mode::StallAfterHandshake);
	ASSERT_TRUE(WaitForPipe(server.Name(), std::chrono::milliseconds(3000)));
	IpcSendQueue queue;
	queue.Start(server.Name(), ShortDeadline());

	for (int i = 0; i < 5; ++i) {
		queue.Enqueue(protocol::Request(protocol::RequestSetDeviceTransform), "d1");
	}

	const auto status = queue.GetStatus();
	EXPECT_GE(status.coalesced, 3u) << "burst of same-key entries must collapse to at most the in-flight one"
	                                   " plus one queued latest value";
	EXPECT_LE(status.queueDepth, 2u);
	queue.Stop();
}

TEST(IpcSendQueue, StopIsBoundedWithAStalledDriver)
{
	RecordingPipeServer server(RecordingPipeServer::Mode::StallAfterHandshake);
	ASSERT_TRUE(WaitForPipe(server.Name(), std::chrono::milliseconds(3000)));
	IpcSendQueue queue;
	queue.Start(server.Name(), ShortDeadline());
	for (int i = 0; i < 10; ++i) {
		queue.Enqueue(protocol::Request(protocol::RequestSetDeviceTransform), "d" + std::to_string(i));
	}

	const auto start = std::chrono::steady_clock::now();
	queue.Stop();
	const auto elapsed =
	    std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start);
	EXPECT_LT(elapsed.count(), 5000)
	    << "Stop must join after at most one in-flight deadline, discarding the rest of the queue";

	const auto status = queue.GetStatus();
	EXPECT_EQ(status.queueDepth, 0u);
}

TEST(IpcSendQueue, EnqueueWithoutStartDropsAndCounts)
{
	IpcSendQueue queue;
	queue.Enqueue(protocol::Request(protocol::RequestSetDeviceTransform), "d1");
	EXPECT_EQ(queue.GetStatus().dropped, 1u);
}
