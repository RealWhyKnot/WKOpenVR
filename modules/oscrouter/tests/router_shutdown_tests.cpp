#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include <gtest/gtest.h>

#include "OscRouter.h"
#include "ProtocolNames.h"

#include <algorithm>
#include <atomic>
#include <cstring>
#include <thread>

namespace {

struct TestHandle
{
	HANDLE value = INVALID_HANDLE_VALUE;

	TestHandle() = default;
	explicit TestHandle(HANDLE handle) : value(handle) {}
	TestHandle(const TestHandle&) = delete;
	TestHandle& operator=(const TestHandle&) = delete;
	TestHandle(TestHandle&& other) noexcept : value(other.value) { other.value = INVALID_HANDLE_VALUE; }
	TestHandle& operator=(TestHandle&& other) noexcept
	{
		if (this != &other) {
			reset(other.value);
			other.value = INVALID_HANDLE_VALUE;
		}
		return *this;
	}

	~TestHandle()
	{
		if (value != INVALID_HANDLE_VALUE) CloseHandle(value);
	}

	HANDLE get() const { return value; }

	void reset(HANDLE next = INVALID_HANDLE_VALUE)
	{
		if (value != INVALID_HANDLE_VALUE) CloseHandle(value);
		value = next;
	}
};

TestHandle OpenPubPipeForWrite(DWORD timeoutMs)
{
	const ULONGLONG deadline = GetTickCount64() + timeoutMs;
	for (;;) {
		HANDLE pipe = CreateFileA(OPENVR_PAIRDRIVER_OSCROUTER_PUB_PIPE_NAME, GENERIC_WRITE, 0, nullptr, OPEN_EXISTING,
		                          FILE_ATTRIBUTE_NORMAL, nullptr);
		if (pipe != INVALID_HANDLE_VALUE) return TestHandle(pipe);

		const DWORD err = GetLastError();
		if (err == ERROR_PIPE_BUSY) {
			WaitNamedPipeA(OPENVR_PAIRDRIVER_OSCROUTER_PUB_PIPE_NAME, 50);
		}
		else {
			Sleep(25);
		}

		if (GetTickCount64() >= deadline) return {};
	}
}

bool WriteSourceId(HANDLE pipe, const char* source)
{
	char sourceId[32] = {};
	if (source) {
		memcpy(sourceId, source, std::min<size_t>(strlen(source), sizeof(sourceId)));
	}
	DWORD written = 0;
	return WriteFile(pipe, sourceId, sizeof(sourceId), &written, nullptr) && written == sizeof(sourceId);
}

} // namespace

TEST(OscRouterShutdown, ShutdownCancelsIdleConnectedPubPipeRead)
{
	oscrouter::OscRouter router;
	DriverModuleContext ctx{};
	ASSERT_TRUE(router.Init(ctx));

	TestHandle pipe = OpenPubPipeForWrite(5000);
	ASSERT_NE(pipe.get(), INVALID_HANDLE_VALUE);
	ASSERT_TRUE(WriteSourceId(pipe.get(), "shutdown-test"));

	Sleep(100);

	std::atomic<bool> shutdownDone{false};
	const ULONGLONG started = GetTickCount64();
	std::thread shutdownThread([&] {
		router.Shutdown();
		shutdownDone.store(true, std::memory_order_release);
	});

	for (int i = 0; i < 40 && !shutdownDone.load(std::memory_order_acquire); ++i) {
		Sleep(50);
	}

	if (!shutdownDone.load(std::memory_order_acquire)) {
		pipe.reset();
		shutdownThread.join();
		FAIL() << "OscRouter::Shutdown did not return while a pub-pipe client was idle";
	}

	shutdownThread.join();
	const ULONGLONG elapsedMs = GetTickCount64() - started;
	EXPECT_LT(elapsedMs, 2000ULL);
}
