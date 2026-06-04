#pragma once

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>

#include <cstdint>
#include <cstddef>
#include <string>

namespace oscrouter {

// Send-only UDP socket targeting one fixed endpoint.
// Not thread-safe -- call only from the send worker thread.
class UdpSender
{
public:
	UdpSender() = default;
	~UdpSender();

	// Open a UDP socket and resolve the target. Returns false on failure.
	bool Open(const char* host, uint16_t port);

	// Send `len` bytes from `data`. Returns bytes sent or -1 on error.
	int Send(const void* data, size_t len);

	// Close the socket. Safe to call multiple times.
	void Close();

	bool IsOpen() const { return sock_ != INVALID_SOCKET; }

private:
	SOCKET sock_ = INVALID_SOCKET;
	sockaddr_in target_{};
	uint64_t sendErrorCount_ = 0;
	static bool wsaInit_;
};

} // namespace oscrouter
