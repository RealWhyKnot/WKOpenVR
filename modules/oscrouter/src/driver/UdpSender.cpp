#include "UdpSender.h"
#include "Logging.h"

#pragma comment(lib, "Ws2_32.lib")

namespace oscrouter {

bool UdpSender::wsaInit_ = false;

UdpSender::~UdpSender()
{
    Close();
}

bool UdpSender::Open(const char *host, uint16_t port)
{
    Close();

    if (!wsaInit_) {
        WSADATA wd;
        if (WSAStartup(MAKEWORD(2, 2), &wd) != 0) {
            OR_LOG("WSAStartup failed: %d", WSAGetLastError());
            return false;
        }
        wsaInit_ = true;
    }

    sock_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock_ == INVALID_SOCKET) {
        OR_LOG("UdpSender::Open socket() failed: %d", WSAGetLastError());
        return false;
    }

    ZeroMemory(&target_, sizeof(target_));
    target_.sin_family = AF_INET;
    target_.sin_port   = htons(port);
    if (inet_pton(AF_INET, host, &target_.sin_addr) != 1) {
        OR_LOG("UdpSender::Open inet_pton(%s) failed", host);
        Close();
        return false;
    }

    OR_LOG("UdpSender: sending to %s:%u", host, (unsigned)port);
    return true;
}

int UdpSender::Send(const void *data, size_t len)
{
    if (sock_ == INVALID_SOCKET) return -1;
    int sent = sendto(sock_,
        reinterpret_cast<const char*>(data),
        static_cast<int>(len),
        0,
        reinterpret_cast<const sockaddr*>(&target_),
        sizeof(target_));
    if (sent == SOCKET_ERROR) {
        const int err = WSAGetLastError();
        ++sendErrorCount_;
        if (sendErrorCount_ == 1 || (sendErrorCount_ % 1024) == 0) {
            OR_LOG("UdpSender::Send sendto failed: err=%d total=%llu",
                err,
                static_cast<unsigned long long>(sendErrorCount_));
        }
        return -1;
    }
    return sent;
}

void UdpSender::Close()
{
    if (sock_ != INVALID_SOCKET) {
        closesocket(sock_);
        sock_ = INVALID_SOCKET;
    }
    sendErrorCount_ = 0;
}

} // namespace oscrouter
