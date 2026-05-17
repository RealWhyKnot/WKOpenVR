#pragma once

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

#include "DriverModule.h"
#include "Protocol.h"
#include "RouteTable.h"
#include "UdpSender.h"

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

namespace oscrouter {

// Bounded MPSC send-queue entry. Holds one serialised OSC packet.
static constexpr size_t kSendQueueEntryMaxSize = 512;
static constexpr size_t kSendQueueCapacity     = 1024;

struct SendEntry
{
    uint8_t data[kSendQueueEntryMaxSize];
    size_t  size = 0;
};

// OscRouter: subsystem entry point. Owns the RouteTable, UdpSender, send
// worker thread, and publish pipe server thread. Implements DriverModule so
// ServerTrackedDeviceProvider can manage its lifecycle uniformly.
class OscRouter : public DriverModule
{
public:
    OscRouter();
    ~OscRouter() override;

    // DriverModule interface.
    const char   *Name()        const override { return "OscRouter"; }
    uint32_t      FeatureMask() const override;
    const char   *PipeName()    const override;
    bool          Init(DriverModuleContext &ctx) override;
    void          Shutdown() override;
    bool          HandleRequest(const protocol::Request &req, protocol::Response &resp) override;

    // In-process publish from other driver modules (RouterPublishApi).
    // Thread-safe. Returns false if the queue is full or router is stopped.
    bool PublishOsc(const char *source_id,
                    const char *address,
                    const char *typetag,
                    const void *args, size_t arg_len);

    // Test harness hook: configure the UDP sink before Init() so fake VRChat
    // receivers can bind an isolated localhost port.
    void SetSendEndpointForTesting(const char *host, uint16_t port);

private:
    // Configuration (written at init / config push, read by send thread).
    std::string sendHost_  = "127.0.0.1";
    uint16_t    sendPort_  = 9000;
    bool        enabled_   = true;

    // Live port edit signal: when the overlay pushes a new send_port via
    // RequestSetOscRouterConfig, the IPC thread bumps this atomic and the
    // send-worker thread re-opens its socket on the next loop iteration. 0
    // (the default) means "no pending change". Keeps the UdpSender confined
    // to the send-worker thread per its threading contract.
    std::atomic<uint16_t> pendingSendPort_ {0};

    RouteTable routeTable_;
    UdpSender  sender_;

    // Bounded MPSC queue. Producers: IPC server thread + any driver thread
    // calling PublishOsc. Consumer: send worker thread only.
    std::mutex              queueMutex_;
    std::condition_variable queueCv_;
    SendEntry               queue_[kSendQueueCapacity];
    size_t                  qHead_ = 0;
    size_t                  qTail_ = 0;
    size_t                  qCount_ = 0;

    std::atomic<uint64_t>   droppedCount_   {0};
    std::atomic<uint64_t>   packetsSent_    {0};
    std::atomic<uint64_t>   bytesSent_      {0};

    std::atomic<bool>       stop_           {false};
    std::thread             sendWorker_;
    std::thread             pubPipeWorker_;

    // Stats shmem, written by the send worker at ~10 Hz.
    protocol::OscRouterStatsShmem statsShmem_;
    std::atomic<bool>             shmemReady_ {false};

    // Enqueue a raw OSC packet for sending. Called under queueMutex_.
    // Returns false on queue full.
    bool EnqueueLocked(const void *data, size_t size);

    // Serialize an OSC message from address+typetag+args and enqueue.
    bool BuildAndEnqueue(const char *address, const char *typetag,
                         const void *args, size_t arg_len);

    // Worker thread bodies.
    void SendWorkerMain();
    void PubPipeWorkerMain();

    // IPC handler helpers.
    bool HandleSubscribe(const protocol::OscRouteSubscribe &req, protocol::Response &resp);
    bool HandleUnsubscribe(const protocol::OscRouteUnsubscribe &req, protocol::Response &resp);
    bool HandlePublish(const protocol::OscPublish &req, protocol::Response &resp);
    bool HandleGetStats(protocol::Response &resp);
    bool HandleSetConfig(const protocol::OscRouterConfig &req, protocol::Response &resp);

    // Publish shmem stats (called from send worker ~10 Hz).
    void PublishStats();
};

// Singleton pointer registered at Init, cleared at Shutdown.
// Used by RouterPublishApi to reach the active OscRouter without a global
// link dep on the OscRouterDriverModule lib. Set before the module list
// that calls PublishOsc is initialized.
extern std::atomic<OscRouter*> g_activeRouter;

} // namespace oscrouter

namespace oscrouter {
std::unique_ptr<DriverModule> CreateDriverModule();
}
