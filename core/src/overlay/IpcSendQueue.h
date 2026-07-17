#pragma once

#include "IpcClientBase.h"
#include "Protocol.h"

#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>

namespace openvr_pair::overlay {

// Off-thread sender for fire-and-forget IPC requests.
//
// The overlay's render thread must never wait on the driver for recurring
// bulk traffic (device-transform republish runs several times per second and
// measured 16-20 ms of pipe time per apply cycle). Callers enqueue; a single
// worker thread owns its own pipe connection outright and drains the queue.
//
// Concurrency contract:
//  - single-owner: only the worker touches its pipe handle; callers only
//    touch the queue under its mutex. No lock is ever held across pipe IO.
//  - bounded everywhere: each pipe operation carries the client's
//    per-operation deadline, failed connects arm the client's backoff, the
//    queue has a hard cap, and Stop() joins after at most one in-flight
//    operation (the remaining queue is discarded -- entries are periodic
//    republish traffic that the next session re-sends anyway).
//  - ordering: one FIFO worker preserves enqueue order end to end.
//
// Entries enqueued with a non-empty key coalesce latest-wins against a
// queued entry with the same key: for same-device transforms the newest
// value supersedes older ones, so a slow driver bounds the backlog instead
// of growing it.
class IpcSendQueue
{
public:
	struct Status
	{
		bool connected = false;
		uint32_t queueDepth = 0;
		uint64_t sent = 0;
		uint64_t sendFailures = 0;
		uint64_t coalesced = 0;
		uint64_t dropped = 0; // enqueue attempts refused by the full-queue cap
		// The worker client's connection generation. A change means the pipe
		// reconnected: anything the peer learned from earlier sends may be
		// gone, so callers keeping sent-value dedupe caches must invalidate
		// them (sendFailures moving is the same signal for dropped entries).
		uint64_t connectionGeneration = 0;
	};

	IpcSendQueue() = default;
	~IpcSendQueue();

	IpcSendQueue(const IpcSendQueue&) = delete;
	IpcSendQueue& operator=(const IpcSendQueue&) = delete;

	void Start(std::string pipeName, IpcClientConnectOptions clientOptions = {});
	void Stop();
	bool IsRunning() const;

	// Fire-and-forget. A non-empty key coalesces latest-wins with a queued
	// entry carrying the same key. Silently drops (and counts) when the
	// queue is full or the worker is not running.
	void Enqueue(const protocol::Request& request, std::string coalesceKey = {});

	Status GetStatus() const;

private:
	void Run();

	struct Entry
	{
		protocol::Request request;
		std::string key;
	};

	static constexpr size_t kMaxQueue = 256;

	mutable std::mutex mutex_;
	std::condition_variable cv_;
	std::deque<Entry> queue_;
	bool stop_ = false;
	bool started_ = false;
	std::thread worker_;
	std::string pipeName_;
	IpcClientConnectOptions clientOptions_;
	Status status_;
};

} // namespace openvr_pair::overlay
