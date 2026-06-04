#pragma once

#include "Protocol.h"

#include <atomic>
#include <cstdint>
#include <cstring>
#include <mutex>

namespace oscrouter {

// Maximum simultaneous routes. Matches OSC_ROUTER_ROUTE_SLOTS in Protocol.h.
static constexpr uint32_t kMaxRoutes = protocol::OSC_ROUTER_ROUTE_SLOTS;

struct RouteEntry
{
	protocol::OscSubscriberId subscriber_id;
	char pattern[protocol::OSC_ROUTE_ADDR_LEN];
	char subscriber_label[32]; // for shmem stats display
	std::atomic<uint64_t> match_count{0};
	std::atomic<uint64_t> drop_count{0};
	std::atomic<uint64_t> last_match_tick{0};
	bool active = false;
};

// Thread-safe flat route table. Written by the IPC server thread when the
// overlay subscribes or unsubscribes; read by the send worker on every
// published OSC message to determine which routes match and bump counters.
//
// Size is fixed at kMaxRoutes. A full table silently rejects new subscriptions
// and logs the failure -- the caller sees ResponseSuccess=false.
class RouteTable
{
public:
	RouteTable() = default;

	// Add or update a route. Returns false if the table is full and
	// the subscriber_id doesn't already exist (no update slot available).
	bool Subscribe(protocol::OscSubscriberId id, const char* pattern, const char* label);

	// Remove a route by subscriber id. No-op if not found.
	void Unsubscribe(protocol::OscSubscriberId id);

	// Called from the send worker thread on every published OSC address.
	// Iterates entries, tests the pattern, and bumps match_count.
	// `matched_out` is set to true if at least one route matched.
	void Dispatch(const char* address, uint64_t qpc_tick, bool& matched_out);

	// Bump drop_count on all routes that currently match `address`.
	// Called when the send queue is full and a matched message is dropped.
	void BumpDropCount(const char* address);

	// Snapshot: write all active routes into the shmem segment.
	void PublishToShmem(protocol::OscRouterStatsShmem& shmem) const;

	// Count of active routes.
	uint32_t ActiveCount() const;

private:
	mutable std::mutex mutex_;
	RouteEntry entries_[kMaxRoutes];

	// Find slot by subscriber_id (call with mutex held).
	int FindSlot(protocol::OscSubscriberId id) const;
	// Find first empty slot (call with mutex held).
	int FindFreeSlot() const;
};

} // namespace oscrouter
