#pragma once

#include "Protocol.h"

// Reads per-route stats from the OscRouter stats shmem segment at ~10 Hz.
// Opened lazily on first Tick(); closed on destruction or Close().

class OscRouterStatsReader
{
public:
	OscRouterStatsReader() = default;
	~OscRouterStatsReader() { Close(); }

	// Attempt to open the shmem segment. Returns false if the driver has not
	// created it yet (oscrouter not enabled or driver not running).
	bool TryOpen();

	void Close();

	bool IsOpen() const { return shmem_; }

	// Snapshot global stats. Returns false if not open.
	bool ReadGlobal(protocol::OscRouterStats& out) const;

	// Read one route slot. Returns false if the slot is inactive or unreadable.
	bool ReadRoute(uint32_t index, protocol::OscRouterRouteSlot& out) const;

	static uint32_t RouteSlotCount() { return protocol::OSC_ROUTER_ROUTE_SLOTS; }

private:
	protocol::OscRouterStatsShmem shmem_;
};
