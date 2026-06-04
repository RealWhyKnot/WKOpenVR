#include "RouteTable.h"
#include "OscWire.h"
#include "Logging.h"

#include <cstring>

namespace oscrouter {

int RouteTable::FindSlot(protocol::OscSubscriberId id) const
{
	for (int i = 0; i < static_cast<int>(kMaxRoutes); ++i) {
		if (entries_[i].active && entries_[i].subscriber_id == id) return i;
	}
	return -1;
}

int RouteTable::FindFreeSlot() const
{
	for (int i = 0; i < static_cast<int>(kMaxRoutes); ++i) {
		if (!entries_[i].active) return i;
	}
	return -1;
}

bool RouteTable::Subscribe(protocol::OscSubscriberId id, const char* pattern, const char* label)
{
	char loggedPattern[protocol::OSC_ROUTE_ADDR_LEN] = {};
	if (pattern) {
		size_t n = 0;
		for (; n < sizeof(loggedPattern) - 1 && pattern[n]; ++n)
			loggedPattern[n] = pattern[n];
		loggedPattern[n] = '\0';
	}
	std::lock_guard<std::mutex> lk(mutex_);
	int slot = FindSlot(id);
	const bool update = slot >= 0;
	if (slot < 0) {
		slot = FindFreeSlot();
		if (slot < 0) {
			OR_LOG("[OscRouter] route table full; rejected subscriber=%u pattern='%s'", (unsigned)id, loggedPattern);
			return false;
		}
	}
	RouteEntry& e = entries_[slot];
	e.subscriber_id = id;
	{
		size_t n = 0;
		for (; n < sizeof(e.pattern) - 1 && pattern[n]; ++n)
			e.pattern[n] = pattern[n];
		e.pattern[n] = '\0';
	}
	{
		size_t n = 0;
		for (; n < sizeof(e.subscriber_label) - 1 && label[n]; ++n)
			e.subscriber_label[n] = label[n];
		e.subscriber_label[n] = '\0';
	}
	e.match_count.store(0, std::memory_order_relaxed);
	e.drop_count.store(0, std::memory_order_relaxed);
	e.last_match_tick.store(0, std::memory_order_relaxed);
	e.active = true;
	OR_LOG("[OscRouter] route %s slot=%d subscriber=%u pattern='%s' label='%s'", update ? "updated" : "added", slot,
	       (unsigned)id, e.pattern, e.subscriber_label);
	return true;
}

void RouteTable::Unsubscribe(protocol::OscSubscriberId id)
{
	std::lock_guard<std::mutex> lk(mutex_);
	int slot = FindSlot(id);
	if (slot >= 0) {
		entries_[slot].active = false;
		OR_LOG("[OscRouter] route removed slot=%d subscriber=%u", slot, (unsigned)id);
	}
	else {
		OR_LOG("[OscRouter] route remove ignored; subscriber=%u not found", (unsigned)id);
	}
}

void RouteTable::Dispatch(const char* address, uint64_t qpc_tick, bool& matched_out)
{
	// No mutex here -- the send worker reads, and Subscribe/Unsubscribe write.
	// We tolerate a brief stale view on the active flag (set under mutex) since
	// entries_ is accessed word-at-a-time on x86 and the worst case is one
	// missed counter bump. The match_count atomics are relaxed-ordered.
	matched_out = false;
	for (uint32_t i = 0; i < kMaxRoutes; ++i) {
		if (!entries_[i].active) continue;
		if (OscPatternMatch(entries_[i].pattern, address)) {
			entries_[i].match_count.fetch_add(1, std::memory_order_relaxed);
			entries_[i].last_match_tick.store(qpc_tick, std::memory_order_relaxed);
			matched_out = true;
		}
	}
}

void RouteTable::BumpDropCount(const char* address)
{
	for (uint32_t i = 0; i < kMaxRoutes; ++i) {
		if (!entries_[i].active) continue;
		if (OscPatternMatch(entries_[i].pattern, address)) {
			entries_[i].drop_count.fetch_add(1, std::memory_order_relaxed);
		}
	}
}

void RouteTable::PublishToShmem(protocol::OscRouterStatsShmem& shmem) const
{
	// Snapshot under lock, then write outside -- avoid holding the lock
	// during shmem seqlock writes.
	struct Snap
	{
		char pattern[protocol::OSC_ROUTE_ADDR_LEN];
		char label[32];
		uint64_t match, drop, tick;
		bool active;
	} snaps[kMaxRoutes];

	{
		std::lock_guard<std::mutex> lk(mutex_);
		for (uint32_t i = 0; i < kMaxRoutes; ++i) {
			snaps[i].active = entries_[i].active;
			if (!snaps[i].active) continue;
			memcpy(snaps[i].pattern, entries_[i].pattern, sizeof(snaps[i].pattern));
			memcpy(snaps[i].label, entries_[i].subscriber_label, sizeof(snaps[i].label));
			snaps[i].match = entries_[i].match_count.load(std::memory_order_relaxed);
			snaps[i].drop = entries_[i].drop_count.load(std::memory_order_relaxed);
			snaps[i].tick = entries_[i].last_match_tick.load(std::memory_order_relaxed);
		}
	}

	for (uint32_t i = 0; i < kMaxRoutes; ++i) {
		shmem.WriteRoute(i, snaps[i].active ? snaps[i].pattern : "", snaps[i].active ? snaps[i].label : "",
		                 snaps[i].match, snaps[i].drop, snaps[i].tick, snaps[i].active);
	}
}

uint32_t RouteTable::ActiveCount() const
{
	std::lock_guard<std::mutex> lk(mutex_);
	uint32_t n = 0;
	for (uint32_t i = 0; i < kMaxRoutes; ++i)
		if (entries_[i].active) ++n;
	return n;
}

} // namespace oscrouter
