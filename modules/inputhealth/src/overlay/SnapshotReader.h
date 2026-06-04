#pragma once

#include "Protocol.h"

#include <chrono>
#include <string>
#include <unordered_map>
#include <vector>

// Reader for OPENVR_PAIRDRIVER_INPUTHEALTH_SHMEM_NAME. Wraps the underlying
// InputHealthSnapshotShmem class with caching: each Refresh() call walks all
// non-empty slots, validates each via the per-slot seqlock, and stores the
// resulting body in `entries_by_handle`. The UI reads from that map without
// any further synchronization.
//
// Open() must succeed before Refresh(); a failed Open() leaves the reader in
// a state where Refresh() is a no-op so the UI can render a "waiting for
// driver" placeholder gracefully.

class SnapshotReader
{
public:
	struct Entry
	{
		protocol::InputHealthSnapshotBody body;

		// Wall-clock millis since the publisher's BumpPublishTick last
		// changed for this entry. Used by the UI to show stale-data hints
		// when a controller is asleep.
		uint64_t last_seen_publish_tick = 0;
	};

	// Try to open the shmem segment. Returns true on success. On failure,
	// stores the error message in `LastError()` and leaves the reader idle.
	bool TryOpen();

	// Walk every slot, copy out fresh entries, drop entries whose handle
	// went to zero (driver Shutdown) or whose generation has not advanced
	// for more than `stale_ticks` publish ticks.
	void Refresh();
	void Close();

	bool IsOpen() const { return shmem_; }

	// Most recent error from TryOpen() / Refresh(); empty when healthy.
	const std::string& LastError() const { return last_error_; }

	// True if the last TryOpen() failure was a magic/version mismatch (driver
	// and overlay are from different builds). False means "not ready yet" or
	// some other transient error.
	bool LastErrorIsVersionMismatch() const { return last_error_is_version_mismatch_; }

	// Most recent publish tick observed at Refresh() time.
	uint64_t LastPublishTick() const { return last_publish_tick_; }

	// Snapshot of the live entries, keyed by component handle. Order
	// is unspecified; callers needing a stable display order should sort
	// after copying out (typically by device serial hash + path).
	const std::unordered_map<uint64_t, Entry>& EntriesByHandle() const { return entries_by_handle_; }

private:
	protocol::InputHealthSnapshotShmem shmem_;
	std::unordered_map<uint64_t, Entry> entries_by_handle_;
	std::string last_error_;
	bool last_error_is_version_mismatch_ = false;
	uint64_t last_publish_tick_ = 0;
	std::chrono::steady_clock::time_point last_publish_tick_change_{};
};
