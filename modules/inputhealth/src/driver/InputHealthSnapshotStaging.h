#pragma once

#include "Protocol.h"

#include <cstdint>
#include <vector>

namespace inputhealth {

// Per-snapshot record + handle pair emitted to the snapshot publisher.
struct StagedSnapshot
{
	uint64_t handle;
	protocol::InputHealthSnapshotBody body;
};

// Apply an InputHealthResetStats request to the per-component map. Resolves
// each entry's owning device serial via vr::VRProperties() lazily (cached
// per-handle for the lifetime of the entry) and clears the stats categories
// the request asked for on every entry whose serial hash matches. A request
// with device_serial_hash == kSerialHashAllDevices targets every entry.
//
// Runs on the IPC dispatcher thread. Safe to call concurrently with the
// per-tick detour bodies; both paths take g_componentMutex.
void ApplyResetRequest(const protocol::InputHealthResetStats& req);

// Build wire-format snapshot bodies for every component currently registered
// in g_componentStats and append them to `out`. Holds g_componentMutex for
// the duration. The publisher calls this once per ~10 Hz tick and then
// writes the staged bodies to shmem under the per-slot seqlock without
// holding the mutex.
void StageSnapshots(std::vector<StagedSnapshot>& out);

} // namespace inputhealth
