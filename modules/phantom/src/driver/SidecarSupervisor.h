#pragma once

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

#include <atomic>
#include <string>
#include <thread>

namespace phantom {

// Supervises the out-of-process phantom inference sidecar
// (WKOpenVRPhantomSidecar.exe). Spawns on Start(), monitors the child
// process from a worker thread, restarts with exponential backoff on
// exit (1 s -> 2 s -> 4 s -> 8 s -> 16 s -> 32 s -> 60 s, capped) up
// to a five-attempts-in-ten-minutes circuit breaker that matches the
// translator / face-tracking host pattern.
//
// The supervisor does not own the shmem segments; PhantomDriverModule
// creates the IN segment and the sidecar creates the OUT segment.
// Spawn timing: Start() returns before the child finishes booting, so
// the bridge tolerates a missing OUT segment for the first second or
// so after Start().
class SidecarSupervisor
{
public:
	SidecarSupervisor() = default;
	~SidecarSupervisor() { Stop(); }
	SidecarSupervisor(const SidecarSupervisor&) = delete;
	SidecarSupervisor& operator=(const SidecarSupervisor&) = delete;

	// Resolves the sidecar exe path relative to the loaded driver DLL,
	// matching the face-tracking + translator host walk-up pattern. The
	// walk-up is THREE segments (filename, win64, bin) to reach the
	// driver root, then we append "\resources\phantom\host\<exe>".
	// Returns false if the driver DLL path cannot be resolved.
	bool Start();
	void Stop();

	bool Running() const { return running_.load(std::memory_order_acquire); }

	// Last observed child exit code; 0 if the supervisor has never seen
	// an exit. Surfaced to the overlay's Diagnostics tab in a follow-up
	// pass; for Phase 3 scaffold it is read-only.
	uint32_t last_exit_code() const { return last_exit_code_.load(std::memory_order_acquire); }

	// Whether the circuit breaker has tripped (5 fast exits in 10 min).
	// When true, the supervisor stops trying to respawn until Stop +
	// Start cycles the state.
	bool halted() const { return halted_.load(std::memory_order_acquire); }

private:
	void SuperviseLoop();
	std::wstring ResolveSidecarPath() const;

	std::atomic<bool> running_{false};
	std::atomic<bool> stop_requested_{false};
	std::atomic<bool> halted_{false};
	std::atomic<uint32_t> last_exit_code_{0};
	std::thread worker_;
	HANDLE child_process_ = nullptr;
};

} // namespace phantom
