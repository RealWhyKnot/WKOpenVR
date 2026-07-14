#pragma once

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

#include "ModuleRegistry.h"

#include <atomic>
#include <cstdarg>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "SidecarOwnerLease.h"

namespace openvr_pair::common {

// Pure decision for the dev-only sidecar hot-reload watch: should the supervisor
// kill and respawn its host to pick up a rebuilt binary? True only when the host
// is one we spawned ourselves (not attached to a prior-session process), we have
// a spawn-time baseline, and the on-disk last-write time now differs. Extracted
// as a free function so it can be unit-tested without a live process.
inline bool ShouldHotReloadHost(bool attachedToExisting, bool haveHandle, uint64_t baselineWrite, uint64_t diskWrite)
{
	if (attachedToExisting || !haveHandle) return false;
	if (baselineWrite == 0 || diskWrite == 0) return false;
	return diskWrite != baselineWrite;
}

// Pure companion for the crash-loop halt state: a supervisor halted by fast
// exits has no live process handle, but a rebuilt exe on disk (differing from
// the spawn-time baseline of the build that crash-looped) should clear the
// halt so the fixed build can take over without a SteamVR restart.
inline bool ShouldUnhaltForNewHostExe(bool attachedToExisting, uint64_t baselineWrite, uint64_t diskWrite)
{
	if (attachedToExisting) return false;
	if (baselineWrite == 0 || diskWrite == 0) return false;
	return diskWrite != baselineWrite;
}

// Respawn tuning shared by the supervisor and the exit-decision helper below.
inline constexpr int kFastExitThresholdMs = 2000;
inline constexpr int kCircuitBreakerThreshold = 5;
inline constexpr int kBackoffStartMs = 1000;
inline constexpr int kBackoffMaxMs = 30000;
inline constexpr int kSingletonRetryDelayMs = 500;

// Exit codes 3/4 mean the host found another live instance holding the
// per-user singleton mutex and exited on purpose -- a clean duplicate exit,
// not a host failure.
inline bool IsCleanSingletonExit(DWORD code)
{
	return code == 3 || code == 4;
}

// What the monitor should do after the tracked host handle signals. Pure so
// the decision can be unit-tested without a live process; the supervisor
// supplies generationMatches from its spawn-generation counter.
struct HostExitAction
{
	// The process the monitor waited on was already replaced or reaped by a
	// concurrent Restart()/Kill(); the tracked handle may belong to a live
	// replacement host. Touch nothing, count nothing, keep monitoring.
	bool ignore = false;
	// Fast-exit counter value after accounting for this exit.
	int consecutiveFastExits = 0;
	// Circuit breaker tripped: halt respawns.
	bool halt = false;
	// Delay before the next spawn attempt. Unused when ignore/halt is set.
	int respawnDelayMs = 0;
};

inline HostExitAction DecideHostExitAction(bool generationMatches, DWORD exitCode, long long uptimeMs,
                                           int consecutiveFastExitsBefore, int backoffMs)
{
	HostExitAction action;
	action.consecutiveFastExits = consecutiveFastExitsBefore;
	if (!generationMatches) {
		action.ignore = true;
		return action;
	}
	if (IsCleanSingletonExit(exitCode)) {
		// Another instance holds the singleton mutex. Respawning immediately
		// just burns process spawns while that instance finishes starting up
		// (its control pipe can trail the mutex by seconds); pace the retry.
		action.respawnDelayMs = kSingletonRetryDelayMs;
		return action;
	}
	action.consecutiveFastExits = (uptimeMs < kFastExitThresholdMs) ? consecutiveFastExitsBefore + 1 : 0;
	if (action.consecutiveFastExits >= kCircuitBreakerThreshold) {
		action.halt = true;
		return action;
	}
	action.respawnDelayMs = backoffMs;
	return action;
}

// Shared base for the C++ supervisors that spawn and manage feature-host
// sidecar processes -- currently the facetracking C# host and the captions
// C++ host.
//
// The base owns:
//   - Win32 CreateProcessW spawn under CREATE_NO_WINDOW.
//   - Job object with JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE so the child is
//     reaped automatically if the driver exits abnormally.
//   - Monitor thread, exponential backoff (1/2/4/8/16/30 s cap), fast-exit
//     counter, circuit breaker at kCircuitBreakerThreshold consecutive
//     fast exits.
//   - Connect-first attach: if a host from a prior session is already
//     responsive on the control pipe, the supervisor attaches without
//     spawning a duplicate.
//
// Subclasses plug in their feature-specific bits:
//   - ControlPipeName()    -- which named pipe the host listens on.
//   - BuildCommandLine()   -- extra arguments after argv[0].
//   - OnHostReady()        -- flush queued control-pipe messages.
//   - OnHostExited()       -- reset queued-message sent flags.
//   - LogV()               -- route printf-style log lines.
//   - DescribeExitCode()   -- map exit codes to human descriptions.
class HostSupervisorBase
{
public:
	explicit HostSupervisorBase(std::string host_exe_path);
	virtual ~HostSupervisorBase();

	HostSupervisorBase(const HostSupervisorBase&) = delete;
	HostSupervisorBase& operator=(const HostSupervisorBase&) = delete;

	bool Start();
	void Stop();
	void Restart();
	bool IsRunning() const;
	bool IsHalted() const;

	// 0 if no exit has been recorded yet.
	uint32_t LastExitCode() const;
	std::string LastExitDescription() const;

	// If a stale host from a previous SteamVR session is still holding the
	// singleton mutex but its control pipe has gone unresponsive, terminate
	// any process matching the host's image name. Call this from the driver
	// module's Init() BEFORE Start() so the supervisor's connect-first
	// attach path does not race against a wedged host.
	//
	// Returns the number of stale processes terminated (zero if the host
	// image isn't running, or if the live host's pipe answered the probe).
	int CleanupStaleHostIfWedged();

protected:
	virtual std::string ControlPipeName() const = 0;

	// Per-user singleton mutex name (e.g. Global\WKOpenVR-FaceModuleHost-
	// Singleton-<sid-or-username>). Empty means "host has no singleton
	// mutex", and the supervisor falls back to pipe-only attach detection.
	virtual std::wstring SingletonMutexName() const { return {}; }

	// Default appends nothing. Subclasses override to add their own flags.
	virtual void BuildCommandLine(std::wstring& commandLine, const std::wstring& exe_path) const;

	// Called whenever the host transitions to a reachable state (post-spawn
	// and once per monitor tick while alive/attached). Subclass flushes any
	// queued control-pipe message here.
	virtual void OnHostReady() {}

	// Called once each time the host process exits. Subclass resets any
	// "sent" flags so the queued message is retried against the next host.
	virtual void OnHostExited() {}

	// Called before forceful termination when the driver intentionally stops
	// this host. Subclasses with a control-pipe shutdown command override it.
	virtual void RequestGracefulShutdown() {}

	// Default returns empty. Captions overrides to provide a table of
	// known exit codes.
	virtual std::string DescribeExitCode(DWORD code) const
	{
		(void)code;
		return {};
	}

	// Module that owns this host, for per-module perf attribution. When set,
	// the supervisor registers the spawned child process plus its own monitor
	// thread with the perf registry. Attach-to-existing hosts hold no process
	// handle, so they are not attributed until the next respawn.
	virtual std::optional<modules::ModuleId> PerfModuleId() const { return std::nullopt; }

	// Module that owns this sidecar for the owner-liveness lease. Defaults to
	// the perf owner because every HostSupervisorBase instance is a feature
	// host and the current concrete supervisors already declare their module.
	virtual std::optional<modules::ModuleId> SidecarOwnerModuleId() const { return PerfModuleId(); }

	// Route a printf-style log line to the subclass's logger. Pure: there is
	// no shared driver log file, each feature has its own (FT_LOG_DRV /
	// TR_LOG_DRV) routed to a feature-specific file in %LocalAppDataLow%.
	virtual void LogV(const char* fmt, va_list args) = 0;

	// Convenience.
	void Log(const char* fmt, ...);

	// Helpers for subclasses ------------------------------------------------

	// Open the control pipe for write and push `data`. Returns true on a
	// successful WriteFile of the full payload. The pipe is closed before
	// return. Used by subclass control-pipe senders.
	bool SendBytesOverControlPipe(const void* data, size_t len);

	// True if the host's control pipe is responsive within timeout_ms.
	bool CanConnectToHost(int timeout_ms) const;

	// True if the per-user singleton mutex named by SingletonMutexName()
	// exists. Always returns false when SingletonMutexName() is empty.
	bool IsSingletonMutexHeld() const;

	bool IsStopRequested() const { return stop_requested_.load(std::memory_order_acquire); }

private:
	bool Spawn();
	void Kill();
	void MonitorLoop();

	// Last-write time (FILETIME as a 64-bit value) of host_exe_path_ as it was
	// when the currently tracked process was spawned. 0 when unknown/attached.
	// Drives the dev-only hot-reload watch; see ShouldHotReloadHost.
	uint64_t QueryExeWriteTime() const;
	bool EnsureOwnerLease();
	void HeartbeatOwnerLease(sidecar_owner::LeaseState state = sidecar_owner::LeaseState::Alive);
	void MarkOwnerLeaseShuttingDown();
	void MarkOwnerLeaseDisabled();
	void AppendOwnerLivenessArgs(std::wstring& commandLine) const;

	std::string host_exe_path_;
	std::atomic<bool> stop_requested_{false};
	std::atomic<bool> running_{false};

	// process_handle_ is read/written by both MonitorLoop and Restart/Kill;
	// all accesses must hold process_mutex_.
	mutable std::mutex process_mutex_;
	HANDLE process_handle_ = INVALID_HANDLE_VALUE;
	// Bumped whenever process_handle_ changes (spawn, reap, kill) so the
	// monitor can tell whether the handle it waited on is still the tracked
	// one; a raw handle-value compare could alias after handle reuse.
	uint64_t spawn_generation_ = 0; // process_mutex_
	HANDLE job_handle_ = nullptr;
	bool attached_to_existing_ = false;
	int consecutive_fast_exits_ = 0;
	bool halted_ = false;
	uint64_t watched_exe_write_ = 0; // exe mtime at last spawn; process_mutex_
	uint32_t last_exit_code_ = 0;
	std::string last_exit_description_;

	std::thread monitor_thread_;
	mutable std::mutex owner_lease_mutex_;
	std::unique_ptr<sidecar_owner::LeaseOwner> owner_lease_;
};

} // namespace openvr_pair::common
