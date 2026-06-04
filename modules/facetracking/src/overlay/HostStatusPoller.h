#pragma once

// Polls the C# host's host_status.json sidecar so the overlay can display
// live state (active module, installed modules) without requiring a new
// IPC channel + Protocol.h version bump. The host writes the file
// atomically (.tmp + rename) once per second; we re-read at the same
// cadence on the overlay's tick thread.
//
// File location: %LocalAppDataLow%\WKOpenVR\facetracking\host_status.json
//
// All accessors are safe to call on the same thread that calls Tick();
// the parsed snapshot is updated in-place during Tick() and read by the
// UI draw functions immediately after.

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace facetracking {

struct HostStatusActiveModule
{
	std::string uuid;
	std::string name;
	std::string vendor;
	std::string version;
};

struct HostStatusInstalledModule
{
	std::string uuid;
	std::string name;
	std::string vendor;
	std::string version;
};

struct HostStatusSnapshot
{
	bool valid = false;
	bool stale = false;
	int host_pid = 0;
	std::string host_started_at;
	int host_uptime_seconds = 0;
	bool host_shutting_down = false;
	std::string phase;
	std::string last_error;
	int module_count = 0;
	std::string active_module_uuid;
	std::string active_module_name;
	uint64_t frames_written = 0;
	uint64_t frames_read = 0;
	uint64_t osc_messages_sent = 0;
	int last_exit_code = 0;
	std::string last_restart_time;
	std::optional<HostStatusActiveModule> active_module;
	std::vector<HostStatusInstalledModule> installed_modules;
};

class HostStatusPoller
{
public:
	HostStatusPoller();

	// Call once per overlay frame. Re-reads the file only when the on-disk
	// mtime advances, so the cost is amortised down to a stat() per tick on
	// steady state.
	void Tick();

	// Latest snapshot. The `valid` flag is true iff the file existed and
	// parsed successfully at least once; `stale` becomes true if the file
	// hasn't been refreshed in the last ~10 seconds (host crashed or wasn't
	// started).
	const HostStatusSnapshot& Snapshot() const noexcept { return snapshot_; }

	// For diagnostics: returns the path the poller is reading from.
	const std::string& PathUtf8() const noexcept { return path_utf8_; }

private:
	void ResolvePath();
	void ReadFile();

	std::string path_utf8_;
	std::chrono::steady_clock::time_point last_read_attempt_{};
	std::chrono::steady_clock::time_point last_successful_read_{};
	std::chrono::steady_clock::time_point last_stale_warn_{};
	int64_t last_observed_mtime_ = 0;
	HostStatusSnapshot snapshot_;
};

} // namespace facetracking
