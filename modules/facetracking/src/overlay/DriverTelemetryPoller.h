#pragma once

// Polls the driver's driver_telemetry.json sidecar so the overlay can display
// live host/driver status and vergence readout without requiring a new IPC
// channel. The driver writes the file atomically (.tmp + rename) every ~500 ms
// from the worker thread; we re-read at the same cadence on the overlay tick.
//
// File location: %LocalAppDataLow%\WKOpenVR\facetracking\driver_telemetry.json
//
// All accessors are safe to call on the same thread that calls Tick(); the
// parsed snapshot is updated in-place during Tick() and read by the UI draw
// functions immediately after.

#include <chrono>
#include <cstdint>
#include <string>

namespace facetracking {

struct DriverTelemetrySnapshot
{
	bool valid = false; // file existed and parsed at least once
	bool stale = false; // file mtime > 5 s old

	int driver_pid = 0;
	uint64_t frames_processed = 0;
	uint64_t frames_read = 0;
	uint64_t osc_messages_sent = 0;
	uint64_t osc_messages_dropped = 0;
	std::string active_module_uuid;

	// Vergence lock readout.
	bool vergence_enabled = false;
	float focus_distance_m = 0.f;
	float ipd_m = 0.f;
};

class DriverTelemetryPoller
{
public:
	DriverTelemetryPoller();

	// Call once per overlay frame. Re-reads the file only when the on-disk
	// mtime advances; cost is a stat() per tick in steady state.
	void Tick();

	const DriverTelemetrySnapshot& Snapshot() const noexcept { return snapshot_; }

	const std::string& PathUtf8() const noexcept { return path_utf8_; }

private:
	void ResolvePath();
	void ReadFile();

	std::string path_utf8_;
	std::chrono::steady_clock::time_point last_read_attempt_{};
	std::chrono::steady_clock::time_point last_successful_read_{};
	int64_t last_observed_mtime_ = 0;
	DriverTelemetrySnapshot snapshot_;
};

} // namespace facetracking
