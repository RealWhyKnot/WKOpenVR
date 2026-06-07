#pragma once

#include <chrono>
#include <cstdint>
#include <string>

namespace captions {

struct HostStatusSnapshot
{
	bool valid = false;
	bool stale = false;
	bool host_halted = false; // circuit breaker tripped in the driver supervisor
	uint32_t last_exit_code = 0;
	std::string last_exit_description;
	int host_pid = 0;
	int state = 0; // HostStatus::State int value
	std::string phase;
	std::string mic_name;
	std::string last_transcript;
	std::string last_translation;
	std::string last_error;
	bool ptt_available = false;
	bool ptt_registered = false;
	std::string ptt_app_key;
	std::string ptt_error;
	bool speech_pack_installed = false;
	bool vad_runtime_available = false;
	bool translation_runtime_available = false;
	bool translation_pack_installed = false;
	std::string active_translation_pair;
	long long captions_completed = 0;
	long long packets_sent = 0;
};

// Polls %LocalAppDataLow%\WKOpenVR\captions\host_status.json.
// Throttled to a stat() every 500 ms; only re-reads on mtime change.
class HostStatusPoller
{
public:
	HostStatusPoller();

	void Tick();

	const HostStatusSnapshot& Snapshot() const noexcept { return snapshot_; }
	const std::string& PathUtf8() const noexcept { return path_utf8_; }

	// Set host_halted on the snapshot directly (driven by the driver IPC query).
	void SetSupervisorStatus(bool halted, uint32_t last_exit_code, const std::string& last_exit_description)
	{
		snapshot_.host_halted = halted;
		snapshot_.last_exit_code = last_exit_code;
		snapshot_.last_exit_description = last_exit_description;
	}

private:
	void ResolvePath();
	void ReadFile();

	std::string path_utf8_;
	std::chrono::steady_clock::time_point last_read_attempt_{};
	std::chrono::steady_clock::time_point last_successful_read_{};
	int64_t last_observed_mtime_ = 0;
	HostStatusSnapshot snapshot_;
};

} // namespace captions
