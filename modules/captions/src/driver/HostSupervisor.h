#pragma once

#include "HostSupervisorBase.h"

#include <cstdarg>
#include <mutex>
#include <string>

namespace captions {

// Spawns and supervises WKOpenVR.CaptionsHost.exe.
//
// Lifecycle, job-object kill-on-driver-exit, exponential backoff, and the
// 5-strike circuit breaker live in HostSupervisorBase. This class plugs in
// the captions pipe name, an exit-code description table for the loader
// failures most captions hosts hit (missing CUDA runtime, etc.), and the
// raw-bytes "host config" message the driver pushes after spawn.
class HostSupervisor : public openvr_pair::common::HostSupervisorBase
{
public:
	explicit HostSupervisor(const std::string& host_exe_path);

	// Queue and send the current host config blob. If the control pipe is
	// not available yet, retry after the next spawn/reconnect.
	void SetHostConfigCommand(const std::string& command);

	// Ask a responsive host to exit cleanly. Stop() still handles tracked
	// spawned processes; this covers connect-first attachments.
	void RequestHostShutdown();

protected:
	std::string ControlPipeName() const override;
	std::wstring SingletonMutexName() const override;
	void OnHostReady() override;
	void OnHostExited() override;
	std::string DescribeExitCode(DWORD code) const override;
	void LogV(const char* fmt, va_list args) override;

private:
	bool TrySendCommand(const std::string& command);

	std::mutex command_mutex_;
	std::string pending_command_;
	bool has_pending_command_ = false;
	bool command_sent_ = false;
};

} // namespace captions
