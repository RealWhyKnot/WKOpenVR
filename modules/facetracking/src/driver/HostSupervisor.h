#pragma once

#include "HostSupervisorBase.h"

#include <cstdarg>
#include <mutex>
#include <string>

namespace facetracking {

// Spawns and supervises WKOpenVR.FaceModuleHost.exe.
//
// Lifecycle, job-object kill-on-driver-exit, exponential backoff, and the
// 5-strike circuit breaker live in HostSupervisorBase. This class plugs in
// the face-tracking pipe name, command-line arguments, and the SelectModule
// CBOR encoder for the active-module uuid.
class HostSupervisor : public openvr_pair::common::HostSupervisorBase
{
public:
	explicit HostSupervisor(const std::string& host_exe_path);

	// Send the active-module uuid to the host via the control pipe.
	// If the pipe isn't up the uuid is queued for the next reconnect.
	void SetActiveModuleUuid(const char* uuid);

protected:
	std::string ControlPipeName() const override;
	std::wstring SingletonMutexName() const override;
	void BuildCommandLine(std::wstring& commandLine, const std::wstring& exe_path) const override;
	void OnHostReady() override;
	void OnHostExited() override;
	void LogV(const char* fmt, va_list args) override;

private:
	bool TrySendUuid(const std::string& uuid);

	std::mutex uuid_mutex_;
	std::string pending_uuid_;
	bool has_pending_uuid_ = false;
	bool uuid_sent_ = false;
};

} // namespace facetracking
