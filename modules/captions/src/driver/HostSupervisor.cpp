#define _CRT_SECURE_NO_DEPRECATE
#include "HostSupervisor.h"
#include "Logging.h"

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

#include <cstdarg>

#define TR_HOST_CONTROL_PIPE_NAME "\\\\.\\pipe\\WKOpenVR-Captions.host"

namespace captions {
namespace {

// Matches the username discriminator the captions host writes in
// modules/captions/src/host/main.cpp (GetUserNameW result). Empty
// on failure so the supervisor falls back to pipe-only attach.
std::wstring GetCurrentUserNameString()
{
	wchar_t user[256] = {};
	DWORD len = 256;
	if (!GetUserNameW(user, &len) || len == 0) return {};
	return std::wstring(user);
}

} // namespace

HostSupervisor::HostSupervisor(const std::string& host_exe_path) : HostSupervisorBase(host_exe_path) {}

std::string HostSupervisor::ControlPipeName() const
{
	return TR_HOST_CONTROL_PIPE_NAME;
}

std::wstring HostSupervisor::SingletonMutexName() const
{
	std::wstring user = GetCurrentUserNameString();
	if (user.empty()) return {};
	return L"Global\\WKOpenVR-CaptionsHost-Singleton-" + user;
}

void HostSupervisor::LogV(const char* fmt, va_list args)
{
	TrDrvLogV(fmt, args);
}

std::string HostSupervisor::DescribeExitCode(DWORD code) const
{
	// Windows loader failures.
	if (code == 0xC0000135)
		return "STATUS_DLL_NOT_FOUND -- a hard-import DLL is missing; "
		       "check openvr_api.dll and VC runtime staging beside "
		       "WKOpenVR.CaptionsHost.exe";
	if (code == 0xC0000005)
		return "STATUS_ACCESS_VIOLATION -- crash inside native lib; "
		       "see captions_host_crash_<pid>.txt in "
		       "%LocalAppDataLow%\\WKOpenVR\\Logs";
	if (code == 0xC000007B) return "STATUS_INVALID_IMAGE_FORMAT -- 32/64-bit mismatch";
	if (code == 0xC0000142)
		return "STATUS_DLL_INIT_FAILED -- DLL DllMain returned FALSE; a "
		       "required dependency of a loaded DLL is itself missing";

	// Our delay-load failure range: 0xCEE0DC00 | (err & 0xFF).
	if ((code & 0xFFFFFF00u) == 0xCEE0DC00u) {
		DWORD winerr = code & 0xFF;
		if (winerr == 126) {
			return "delay-load failed: ERROR_MOD_NOT_FOUND (126) -- "
			       "optional captions runtime DLL missing; install the "
			       "relevant Captions pack; see captions_host_crash_<pid>.txt "
			       "for the exact DLL name";
		}
		return "delay-load failed -- optional captions runtime DLL missing; "
		       "see captions_host_crash_<pid>.txt for details";
	}

	// Clean exits from the host's singleton check.
	if (code == 3) return "clean singleton exit (another host already running)";
	if (code == 4) return "clean pipe-busy exit (another host owns the control pipe)";

	if (code == 0) return "normal exit (code 0)";

	return "unknown";
}

void HostSupervisor::SetHostConfigCommand(const std::string& command)
{
	{
		std::lock_guard<std::mutex> lk(command_mutex_);
		pending_command_ = command;
		has_pending_command_ = !command.empty();
		command_sent_ = false;
	}
	if (!command.empty()) TrySendCommand(command);
}

void HostSupervisor::OnHostReady()
{
	std::string command;
	bool should_send = false;
	{
		std::lock_guard<std::mutex> lk(command_mutex_);
		if (has_pending_command_ && !command_sent_) {
			command = pending_command_;
			should_send = true;
		}
	}
	if (should_send) TrySendCommand(command);
}

void HostSupervisor::OnHostExited()
{
	std::lock_guard<std::mutex> lk(command_mutex_);
	if (has_pending_command_) command_sent_ = false;
}

bool HostSupervisor::TrySendCommand(const std::string& command)
{
	if (!SendBytesOverControlPipe(command.data(), command.size())) {
		return false;
	}
	{
		std::lock_guard<std::mutex> lk(command_mutex_);
		if (pending_command_ == command) command_sent_ = true;
	}
	Log("[host] sent captions host config (%lu bytes)", static_cast<unsigned long>(command.size()));
	return true;
}

} // namespace captions
