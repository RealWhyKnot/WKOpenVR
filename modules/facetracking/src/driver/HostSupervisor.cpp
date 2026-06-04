#define _CRT_SECURE_NO_DEPRECATE
#include "HostSupervisor.h"
#include "DebugLogging.h"
#include "LogPaths.h"
#include "Logging.h"
#include "Win32Paths.h"

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <sddl.h>

#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <vector>

#define FT_HOST_CONTROL_PIPE_NAME "\\\\.\\pipe\\WKOpenVR-FaceTracking.host"

namespace facetracking {
namespace {

std::wstring QuoteArg(const std::wstring& value)
{
	std::wstring out = L"\"";
	for (wchar_t ch : value) {
		if (ch == L'"')
			out += L"\\\"";
		else
			out.push_back(ch);
	}
	out += L"\"";
	return out;
}

void AppendPathArg(std::wstring& commandLine, const wchar_t* name, const std::wstring& value)
{
	if (value.empty()) return;
	commandLine += L" ";
	commandLine += name;
	commandLine += L" ";
	commandLine += QuoteArg(value);
}

// Minimal CBOR encoder for the control-pipe wire. The host's
// HostControlPipeServer reads [4-byte LE length][CBOR map]. We only need
// to emit one shape:
//   { "type": "SelectModule", "uuid": "<uuid>" }
// All keys and values are short ASCII text strings.
//
// CBOR text-string framing:
//   0..23  : single byte 0x60 + len, followed by the bytes
//   24..255: byte 0x78, one length byte, then the bytes
// All strings used here fit one of those two cases.
void CborAppendTextString(std::string& out, const char* s, size_t len)
{
	if (len < 24) {
		out.push_back(static_cast<char>(0x60 | static_cast<unsigned char>(len)));
	}
	else {
		out.push_back(static_cast<char>(0x78));
		out.push_back(static_cast<char>(len & 0xff));
	}
	out.append(s, len);
}

std::string EncodeSelectModule(const std::string& uuid)
{
	std::string body;
	body.push_back(static_cast<char>(0xA2)); // map with 2 pairs
	CborAppendTextString(body, "type", 4);
	CborAppendTextString(body, "SelectModule", 12);
	CborAppendTextString(body, "uuid", 4);
	CborAppendTextString(body, uuid.c_str(), uuid.size());

	std::string wire;
	wire.reserve(4 + body.size());
	const uint32_t len = static_cast<uint32_t>(body.size());
	wire.push_back(static_cast<char>(len & 0xff));
	wire.push_back(static_cast<char>((len >> 8) & 0xff));
	wire.push_back(static_cast<char>((len >> 16) & 0xff));
	wire.push_back(static_cast<char>((len >> 24) & 0xff));
	wire.append(body);
	return wire;
}

// Returns the current user's SID as the canonical S-1-... string. Matches
// what the C# host writes via WindowsIdentity.GetCurrent().User.Value so
// the supervisor can probe the same mutex name from the driver side.
// Returns an empty string on failure (caller falls back to pipe-only
// attach detection).
std::wstring GetCurrentUserSidString()
{
	HANDLE token = nullptr;
	if (!OpenProcessToken(GetCurrentProcess(), TOKEN_QUERY, &token)) return {};

	DWORD needed = 0;
	GetTokenInformation(token, TokenUser, nullptr, 0, &needed);
	if (needed == 0) {
		CloseHandle(token);
		return {};
	}

	std::vector<unsigned char> buf(needed);
	if (!GetTokenInformation(token, TokenUser, buf.data(), needed, &needed)) {
		CloseHandle(token);
		return {};
	}
	CloseHandle(token);

	PSID sid = reinterpret_cast<TOKEN_USER*>(buf.data())->User.Sid;
	LPWSTR sidStr = nullptr;
	if (!ConvertSidToStringSidW(sid, &sidStr) || !sidStr) return {};

	std::wstring result(sidStr);
	LocalFree(sidStr);
	return result;
}

} // namespace

HostSupervisor::HostSupervisor(const std::string& host_exe_path) : HostSupervisorBase(host_exe_path) {}

std::string HostSupervisor::ControlPipeName() const
{
	return FT_HOST_CONTROL_PIPE_NAME;
}

std::wstring HostSupervisor::SingletonMutexName() const
{
	std::wstring sid = GetCurrentUserSidString();
	if (sid.empty()) return {};
	return L"Global\\WKOpenVR-FaceModuleHost-Singleton-" + sid;
}

void HostSupervisor::BuildCommandLine(std::wstring& commandLine, const std::wstring& /*exe_path*/) const
{
	const bool debugLogging = openvr_pair::common::IsDebugLoggingEnabled();
	if (debugLogging) {
		commandLine += L" --debug-logging 1";
	}
	std::wstring faceDir = openvr_pair::common::WkOpenVrSubdirectoryPath(L"facetracking", true);
	AppendPathArg(commandLine, L"--status-file", faceDir.empty() ? std::wstring{} : faceDir + L"\\host_status.json");
	AppendPathArg(commandLine, L"--modules-dir", faceDir.empty() ? std::wstring{} : faceDir + L"\\modules");
	if (debugLogging) {
		AppendPathArg(commandLine, L"--log-file", openvr_pair::common::TimestampedLogPath(L"facetracking_host_log"));
	}
}

void HostSupervisor::LogV(const char* fmt, va_list args)
{
	if (!FtDrvEnsureLogFileOpen()) return;
	tm now = FtDrvTimeForLog();
	fprintf(FtDrvLogFile, "[%02d:%02d:%02d] ", now.tm_hour, now.tm_min, now.tm_sec);
	vfprintf(FtDrvLogFile, fmt, args);
	fputc('\n', FtDrvLogFile);
	FtDrvLogFlush();
}

void HostSupervisor::SetActiveModuleUuid(const char* uuid)
{
	std::string s = uuid ? std::string(uuid) : std::string();
	{
		std::lock_guard<std::mutex> lk(uuid_mutex_);
		pending_uuid_ = s;
		has_pending_uuid_ = true;
		uuid_sent_ = false;
	}
	TrySendUuid(s);
}

void HostSupervisor::OnHostReady()
{
	std::string uuid;
	bool should_send = false;
	{
		std::lock_guard<std::mutex> lk(uuid_mutex_);
		if (has_pending_uuid_ && !uuid_sent_) {
			uuid = pending_uuid_;
			should_send = true;
		}
	}
	if (should_send) TrySendUuid(uuid);
}

void HostSupervisor::OnHostExited()
{
	std::lock_guard<std::mutex> lk(uuid_mutex_);
	if (has_pending_uuid_) uuid_sent_ = false;
}

bool HostSupervisor::TrySendUuid(const std::string& uuid)
{
	std::string msg = EncodeSelectModule(uuid);
	if (!SendBytesOverControlPipe(msg.data(), msg.size())) {
		return false;
	}
	{
		std::lock_guard<std::mutex> lk(uuid_mutex_);
		uuid_sent_ = true;
	}
	Log("[host] sent SelectModule uuid '%s'", uuid.c_str());
	return true;
}

} // namespace facetracking
