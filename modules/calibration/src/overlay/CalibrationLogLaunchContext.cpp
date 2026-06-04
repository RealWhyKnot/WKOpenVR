#include "CalibrationLogLaunchContext.h"

#include "Win32Text.h"

#include <cstdio>
#include <string>
#include <vector>

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <psapi.h>
#include <sddl.h>
#include <tlhelp32.h>

#pragma comment(lib, "Advapi32.lib")

namespace Metrics {
namespace {

std::string WideToUtf8(const wchar_t* w)
{
	return w ? openvr_pair::common::WideToUtf8(w) : std::string();
}

} // namespace

void WriteLaunchContextBanner(std::ostream& out)
{
	const DWORD pid = GetCurrentProcessId();
	out << "# launch_pid=" << pid << "\n";

	wchar_t exePath[MAX_PATH] = {};
	if (GetModuleFileNameW(nullptr, exePath, MAX_PATH)) {
		out << "# launch_exe_path=" << WideToUtf8(exePath) << "\n";
	}
	else {
		out << "# launch_exe_path=GetModuleFileNameW_failed_err=" << GetLastError() << "\n";
	}

	LPWSTR cmdline = GetCommandLineW();
	if (cmdline) {
		out << "# launch_command_line=" << WideToUtf8(cmdline) << "\n";
	}

	wchar_t cwd[MAX_PATH] = {};
	DWORD cwdLen = GetCurrentDirectoryW(MAX_PATH, cwd);
	if (cwdLen > 0 && cwdLen < MAX_PATH) {
		out << "# launch_cwd=" << WideToUtf8(cwd) << "\n";
	}
	else {
		out << "# launch_cwd=GetCurrentDirectoryW_failed_err=" << GetLastError() << "\n";
	}

	DWORD parentPid = 0;
	{
		HANDLE snap = CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0);
		if (snap != INVALID_HANDLE_VALUE) {
			PROCESSENTRY32W pe{};
			pe.dwSize = sizeof pe;
			if (Process32FirstW(snap, &pe)) {
				do {
					if (pe.th32ProcessID == pid) {
						parentPid = pe.th32ParentProcessID;
						break;
					}
				} while (Process32NextW(snap, &pe));
			}
			CloseHandle(snap);
		}
		else {
			out << "# launch_parent_lookup=snapshot_failed_err=" << GetLastError() << "\n";
		}
	}
	out << "# launch_parent_pid=" << parentPid << "\n";

	if (parentPid != 0) {
		HANDLE hParent = OpenProcess(PROCESS_QUERY_LIMITED_INFORMATION, FALSE, parentPid);
		if (hParent) {
			wchar_t parentExe[MAX_PATH] = {};
			DWORD len = MAX_PATH;
			if (QueryFullProcessImageNameW(hParent, 0, parentExe, &len)) {
				out << "# launch_parent_exe=" << WideToUtf8(parentExe) << "\n";
			}
			else {
				out << "# launch_parent_exe=QueryFullProcessImageNameW_failed_err=" << GetLastError() << "\n";
			}
			CloseHandle(hParent);
		}
		else {
			out << "# launch_parent_exe=OpenProcess_failed_err=" << GetLastError() << "\n";
		}
	}

	HANDLE token = nullptr;
	if (OpenProcessToken(GetCurrentProcess(), TOKEN_QUERY, &token) && token) {
		DWORD size = 0;
		GetTokenInformation(token, TokenIntegrityLevel, nullptr, 0, &size);
		if (size > 0) {
			std::vector<BYTE> buf(size);
			if (GetTokenInformation(token, TokenIntegrityLevel, buf.data(), size, &size)) {
				auto* tml = reinterpret_cast<TOKEN_MANDATORY_LABEL*>(buf.data());
				DWORD subAuth = *GetSidSubAuthority(tml->Label.Sid, static_cast<DWORD>(static_cast<UCHAR>(
				                                                        *GetSidSubAuthorityCount(tml->Label.Sid) - 1)));
				const char* levelName = "unknown";
				if (subAuth < 0x1000)
					levelName = "untrusted";
				else if (subAuth < 0x2000)
					levelName = "low";
				else if (subAuth < 0x3000)
					levelName = "medium";
				else if (subAuth < 0x4000)
					levelName = "high";
				else
					levelName = "system";
				char tmp[64];
				sprintf_s(tmp, "%s(0x%lx)", levelName, subAuth);
				out << "# launch_token_integrity=" << tmp << "\n";
			}
		}

		TOKEN_ELEVATION elevation{};
		DWORD elevSize = sizeof elevation;
		if (GetTokenInformation(token, TokenElevation, &elevation, elevSize, &elevSize)) {
			out << "# launch_token_elevated=" << (elevation.TokenIsElevated ? "yes" : "no") << "\n";
		}

		TOKEN_ELEVATION_TYPE elevType{};
		DWORD elevTypeSize = sizeof elevType;
		if (GetTokenInformation(token, TokenElevationType, &elevType, elevTypeSize, &elevTypeSize)) {
			const char* etName = "unknown";
			switch (elevType) {
				case TokenElevationTypeDefault:
					etName = "default";
					break;
				case TokenElevationTypeFull:
					etName = "full";
					break;
				case TokenElevationTypeLimited:
					etName = "limited";
					break;
			}
			out << "# launch_token_elevation_type=" << etName << "\n";
		}

		DWORD sessionId = 0;
		DWORD sessSize = sizeof sessionId;
		if (GetTokenInformation(token, TokenSessionId, &sessionId, sessSize, &sessSize)) {
			out << "# launch_session_id=" << sessionId << "\n";
		}

		DWORD tuSize = 0;
		GetTokenInformation(token, TokenUser, nullptr, 0, &tuSize);
		if (tuSize > 0) {
			std::vector<BYTE> tuBuf(tuSize);
			if (GetTokenInformation(token, TokenUser, tuBuf.data(), tuSize, &tuSize)) {
				auto* tu = reinterpret_cast<TOKEN_USER*>(tuBuf.data());
				LPWSTR sidStr = nullptr;
				if (ConvertSidToStringSidW(tu->User.Sid, &sidStr)) {
					out << "# launch_user_sid=" << WideToUtf8(sidStr) << "\n";
					LocalFree(sidStr);
				}
			}
		}

		CloseHandle(token);
	}
	else {
		out << "# launch_token=OpenProcessToken_failed_err=" << GetLastError() << "\n";
	}

	HWINSTA hwinsta = GetProcessWindowStation();
	if (hwinsta) {
		wchar_t wsname[256] = {};
		DWORD needed = 0;
		if (GetUserObjectInformationW(hwinsta, UOI_NAME, wsname, sizeof wsname, &needed)) {
			out << "# launch_winstation=" << WideToUtf8(wsname) << "\n";
		}
	}
	HDESK hdesk = GetThreadDesktop(GetCurrentThreadId());
	if (hdesk) {
		wchar_t deskname[256] = {};
		DWORD needed = 0;
		if (GetUserObjectInformationW(hdesk, UOI_NAME, deskname, sizeof deskname, &needed)) {
			out << "# launch_desktop=" << WideToUtf8(deskname) << "\n";
		}
	}

	HWND console = GetConsoleWindow();
	out << "# launch_console_attached=" << (console != nullptr ? "yes" : "no") << "\n";

	STARTUPINFOW si{};
	si.cb = sizeof si;
	GetStartupInfoW(&si);
	char sibuf[128];
	sprintf_s(sibuf, "0x%lx show_window=0x%x has_title=%d", si.dwFlags, static_cast<unsigned>(si.wShowWindow),
	          si.lpTitle ? 1 : 0);
	out << "# launch_startup_info=" << sibuf << "\n";
	if (si.lpTitle) {
		out << "# launch_startup_title=" << WideToUtf8(si.lpTitle) << "\n";
	}

	LPWCH env = GetEnvironmentStringsW();
	int envCount = 0;
	std::string vrEnvDump;
	if (env) {
		LPWCH p = env;
		while (*p) {
			envCount++;
			std::wstring entry(p);
			if (entry.rfind(L"VR_", 0) == 0 || entry.rfind(L"OPENVR_", 0) == 0 || entry.rfind(L"XR_", 0) == 0 ||
			    entry.rfind(L"STEAM_", 0) == 0 || entry.rfind(L"OVR_", 0) == 0 || entry.rfind(L"VRPATH", 0) == 0) {
				vrEnvDump += "# launch_vr_env=";
				vrEnvDump += WideToUtf8(p);
				vrEnvDump += "\n";
			}
			while (*p)
				p++;
			p++;
		}
		FreeEnvironmentStringsW(env);
	}
	out << "# launch_env_var_count=" << envCount << "\n";
	out << vrEnvDump;
}

} // namespace Metrics
